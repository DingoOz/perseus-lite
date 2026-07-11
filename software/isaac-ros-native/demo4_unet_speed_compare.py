"""
Demo 4 (see plan.md): U-Net segmentation TensorRT (GPU) vs ONNX Runtime
(CPU) -- same direct-Python-API structure as Demo 3 (no ROS in the timing
loop at all), applied to Stage 7's dense per-pixel segmentation model
instead of YOLOv8's detector. Segmentation does far more FLOPs/pixel than
detection, so the GPU speedup ratio is expected to be the largest of any
demo in this series.

model.dummy.onnx (Stage 7's fixture, random weights -- see
unet_launch.py/unet_check.py) has a DYNAMIC batch dimension
((-1, 3, 544, 960) input, (-1, 544, 960, 20) output) unlike YOLOv8's
fixed batch of 1, so this needs an explicit TensorRT optimization
profile (min=opt=max=1) at engine-build time and a per-context
set_input_shape() call before inference -- YOLOv8's model didn't need
either.
"""
import json
import pathlib
import sys
import time

sys.path.insert(0, '/usr/lib/python3.12/dist-packages')

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import tensorrt as trt
from cuda.bindings import runtime as cudart

THIS_DIR = pathlib.Path(__file__).parent
MODEL_PATH = THIS_DIR / 'models' / 'model.dummy.onnx'
ENGINE_PATH = THIS_DIR / 'models' / 'unet_dummy_demo4.plan'
IMAGE_PATH = (THIS_DIR / 'isaac_ros_image_segmentation' / 'isaac_ros_unet' /
              'test' / 'test_cases' / 'unet_sample' / 'image.jpg')

NETWORK_WIDTH = 960
NETWORK_HEIGHT = 544
INPUT_SHAPE = (1, 3, NETWORK_HEIGHT, NETWORK_WIDTH)
OUTPUT_SHAPE = (1, NETWORK_HEIGHT, NETWORK_WIDTH, 20)
INPUT_NAME = 'input_1'
OUTPUT_NAME = 'softmax_1'
NUM_WARMUP = 5
NUM_TIMED = 30  # heavier model than YOLOv8s -- fewer timed iterations


def load_input_tensor():
    bgr = cv2.imread(str(IMAGE_PATH))
    bgr = cv2.resize(bgr, (NETWORK_WIDTH, NETWORK_HEIGHT))
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    chw = np.transpose(rgb, (2, 0, 1))
    return np.ascontiguousarray(chw[np.newaxis, ...])


def build_or_load_engine():
    logger = trt.Logger(trt.Logger.WARNING)
    if ENGINE_PATH.exists():
        print(f'Loading cached engine: {ENGINE_PATH}')
        runtime = trt.Runtime(logger)
        with open(ENGINE_PATH, 'rb') as f:
            return runtime.deserialize_cuda_engine(f.read())

    print('Building TensorRT engine (first run only, cached after)...')
    builder = trt.Builder(logger)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser = trt.OnnxParser(network, logger)
    with open(MODEL_PATH, 'rb') as f:
        if not parser.parse(f.read()):
            for i in range(parser.num_errors):
                print(parser.get_error(i))
            raise RuntimeError('Failed to parse ONNX model')

    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 512 * 1024 * 1024)

    # Dynamic batch dim -- fix it at 1 via an explicit optimization profile.
    profile = builder.create_optimization_profile()
    profile.set_shape(INPUT_NAME, INPUT_SHAPE, INPUT_SHAPE, INPUT_SHAPE)
    config.add_optimization_profile(profile)

    t0 = time.time()
    serialized = builder.build_serialized_network(network, config)
    print(f'Engine built in {time.time() - t0:.1f}s')

    with open(ENGINE_PATH, 'wb') as f:
        f.write(serialized)

    runtime = trt.Runtime(logger)
    return runtime.deserialize_cuda_engine(serialized)


def benchmark_tensorrt(input_np):
    engine = build_or_load_engine()
    context = engine.create_execution_context()
    context.set_input_shape(INPUT_NAME, INPUT_SHAPE)

    input_nbytes = int(np.prod(INPUT_SHAPE)) * 4
    output_nbytes = int(np.prod(OUTPUT_SHAPE)) * 4

    err, d_input = cudart.cudaMalloc(input_nbytes)
    if err != cudart.cudaError_t.cudaSuccess:
        raise RuntimeError(f'cudaMalloc(input) failed: {err}')
    err, d_output = cudart.cudaMalloc(output_nbytes)
    if err != cudart.cudaError_t.cudaSuccess:
        raise RuntimeError(f'cudaMalloc(output) failed: {err}')

    context.set_tensor_address(INPUT_NAME, d_input)
    context.set_tensor_address(OUTPUT_NAME, d_output)

    err, stream = cudart.cudaStreamCreate()
    if err != cudart.cudaError_t.cudaSuccess:
        raise RuntimeError(f'cudaStreamCreate failed: {err}')

    host_input = np.ascontiguousarray(input_np, dtype=np.float32)
    host_output = np.empty(OUTPUT_SHAPE, dtype=np.float32)

    def run_once():
        (err,) = cudart.cudaMemcpyAsync(
            d_input, host_input.ctypes.data, input_nbytes,
            cudart.cudaMemcpyKind.cudaMemcpyHostToDevice, stream)
        if err != cudart.cudaError_t.cudaSuccess:
            raise RuntimeError(f'H2D memcpy failed: {err}')
        context.execute_async_v3(stream_handle=stream)
        (err,) = cudart.cudaMemcpyAsync(
            host_output.ctypes.data, d_output, output_nbytes,
            cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost, stream)
        if err != cudart.cudaError_t.cudaSuccess:
            raise RuntimeError(f'D2H memcpy failed: {err}')
        (err,) = cudart.cudaStreamSynchronize(stream)
        if err != cudart.cudaError_t.cudaSuccess:
            raise RuntimeError(f'stream sync failed: {err}')

    for _ in range(NUM_WARMUP):
        run_once()

    times = []
    for _ in range(NUM_TIMED):
        t0 = time.perf_counter()
        run_once()
        times.append(time.perf_counter() - t0)

    cudart.cudaFree(d_input)
    cudart.cudaFree(d_output)
    cudart.cudaStreamDestroy(stream)

    return times, host_output.copy()


def benchmark_onnxruntime(input_np):
    import onnxruntime as ort
    session = ort.InferenceSession(str(MODEL_PATH), providers=['CPUExecutionProvider'])
    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name

    host_input = np.ascontiguousarray(input_np, dtype=np.float32)

    for _ in range(NUM_WARMUP):
        session.run([output_name], {input_name: host_input})

    times = []
    output = None
    for _ in range(NUM_TIMED):
        t0 = time.perf_counter()
        output = session.run([output_name], {input_name: host_input})[0]
        times.append(time.perf_counter() - t0)

    return times, output


def summarize(name, times):
    arr = np.array(times) * 1000.0
    mean_ms = float(arr.mean())
    p50 = float(np.percentile(arr, 50))
    p99 = float(np.percentile(arr, 99))
    fps = 1000.0 / mean_ms if mean_ms > 0 else 0.0
    print(f'{name}: mean={mean_ms:.3f}ms p50={p50:.3f}ms p99={p99:.3f}ms -> {fps:.1f} fps')
    return {'mean_ms': mean_ms, 'p50_ms': p50, 'p99_ms': p99, 'fps': fps}


def build_chart_image(trt_stats, ort_stats, speedup):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 4))

    names = ['TensorRT\n(GPU)', 'ONNX Runtime\n(CPU)']
    means = [trt_stats['mean_ms'], ort_stats['mean_ms']]
    colors = ['#3f8a6f', '#c9822f']
    ax1.bar(names, means, color=colors)
    for i, m in enumerate(means):
        ax1.text(i, m + max(means) * 0.02, f'{m:.2f}ms', ha='center', fontsize=10)
    ax1.set_ylabel('mean inference time (ms)')
    ax1.set_title(f'U-Net: TensorRT vs ONNX Runtime\n{speedup:.1f}x speedup (GPU vs CPU)')

    fpss = [trt_stats['fps'], ort_stats['fps']]
    ax2.bar(names, fpss, color=colors)
    for i, f in enumerate(fpss):
        ax2.text(i, f + max(fpss) * 0.02, f'{f:.1f} fps', ha='center', fontsize=10)
    ax2.set_ylabel('throughput (fps)')
    ax2.set_title('Direct Python API, no ROS overhead')

    fig.tight_layout()
    fig.canvas.draw()
    buf = np.asarray(fig.canvas.buffer_rgba())
    bgr = cv2.cvtColor(buf, cv2.COLOR_RGBA2BGR)
    plt.close(fig)
    return bgr


def publish_result(image_bgr, duration_sec=120):
    import rclpy
    from rclpy.node import Node
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image as RosImage

    class ResultPublisher(Node):
        def __init__(self):
            super().__init__('demo4_unet_result_publisher')
            self.pub = self.create_publisher(RosImage, 'demo4_unet_comparison', 10)
            self.bridge = CvBridge()
            self.msg = self.bridge.cv2_to_imgmsg(image_bgr, encoding='bgr8')
            self.create_timer(1.0, self.publish_once)
            self.get_logger().info(
                f'Publishing result to /demo4_unet_comparison for {duration_sec}s -- '
                'view with RViz2 (Image display) or rqt_image_view on a laptop on this '
                'ROS domain')

        def publish_once(self):
            self.msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self.msg)

    rclpy.init()
    node = ResultPublisher()
    end_time = time.time() + duration_sec
    while time.time() < end_time:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


def main():
    input_np = load_input_tensor()

    print(f'--- TensorRT GPU: {NUM_WARMUP} warmup + {NUM_TIMED} timed calls ---')
    trt_times, trt_output = benchmark_tensorrt(input_np)
    trt_stats = summarize('TensorRT (GPU)', trt_times)

    print(f'--- ONNX Runtime CPU: {NUM_WARMUP} warmup + {NUM_TIMED} timed calls ---')
    ort_times, ort_output = benchmark_onnxruntime(input_np)
    ort_stats = summarize('ONNX Runtime (CPU)', ort_times)

    speedup = ort_stats['mean_ms'] / trt_stats['mean_ms']
    print(f'\nSpeedup (GPU vs CPU): {speedup:.1f}x')

    with open(THIS_DIR / 'demo4_result.json', 'w') as f:
        json.dump({'tensorrt_gpu': trt_stats, 'onnxruntime_cpu': ort_stats,
                    'speedup': speedup, 'num_timed_calls': NUM_TIMED}, f, indent=2)
    print(f'Saved {THIS_DIR / "demo4_result.json"}')

    chart = build_chart_image(trt_stats, ort_stats, speedup)
    cv2.imwrite(str(THIS_DIR / 'demo4_result.png'), chart)
    print(f'Saved {THIS_DIR / "demo4_result.png"}')

    publish_result(chart)


if __name__ == '__main__':
    main()
