"""
Demo 3 (see plan.md): YOLOv8 TensorRT (GPU) vs ONNX Runtime (CPU),
direct Python API on both sides -- no ROS in the timing loop at all.

Demo 1 spent most of its effort discovering that ROS2/DDS
message-passing throughput, not detector compute time, dominated a
live comparison built on ROS pub/sub. Rather than risk repeating that,
this demo sidesteps ROS entirely for the timed portion: TensorRT
(python3-libnvinfer, the system apt package matching the already-
installed tensorrt-dev -- not on PyPI for Jetson, so it's reached via
PYTHONPATH into /usr/lib/python3.12/dist-packages, not a pixi
dependency) builds and runs the same dummy_yolov8s.onnx model Stage 5
follow-up already validated end to end through the ROS pipeline;
onnxruntime's CPUExecutionProvider runs the identical model. Both
inference calls are timed directly, in a tight loop, in the same
process -- no publish/subscribe, no serialization, no DDS.

dummy_yolov8s.onnx has RANDOM weights (see yolov8_launch.py/
yolov8_check.py's docstrings) -- detection content is meaningless, so
this is purely a throughput comparison, same scope as every dummy-model
stage before it. There's no recorded video handy on this headless
robot, so the same single 640x640 test image already used by Stage 5
follow-up (isaac_ros_yolov8's own people_cycles.jpg fixture) is reused
as every "frame" -- fine for a throughput benchmark since the network
does the identical amount of work regardless of pixel content.

GPU device buffer allocation/copy uses cuda-python's newer
cuda.bindings.runtime API (cuda-python 13.x moved off the older
cuda.cuda/cuda.cudart module names used in most older TensorRT sample
code still floating around online) -- confirmed working with a
standalone cudaMalloc/cudaMemcpy round-trip test before relying on it.
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
MODEL_PATH = THIS_DIR / 'models' / 'dummy_yolov8s.onnx'
ENGINE_PATH = THIS_DIR / 'models' / 'dummy_yolov8s_demo3.plan'
IMAGE_PATH = (THIS_DIR / 'isaac_ros_object_detection' / 'isaac_ros_yolov8' /
              'test' / 'test_cases' / 'single_detection' / 'people_cycles.jpg')

INPUT_SHAPE = (1, 3, 640, 640)
OUTPUT_SHAPE = (1, 84, 8400)
NUM_WARMUP = 5
NUM_TIMED = 100


def load_input_tensor():
    bgr = cv2.imread(str(IMAGE_PATH))
    bgr = cv2.resize(bgr, (640, 640))
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    chw = np.transpose(rgb, (2, 0, 1))
    return np.ascontiguousarray(chw[np.newaxis, ...])  # (1, 3, 640, 640)


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

    input_nbytes = int(np.prod(INPUT_SHAPE)) * 4
    output_nbytes = int(np.prod(OUTPUT_SHAPE)) * 4

    err, d_input = cudart.cudaMalloc(input_nbytes)
    if err != cudart.cudaError_t.cudaSuccess:
        raise RuntimeError(f'cudaMalloc(input) failed: {err}')
    err, d_output = cudart.cudaMalloc(output_nbytes)
    if err != cudart.cudaError_t.cudaSuccess:
        raise RuntimeError(f'cudaMalloc(output) failed: {err}')

    context.set_tensor_address('images', d_input)
    context.set_tensor_address('output0', d_output)

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
    arr = np.array(times) * 1000.0  # ms
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
    ax1.set_title(f'YOLOv8: TensorRT vs ONNX Runtime\n{speedup:.1f}x speedup (GPU vs CPU)')

    fpss = [trt_stats['fps'], ort_stats['fps']]
    ax2.bar(names, fpss, color=colors)
    for i, f in enumerate(fpss):
        ax2.text(i, f + max(fpss) * 0.02, f'{f:.0f} fps', ha='center', fontsize=10)
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
            super().__init__('demo3_yolov8_result_publisher')
            self.pub = self.create_publisher(RosImage, 'demo3_yolov8_comparison', 10)
            self.bridge = CvBridge()
            self.msg = self.bridge.cv2_to_imgmsg(image_bgr, encoding='bgr8')
            self.create_timer(1.0, self.publish_once)
            self.get_logger().info(
                f'Publishing result to /demo3_yolov8_comparison for {duration_sec}s -- '
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

    with open(THIS_DIR / 'demo3_result.json', 'w') as f:
        json.dump({'tensorrt_gpu': trt_stats, 'onnxruntime_cpu': ort_stats,
                    'speedup': speedup, 'num_timed_calls': NUM_TIMED}, f, indent=2)
    print(f'Saved {THIS_DIR / "demo3_result.json"}')

    chart = build_chart_image(trt_stats, ort_stats, speedup)
    cv2.imwrite(str(THIS_DIR / 'demo3_result.png'), chart)
    print(f'Saved {THIS_DIR / "demo3_result.png"}')

    publish_result(chart)


if __name__ == '__main__':
    main()
