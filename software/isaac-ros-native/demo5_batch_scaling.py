"""
Demo 5 (see plan.md): batch-size scaling study, TensorRT (GPU) vs ONNX
Runtime (CPU).

Design note: the original plan.md description for #5 ("quantitative
throughput benchmark... plot images/sec and p50/p90/p99 latency") turned
out to be largely redundant with what Demos 3 and 4 already deliver --
both already compute mean/p50/p99 latency and an fps chart from a tight
direct-API timing loop. The one dimension neither demo tested is
batching, so #5 is reshaped into a batch-size scaling study instead:
sweep batch size on Stage 7's dynamic-batch U-Net model and plot
throughput vs batch size for both backends. This is the sharpest way to
show *why* GPU inference matters beyond raw single-image latency: a GPU
can process a batch largely in parallel, so throughput keeps climbing as
batch size grows, while a CPU's per-image cost is roughly fixed --
throughput stays flat (or worsens slightly from cache pressure).

Reuses model.dummy.onnx (same fixture as Demo 4), but builds a *separate*
TensorRT engine (unet_dummy_demo5.plan) with an optimization profile
spanning the full batch range (min=1, opt=mid, max=MAX_BATCH) instead of
Demo 4's fixed batch=1 profile -- a fixed-batch=1 engine cannot run
larger batches at all.

ONNX Runtime CPU is intentionally given fewer warmup/timed iterations at
each batch size (see NUM_WARMUP_CPU/NUM_TIMED_CPU) since its per-image
cost is roughly 25x TensorRT's (see Demo 4's result) and scales
~linearly with batch size -- without this, the full sweep would take
several minutes longer for no extra signal.

Actual result did not confirm the "GPU throughput climbs with batch"
hypothesis above: TensorRT's per-image latency scaled essentially
perfectly linearly with batch size (8.68/8.71/8.74/8.74 img/s at
batch 1/2/4/8 -- 1.01x from batch=1 to batch=8's best), i.e. no
parallelism headroom was exploited. See plan.md's Demo 5 section for
the full discussion of why (the Orin Nano's iGPU is compute-bound per
image at this model size, not launch/dispatch-overhead-bound, so
there's no idle throughput for batching to reclaim).
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
ENGINE_PATH = THIS_DIR / 'models' / 'unet_dummy_demo5.plan'
IMAGE_PATH = (THIS_DIR / 'isaac_ros_image_segmentation' / 'isaac_ros_unet' /
              'test' / 'test_cases' / 'unet_sample' / 'image.jpg')

NETWORK_WIDTH = 960
NETWORK_HEIGHT = 544
INPUT_NAME = 'input_1'
OUTPUT_NAME = 'softmax_1'

BATCH_SIZES = [1, 2, 4, 8]
MAX_BATCH = max(BATCH_SIZES)

NUM_WARMUP_GPU = 2
NUM_TIMED_GPU = 5
# CPU is ~25x slower per image (see Demo 4) and scales ~linearly with
# batch size -- fewer repeats keep the whole sweep under ~2 minutes.
NUM_WARMUP_CPU = 1
NUM_TIMED_CPU = 2


def load_single_image():
    bgr = cv2.imread(str(IMAGE_PATH))
    bgr = cv2.resize(bgr, (NETWORK_WIDTH, NETWORK_HEIGHT))
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    chw = np.transpose(rgb, (2, 0, 1))
    return chw


def make_batch(single_chw, batch_size):
    batch = np.repeat(single_chw[np.newaxis, ...], batch_size, axis=0)
    return np.ascontiguousarray(batch, dtype=np.float32)


def build_or_load_engine():
    logger = trt.Logger(trt.Logger.WARNING)
    if ENGINE_PATH.exists():
        print(f'Loading cached engine: {ENGINE_PATH}')
        runtime = trt.Runtime(logger)
        with open(ENGINE_PATH, 'rb') as f:
            return runtime.deserialize_cuda_engine(f.read())

    print(f'Building TensorRT engine with batch profile 1..{MAX_BATCH} '
          '(first run only, cached after)...')
    builder = trt.Builder(logger)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser = trt.OnnxParser(network, logger)
    with open(MODEL_PATH, 'rb') as f:
        if not parser.parse(f.read()):
            for i in range(parser.num_errors):
                print(parser.get_error(i))
            raise RuntimeError('Failed to parse ONNX model')

    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1024 * 1024 * 1024)

    min_shape = (1, 3, NETWORK_HEIGHT, NETWORK_WIDTH)
    opt_shape = (MAX_BATCH // 2, 3, NETWORK_HEIGHT, NETWORK_WIDTH)
    max_shape = (MAX_BATCH, 3, NETWORK_HEIGHT, NETWORK_WIDTH)
    profile = builder.create_optimization_profile()
    profile.set_shape(INPUT_NAME, min_shape, opt_shape, max_shape)
    config.add_optimization_profile(profile)

    t0 = time.time()
    serialized = builder.build_serialized_network(network, config)
    print(f'Engine built in {time.time() - t0:.1f}s')

    with open(ENGINE_PATH, 'wb') as f:
        f.write(serialized)

    runtime = trt.Runtime(logger)
    return runtime.deserialize_cuda_engine(serialized)


def benchmark_tensorrt_at_batch(engine, batch_size, single_chw):
    input_shape = (batch_size, 3, NETWORK_HEIGHT, NETWORK_WIDTH)
    output_shape = (batch_size, NETWORK_HEIGHT, NETWORK_WIDTH, 20)

    context = engine.create_execution_context()
    context.set_input_shape(INPUT_NAME, input_shape)

    input_nbytes = int(np.prod(input_shape)) * 4
    output_nbytes = int(np.prod(output_shape)) * 4

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

    host_input = make_batch(single_chw, batch_size)
    host_output = np.empty(output_shape, dtype=np.float32)

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

    for _ in range(NUM_WARMUP_GPU):
        run_once()

    times = []
    for _ in range(NUM_TIMED_GPU):
        t0 = time.perf_counter()
        run_once()
        times.append(time.perf_counter() - t0)

    cudart.cudaFree(d_input)
    cudart.cudaFree(d_output)
    cudart.cudaStreamDestroy(stream)

    return times


def benchmark_onnxruntime_at_batch(session, batch_size, single_chw):
    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name
    host_input = make_batch(single_chw, batch_size)

    for _ in range(NUM_WARMUP_CPU):
        session.run([output_name], {input_name: host_input})

    times = []
    for _ in range(NUM_TIMED_CPU):
        t0 = time.perf_counter()
        session.run([output_name], {input_name: host_input})
        times.append(time.perf_counter() - t0)

    return times


def summarize_batch(batch_size, times):
    arr = np.array(times) * 1000.0
    mean_ms = float(arr.mean())
    images_per_sec = (batch_size * 1000.0) / mean_ms if mean_ms > 0 else 0.0
    print(f'  batch={batch_size:2d}: mean={mean_ms:8.2f} ms/call -> '
          f'{images_per_sec:7.1f} images/sec')
    return {'batch_size': batch_size, 'mean_ms_per_call': mean_ms,
            'images_per_sec': images_per_sec}


def build_chart_image(gpu_results, cpu_results):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))

    gpu_batches = [r['batch_size'] for r in gpu_results]
    gpu_fps = [r['images_per_sec'] for r in gpu_results]
    cpu_batches = [r['batch_size'] for r in cpu_results]
    cpu_fps = [r['images_per_sec'] for r in cpu_results]

    ax1.plot(gpu_batches, gpu_fps, 'o-', color='#3f8a6f', label='TensorRT (GPU)')
    ax1.plot(cpu_batches, cpu_fps, 's-', color='#c9822f', label='ONNX Runtime (CPU)')
    ax1.set_xlabel('batch size')
    ax1.set_ylabel('throughput (images/sec)')
    ax1.set_title('U-Net throughput vs batch size')
    ax1.set_xticks(BATCH_SIZES)
    ax1.legend()
    ax1.grid(alpha=0.3)

    ax2.plot(gpu_batches, gpu_fps, 'o-', color='#3f8a6f', label='TensorRT (GPU)')
    ax2.plot(cpu_batches, cpu_fps, 's-', color='#c9822f', label='ONNX Runtime (CPU)')
    ax2.set_yscale('log')
    ax2.set_xlabel('batch size')
    ax2.set_ylabel('throughput (images/sec, log scale)')
    ax2.set_title('Same data, log scale\n(neither backend gains throughput from batching here)')
    ax2.set_xticks(BATCH_SIZES)
    ax2.legend()
    ax2.grid(alpha=0.3, which='both')

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
            super().__init__('demo5_batch_scaling_result_publisher')
            self.pub = self.create_publisher(RosImage, 'demo5_batch_scaling', 10)
            self.bridge = CvBridge()
            self.msg = self.bridge.cv2_to_imgmsg(image_bgr, encoding='bgr8')
            self.create_timer(1.0, self.publish_once)
            self.get_logger().info(
                f'Publishing result to /demo5_batch_scaling for {duration_sec}s -- '
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
    import onnxruntime as ort

    single_chw = load_single_image()

    print(f'--- TensorRT GPU: batch sweep {BATCH_SIZES} '
          f'({NUM_WARMUP_GPU} warmup + {NUM_TIMED_GPU} timed per batch) ---')
    engine = build_or_load_engine()
    gpu_results = []
    for batch_size in BATCH_SIZES:
        times = benchmark_tensorrt_at_batch(engine, batch_size, single_chw)
        gpu_results.append(summarize_batch(batch_size, times))

    print(f'--- ONNX Runtime CPU: batch sweep {BATCH_SIZES} '
          f'({NUM_WARMUP_CPU} warmup + {NUM_TIMED_CPU} timed per batch) ---')
    session = ort.InferenceSession(str(MODEL_PATH), providers=['CPUExecutionProvider'])
    cpu_results = []
    for batch_size in BATCH_SIZES:
        times = benchmark_onnxruntime_at_batch(session, batch_size, single_chw)
        cpu_results.append(summarize_batch(batch_size, times))

    gpu_best = max(r['images_per_sec'] for r in gpu_results)
    cpu_best = max(r['images_per_sec'] for r in cpu_results)
    print(f'\nBest throughput -- TensorRT: {gpu_best:.1f} img/s (batch scaling: '
          f'{gpu_best / gpu_results[0]["images_per_sec"]:.2f}x from batch=1), '
          f'ONNX Runtime CPU: {cpu_best:.1f} img/s (batch scaling: '
          f'{cpu_best / cpu_results[0]["images_per_sec"]:.2f}x from batch=1)')

    with open(THIS_DIR / 'demo5_result.json', 'w') as f:
        json.dump({'tensorrt_gpu': gpu_results, 'onnxruntime_cpu': cpu_results,
                    'batch_sizes': BATCH_SIZES}, f, indent=2)
    print(f'Saved {THIS_DIR / "demo5_result.json"}')

    chart = build_chart_image(gpu_results, cpu_results)
    cv2.imwrite(str(THIS_DIR / 'demo5_result.png'), chart)
    print(f'Saved {THIS_DIR / "demo5_result.png"}')

    publish_result(chart)


if __name__ == '__main__':
    main()
