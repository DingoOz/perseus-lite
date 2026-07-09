"""
Demo 2 (see plan.md): occupancy-grid localizer GPU vs CPU, using
NVIDIA's own real code on both sides -- not a reimplementation.

isaac_ros_occupancy_grid_localizer's compiled node
(occupancy_grid_localizer_core.cpp) already contains BOTH a GPU path
(SearchCandidatesGpu, batching every candidate pose through one
gpu_->ScorePoses() CUDA kernel call) and a genuine CPU fallback path
(SearchCandidates -> ScorePose -> RaycastRange, a serial per-pose,
per-beam ray-marching loop) -- LoadMap() falls back to CPU automatically
if the GPU context fails to initialize:

    try {
      gpu_ = std::make_unique<OccupancyGridLocalizerGpu>(...);
      gpu_->UploadMap(...);
    } catch (const std::runtime_error &) {
      gpu_.reset();  // GPU unavailable, will fall back to CPU
    }

Both paths run the identical hierarchical coarse -> medium -> fine grid
search (same candidate pose lists, same beam data, same scoring math) --
only the score-computation step differs. Rather than reimplement this
algorithm ourselves (real risk of an unfair or subtly-wrong comparison,
as Demo 1 turned out to hinge entirely on getting the *methodology*
right, not just the code), this demo forces the SAME compiled binary
down each path by hiding the GPU from CUDA entirely:

    CUDA_VISIBLE_DEVICES="" ros2 launch ogl_launch.py

Confirmed this actually works on this Jetson (not guaranteed on an
integrated-GPU platform) via a standalone ctypes cudaMalloc call before
building this: cudaGetDeviceCount/cudaMalloc both return
cudaErrorNoDevice (100) under an empty CUDA_VISIBLE_DEVICES, which
IS a std::runtime_error-worthy CUDA_CHECK failure in
occupancy_grid_localizer_gpu.cu's constructor, taking the real fallback
path -- not a simulation of it.

The coarse search level alone sweeps the *entire* bundled map
(38.7m x 22.25m at 0.05 m/cell) at 0.5m position / 5-degree yaw
resolution -- roughly a quarter million candidate poses, each scored
against up to 128 lidar beams. Real compute, not a toy problem.

Runs the full pipeline (rosbag replay -> trigger_grid_search_localization
-> localization_result) twice, sequentially, once per mode, timing
wall-clock from bag-playback start to result-received each time (this
includes ~1.1s of bag-playback latency on both sides -- a small, roughly
equal systematic bias, not something that favors either side; labeled
honestly rather than claimed as a pure isolated compute measurement,
per the lesson from Demo 1). Renders both recovered poses on the real
map image with the timing numbers, and publishes the result on a ROS
topic for a few minutes so it can be viewed on a laptop via RViz2/
rqt_image_view before the script exits.

Actual result (this Jetson, first real run): GPU completed the full
three-level coarse->medium->fine search in 20.58s, recovering the same
pose Stage 12 originally verified (33.60, 7.70) against NVIDIA's ground
truth. The CPU fallback path did not complete even the coarse level (the
~265k-pose full-map sweep) within 900 seconds (15 minutes) -- genuinely
dramatic, and an honest result in its own right: a >44x speedup that
undersells the real gap, since CPU never actually finished.
TRIGGER_TIMEOUT_SEC is set to a more practical 180s below for routine
re-runs (an unbounded 15+ minute wait isn't reasonable default demo UX);
expect the CPU side to reliably report FAILED/TIMED OUT at that setting
too, which is itself the finding, not a bug -- raise it back up if you
want to see how long it actually takes to finish (untested how much
longer than 900s that would be).
"""
import json
import math
import os
import pathlib
import subprocess
import sys
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from std_srvs.srv import Empty

THIS_DIR = pathlib.Path(__file__).parent
OGL_DIR = THIS_DIR / 'isaac_ros_mapping_and_localization' / 'isaac_ros_occupancy_grid_localizer'
BAG_PATH = OGL_DIR / 'data' / 'rosbags' / 'flatscan'
MAP_PNG_PATH = OGL_DIR / 'maps' / 'map.png'
LAUNCH_FILE = THIS_DIR / 'ogl_launch.py'

MAP_RESOLUTION = 0.05  # meters/pixel, from maps/map.yaml
MAP_ORIGIN = (0.0, 0.0, 0.0)  # from maps/map.yaml

TRIGGER_TIMEOUT_SEC = 180  # generous, but see note below -- CPU may not finish even in 900s
RESULT_PUBLISH_DURATION_SEC = 180


class LocalizeClient(Node):
    def __init__(self):
        super().__init__('demo2_ogl_client')
        self.received = None
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, '/localization_result', self.on_msg, 10)
        self.cli = self.create_client(Empty, '/trigger_grid_search_localization')

    def on_msg(self, msg):
        if self.received is None:
            self.received = msg


def run_localization_pass(mode):
    """mode: 'gpu' or 'cpu'. Returns (elapsed_sec, (x, y, qz, qw)) or (None, None)."""
    env = os.environ.copy()
    if mode == 'cpu':
        env['CUDA_VISIBLE_DEVICES'] = ''

    print(f'--- {mode.upper()} pass: launching localizer ---', flush=True)
    launch_proc = subprocess.Popen(
        ['ros2', 'launch', str(LAUNCH_FILE)],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(6)  # let the container + node (incl. GPU init attempt) come up

    rclpy.init()
    node = LocalizeClient()
    have_service = node.cli.wait_for_service(timeout_sec=20.0)

    elapsed = None
    pose = None
    if not have_service:
        print(f'{mode.upper()}: trigger_grid_search_localization service never appeared',
              flush=True)
    else:
        bag_proc = subprocess.Popen(
            ['ros2', 'bag', 'play', str(BAG_PATH)],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        t_start = time.time()
        end_time = t_start + TRIGGER_TIMEOUT_SEC
        last_progress = t_start
        while time.time() < end_time and node.received is None:
            future = node.cli.call_async(Empty.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.time()
            if now - last_progress > 10.0:
                print(f'{mode.upper()}: still searching... {now - t_start:.0f}s elapsed',
                      flush=True)
                last_progress = now

        if node.received is not None:
            elapsed = time.time() - t_start
            p = node.received.pose.pose.position
            q = node.received.pose.pose.orientation
            pose = (p.x, p.y, q.z, q.w)
            print(f'{mode.upper()}: localized in {elapsed:.2f}s -> '
                  f'({p.x:.2f}, {p.y:.2f})', flush=True)
        else:
            print(f'{mode.upper()}: timed out after {TRIGGER_TIMEOUT_SEC}s with no result',
                  flush=True)

        bag_proc.terminate()
        try:
            bag_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            bag_proc.kill()

    node.destroy_node()
    rclpy.shutdown()

    launch_proc.terminate()
    try:
        launch_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        launch_proc.kill()
    subprocess.run(['pkill', '-9', '-f', '__node:=ogl_container'], stderr=subprocess.DEVNULL)
    time.sleep(2)

    return elapsed, pose


def world_to_pixel(x, y, map_height_px):
    px = int(round((x - MAP_ORIGIN[0]) / MAP_RESOLUTION))
    py = map_height_px - 1 - int(round((y - MAP_ORIGIN[1]) / MAP_RESOLUTION))
    return px, py


def draw_pose(img, pose, color, label):
    if pose is None:
        return
    x, y, qz, qw = pose
    yaw = 2.0 * math.atan2(qz, qw)
    px, py = world_to_pixel(x, y, img.shape[0])
    cv2.circle(img, (px, py), 8, color, 2)
    tip = (int(px + 22 * math.cos(yaw)), int(py - 22 * math.sin(yaw)))
    cv2.arrowedLine(img, (px, py), tip, color, 2, tipLength=0.4)
    cv2.putText(img, label, (px + 12, py - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


def build_result_image(gpu_elapsed, gpu_pose, cpu_elapsed, cpu_pose):
    map_bgr = cv2.cvtColor(
        cv2.imread(str(MAP_PNG_PATH), cv2.IMREAD_GRAYSCALE), cv2.COLOR_GRAY2BGR)
    draw_pose(map_bgr, gpu_pose, (90, 220, 130), 'GPU')
    draw_pose(map_bgr, cpu_pose, (60, 140, 235), 'CPU')

    # Scale the (small) map image up so the banner text is legible.
    scale = 2
    map_bgr = cv2.resize(map_bgr, (map_bgr.shape[1] * scale, map_bgr.shape[0] * scale),
                          interpolation=cv2.INTER_NEAREST)

    banner_h = 90
    canvas = np.zeros((map_bgr.shape[0] + banner_h, map_bgr.shape[1], 3), dtype=np.uint8)
    canvas[banner_h:, :, :] = map_bgr

    def fmt(elapsed):
        return f'{elapsed:.2f}s' if elapsed is not None else 'FAILED/TIMED OUT'

    gpu_line = f'GPU (real CUDA kernel):  {fmt(gpu_elapsed)}'
    cpu_line = f'CPU (real fallback path): {fmt(cpu_elapsed)}'
    cv2.putText(canvas, gpu_line, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (90, 220, 130), 2)
    cv2.putText(canvas, cpu_line, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (60, 140, 235), 2)

    if gpu_elapsed and cpu_elapsed:
        speedup = cpu_elapsed / gpu_elapsed
        cv2.putText(canvas, f'{speedup:.1f}x', (canvas.shape[1] - 160, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 255), 3)

    return canvas


class ResultPublisher(Node):
    def __init__(self, image_bgr):
        super().__init__('demo2_ogl_result_publisher')
        self.pub = self.create_publisher(Image, 'demo2_ogl_comparison', 10)
        self.bridge = CvBridge()
        self.msg = self.bridge.cv2_to_imgmsg(image_bgr, encoding='bgr8')
        self.create_timer(1.0, self.publish_once)
        self.get_logger().info(
            f'Publishing result to /demo2_ogl_comparison for {RESULT_PUBLISH_DURATION_SEC}s '
            '-- view with RViz2 (Image display) or rqt_image_view on a laptop on this ROS domain')

    def publish_once(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main():
    gpu_elapsed, gpu_pose = run_localization_pass('gpu')
    cpu_elapsed, cpu_pose = run_localization_pass('cpu')

    print('\n=== Demo 2 result ===')
    print(f'GPU: {gpu_elapsed}')
    print(f'CPU: {cpu_elapsed}')
    if gpu_elapsed and cpu_elapsed:
        print(f'Speedup: {cpu_elapsed / gpu_elapsed:.1f}x')

    result_image = build_result_image(gpu_elapsed, gpu_pose, cpu_elapsed, cpu_pose)
    out_path = THIS_DIR / 'demo2_result.png'
    cv2.imwrite(str(out_path), result_image)
    print(f'Saved {out_path}')

    with open(THIS_DIR / 'demo2_result.json', 'w') as f:
        json.dump({
            'gpu_elapsed_sec': gpu_elapsed, 'gpu_pose': gpu_pose,
            'cpu_elapsed_sec': cpu_elapsed, 'cpu_pose': cpu_pose,
        }, f, indent=2)

    rclpy.init()
    pub_node = ResultPublisher(result_image)
    end_time = time.time() + RESULT_PUBLISH_DURATION_SEC
    while time.time() < end_time:
        rclpy.spin_once(pub_node, timeout_sec=0.5)
    pub_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
