"""
Demo 1 (see plan.md): frame-rate decoupling pump, with alternating GPU/CPU
phases.

The C920's usb_cam config is fixed at 10 fps (camera_c920_params.yaml,
YUYV mode). Subscribing both detectors directly to image_rect would cap
BOTH at ~10 fps regardless of how much faster one backend actually is,
hiding the entire comparison this demo exists to show. This node
republishes the latest camera frame on a fast timer instead, so each
detector can process faster than the camera itself produces frames.

Design history, in order tried (kept for context -- each finding shaped
the next attempt):
  1. Republished full-res (1280x720 bgr8, 2.76 MB/frame) at a 200 Hz
     timer target -- only achieved ~4.3 Hz. `ros2 topic bw` showed a
     ~12.6 MB/s ceiling.
  2. Tried mono8 to shrink messages -- cuAprilTags (GPU) rejects mono8
     outright (`Unsupported image encoding: mono8`), crashing the whole
     container. Reverted to bgr8.
  3. Shrank resolution instead (640x360, then 320x180, then 160x90) to
     fit under the apparent bandwidth ceiling. Rate scaled with pixel
     count as expected, but GPU and CPU stayed locked to *each other*
     and to the pump rate the whole way -- never revealing either
     detector's real ceiling. An isolated 2-process pub/sub diagnostic
     (no NITROS, no Isaac ROS) sustained ~37-45 Hz at the ORIGINAL full
     message size -- 8-10x this pipeline's real number at that size,
     ruling out a hard transport limit.
  4. Found the actual bug: `pump()` called `cv2_to_imgmsg()` (a fresh
     copy/serialize) on every timer tick, not just once per real camera
     frame. Fixed to convert once per arrival and republish the cached
     message. This helped at small resolutions but 640x480 was still
     only ~12 Hz -- consistent with genuine DDS serialization cost
     scaling with message size at larger resolutions, now compounded by
     CPU contention between five concurrently-busy processes (GPU
     container, CPU detector doing real compute, frame pump, compare
     node) on a 6-core board -- more processes truly competing for CPU
     than the isolated 2-process diagnostic had.
  5. Rather than keep shrinking images to fight contention, removed the
     contention itself: the pump alternates between a GPU-only phase and
     a CPU-only phase (PHASE_DURATION_SEC each), publishing only to the
     currently-active detector's topic. Result: GPU-alone and CPU-alone
     landed within 1% of each other AND of the pump rate at 640x480
     (~12.1-12.2 Hz), just as they had running concurrently -- ruling out
     inter-process contention as the explanation too.
  6. Retested 320x180 with both fixes (per-tick conversion removed,
     phases isolated) combined: GPU-alone ~68.0 Hz, CPU-alone ~67.7 Hz --
     STILL matched, at three very different resolutions (12/68/262 Hz)
     and under two different contention conditions. That consistency is
     itself the finding: across every configuration tried, the ROS2/DDS
     message-passing pipeline's own achievable throughput was the
     binding constraint, not either detector's compute time -- neither
     cuAprilTag (GPU, with NITROS's own per-frame type-negotiation
     overhead) nor pupil-apriltags (CPU, 2 threads) was ever slow enough
     to be the limiting factor within the range this pipeline can
     deliver frames at.

Given that, this final version reverts to 640x480 (a realistic
detection resolution, not the artificially tiny sizes tried chasing a
higher pump ceiling) and stops trying to out-run the message-passing
ceiling. demo1_cpu_apriltag_node.py separately instruments its own
detector.detect() call directly (wall-clock around just that call, no
ROS overhead) -- the one number in this demo that's fully decoupled
from the pipeline throughput ceiling documented above, since the GPU
side is a closed compiled binary and can't be instrumented the same
way. The live GPU-vs-CPU message-rate comparison is kept and still
displayed (it's a real, honest measurement of end-to-end pipeline
throughput), just labeled for what it actually shows.
"""
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo, Image

PUMP_RATE_HZ = 500.0
PUMP_WIDTH = 640
PUMP_HEIGHT = 480
PHASE_DURATION_SEC = 10.0


class FramePumpNode(Node):
    def __init__(self):
        super().__init__('frame_pump')
        self.bridge = CvBridge()
        self.latest_image_msg = None
        self.latest_info_msg = None

        self.phase = 'gpu'
        self.phase_start = self.get_clock().now()

        self.image_sub = self.create_subscription(
            Image, 'image_rect', self.on_image, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, 'camera_info_rect', self.on_info, 10)

        self.gpu_image_pub = self.create_publisher(Image, 'image_gpu_active', 10)
        self.gpu_info_pub = self.create_publisher(CameraInfo, 'camera_info_gpu_active', 10)
        self.cpu_image_pub = self.create_publisher(Image, 'image_cpu_active', 10)
        self.cpu_info_pub = self.create_publisher(CameraInfo, 'camera_info_cpu_active', 10)

        self.create_timer(1.0 / PUMP_RATE_HZ, self.pump)
        self.create_timer(0.5, self.check_phase)
        self.get_logger().info(
            f'Frame pump ready: {PUMP_WIDTH}x{PUMP_HEIGHT} bgr8, alternating GPU/CPU phases '
            f'every {PHASE_DURATION_SEC:.0f}s, up to {PUMP_RATE_HZ:.0f} Hz within a phase')
        self.get_logger().info('Phase: GPU (first)')

    def check_phase(self):
        elapsed = (self.get_clock().now() - self.phase_start).nanoseconds / 1e9
        if elapsed >= PHASE_DURATION_SEC:
            self.phase = 'cpu' if self.phase == 'gpu' else 'gpu'
            self.phase_start = self.get_clock().now()
            self.get_logger().info(f'Phase: {self.phase.upper()}')

    def on_image(self, msg):
        # Convert/resize/serialize once per real camera frame (~2-10 Hz),
        # not once per pump tick (up to 500 Hz) -- see module docstring,
        # point 4. bgr8, not mono8 -- see point 2.
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        small = cv2.resize(bgr, (PUMP_WIDTH, PUMP_HEIGHT), interpolation=cv2.INTER_AREA)
        self.latest_image_msg = self.bridge.cv2_to_imgmsg(small, encoding='bgr8')
        self.latest_image_msg.header.frame_id = msg.header.frame_id

    def on_info(self, msg):
        sx = PUMP_WIDTH / msg.width
        sy = PUMP_HEIGHT / msg.height
        scaled = CameraInfo()
        scaled.header = msg.header
        scaled.width = PUMP_WIDTH
        scaled.height = PUMP_HEIGHT
        scaled.distortion_model = msg.distortion_model
        scaled.d = msg.d
        scaled.k = [msg.k[0] * sx, msg.k[1], msg.k[2] * sx,
                    msg.k[3], msg.k[4] * sy, msg.k[5] * sy,
                    msg.k[6], msg.k[7], msg.k[8]]
        scaled.r = msg.r
        scaled.p = [msg.p[0] * sx, msg.p[1], msg.p[2] * sx, msg.p[3] * sx,
                    msg.p[4], msg.p[5] * sy, msg.p[6] * sy, msg.p[7] * sy,
                    msg.p[8], msg.p[9], msg.p[10], msg.p[11]]
        self.latest_info_msg = scaled

    def pump(self):
        if self.latest_image_msg is None:
            return
        # Re-stamp with the current time on every pump tick, not the
        # original camera capture time -- both detectors' stamp-to-arrival
        # latency measurement (demo1_speed_compare_node.py) needs a stamp
        # from *this* publish, or every repeated frame would measure
        # latency against an increasingly stale original capture time.
        # Mutates and republishes the SAME cached message -- no per-tick
        # cv2_to_imgmsg call.
        stamp = self.get_clock().now().to_msg()
        self.latest_image_msg.header.stamp = stamp

        if self.phase == 'gpu':
            self.gpu_image_pub.publish(self.latest_image_msg)
            if self.latest_info_msg is not None:
                self.latest_info_msg.header.stamp = stamp
                self.gpu_info_pub.publish(self.latest_info_msg)
        else:
            self.cpu_image_pub.publish(self.latest_image_msg)
            if self.latest_info_msg is not None:
                self.latest_info_msg.header.stamp = stamp
                self.cpu_info_pub.publish(self.latest_info_msg)


def main():
    rclpy.init()
    node = FramePumpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
