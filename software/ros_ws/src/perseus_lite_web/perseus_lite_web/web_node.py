"""Perseus Lite web telemetry node.

Runs a tiny stdlib HTTP server in a background thread that:
  - serves a single static dashboard page (black/grey/white, Tesla/Starlink vibe)
  - pushes a JSON telemetry snapshot to connected browsers over
    Server-Sent Events (SSE) at /events

Everything lives in one Python process so there are no extra dependencies
beyond rclpy. Topics subscribed:

  /odom                              nav_msgs/Odometry
  /power_monitor/battery_state       sensor_msgs/BatteryState
  /imu/data                          sensor_msgs/Imu
  /joint_states                      sensor_msgs/JointState
  /scan                              sensor_msgs/LaserScan
"""

import json
import math
import os
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu, JointState, LaserScan


def _quat_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class TelemetryState:
    """Thread-safe snapshot of the latest telemetry."""

    def __init__(self):
        self._lock = threading.Lock()
        self._data = {
            "ts": 0.0,
            "battery": None,
            "odom": None,
            "imu": None,
            "joints": None,
            "lidar": None,
        }

    def update(self, key: str, value):
        with self._lock:
            self._data[key] = value
            self._data["ts"] = time.time()

    def snapshot(self) -> dict:
        with self._lock:
            return json.loads(json.dumps(self._data))


class SSEBroker:
    """Fan-out of telemetry snapshots to every open SSE client."""

    def __init__(self):
        self._clients: list[queue.Queue] = []
        self._lock = threading.Lock()

    def subscribe(self) -> queue.Queue:
        q: queue.Queue = queue.Queue(maxsize=16)
        with self._lock:
            self._clients.append(q)
        return q

    def unsubscribe(self, q: queue.Queue):
        with self._lock:
            if q in self._clients:
                self._clients.remove(q)

    def publish(self, payload: str):
        with self._lock:
            dead = []
            for q in self._clients:
                try:
                    q.put_nowait(payload)
                except queue.Full:
                    dead.append(q)
            for q in dead:
                self._clients.remove(q)

    def client_count(self) -> int:
        with self._lock:
            return len(self._clients)


def _make_handler(static_dir: str, state: TelemetryState, broker: SSEBroker):
    class Handler(BaseHTTPRequestHandler):
        server_version = "PerseusLiteWeb/0.1"

        def log_message(self, fmt, *args):
            # Silence default stderr access-log noise; the ROS node logs
            # the interesting lifecycle events itself.
            pass

        def _send_static(self, path: str, content_type: str):
            full = os.path.join(static_dir, path)
            if not os.path.isfile(full):
                self.send_error(404)
                return
            with open(full, "rb") as f:
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._send_static("index.html", "text/html; charset=utf-8")
                return
            if self.path == "/snapshot":
                body = json.dumps(state.snapshot()).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path == "/events":
                self._stream_events()
                return
            self.send_error(404)

        def _stream_events(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()

            q = broker.subscribe()
            try:
                initial = json.dumps(state.snapshot())
                self.wfile.write(f"data: {initial}\n\n".encode("utf-8"))
                self.wfile.flush()
                while True:
                    try:
                        payload = q.get(timeout=15.0)
                        self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                    except queue.Empty:
                        # keep-alive ping so proxies don't drop the connection
                        self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                broker.unsubscribe(q)

    return Handler


class WebNode(Node):
    def __init__(self):
        super().__init__("perseus_lite_web")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("battery_topic", "/power_monitor/battery_state")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("scan_topic", "/scan")

        host = self.get_parameter("host").value
        port = int(self.get_parameter("port").value)
        rate = float(self.get_parameter("publish_rate_hz").value)

        self._state = TelemetryState()
        self._broker = SSEBroker()
        self._start_time = time.time()

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self._odom_cb,
            reliable_qos,
        )
        self.create_subscription(
            BatteryState,
            self.get_parameter("battery_topic").value,
            self._battery_cb,
            reliable_qos,
        )
        self.create_subscription(
            Imu,
            self.get_parameter("imu_topic").value,
            self._imu_cb,
            sensor_qos,
        )
        self.create_subscription(
            JointState,
            self.get_parameter("joint_states_topic").value,
            self._joint_state_cb,
            reliable_qos,
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self._scan_cb,
            sensor_qos,
        )

        static_dir = self._resolve_static_dir()
        handler_cls = _make_handler(static_dir, self._state, self._broker)
        self._httpd: Optional[ThreadingHTTPServer] = ThreadingHTTPServer(
            (host, port), handler_cls
        )
        self._httpd_thread = threading.Thread(
            target=self._httpd.serve_forever, name="perseus_lite_web_http", daemon=True
        )
        self._httpd_thread.start()

        period = max(1.0 / rate, 0.05)
        self._publish_timer = self.create_timer(period, self._publish_tick)

        self.get_logger().info(
            f"Perseus Lite web dashboard on http://{host}:{port}/ "
            f"(push rate {rate:.1f} Hz)"
        )

    def _resolve_static_dir(self) -> str:
        share_dir = get_package_share_directory("perseus_lite_web")
        candidate = os.path.join(share_dir, "static")
        if os.path.isdir(candidate):
            return candidate
        # Fallback for colcon --symlink-install + editable dev runs where
        # static files weren't copied into install/.
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(here, "static")

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = _quat_to_yaw(msg.pose.pose.orientation)
        self._state.update(
            "odom",
            {
                "x": p.x,
                "y": p.y,
                "yaw_deg": math.degrees(yaw),
                "linear_x": msg.twist.twist.linear.x,
                "angular_z": msg.twist.twist.angular.z,
                "frame_id": msg.header.frame_id,
            },
        )

    def _battery_cb(self, msg: BatteryState):
        percent = msg.percentage * 100.0 if msg.percentage > 0 else None
        self._state.update(
            "battery",
            {
                "voltage": msg.voltage if not math.isnan(msg.voltage) else None,
                "current": msg.current if not math.isnan(msg.current) else None,
                "percentage": percent,
                "present": bool(msg.present),
            },
        )

    def _imu_cb(self, msg: Imu):
        yaw = _quat_to_yaw(msg.orientation)
        self._state.update(
            "imu",
            {
                "yaw_deg": math.degrees(yaw),
                "accel_x": msg.linear_acceleration.x,
                "accel_y": msg.linear_acceleration.y,
                "accel_z": msg.linear_acceleration.z,
                "gyro_z": msg.angular_velocity.z,
            },
        )

    def _joint_state_cb(self, msg: JointState):
        names = list(msg.name)
        joints = []
        for i, n in enumerate(names):
            joints.append(
                {
                    "name": n,
                    "position": msg.position[i] if i < len(msg.position) else None,
                    "velocity": msg.velocity[i] if i < len(msg.velocity) else None,
                    "effort": msg.effort[i] if i < len(msg.effort) else None,
                }
            )
        self._state.update("joints", joints)

    def _scan_cb(self, msg: LaserScan):
        ranges = [
            r
            for r in msg.ranges
            if math.isfinite(r) and msg.range_min < r < msg.range_max
        ]
        if not ranges:
            self._state.update(
                "lidar",
                {"min": None, "max": None, "count": 0, "frame_id": msg.header.frame_id},
            )
            return
        self._state.update(
            "lidar",
            {
                "min": min(ranges),
                "max": max(ranges),
                "count": len(ranges),
                "frame_id": msg.header.frame_id,
            },
        )

    def _publish_tick(self):
        snap = self._state.snapshot()
        snap["uptime_s"] = time.time() - self._start_time
        snap["clients"] = self._broker.client_count()
        self._broker.publish(json.dumps(snap))

    def destroy_node(self):
        if self._httpd is not None:
            try:
                self._httpd.shutdown()
                self._httpd.server_close()
            except Exception:
                pass
            self._httpd = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
