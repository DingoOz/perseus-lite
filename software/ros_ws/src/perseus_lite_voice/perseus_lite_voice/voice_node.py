"""ROS 2 orchestrator for the Perseus Lite voice assistant.

Wires the ported ``Speaker`` / ``Listener`` / ``WakeListener`` / ``Vision``
components into a normal ROS 2 node, and adds an intent router that can drive
the rover (publish ``/joy_vel`` and toggle ``frontier_explorer``).

Topics
------
Subscribed:
  ~/say     std_msgs/String          — speak text directly (no LLM)
  ~/prompt  std_msgs/String          — push prompt through Ollama, speak reply
  <vision_topic>  sensor_msgs/Image  — latest-frame buffer for YOLO

Published:
  ~/heard       std_msgs/String       — every transcribed utterance
  ~/reply       std_msgs/String       — final Ollama reply text
  ~/state       std_msgs/String       — what the StatusBar would have shown
  ~/intent      std_msgs/String       — matched intent name (or "chat")
  ~/detections  std_msgs/String       — JSON list of YOLO detections
  <cmd_vel_topic>  geometry_msgs/TwistStamped  — voice → robot velocity

Services:
  ~/reset_history  std_srvs/Trigger
  ~/detect         std_srvs/Trigger
"""

from __future__ import annotations

import json
import queue
import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .intents import Intent, route
from .listener import Listener
from .ollama_client import stream_chat
from .speaker import Speaker
from .speech_text import split_sentences
from .vision import Vision, describe_detections
from .wake_listener import WakeListener


def _resolve_yolo_weights(node: Node, configured: str) -> str:
    """Return an absolute path to the YOLO weights, preferring bundled."""
    p = Path(configured)
    if p.is_absolute() and p.exists():
        return str(p)
    try:
        share = Path(get_package_share_directory("perseus_lite_voice"))
        bundled = share / "models" / p.name
        if bundled.exists():
            return str(bundled)
    except Exception:  # noqa: BLE001
        pass
    # Let ultralytics auto-download by name.
    return configured


def _yuv422_to_bgr(buf, height: int, width: int):
    """Convert a YUYV 4:2:2 buffer to BGR. Used for /image_raw frames since
    perseus_lite publishes 320x240 YUYV (perseus_lite.launch.py:194)."""
    import numpy as np

    yuv = np.frombuffer(buf, dtype=np.uint8).reshape(height, width, 2)
    y = yuv[:, :, 0].astype(np.int32)
    u_full = np.empty((height, width), dtype=np.int32)
    v_full = np.empty((height, width), dtype=np.int32)
    u = yuv[:, 0::2, 1].astype(np.int32)
    v = yuv[:, 1::2, 1].astype(np.int32)
    u_full[:, 0::2] = u
    u_full[:, 1::2] = u
    v_full[:, 0::2] = v
    v_full[:, 1::2] = v
    c = y - 16
    d = u_full - 128
    e = v_full - 128
    r = np.clip((298 * c + 409 * e + 128) >> 8, 0, 255)
    g = np.clip((298 * c - 100 * d - 208 * e + 128) >> 8, 0, 255)
    b = np.clip((298 * c + 516 * d + 128) >> 8, 0, 255)
    return np.stack([b, g, r], axis=-1).astype(np.uint8)


def _image_msg_to_bgr(msg: Image):
    """Convert a sensor_msgs/Image to BGR. Tries cv_bridge first, falls back
    to a small manual decoder for the encodings we expect on Perseus Lite."""
    enc = msg.encoding.lower()
    try:
        from cv_bridge import CvBridge

        bridge = CvBridge()
        return bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception:
        pass
    if enc in ("yuyv", "yuv422", "yuv422_yuy2"):
        return _yuv422_to_bgr(msg.data, msg.height, msg.width)
    if enc in ("bgr8",):
        import numpy as np

        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    if enc in ("rgb8",):
        import numpy as np

        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        return rgb[:, :, ::-1].copy()
    raise ValueError(f"unsupported image encoding: {msg.encoding}")


class VoiceNode(Node):
    def __init__(self) -> None:
        super().__init__("perseus_lite_voice")

        # --- Parameters (defaults match config/voice_params.yaml) ---
        self.declare_parameter("ollama_url", "http://localhost:11434/api/chat")
        self.declare_parameter("ollama_model", "gemma4:e2b")
        self.declare_parameter(
            "system_prompt",
            "You are a friendly voice assistant on a small rover named Perseus "
            "Lite. Replies will be spoken aloud, so keep them short and "
            "conversational. Avoid markdown.",
        )
        self.declare_parameter(
            "voice_path",
            "/home/dingo/Programming/Piper/voices/en_US-amy-medium.onnx",
        )
        self.declare_parameter("audio_device", "plughw:1,0")
        self.declare_parameter("initial_volume", 0.2)
        self.declare_parameter("mic_enabled", True)
        self.declare_parameter("mic_device", "plughw:0,0")
        self.declare_parameter("whisper_model", "tiny")
        self.declare_parameter("wake_enabled", False)
        self.declare_parameter("wake_model", "hey_jarvis_v0.1")
        self.declare_parameter("wake_threshold", 0.5)
        self.declare_parameter("vision_enabled", True)
        self.declare_parameter("vision_topic", "/image_raw")
        self.declare_parameter("vision_max_fps", 2.0)
        self.declare_parameter("yolo_model", "yolov8n.pt")
        self.declare_parameter("enable_robot_bridge", True)
        self.declare_parameter("cmd_vel_topic", "/joy_vel")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("explorer_node", "frontier_explorer")
        self.declare_parameter("explorer_default_period_sec", 3.0)
        self.declare_parameter("explorer_pause_period_sec", 999999.0)
        self.declare_parameter("motion_linear_speed", 0.2)
        self.declare_parameter("motion_angular_speed", 0.5)
        self.declare_parameter("motion_pulse_seconds", 1.0)

        gp = self.get_parameter
        self._ollama_url = gp("ollama_url").get_parameter_value().string_value
        self._ollama_model = gp("ollama_model").get_parameter_value().string_value
        self._system_prompt = gp("system_prompt").get_parameter_value().string_value
        self._mic_enabled = gp("mic_enabled").get_parameter_value().bool_value
        self._wake_enabled = gp("wake_enabled").get_parameter_value().bool_value
        self._vision_enabled = gp("vision_enabled").get_parameter_value().bool_value
        self._enable_bridge = gp("enable_robot_bridge").get_parameter_value().bool_value
        self._base_frame_id = gp("base_frame_id").get_parameter_value().string_value
        self._explorer_node = gp("explorer_node").get_parameter_value().string_value
        self._explorer_default_period = (
            gp("explorer_default_period_sec").get_parameter_value().double_value
        )
        self._explorer_pause_period = (
            gp("explorer_pause_period_sec").get_parameter_value().double_value
        )
        self._lin_speed = gp("motion_linear_speed").get_parameter_value().double_value
        self._ang_speed = gp("motion_angular_speed").get_parameter_value().double_value
        self._pulse_s = gp("motion_pulse_seconds").get_parameter_value().double_value

        # --- Publishers (created early; subscribers reference them) ---
        self._heard_pub = self.create_publisher(String, "~/heard", 10)
        self._reply_pub = self.create_publisher(String, "~/reply", 10)
        self._state_pub = self.create_publisher(String, "~/state", 10)
        self._intent_pub = self.create_publisher(String, "~/intent", 10)
        self._detections_pub = self.create_publisher(String, "~/detections", 10)

        cmd_vel_topic = gp("cmd_vel_topic").get_parameter_value().string_value
        self._cmd_vel_pub = (
            self.create_publisher(TwistStamped, cmd_vel_topic, 10)
            if self._enable_bridge
            else None
        )

        # --- Speaker (TTS) ---
        voice_path = Path(gp("voice_path").get_parameter_value().string_value)
        audio_device = gp("audio_device").get_parameter_value().string_value
        initial_volume = gp("initial_volume").get_parameter_value().double_value
        if not voice_path.exists():
            self.get_logger().error(
                f"Piper voice model not found at {voice_path}. TTS will fail until "
                f"the path is corrected via the voice_path parameter."
            )
        self._speaker = Speaker(
            voice=voice_path,
            device=audio_device,
            volume=initial_volume,
            log_error=lambda m: self.get_logger().error(m),
        )

        # --- LLM history ---
        self._history: list[dict] = [{"role": "system", "content": self._system_prompt}]
        self._history_lock = threading.Lock()

        # --- Listener (push-to-talk) and WakeListener ---
        mic_device = gp("mic_device").get_parameter_value().string_value
        whisper_model = gp("whisper_model").get_parameter_value().string_value
        self._listener: Optional[Listener] = None
        if self._mic_enabled or self._wake_enabled:
            self._listener = Listener(
                mic_device=mic_device,
                model_name=whisper_model,
                set_state=self._publish_state,
            )
        self._wake: Optional[WakeListener] = None
        if self._wake_enabled:
            if self._listener is None:
                self.get_logger().warning(
                    "wake_enabled=true forces mic_enabled=true; enabling listener"
                )
                self._listener = Listener(
                    mic_device=mic_device,
                    model_name=whisper_model,
                    set_state=self._publish_state,
                )
            try:
                self._wake = WakeListener(
                    listener=self._listener,
                    speaker=self._speaker,
                    mic_device=mic_device,
                    wake_model=gp("wake_model").get_parameter_value().string_value,
                    threshold=gp("wake_threshold").get_parameter_value().double_value,
                    set_state=self._publish_state,
                    log_error=lambda m: self.get_logger().error(m),
                )
                self._wake.start()
                self.get_logger().info("wake listener started")
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"wake listener disabled: {e}")
                self._wake = None

        # --- Vision ---
        self._vision: Optional[Vision] = None
        if self._vision_enabled:
            yolo_path = _resolve_yolo_weights(
                self, gp("yolo_model").get_parameter_value().string_value
            )
            self._vision = Vision(
                model_path=yolo_path,
                max_fps=gp("vision_max_fps").get_parameter_value().double_value,
                log_error=lambda m: self.get_logger().error(m),
                log_info=lambda m: self.get_logger().info(m),
            )
            vision_topic = gp("vision_topic").get_parameter_value().string_value
            self.create_subscription(Image, vision_topic, self._on_image, 1)
            self.get_logger().info(f"vision subscribed to {vision_topic}")

        # --- Subscribers (input commands) ---
        prompt_cb_group = ReentrantCallbackGroup()
        self.create_subscription(String, "~/say", self._on_say, 10)
        self.create_subscription(
            String, "~/prompt", self._on_prompt, 10, callback_group=prompt_cb_group
        )

        # --- Services ---
        srv_cb_group = MutuallyExclusiveCallbackGroup()
        self.create_service(
            Trigger,
            "~/reset_history",
            self._srv_reset_history,
            callback_group=srv_cb_group,
        )
        self.create_service(
            Trigger, "~/detect", self._srv_detect, callback_group=srv_cb_group
        )

        # --- Explorer SetParameters client (used by intent bridge) ---
        self._explorer_set_cli = self.create_client(
            SetParameters, f"/{self._explorer_node}/set_parameters"
        )

        # --- Wake-command worker thread ---
        self._stop = threading.Event()
        self._wake_worker: Optional[threading.Thread] = None
        if self._wake is not None:
            self._wake_worker = threading.Thread(
                target=self._wake_command_loop, daemon=True, name="voice-wake-worker"
            )
            self._wake_worker.start()

        self._publish_state("idle")
        self.get_logger().info(
            f"perseus_lite_voice ready (mic={self._mic_enabled} "
            f"wake={self._wake is not None} vision={self._vision is not None} "
            f"bridge={self._enable_bridge})"
        )

    # ------------------------------------------------------------------ helpers

    def _publish_state(self, label: Optional[str]) -> None:
        msg = String()
        msg.data = label or "idle"
        try:
            self._state_pub.publish(msg)
        except Exception:  # noqa: BLE001
            # Publishing during shutdown can throw; ignore.
            pass

    def _publish(self, pub, text: str) -> None:
        m = String()
        m.data = text
        pub.publish(m)

    # ------------------------------------------------------------------ subs

    def _on_say(self, msg: String) -> None:
        if msg.data:
            self._speaker.say(msg.data)

    def _on_prompt(self, msg: String) -> None:
        text = (msg.data or "").strip()
        if not text:
            return
        self._handle_user_turn(text)

    def _on_image(self, msg: Image) -> None:
        if self._vision is None:
            return
        try:
            bgr = _image_msg_to_bgr(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(
                f"image decode failed: {e}", throttle_duration_sec=10
            )
            return
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self._vision.update_frame(bgr, stamp, msg.header.frame_id)

    # ------------------------------------------------------------------ services

    def _srv_reset_history(self, _req, resp):
        with self._history_lock:
            self._history = [{"role": "system", "content": self._system_prompt}]
        resp.success = True
        resp.message = "history cleared"
        return resp

    def _srv_detect(self, _req, resp):
        if self._vision is None:
            resp.success = False
            resp.message = "vision disabled"
            return resp
        detections = self._vision.detect()
        self._publish(self._detections_pub, json.dumps(detections, default=str))
        spoken = describe_detections(detections)
        self._speaker.say(spoken)
        resp.success = True
        resp.message = spoken
        return resp

    # ------------------------------------------------------------------ wake worker

    def _wake_command_loop(self) -> None:
        assert self._wake is not None
        while not self._stop.is_set():
            try:
                text = self._wake.commands.get(timeout=0.25)
            except queue.Empty:
                continue
            self._publish(self._heard_pub, text)
            self._handle_user_turn(text)

    # ------------------------------------------------------------------ intents

    def _handle_user_turn(self, text: str) -> None:
        intent = route(text)
        self._publish(self._intent_pub, intent.name)
        if intent.speak:
            self._speaker.say(intent.speak)
        try:
            self._dispatch(intent)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"intent {intent.name!r} raised: {e}")

    def _dispatch(self, intent: Intent) -> None:
        name = intent.name
        if name == "chat":
            self._chat_with_ollama(intent.text)
            return
        if name == "see":
            if self._vision is None:
                self._speaker.say("Vision is disabled.")
                return
            detections = self._vision.detect()
            self._publish(self._detections_pub, json.dumps(detections, default=str))
            self._speaker.say(describe_detections(detections))
            return
        if name == "reset_history":
            with self._history_lock:
                self._history = [{"role": "system", "content": self._system_prompt}]
            return
        if name in ("start_exploring", "stop_exploring"):
            period = (
                self._explorer_default_period
                if name == "start_exploring"
                else self._explorer_pause_period
            )
            self._set_explorer_period(period)
            return
        # Velocity intents — only if the bridge is enabled.
        if not self._enable_bridge or self._cmd_vel_pub is None:
            self._speaker.say("Robot bridge disabled.")
            return
        if name == "stop":
            self._publish_velocity(0.0, 0.0)
            return
        if name == "forward":
            self._publish_velocity_pulse(self._lin_speed, 0.0)
            return
        if name == "backward":
            self._publish_velocity_pulse(-self._lin_speed, 0.0)
            return
        if name == "turn_left":
            self._publish_velocity_pulse(0.0, self._ang_speed)
            return
        if name == "turn_right":
            self._publish_velocity_pulse(0.0, -self._ang_speed)
            return
        # Unknown — fall back to chat.
        self._chat_with_ollama(intent.text)

    def _publish_velocity(self, linear: float, angular: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame_id
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self._cmd_vel_pub.publish(msg)

    def _publish_velocity_pulse(self, linear: float, angular: float) -> None:
        """Pulse the requested velocity for ``motion_pulse_seconds`` then zero.

        Spawns a short-lived thread so the dispatcher returns immediately and
        the executor stays responsive (e.g. to a follow-up "stop")."""

        def worker():
            end_at = time.monotonic() + self._pulse_s
            # Re-publish at ~10 Hz so the twist_mux/collision_monitor stack
            # sees a fresh command and doesn't time out (twist_mux.yaml uses
            # 0.5s timeouts).
            while time.monotonic() < end_at and not self._stop.is_set():
                self._publish_velocity(linear, angular)
                time.sleep(0.1)
            self._publish_velocity(0.0, 0.0)

        threading.Thread(target=worker, daemon=True, name="voice-pulse").start()

    def _set_explorer_period(self, period_sec: float) -> None:
        if not self._explorer_set_cli.service_is_ready():
            self.get_logger().warning(
                f"{self._explorer_node} set_parameters not yet discovered"
            )
            return
        req = SetParameters.Request()
        p = Parameter()
        p.name = "planning_period_sec"
        p.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(period_sec)
        )
        req.parameters = [p]
        self._explorer_set_cli.call_async(req)

    # ------------------------------------------------------------------ Ollama

    def _chat_with_ollama(self, user_text: str) -> None:
        with self._history_lock:
            self._history.append({"role": "user", "content": user_text})
            history_snapshot = list(self._history)

        self._publish_state("thinking")
        full_reply = ""
        buffer = ""
        try:
            for chunk in stream_chat(
                self._ollama_url, self._ollama_model, history_snapshot
            ):
                full_reply += chunk
                buffer += chunk
                sentences, buffer = split_sentences(buffer)
                for s in sentences:
                    self._speaker.say(s)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"ollama error: {e}")
            with self._history_lock:
                # Roll back the unanswered user turn so retries don't double-up.
                if self._history and self._history[-1].get("role") == "user":
                    self._history.pop()
            self._publish_state("idle")
            return

        tail = buffer.strip()
        if tail:
            self._speaker.say(tail)
        with self._history_lock:
            self._history.append({"role": "assistant", "content": full_reply})
        self._publish(self._reply_pub, full_reply)
        self._publish_state("idle")

    # ------------------------------------------------------------------ shutdown

    def shutdown(self) -> None:
        self._stop.set()
        if self._wake is not None:
            try:
                self._wake.stop()
            except Exception:  # noqa: BLE001
                pass
        try:
            self._speaker.shutdown()
        except Exception:  # noqa: BLE001
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoiceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
