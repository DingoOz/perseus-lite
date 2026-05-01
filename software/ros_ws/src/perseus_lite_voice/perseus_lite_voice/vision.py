"""YOLOv8 object detection over a ROS image stream.

Differs from ollama_voice/chat.py: instead of shelling out to ffmpeg/v4l2 to
grab a frame, we keep the most recent ``sensor_msgs/Image`` from the camera
topic in memory and run YOLO on demand. The model loads lazily on first call.
"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Callable, Optional


class Vision:
    """Latest-frame buffer + YOLO detector."""

    def __init__(
        self,
        model_path: str,
        max_fps: float = 2.0,
        log_error: Optional[Callable[[str], None]] = None,
        log_info: Optional[Callable[[str], None]] = None,
    ):
        self._model_path = model_path
        self._min_period_s = 1.0 / max(0.1, float(max_fps))
        self._log_error = log_error or (lambda _msg: None)
        self._log_info = log_info or (lambda _msg: None)
        self._lock = threading.Lock()
        self._latest_bgr = None  # numpy HxWx3 uint8
        self._latest_stamp = None  # rclpy time-tuple (sec, nanosec)
        self._latest_frame_id: str = ""
        self._last_run_at = 0.0
        self._model = None

    def update_frame(self, bgr, stamp, frame_id: str) -> None:
        with self._lock:
            self._latest_bgr = bgr
            self._latest_stamp = stamp
            self._latest_frame_id = frame_id

    def has_frame(self) -> bool:
        with self._lock:
            return self._latest_bgr is not None

    def _ensure_model(self):
        if self._model is not None:
            return self._model
        # Throttle re-init attempts so a missing weights file doesn't spam.
        from ultralytics import YOLO  # lazy

        weights = Path(self._model_path)
        if not weights.exists():
            self._log_info(
                f"YOLO weights {weights} not found; ultralytics will auto-download"
            )
        self._model = YOLO(str(weights))
        return self._model

    def detect(self) -> list[dict]:
        """Run YOLO on the latest frame and return a list of detections.

        Each detection: ``{"label": str, "conf": float, "bbox": [x1,y1,x2,y2],
        "frame_id": str, "stamp": (sec, nanosec)}``.
        Returns ``[]`` if no frame is buffered or the throttle is engaged.
        """
        now = time.monotonic()
        if now - self._last_run_at < self._min_period_s:
            return []
        with self._lock:
            frame = self._latest_bgr
            stamp = self._latest_stamp
            frame_id = self._latest_frame_id
        if frame is None:
            return []
        try:
            model = self._ensure_model()
            results = model(frame, verbose=False)
        except Exception as e:  # noqa: BLE001
            self._log_error(f"yolo error: {e}")
            return []
        self._last_run_at = now

        detections: list[dict] = []
        for r in results:
            names = r.names if hasattr(r, "names") else {}
            boxes = getattr(r, "boxes", None)
            if boxes is None:
                continue
            for i in range(len(boxes)):
                cls_id = int(boxes.cls[i].item()) if boxes.cls is not None else -1
                conf = float(boxes.conf[i].item()) if boxes.conf is not None else 0.0
                xyxy = (
                    boxes.xyxy[i].tolist() if boxes.xyxy is not None else [0, 0, 0, 0]
                )
                detections.append(
                    {
                        "label": names.get(cls_id, str(cls_id)),
                        "conf": round(conf, 3),
                        "bbox": [float(v) for v in xyxy],
                        "frame_id": frame_id,
                        "stamp": stamp,
                    }
                )
        return detections


def describe_detections(detections: list[dict]) -> str:
    """Human-readable summary of a detections list, suitable for TTS."""
    if not detections:
        return "I don't see anything I recognise."
    counts: dict[str, int] = {}
    for d in detections:
        label = d.get("label", "thing")
        counts[label] = counts.get(label, 0) + 1
    parts: list[str] = []
    for label, n in counts.items():
        if n == 1:
            parts.append(f"a {label}")
        else:
            parts.append(f"{n} {label}s")
    if len(parts) == 1:
        return f"I see {parts[0]}."
    if len(parts) == 2:
        return f"I see {parts[0]} and {parts[1]}."
    return "I see " + ", ".join(parts[:-1]) + ", and " + parts[-1] + "."
