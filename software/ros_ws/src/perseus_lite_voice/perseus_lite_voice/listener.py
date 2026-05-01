"""Push-to-talk recorder + Whisper transcriber.

Ported from ollama_voice/chat.py:180-343. Whisper is lazy-loaded on first use.
"""

from __future__ import annotations

import subprocess
import threading
from typing import Callable, Optional


class Listener:
    """Capture from a mic with simple energy VAD, then transcribe with Whisper.

    Whisper model loads lazily on first use (~73MB for 'tiny', a few seconds
    on Jetson CPU) so users who never invoke voice input pay no cost.
    """

    SAMPLE_RATE = 16000
    FRAME_MS = 30
    FRAME_SAMPLES = SAMPLE_RATE * FRAME_MS // 1000  # 480
    FRAME_BYTES = FRAME_SAMPLES * 2  # int16 mono
    CALIBRATE_FRAMES = 10  # ~300 ms
    SPEECH_TRIGGER_FRAMES = 3  # ~90 ms above threshold to start
    SILENCE_HANG_FRAMES = 50  # ~1.5 s of silence ends utterance
    PREROLL_FRAMES = 6  # keep ~180 ms before trigger
    MIN_RMS_THRESHOLD = 800.0  # floor for very quiet rooms
    MAX_RECORD_SECONDS = 30
    PRE_TRIGGER_TIMEOUT_S = 8  # give up if no speech detected

    def __init__(
        self,
        mic_device: str,
        model_name: str = "tiny",
        set_state: Optional[Callable[[Optional[str]], None]] = None,
    ):
        self.mic_device = mic_device
        self.model_name = model_name
        self._model = None
        self._lock = threading.Lock()
        self._set_state = set_state or (lambda _label: None)

    def _ensure_model(self):
        if self._model is not None:
            return self._model
        with self._lock:
            if self._model is None:
                self._set_state(f"loading whisper {self.model_name}")
                import whisper  # lazy: ~1s import + torch

                self._model = whisper.load_model(self.model_name)
                self._set_state(None)
        return self._model

    def record_until_silence(self):
        """Block until an utterance is captured. Returns float32 numpy array
        at 16 kHz, or None if cancelled / nothing captured."""
        import numpy as np

        proc = subprocess.Popen(
            [
                "arecord",
                "-D",
                self.mic_device,
                "-r",
                str(self.SAMPLE_RATE),
                "-f",
                "S16_LE",
                "-c",
                "1",
                "-t",
                "raw",
                "-q",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )

        def read_frame() -> bytes | None:
            assert proc.stdout is not None
            buf = b""
            while len(buf) < self.FRAME_BYTES:
                chunk = proc.stdout.read(self.FRAME_BYTES - len(buf))
                if not chunk:
                    return None
                buf += chunk
            return buf

        def rms(frame_bytes: bytes) -> float:
            samples = np.frombuffer(frame_bytes, dtype=np.int16).astype(np.float32)
            if samples.size == 0:
                return 0.0
            return float(np.sqrt(np.mean(samples * samples)))

        captured: list[bytes] = []
        try:
            # Calibrate noise floor.
            self._set_state("calibrating mic")
            noise_levels = []
            for _ in range(self.CALIBRATE_FRAMES):
                f = read_frame()
                if f is None:
                    return None
                noise_levels.append(rms(f))
            noise_floor = sum(noise_levels) / len(noise_levels)
            threshold = max(noise_floor * 3.0, self.MIN_RMS_THRESHOLD)

            # Wait for speech, keeping a small pre-roll buffer.
            self._set_state("listening")
            preroll: list[bytes] = []
            consecutive_speech = 0
            max_pre_frames = int(self.PRE_TRIGGER_TIMEOUT_S * 1000 / self.FRAME_MS)
            triggered = False
            for _ in range(max_pre_frames):
                f = read_frame()
                if f is None:
                    return None
                preroll.append(f)
                if len(preroll) > self.PREROLL_FRAMES:
                    preroll.pop(0)
                if rms(f) > threshold:
                    consecutive_speech += 1
                    if consecutive_speech >= self.SPEECH_TRIGGER_FRAMES:
                        triggered = True
                        break
                else:
                    consecutive_speech = 0

            if not triggered:
                return None

            captured.extend(preroll)

            # Record until silence_hang frames are below threshold.
            self._set_state("recording")
            silent_run = 0
            max_frames = int(self.MAX_RECORD_SECONDS * 1000 / self.FRAME_MS)
            for _ in range(max_frames):
                f = read_frame()
                if f is None:
                    break
                captured.append(f)
                if rms(f) > threshold:
                    silent_run = 0
                else:
                    silent_run += 1
                    if silent_run >= self.SILENCE_HANG_FRAMES:
                        break
        finally:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    proc.kill()
            self._set_state(None)

        if not captured:
            return None
        raw = b"".join(captured)
        samples = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
        return samples

    def transcribe(self, samples) -> str:
        if samples is None or len(samples) == 0:
            return ""
        self._ensure_model()
        self._set_state("transcribing")
        try:
            result = self._model.transcribe(
                samples,
                language="en",
                fp16=False,  # CPU
                condition_on_previous_text=False,
            )
        finally:
            self._set_state(None)
        return (result.get("text") or "").strip()
