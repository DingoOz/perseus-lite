"""Always-on wake-word listener. Ported from ollama_voice/chat.py:346-570.

Owns the single ``arecord`` stream and runs a state machine that alternates
between WAKE_LISTENING (frames go to openwakeword) and CAPTURING (energy-VAD
recording of the user's command after the wake word). Detected commands are
pushed onto ``commands`` for the consumer to pull.

Critical invariants (from ollama_voice CLAUDE.md):

1. One mic, one stream — arecord cannot be opened twice. WakeListener is the
   sole owner; the push-to-talk path is bypassed when wake mode is enabled.
2. Self-trigger guard — detection is suppressed while the speaker is playing
   or has queued audio, then ``oww.reset()``. Without this, the assistant
   wakes itself off its own TTS.
"""

from __future__ import annotations

import queue
import subprocess
import threading
import time
from typing import Callable, Optional

from .listener import Listener
from .speaker import Speaker


class WakeListener:
    SAMPLE_RATE = 16000
    FRAME_SAMPLES = 1280  # openwakeword's expected chunk
    FRAME_BYTES = FRAME_SAMPLES * 2  # int16 mono
    FRAME_MS = FRAME_SAMPLES * 1000 // SAMPLE_RATE  # 80 ms
    SILENCE_HANG_FRAMES = 19  # ~1.5 s of silence ends utterance
    SPEECH_TRIGGER_FRAMES = 2  # ~160 ms above threshold to count as speech-started
    MAX_CAPTURE_FRAMES = 30 * 1000 // FRAME_MS  # 30 s hard cap
    POST_DETECT_COOLDOWN_S = 1.5  # ignore predictions briefly after detection
    MIN_RMS_THRESHOLD = 800.0  # floor for very quiet rooms

    def __init__(
        self,
        listener: Listener,
        speaker: Speaker,
        mic_device: str,
        wake_model: str = "hey_jarvis_v0.1",
        threshold: float = 0.5,
        set_state: Optional[Callable[[Optional[str]], None]] = None,
        log_error: Optional[Callable[[str], None]] = None,
    ):
        self.listener = listener
        self.speaker = speaker
        self.mic_device = mic_device
        self.wake_model = wake_model
        self.threshold = float(threshold)
        self.commands: "queue.Queue[str]" = queue.Queue()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._oww = None
        self._proc: subprocess.Popen | None = None
        self._set_state = set_state or (lambda _label: None)
        self._log_error = log_error or (lambda _msg: None)

    def _load_oww(self):
        if self._oww is not None:
            return self._oww
        self._set_state(f"loading wake model {self.wake_model}")
        from openwakeword.model import Model

        self._oww = Model(
            wakeword_models=[self.wake_model],
            inference_framework="onnx",
        )
        self._set_state(None)
        return self._oww

    def start(self) -> None:
        # Load model + Whisper up-front so the first wake doesn't pause for
        # a multi-second model load.
        self._load_oww()
        self.listener._ensure_model()
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="voice-wake-listener"
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._proc and self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        if self._thread:
            self._thread.join(timeout=2)

    def _read_frame(self) -> bytes | None:
        assert self._proc is not None and self._proc.stdout is not None
        buf = b""
        while len(buf) < self.FRAME_BYTES:
            if self._stop.is_set():
                return None
            chunk = self._proc.stdout.read(self.FRAME_BYTES - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    @staticmethod
    def _rms(frame_bytes: bytes) -> float:
        import numpy as np

        s = np.frombuffer(frame_bytes, dtype=np.int16).astype(np.float32)
        if s.size == 0:
            return 0.0
        return float((s * s).mean() ** 0.5)

    def _run(self) -> None:
        import numpy as np

        try:
            self._proc = subprocess.Popen(
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
        except FileNotFoundError as e:
            self._log_error(f"wake mic error: {e}")
            return

        self._set_state("waiting for wake word")
        cooldown_until = 0.0
        try:
            while not self._stop.is_set():
                frame = self._read_frame()
                if frame is None:
                    break

                # Suppress detection while we're speaking — the speaker
                # bleeds into the mic and easily false-triggers.
                if self.speaker.is_playing() or self.speaker.pending() > 0:
                    self._oww.reset()
                    cooldown_until = time.monotonic() + 0.5
                    continue

                if time.monotonic() < cooldown_until:
                    continue

                samples = np.frombuffer(frame, dtype=np.int16)
                scores = self._oww.predict(samples)
                top = max(scores.values()) if scores else 0.0
                if top < self.threshold:
                    continue

                # --- WAKE WORD DETECTED ---
                self._set_state(f"wake heard ({top:.2f})")
                self._oww.reset()

                # Capture the following utterance with a simple energy VAD
                # right off the same stream.
                self._set_state("listening")
                captured: list[bytes] = []
                noise_levels: list[float] = []
                for _ in range(3):
                    f = self._read_frame()
                    if f is None:
                        break
                    noise_levels.append(self._rms(f))
                    captured.append(f)
                if not noise_levels:
                    break
                noise_floor = sum(noise_levels) / len(noise_levels)
                vad_threshold = max(noise_floor * 3.0, self.MIN_RMS_THRESHOLD)

                self._set_state("recording")
                speech_started = False
                consecutive_speech = 0
                silent_streak = 0
                for _ in range(self.MAX_CAPTURE_FRAMES):
                    if self._stop.is_set():
                        break
                    f = self._read_frame()
                    if f is None:
                        break
                    captured.append(f)
                    level = self._rms(f)
                    if level > vad_threshold:
                        consecutive_speech += 1
                        if consecutive_speech >= self.SPEECH_TRIGGER_FRAMES:
                            speech_started = True
                        silent_streak = 0
                    else:
                        consecutive_speech = 0
                        if speech_started:
                            silent_streak += 1
                            if silent_streak >= self.SILENCE_HANG_FRAMES:
                                break

                if not speech_started:
                    self._set_state("waiting for wake word")
                    cooldown_until = time.monotonic() + self.POST_DETECT_COOLDOWN_S
                    continue

                self._set_state("transcribing")
                raw = b"".join(captured)
                samples_f = (
                    np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
                )
                try:
                    text = self.listener.transcribe(samples_f)
                except Exception as e:  # noqa: BLE001
                    text = ""
                    self._log_error(f"wake transcribe error: {e}")

                text = text.strip()
                if text:
                    self.commands.put(text)

                self._set_state("waiting for wake word")
                cooldown_until = time.monotonic() + self.POST_DETECT_COOLDOWN_S
        finally:
            self._set_state(None)
            if self._proc and self._proc.poll() is None:
                self._proc.terminate()
                try:
                    self._proc.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    self._proc.kill()
