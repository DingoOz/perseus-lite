"""Background TTS speaker. Ported from ollama_voice/chat.py:65-177.

Spawns short-lived ``piper | aplay`` subprocesses per queued sentence so audio
starts playing as soon as the first sentence of an LLM stream is complete.
"""

from __future__ import annotations

import json
import os
import queue
import shutil
import subprocess
import threading
from pathlib import Path
from typing import Callable, Optional

from .speech_text import clean_for_speech


def _voice_sample_rate(voice_path: Path) -> int:
    config = json.loads((voice_path.with_suffix(".onnx.json")).read_text())
    return int(config["audio"]["sample_rate"])


class Speaker:
    """Background thread that speaks queued text chunks one at a time."""

    def __init__(
        self,
        voice: Path,
        device: str,
        volume: float = 1.0,
        log_error: Optional[Callable[[str], None]] = None,
        piper_bin: Optional[str] = None,
    ):
        self.voice = Path(voice)
        self.device = device
        self.sample_rate = _voice_sample_rate(self.voice)
        self.q: "queue.Queue[str | None]" = queue.Queue()
        self._current: subprocess.Popen | None = None
        self._lock = threading.Lock()
        self._volume = max(0.0, volume)
        self._log_error = log_error or (lambda msg: None)
        self._piper_bin = (
            piper_bin or shutil.which("piper") or str(Path.home() / ".local/bin/piper")
        )
        self.thread = threading.Thread(
            target=self._run, daemon=True, name="voice-speaker"
        )
        self.thread.start()

    @property
    def volume(self) -> float:
        with self._lock:
            return self._volume

    def set_volume(self, value: float) -> float:
        with self._lock:
            self._volume = max(0.0, value)
            return self._volume

    def say(self, text: str) -> None:
        text = clean_for_speech(text)
        if text:
            self.q.put(text)

    def wait_until_idle(self) -> None:
        self.q.join()

    def is_playing(self) -> bool:
        with self._lock:
            return self._current is not None and self._current.poll() is None

    def pending(self) -> int:
        return self.q.qsize()

    def interrupt(self) -> None:
        # Drop pending utterances and kill the one currently playing.
        try:
            while True:
                self.q.get_nowait()
                self.q.task_done()
        except queue.Empty:
            pass
        with self._lock:
            if self._current and self._current.poll() is None:
                self._current.terminate()

    def shutdown(self) -> None:
        self.interrupt()
        self.q.put(None)
        self.thread.join(timeout=2)

    def _run(self) -> None:
        while True:
            item = self.q.get()
            if item is None:
                self.q.task_done()
                return
            try:
                self._speak_blocking(item)
            except Exception as e:  # noqa: BLE001
                self._log_error(f"tts error: {e}")
            finally:
                self.q.task_done()

    def _piper_env(self) -> dict[str, str]:
        """Sanitized env for the piper subprocess.

        ``~/.local/bin/piper`` runs the system Python (e.g. /usr/bin/python3)
        and imports the user-site ``piper`` module. The Nix shell that runs
        this node sets ``PYTHONNOUSERSITE=1`` and points ``PYTHONPATH`` at
        the Nix Python's site-packages — both of those have to be stripped
        for the subprocess so the system Python can find its own modules.
        """
        env = os.environ.copy()
        env.pop("PYTHONNOUSERSITE", None)
        env.pop("PYTHONPATH", None)
        env.pop("PYTHONHOME", None)
        return env

    def _speak_blocking(self, text: str) -> None:
        vol = self.volume
        if vol <= 0.0:
            return
        piper = subprocess.Popen(
            [
                self._piper_bin,
                "-m",
                str(self.voice),
                "--output-raw",
                "--sentence-silence",
                "0.2",
                "--volume",
                f"{vol:.3f}",
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=self._piper_env(),
        )
        aplay = subprocess.Popen(
            [
                "aplay",
                "-D",
                self.device,
                "-r",
                str(self.sample_rate),
                "-f",
                "S16_LE",
                "-c",
                "1",
                "-t",
                "raw",
                "-q",
            ],
            stdin=piper.stdout,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        # Let aplay own the read end of the pipe.
        assert piper.stdout is not None
        piper.stdout.close()
        with self._lock:
            self._current = aplay
        try:
            assert piper.stdin is not None
            piper.stdin.write(text.encode("utf-8"))
            piper.stdin.close()
            aplay.wait()
            piper.wait()
            if piper.returncode not in (0, -15):  # -15 == SIGTERM from interrupt()
                stderr = b""
                if piper.stderr is not None:
                    try:
                        stderr = piper.stderr.read() or b""
                    except Exception:  # noqa: BLE001
                        pass
                snippet = stderr.decode("utf-8", errors="replace").strip()[:400]
                self._log_error(
                    f"piper exited {piper.returncode}: {snippet or '(no stderr)'}"
                )
        finally:
            with self._lock:
                self._current = None
            if piper.stderr is not None:
                try:
                    piper.stderr.close()
                except Exception:  # noqa: BLE001
                    pass
