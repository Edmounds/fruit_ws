"""Subprocess-backed Piper text-to-speech wrapper."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from queue import Queue
import subprocess
from threading import Event
from threading import Lock
from threading import Thread
from typing import Callable, Mapping


@dataclass
class _SpeechJob:
    text: str
    done: Event | None = None
    result: bool = False


class PiperSpeaker:
    """Generate speech with Piper and play it with aplay."""

    def __init__(
        self,
        config: Mapping[str, object],
        logger,
        run: Callable[..., subprocess.CompletedProcess] = subprocess.run,
    ) -> None:
        self._piper = str(config["piper_executable"])
        self._model = str(config["model_path"])
        self._model_config = str(config["config_path"])
        self._aplay = str(config["aplay_executable"])
        self._wav_path = Path(str(config["wav_path"]))
        fallback_timeout = float(config.get("timeout_sec", 20.0))
        self._generation_timeout_sec = float(
            config.get("generation_timeout_sec", fallback_timeout)
        )
        self._playback_timeout_sec = float(
            config.get("playback_timeout_sec", fallback_timeout)
        )
        self._logger = logger
        self._run = run
        self._queue: Queue[_SpeechJob | None] = Queue()
        self._closed = False
        self._close_lock = Lock()
        self._worker = Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

    def announce(self, text: str) -> bool:
        """Queue text for speech without blocking mission execution."""
        if not text:
            return True
        with self._close_lock:
            if self._closed:
                return False
            self._queue.put(_SpeechJob(str(text)))
        return True

    def announce_and_wait(self, text: str) -> bool:
        """Queue text for speech and wait until generation and playback finish."""
        if not text:
            return True
        done = Event()
        job = _SpeechJob(str(text), done)
        with self._close_lock:
            if self._closed:
                return False
            self._queue.put(job)
        done.wait()
        return job.result

    def close(self, wait: bool = False) -> None:
        """Stop the background speaker worker."""
        with self._close_lock:
            if not self._closed:
                self._closed = True
                self._queue.put(None)
        if wait:
            self._worker.join()

    def _worker_loop(self) -> None:
        while True:
            job = self._queue.get()
            try:
                if job is None:
                    return
                job.result = self._speak(job.text)
                if job.done is not None:
                    job.done.set()
            finally:
                self._queue.task_done()

    def _speak(self, text: str) -> bool:
        """Speak text and return whether audio generation and playback succeeded."""
        self._wav_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            self._run(
                [
                    self._piper,
                    "--model",
                    self._model,
                    "--config",
                    self._model_config,
                    "--output_file",
                    str(self._wav_path),
                ],
                input=text,
                text=True,
                check=True,
                timeout=self._generation_timeout_sec,
            )
            self._run(
                [self._aplay, str(self._wav_path)],
                check=True,
                timeout=self._playback_timeout_sec,
            )
        except Exception as error:
            self._logger.error(f"语音播报失败：{error}")
            return False
        return True
