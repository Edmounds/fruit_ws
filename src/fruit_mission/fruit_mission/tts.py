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
import time
import wave


@dataclass
class _SpeechJob:
    text: str
    done: Event | None = None
    playback_started: Event | None = None
    length_scale: float | None = None
    playback_duration_sec: float | None = None
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
        fallback_timeout = float(config.get("timeout_sec", 10.0))
        self._generation_timeout_sec = min(
            10.0, float(config.get("generation_timeout_sec", fallback_timeout))
        )
        self._playback_timeout_sec = min(
            10.0, float(config.get("playback_timeout_sec", fallback_timeout))
        )
        length_scale = config.get("length_scale")
        self._length_scale = None if length_scale is None else float(length_scale)
        self._logger = logger
        self._run = run
        self._queue: Queue[_SpeechJob | None] = Queue()
        self._closed = False
        self._close_lock = Lock()
        self._worker = Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

    def announce(self, text: str, length_scale: float | None = None) -> bool:
        """Queue text for speech without blocking mission execution."""
        if not text:
            return True
        with self._close_lock:
            if self._closed:
                return False
            self._queue.put(_SpeechJob(str(text), length_scale=length_scale))
        return True

    def announce_and_wait(
        self,
        text: str,
        lead_time_sec: float = 0.0,
        length_scale: float | None = None,
    ) -> bool:
        """Queue text and wait until playback finishes or reaches its lead time."""
        if not text:
            return True
        done = Event()
        playback_started = Event()
        job = _SpeechJob(
            str(text),
            done,
            playback_started,
            length_scale=length_scale,
        )
        with self._close_lock:
            if self._closed:
                return False
            self._queue.put(job)
        if lead_time_sec <= 0.0:
            if not done.wait(10.0):
                self._logger.warning('语音等待超时，继续任务')
                return False
            return job.result

        deadline = time.monotonic() + 10.0
        if not playback_started.wait(10.0):
            self._logger.warning('语音开始等待超时，继续任务')
            return False
        if job.playback_duration_sec is None:
            return done.wait(max(0.0, deadline - time.monotonic())) and job.result
        remaining_sec = max(job.playback_duration_sec - float(lead_time_sec), 0.0)
        budget = max(0.0, deadline - time.monotonic())
        if done.wait(min(remaining_sec, budget)):
            return job.result
        return remaining_sec <= budget

    def close(self, wait: bool = False) -> None:
        """Stop the background speaker worker."""
        with self._close_lock:
            if not self._closed:
                self._closed = True
                self._queue.put(None)
        if wait:
            self._worker.join(timeout=10.0)

    def _worker_loop(self) -> None:
        while True:
            job = self._queue.get()
            try:
                if job is None:
                    return
                job.result = self._speak(job)
            finally:
                if job is not None and job.playback_started is not None:
                    job.playback_started.set()
                if job is not None and job.done is not None:
                    job.done.set()
                self._queue.task_done()

    def _speak(self, job: _SpeechJob) -> bool:
        """Speak text and return whether audio generation and playback succeeded."""
        try:
            self._wav_path.parent.mkdir(parents=True, exist_ok=True)
            command = [
                self._piper,
                "--model",
                self._model,
                "--config",
                self._model_config,
                "--output_file",
                str(self._wav_path),
            ]
            length_scale = (
                job.length_scale
                if job.length_scale is not None
                else self._length_scale
            )
            if length_scale is not None:
                command.extend(["--length_scale", str(length_scale)])
            self._run(
                command,
                input=job.text,
                text=True,
                check=True,
                timeout=self._generation_timeout_sec,
            )
            job.playback_duration_sec = self._wav_duration_sec()
            if job.playback_started is not None:
                job.playback_started.set()
            self._run(
                [self._aplay, str(self._wav_path)],
                check=True,
                timeout=self._playback_timeout_sec,
            )
        except Exception as error:
            self._logger.warning(f"语音播报失败，继续任务：{error}")
            return False
        return True

    def _wav_duration_sec(self) -> float | None:
        """Read the generated WAV duration for early mission continuation."""
        try:
            with wave.open(str(self._wav_path), "rb") as wav_file:
                frame_rate = wav_file.getframerate()
                if frame_rate <= 0:
                    return None
                return wav_file.getnframes() / frame_rate
        except (OSError, EOFError, wave.Error):
            return None
