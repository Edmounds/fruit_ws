"""Offline prebuilt WAV spliced text-to-speech speaker."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
from queue import Empty, Queue
import re
import subprocess
from threading import Event, Lock, Thread
import time
from typing import Dict, List, Mapping, Optional, Tuple
import wave

import yaml


@dataclass
class _SpeechJob:
    text: str
    tokens: Optional[List[Tuple[str, Optional[Path], float]]] = None
    done: Event | None = None
    playback_started: Event | None = None
    playback_duration_sec: float | None = None
    result: bool = False


class RecordedSpeaker:
    """Concatenate pre-recorded Wis'adel phrase WAV files and play via aplay."""

    def __init__(
        self,
        config: Mapping[str, object],
        logger,
    ) -> None:
        self._logger = logger
        self._manifest_path = Path(str(config.get(
            "manifest_path",
            "/home/rc4/fruit_ws/src/fruit_mission/config/wisadel_voice.yaml"
        )))
        self._aplay = str(config.get("aplay_executable", "/usr/bin/aplay"))
        self._wav_path = Path(str(config.get(
            "wav_path",
            "/home/rc4/fruit_ws/tmp/fruit_mission/stitched_speech.wav"
        )))
        fallback_timeout = float(config.get("timeout_sec", 10.0))
        self._playback_timeout_sec = min(
            10.0, float(config.get("playback_timeout_sec", fallback_timeout))
        )

        self._sample_rate = 32000
        self._channels = 1
        self._sampwidth = 2
        self._audio_dir: Optional[Path] = None
        self._phrase_map: Dict[str, Path] = {}
        self._sorted_phrases: List[str] = []

        self._load_manifest()

        self._queue: Queue[_SpeechJob | None] = Queue()
        self._closed = False
        self._close_lock = Lock()
        self._current_process: Optional[subprocess.Popen] = None
        self._process_lock = Lock()
        self._abort_event = Event()

        self._worker = Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

    def _load_manifest(self) -> None:
        if not self._manifest_path.is_file():
            self._logger.warning(f"语音清单文件不存在：{self._manifest_path}，将无法发声")
            return
        try:
            with self._manifest_path.open("r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            self._sample_rate = int(data.get("sample_rate", 32000))
            self._channels = int(data.get("channels", 1))
            self._sampwidth = int(data.get("sample_width", 2))

            raw_audio_dir = data.get("audio_dir", "../sounds/wisadel")
            if os.path.isabs(raw_audio_dir):
                self._audio_dir = Path(raw_audio_dir)
            else:
                self._audio_dir = (self._manifest_path.parent / raw_audio_dir).resolve()

            self._phrase_map.clear()
            for item in data.get("phrases", []):
                text = str(item["text"]).strip()
                filename = item["file"]
                self._phrase_map[text] = self._audio_dir / filename

            self._sorted_phrases = sorted(self._phrase_map.keys(), key=len, reverse=True)
            self._logger.info(f"成功加载预生成语音清单：共 {len(self._phrase_map)} 个短语，目录：{self._audio_dir}")
        except Exception as error:
            self._logger.warning(f"读取语音清单失败：{error}")

    def tokenize(self, text: str) -> Optional[List[Tuple[str, Optional[Path], float]]]:
        """Greedy longest-match tokenizer against phrase_map with pause handling."""
        if not self._sorted_phrases:
            return None
        tokens: List[Tuple[str, Optional[Path], float]] = []
        i = 0
        n = len(text)
        while i < n:
            char = text[i]
            if char in ("，", ",", " ", "、"):
                if tokens:
                    last = tokens[-1]
                    tokens[-1] = (last[0], last[1], max(last[2], 0.15))
                i += 1
                continue
            elif char in ("。", ".", "；", ";", "\n"):
                if tokens:
                    last = tokens[-1]
                    tokens[-1] = (last[0], last[1], max(last[2], 0.25))
                i += 1
                continue

            matched = False
            sub = text[i:]
            for phrase in self._sorted_phrases:
                if sub.startswith(phrase):
                    tokens.append((phrase, self._phrase_map[phrase], 0.0))
                    i += len(phrase)
                    matched = True
                    break

            if not matched:
                self._logger.warning(f"语音文案无法完整匹配到预置词表：'{text[i:i+10]}...' (全文: '{text}')")
                return None
        return tokens

    def estimate_duration_sec(self, tokens: List[Tuple[str, Optional[Path], float]]) -> float:
        """Estimate playback duration for a token list in seconds."""
        duration = 0.0
        for _text, wav_path, pause_sec in tokens:
            duration += pause_sec
            if wav_path and wav_path.is_file():
                try:
                    with wave.open(str(wav_path), "rb") as wf:
                        rate = wf.getframerate()
                        if rate > 0:
                            duration += wf.getnframes() / rate
                except Exception:
                    pass
        return duration

    def announce(self, text: str, **_kwargs) -> bool:
        """Queue text for speech without blocking mission execution."""
        if not text:
            return True
        items = [s.strip() for s in re.split(r"[；\n]+", str(text)) if s.strip()]
        if not items:
            return True

        with self._close_lock:
            if self._closed:
                return False
            for item in items:
                tokens = self.tokenize(item)
                self._queue.put(_SpeechJob(item, tokens=tokens))
        return True

    def announce_and_wait(
        self,
        text: str,
        lead_time_sec: float = 0.0,
        **_kwargs,
    ) -> bool:
        """Queue text and wait until playback finishes or reaches its lead time."""
        if not text:
            return True
        items = [s.strip() for s in re.split(r"[；\n]+", str(text)) if s.strip()]
        if not items:
            return True

        jobs: List[_SpeechJob] = []
        total_duration_sec = 0.0
        first_playback_started = Event()
        last_done = Event()

        for idx, item in enumerate(items):
            tokens = self.tokenize(item)
            duration = self.estimate_duration_sec(tokens) if tokens else 0.0
            total_duration_sec += duration
            p_started = first_playback_started if idx == 0 else None
            done = last_done if idx == len(items) - 1 else None
            jobs.append(_SpeechJob(
                item,
                tokens=tokens,
                done=done,
                playback_started=p_started,
                playback_duration_sec=duration,
            ))

        with self._close_lock:
            if self._closed:
                return False
            for j in jobs:
                self._queue.put(j)

        if lead_time_sec <= 0.0:
            if not last_done.wait(10.0):
                self._logger.warning("语音等待超时，继续任务")
                return False
            return jobs[-1].result

        deadline = time.monotonic() + 10.0
        if not first_playback_started.wait(10.0):
            self._logger.warning("语音开始等待超时，继续任务")
            return False

        remaining_sec = max(total_duration_sec - float(lead_time_sec), 0.0)
        budget = max(0.0, deadline - time.monotonic())
        wait_sec = min(remaining_sec, budget)
        if last_done.wait(wait_sec):
            return jobs[-1].result
        return remaining_sec <= budget

    def close(self, wait: bool = False) -> None:
        """Stop playback immediately and discard queued items."""
        with self._close_lock:
            if not self._closed:
                self._closed = True
                self._abort_event.set()
                # Clear queue
                while True:
                    try:
                        self._queue.get_nowait()
                        self._queue.task_done()
                    except Empty:
                        break
                # Kill current aplay if running
                with self._process_lock:
                    if self._current_process is not None and self._current_process.poll() is None:
                        try:
                            self._current_process.terminate()
                            self._current_process.wait(timeout=0.5)
                        except Exception:
                            try:
                                self._current_process.kill()
                            except Exception:
                                pass
                self._queue.put(None)
        if wait:
            self._worker.join(timeout=2.0)

    def _worker_loop(self) -> None:
        while True:
            job = self._queue.get()
            try:
                if job is None:
                    return
                if self._abort_event.is_set():
                    job.result = False
                    return
                job.result = self._speak(job)
            finally:
                if job is not None and job.playback_started is not None:
                    job.playback_started.set()
                if job is not None and job.done is not None:
                    job.done.set()
                self._queue.task_done()

    def _speak(self, job: _SpeechJob) -> bool:
        """Stitch audio from tokens and play via aplay."""
        if job.tokens is None:
            self._logger.warning(f"播报跳过（素材无法覆盖）：'{job.text}'")
            return False

        # Validate that all required WAV files exist and are valid
        for text, wav_path, _pause in job.tokens:
            if wav_path is None or not wav_path.is_file():
                self._logger.warning(f"播报跳过（素材缺失）：'{text}' -> {wav_path}")
                return False

        if not Path(self._aplay).is_file():
            self._logger.warning(f"播报失败（播放器不存在）：{self._aplay}")
            return False

        try:
            self._wav_path.parent.mkdir(parents=True, exist_ok=True)
            tmp_wav = self._wav_path.with_suffix(f".tmp_{os.getpid()}.wav")

            # Concatenate PCM
            total_frames = 0
            with wave.open(str(tmp_wav), "wb") as out_wf:
                out_wf.setnchannels(self._channels)
                out_wf.setsampwidth(self._sampwidth)
                out_wf.setframerate(self._sample_rate)

                for _text, wav_path, pause_sec in job.tokens:
                    with wave.open(str(wav_path), "rb") as in_wf:
                        frames = in_wf.readframes(in_wf.getnframes())
                        out_wf.writeframes(frames)
                        total_frames += in_wf.getnframes()

                    if pause_sec > 0.0:
                        pause_frames = int(self._sample_rate * pause_sec)
                        silence = b"\x00" * (pause_frames * self._channels * self._sampwidth)
                        out_wf.writeframes(silence)
                        total_frames += pause_frames

            tmp_wav.replace(self._wav_path)
            job.playback_duration_sec = total_frames / self._sample_rate

            if job.playback_started is not None:
                job.playback_started.set()

            if self._abort_event.is_set():
                return False

            cmd = [self._aplay, str(self._wav_path)]
            with self._process_lock:
                if self._abort_event.is_set():
                    return False
                self._current_process = subprocess.Popen(
                    cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                )

            try:
                self._current_process.wait(timeout=self._playback_timeout_sec)
            except subprocess.TimeoutExpired:
                self._logger.warning(f"语音播放超时（>{self._playback_timeout_sec}s），强制终止")
                self._current_process.kill()
                return False
            finally:
                with self._process_lock:
                    self._current_process = None

            return True
        except Exception as error:
            self._logger.warning(f"语音播报异常：{error}")
            return False


# Retain PiperSpeaker as alias for compatibility
PiperSpeaker = RecordedSpeaker
