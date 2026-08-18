import subprocess

from fruit_mission.tts import PiperSpeaker


class Logger:
    def __init__(self):
        self.errors = []

    def error(self, message):
        self.errors.append(message)


def _config(tmp_path):
    return {
        "piper_executable": "/tmp/piper",
        "model_path": "/tmp/model.onnx",
        "config_path": "/tmp/model.onnx.json",
        "aplay_executable": "/usr/bin/aplay",
        "wav_path": str(tmp_path / "tts.wav"),
        "generation_timeout_sec": 1.0,
        "playback_timeout_sec": 60.0,
    }


def test_speaker_invokes_piper_then_aplay_without_real_audio(tmp_path):
    calls = []

    def run(command, **kwargs):
        calls.append((command, kwargs))
        return subprocess.CompletedProcess(command, 0)

    speaker = PiperSpeaker(_config(tmp_path), Logger(), run=run)

    assert speaker.announce("C区第一项，四号位置，目标洋葱")
    assert calls[0][0][:5] == [
        "/tmp/piper",
        "--model",
        "/tmp/model.onnx",
        "--config",
        "/tmp/model.onnx.json",
    ]
    assert calls[0][1]["input"] == "C区第一项，四号位置，目标洋葱"
    assert calls[0][1]["timeout"] == 1.0
    assert calls[1][0] == ["/usr/bin/aplay", str(tmp_path / "tts.wav")]
    assert calls[1][1]["timeout"] == 60.0


def test_speaker_logs_failure_and_does_not_raise(tmp_path):
    logger = Logger()

    def run(_command, **_kwargs):
        raise FileNotFoundError("missing piper")

    speaker = PiperSpeaker(_config(tmp_path), logger, run=run)

    assert not speaker.announce("测试")
    assert logger.errors
