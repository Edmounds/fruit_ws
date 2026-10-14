#!/usr/bin/env python3
"""Reset the RealSense device in an isolated component process."""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import yaml


WORKSPACE = Path(__file__).resolve().parents[1]
CONFIG = WORKSPACE / 'src' / 'fruit_perception' / 'config' / 'fruit_perception.yaml'
READY_TEXT = 'RealSense Node Is Up!'
START_TIMEOUT_SEC = 20.0
STOP_TIMEOUT_SEC = 8.0


def component_arguments() -> list[str]:
    config = yaml.safe_load(CONFIG.read_text())
    parameters = config['camera']['ros__parameters']
    arguments = [
        'ros2',
        'component',
        'standalone',
        'realsense2_camera',
        'realsense2_camera::RealSenseNodeFactory',
        '--node-name',
        'camera_usb_preflight',
    ]
    for name, value in parameters.items():
        if name == 'initial_reset':
            continue
        if isinstance(value, bool):
            value = 'true' if value else 'false'
            rendered = value
        elif isinstance(value, str):
            rendered = f'"{value}"'
        else:
            rendered = str(value)
        arguments.extend(['-p', f'{name}:={rendered}'])
    arguments.extend(['-p', 'initial_reset:=true'])
    return arguments


def log_path() -> Path:
    session_dir = os.environ.get('FRUIT_RUNTIME_SESSION_DIR')
    directory = Path(session_dir) if session_dir else Path('/tmp')
    directory.mkdir(parents=True, exist_ok=True)
    return directory / 'camera_usb_preflight.log'


def stop_process(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=STOP_TIMEOUT_SEC)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=2.0)


def main() -> int:
    output_path = log_path()
    with output_path.open('wb') as output:
        process = subprocess.Popen(
            component_arguments(),
            cwd=WORKSPACE,
            stdout=output,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        deadline = time.monotonic() + START_TIMEOUT_SEC
        ready = False
        while time.monotonic() < deadline:
            if process.poll() is not None:
                break
            output.flush()
            try:
                if READY_TEXT in output_path.read_text(errors='replace'):
                    ready = True
                    break
            except OSError:
                pass
            time.sleep(0.25)
        stop_process(process)

    if not ready:
        print(f'Camera USB preflight did not become ready; see {output_path}', file=sys.stderr)
        return 1
    print(f'Camera USB preflight reset completed; see {output_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
