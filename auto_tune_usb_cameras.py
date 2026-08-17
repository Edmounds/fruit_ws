#!/usr/bin/env python3
"""Tune the two USB cameras from their ROS image topics.

Run this script from a shell that has sourced setup_fruit_runtime.bash. It
temporarily starts cameras that are not publishing, evaluates manual exposure
and white-balance values, saves the selected frames under tmp/, and updates
the source camera YAML without reformatting unrelated lines.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import tempfile
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool


CAMERA_NAMES = ('left_camera', 'right_camera')
TOPICS = {
    'left_camera': '/left_camera/image_raw',
    'right_camera': '/right_camera/image_raw',
}
BACKLIGHT_KEYS = {
    'left_camera': 'left_backlight_compensation',
    'right_camera': 'right_backlight_compensation',
}
MIN_EXPOSURE = 10
MAX_EXPOSURE = 626
TARGET_MEDIAN = 175.0
MAX_P95 = 245.0
MAX_CLIPPED_RATIO = 0.03
MANUAL_BRIGHTNESS = 0
MIN_GAIN = 1
MIN_WHITE_BALANCE = 2800
MAX_WHITE_BALANCE = 6500
TARGET_CHANNEL_RATIO = 1.0
CONTROL_SETTLE_FRAMES = 25


@dataclass
class CameraSpec:
    name: str
    device: str
    exposure: int
    backlight_compensation: int
    auto_white_balance: bool
    white_balance: int


@dataclass
class FrameMetrics:
    mean: float
    median: float
    p10: float
    p95: float
    clipped_ratio: float
    bright_ratio: float
    dark_ratio: float
    score: float
    neutral_pixels: int
    blue_ratio: float
    green_ratio: float
    red_ratio: float
    white_balance_score: float


def clamp_exposure(value: int) -> int:
    return max(MIN_EXPOSURE, min(MAX_EXPOSURE, int(value)))


def candidate_exposures(current: int) -> list[int]:
    values = {
        10, 15, 20, 25, 30, 35, 40, 50, 60, 80,
        100, 120, 160, 200, 300,
        clamp_exposure(current),
        clamp_exposure(round(current * 0.75)),
        clamp_exposure(round(current * 1.25)),
    }
    return sorted(values)


def clamp_white_balance(value: int) -> int:
    return max(
        MIN_WHITE_BALANCE,
        min(MAX_WHITE_BALANCE, int(value)),
    )


def candidate_white_balances(current: int) -> list[tuple[bool, int | None]]:
    values = {
        MIN_WHITE_BALANCE,
        3200,
        3600,
        4000,
        4400,
        4600,
        5000,
        5400,
        5800,
        6200,
        MAX_WHITE_BALANCE,
        clamp_white_balance(current),
    }
    candidates = [(False, value) for value in sorted(values)]
    candidates.append((True, None))
    return candidates


def analyze_frame(frame: np.ndarray) -> FrameMetrics:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    values = gray.astype(np.float32)
    p10, median, p95 = np.percentile(values, [10, 50, 95])
    clipped_ratio = float(np.mean(gray >= 250))
    bright_ratio = float(np.mean(gray >= 240))
    dark_ratio = float(np.mean(gray < 20))

    score = abs(float(median) - TARGET_MEDIAN)
    score += max(0.0, float(p95) - MAX_P95) * 2.0
    score += max(0.0, clipped_ratio - MAX_CLIPPED_RATIO) * 800.0
    score += max(0.0, 25.0 - float(p10)) * 0.5

    blue, green, red, neutral_pixels, white_balance_score = (
        analyze_white_balance(frame)
    )
    return FrameMetrics(
        mean=float(values.mean()),
        median=float(median),
        p10=float(p10),
        p95=float(p95),
        clipped_ratio=clipped_ratio,
        bright_ratio=bright_ratio,
        dark_ratio=dark_ratio,
        score=score,
        neutral_pixels=neutral_pixels,
        blue_ratio=blue,
        green_ratio=green,
        red_ratio=red,
        white_balance_score=white_balance_score,
    )


def analyze_white_balance(
    frame: np.ndarray,
) -> tuple[float, float, float, int, float]:
    """Estimate color neutrality from low-chroma, non-clipped pixels.

    The cameras do not see the same scene, so comparing whole-frame colors is
    misleading. White walls, gray floors, and other low-saturation regions are
    a more useful per-camera reference for selecting white balance.
    """
    bgr = frame.astype(np.float32)
    channel_max = bgr.max(axis=2)
    channel_min = bgr.min(axis=2)
    value = bgr.mean(axis=2)
    neutral_mask = (
        (channel_max - channel_min <= 35.0)
        & (value >= 55.0)
        & (value <= 242.0)
    )
    pixels = bgr[neutral_mask]
    if pixels.shape[0] < 100:
        neutral_mask = (
            (channel_max - channel_min <= 55.0)
            & (value >= 40.0)
            & (value <= 247.0)
        )
        pixels = bgr[neutral_mask]
    if pixels.shape[0] == 0:
        return (1.0, 1.0, 1.0, 0, float('inf'))

    channel_medians = np.median(pixels, axis=0)
    channel_mean = float(channel_medians.mean())
    if channel_mean <= 0.0:
        return (1.0, 1.0, 1.0, int(pixels.shape[0]), float('inf'))

    ratios = channel_medians / channel_mean
    score = float(np.mean(np.abs(ratios - TARGET_CHANNEL_RATIO)))
    return (
        float(ratios[0]),
        float(ratios[1]),
        float(ratios[2]),
        int(pixels.shape[0]),
        score,
    )


def run_v4l2_control(device: str, control: str, value: int) -> None:
    result = subprocess.run(
        [
            'v4l2-ctl',
            '--device', device,
            '--set-ctrl', f'{control}={value}',
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        details = (result.stderr or result.stdout).strip()
        raise RuntimeError(
            f'failed to set {control}={value} on {device}: {details}'
        )


def set_camera_controls(
    spec: CameraSpec,
    exposure: int,
    auto_white_balance: bool | None = None,
    white_balance: int | None = None,
) -> None:
    if auto_white_balance is None:
        auto_white_balance = spec.auto_white_balance
    if white_balance is None:
        white_balance = spec.white_balance

    run_v4l2_control(spec.device, 'brightness', MANUAL_BRIGHTNESS)
    run_v4l2_control(spec.device, 'gain', MIN_GAIN)
    run_v4l2_control(
        spec.device,
        'backlight_compensation',
        spec.backlight_compensation,
    )
    run_v4l2_control(
        spec.device,
        'white_balance_automatic',
        int(auto_white_balance),
    )
    if not auto_white_balance:
        run_v4l2_control(
            spec.device,
            'white_balance_temperature',
            int(white_balance),
        )
    run_v4l2_control(spec.device, 'auto_exposure', 1)
    run_v4l2_control(spec.device, 'exposure_time_absolute', exposure)


class TopicCapture(Node):
    def __init__(self):
        super().__init__('usb_camera_auto_tuner')
        self.bridge = CvBridge()
        self.frames: dict[str, np.ndarray] = {}
        self.stamps: dict[str, tuple[int, int] | None] = {
            name: None for name in CAMERA_NAMES
        }
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Image,
            TOPICS['left_camera'],
            lambda message: self._on_image('left_camera', message),
            qos,
        )
        self.create_subscription(
            Image,
            TOPICS['right_camera'],
            lambda message: self._on_image('right_camera', message),
            qos,
        )
        self.capture_clients = {
            name: self.create_client(SetBool, f'/{name}/set_capture')
            for name in CAMERA_NAMES
        }

    def _on_image(self, name: str, message: Image) -> None:
        try:
            self.frames[name] = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding='bgr8',
            )
            self.stamps[name] = (
                int(message.header.stamp.sec),
                int(message.header.stamp.nanosec),
            )
        except Exception as error:
            self.get_logger().error(f'failed to convert {name} image: {error}')

    def latest_stamp(self, name: str) -> tuple[int, int] | None:
        return self.stamps[name]

    def capture_frame(
        self,
        name: str,
        timeout: float,
        frames_needed: int = 5,
        after_stamp: tuple[int, int] | None = None,
    ) -> np.ndarray:
        accepted_frames = 0
        last_accepted_stamp = after_stamp
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            stamp = self.stamps[name]
            if stamp is None:
                continue
            if last_accepted_stamp is not None and stamp <= last_accepted_stamp:
                continue
            accepted_frames += 1
            last_accepted_stamp = stamp
            if accepted_frames >= frames_needed:
                return self.frames[name].copy()
        raise TimeoutError(f'no fresh frame received on {TOPICS[name]}')

    def set_capture(self, name: str, enabled: bool) -> None:
        client = self.capture_clients[name]
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f'capture service unavailable: {client.srv_name}')
        request = SetBool.Request()
        request.data = enabled
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            raise RuntimeError(f'capture service timed out: {client.srv_name}')
        response = future.result()
        expected_message = 'Start Capturing' if enabled else 'Stop Capturing'
        if not response.success and response.message != expected_message:
            raise RuntimeError(
                f'{client.srv_name} rejected {enabled}: {response.message}'
            )


def camera_specs(config: dict) -> list[CameraSpec]:
    controls = config['usb_camera_controls']['ros__parameters']
    specs = []
    for name in CAMERA_NAMES:
        camera = config[name]
        params = camera['ros__parameters']
        if 'video_device' not in params or 'exposure' not in params:
            raise KeyError(f'{name} must define video_device and exposure')
        backlight_key = BACKLIGHT_KEYS[name]
        if backlight_key not in controls:
            raise KeyError(f'usb_camera_controls must define {backlight_key}')
        specs.append(
            CameraSpec(
                name=name,
                device=str(params['video_device']),
                exposure=clamp_exposure(int(params['exposure'])),
                backlight_compensation=int(controls[backlight_key]),
                auto_white_balance=bool(params.get('auto_white_balance', False)),
                white_balance=clamp_white_balance(
                    int(params.get('white_balance', 4600))
                ),
            )
        )
    return specs


def tune_exposure(
    node: TopicCapture,
    spec: CameraSpec,
    timeout: float,
    current_exposure: int,
    auto_white_balance: bool,
    white_balance: int,
) -> tuple[int, list[dict]]:
    records = []
    best = None
    for exposure in candidate_exposures(current_exposure):
        after_stamp = node.latest_stamp(spec.name)
        set_camera_controls(
            spec,
            exposure,
            auto_white_balance=auto_white_balance,
            white_balance=white_balance,
        )
        frame = node.capture_frame(
            spec.name,
            timeout,
            frames_needed=CONTROL_SETTLE_FRAMES,
            after_stamp=after_stamp,
        )
        metrics = analyze_frame(frame)
        record = {'exposure': exposure, **asdict(metrics)}
        records.append(record)
        print(
            f'{spec.name}: exposure={exposure:3d} '
            f'median={metrics.median:6.1f} p95={metrics.p95:6.1f} '
            f'clip={metrics.clipped_ratio:.2%} score={metrics.score:6.2f}'
        )
        if best is None or metrics.score < best[0]:
            best = (metrics.score, exposure)

    if best is None:
        raise RuntimeError(f'no exposure candidates evaluated for {spec.name}')
    return best[1], records


def tune_camera(
    node: TopicCapture,
    spec: CameraSpec,
    output_dir: Path,
    timeout: float,
) -> dict:
    initial_exposure, initial_exposure_records = tune_exposure(
        node,
        spec,
        timeout,
        current_exposure=spec.exposure,
        auto_white_balance=spec.auto_white_balance,
        white_balance=spec.white_balance,
    )

    white_balance_records = []
    best_white_balance = None
    for auto_white_balance, white_balance in candidate_white_balances(
        spec.white_balance
    ):
        after_stamp = node.latest_stamp(spec.name)
        set_camera_controls(
            spec,
            initial_exposure,
            auto_white_balance=auto_white_balance,
            white_balance=white_balance,
        )
        frame = node.capture_frame(
            spec.name,
            timeout,
            frames_needed=CONTROL_SETTLE_FRAMES,
            after_stamp=after_stamp,
        )
        metrics = analyze_frame(frame)
        record = {
            'auto_white_balance': auto_white_balance,
            'white_balance': white_balance,
            **asdict(metrics),
        }
        white_balance_records.append(record)
        label = 'auto' if auto_white_balance else str(white_balance)
        print(
            f'{spec.name}: white_balance={label:>4s} '
            f'neutral={metrics.neutral_pixels:6d} '
            f'B/G/R={metrics.blue_ratio:.3f}/'
            f'{metrics.green_ratio:.3f}/{metrics.red_ratio:.3f} '
            f'score={metrics.white_balance_score:.4f}'
        )
        if (
            best_white_balance is None
            or metrics.white_balance_score < best_white_balance[0]
        ):
            best_white_balance = (
                metrics.white_balance_score,
                auto_white_balance,
                white_balance,
            )

    if best_white_balance is None:
        raise RuntimeError(
            f'no white-balance candidates evaluated for {spec.name}'
        )

    selected_auto_white_balance = best_white_balance[1]
    selected_white_balance = best_white_balance[2]
    selected_white_balance_for_controls = (
        selected_white_balance
        if selected_white_balance is not None
        else spec.white_balance
    )
    selected_exposure, final_exposure_records = tune_exposure(
        node,
        spec,
        timeout,
        current_exposure=initial_exposure,
        auto_white_balance=selected_auto_white_balance,
        white_balance=selected_white_balance_for_controls,
    )
    after_stamp = node.latest_stamp(spec.name)
    set_camera_controls(
        spec,
        selected_exposure,
        auto_white_balance=selected_auto_white_balance,
        white_balance=selected_white_balance_for_controls,
    )
    selected_frame = node.capture_frame(
        spec.name,
        timeout,
        frames_needed=CONTROL_SETTLE_FRAMES,
        after_stamp=after_stamp,
    )
    selected_metrics = analyze_frame(selected_frame)
    output_path = output_dir / f'{spec.name}.png'
    if not cv2.imwrite(str(output_path), selected_frame):
        raise RuntimeError(f'failed to write {output_path}')
    selected_white_balance_label = (
        'auto'
        if selected_auto_white_balance
        else str(selected_white_balance)
    )
    print(
        f'{spec.name}: selected exposure={selected_exposure}, '
        f'white_balance={selected_white_balance_label}, '
        f'median={selected_metrics.median:.1f}, '
        f'p95={selected_metrics.p95:.1f}, '
        f'clip={selected_metrics.clipped_ratio:.2%}, '
        f'color_score={selected_metrics.white_balance_score:.4f}, '
        f'image={output_path}'
    )
    return {
        'exposure': selected_exposure,
        'auto_white_balance': selected_auto_white_balance,
        'white_balance': selected_white_balance_for_controls,
        'metrics': asdict(selected_metrics),
        'initial_exposure': initial_exposure,
        'initial_exposure_candidates': initial_exposure_records,
        'exposure_candidates': final_exposure_records,
        'white_balance_candidates': white_balance_records,
    }


def validate_selected_metrics(results: dict) -> None:
    for name, result in results.items():
        metrics = result['metrics']
        if (
            metrics['p95'] > MAX_P95
            or metrics['clipped_ratio'] > MAX_CLIPPED_RATIO
        ):
            raise RuntimeError(
                f'{name} final frame is still overexposed: '
                f"p95={metrics['p95']:.1f}, "
                f"clip={metrics['clipped_ratio']:.2%}"
            )
        if (
            metrics['neutral_pixels'] < 100
            or not np.isfinite(metrics['white_balance_score'])
        ):
            raise RuntimeError(
                f'{name} final frame has insufficient neutral pixels for '
                'white-balance validation'
            )


def _find_block(
    lines: list[str],
    start: int,
    end: int,
    name: str,
    indentation: int,
) -> tuple[int, int]:
    header = f'{" " * indentation}{name}:'
    block_start = None
    for index in range(start, end):
        body = lines[index].rstrip('\r\n')
        if body == header:
            block_start = index
            break
    if block_start is None:
        raise KeyError(f'YAML block not found: {name}')

    block_end = end
    for index in range(block_start + 1, end):
        body = lines[index].rstrip('\r\n')
        if body and not body.lstrip().startswith('#'):
            current_indent = len(body) - len(body.lstrip(' '))
            if current_indent <= indentation:
                block_end = index
                break
    return block_start, block_end


def _scalar_text(value: object) -> str:
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, int):
        return str(value)
    raise TypeError(f'only bool and int YAML updates are supported: {value!r}')


def update_yaml_values(path: Path, updates: dict[str, dict[str, dict[str, object]]]) -> None:
    original = path.read_text(encoding='utf-8')
    config = yaml.safe_load(original)
    lines = original.splitlines(keepends=True)
    top_level_end = len(lines)

    for camera_name, groups in updates.items():
        if camera_name not in config:
            raise KeyError(f'camera section not found: {camera_name}')
        camera_start, camera_end = _find_block(
            lines, 0, top_level_end, camera_name, 0
        )
        for group_name, values in groups.items():
            if group_name not in config[camera_name]:
                raise KeyError(f'{camera_name}.{group_name} not found')
            group_start, group_end = _find_block(
                lines, camera_start + 1, camera_end, group_name, 2
            )
            for key, value in values.items():
                if key not in config[camera_name][group_name]:
                    raise KeyError(f'{camera_name}.{group_name}.{key} not found')
                prefix = f'    {key}:'
                matches = []
                for index in range(group_start + 1, group_end):
                    body = lines[index].rstrip('\r\n')
                    if body.startswith(prefix):
                        matches.append(index)
                if len(matches) != 1:
                    raise KeyError(
                        f'expected one YAML value for '
                        f'{camera_name}.{group_name}.{key}, found {len(matches)}'
                    )
                index = matches[0]
                newline = lines[index][len(lines[index].rstrip('\r\n')):]
                lines[index] = f'{prefix} {_scalar_text(value)}{newline}'

    updated = ''.join(lines)
    parsed = yaml.safe_load(updated)
    for camera_name, groups in updates.items():
        for group_name, values in groups.items():
            for key, value in values.items():
                if parsed[camera_name][group_name][key] != value:
                    raise ValueError(
                        f'YAML update did not persist for '
                        f'{camera_name}.{group_name}.{key}'
                    )

    mode = path.stat().st_mode & 0o777
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode='w',
            encoding='utf-8',
            dir=path.parent,
            prefix=f'.{path.name}.',
            delete=False,
        ) as temporary:
            temporary.write(updated)
            temporary_path = Path(temporary.name)
        os.chmod(temporary_path, mode)
        os.replace(temporary_path, path)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--config',
        type=Path,
        default=root / 'src/fruit_perception/config/fruit_perception.yaml',
        help='source camera YAML to update',
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=None,
        help='directory for selected frames and metrics',
    )
    parser.add_argument(
        '--frame-timeout',
        type=float,
        default=3.0,
        help='seconds to wait for each topic frame',
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='tune and save images without changing the YAML',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.config.is_file():
        raise FileNotFoundError(args.config)
    output_dir = args.output_dir or (
        Path(__file__).resolve().parent
        / 'tmp'
        / f'auto_tune_usb_cameras_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    config = yaml.safe_load(args.config.read_text(encoding='utf-8'))
    specs = camera_specs(config)
    node = None
    started_by_script = []
    try:
        rclpy.init()
        node = TopicCapture()
        for spec in specs:
            try:
                after_stamp = node.latest_stamp(spec.name)
                node.capture_frame(
                    spec.name,
                    timeout=1.0,
                    frames_needed=1,
                    after_stamp=after_stamp,
                )
                print(f'{spec.name}: topic already active')
            except TimeoutError:
                node.set_capture(spec.name, True)
                started_by_script.append(spec.name)
                print(f'{spec.name}: started capture')

        results = {}
        for spec in specs:
            results[spec.name] = tune_camera(
                node,
                spec,
                output_dir,
                args.frame_timeout,
            )

        (output_dir / 'metrics.json').write_text(
            json.dumps(results, indent=2),
            encoding='utf-8',
        )
        validate_selected_metrics(results)
        updates = {
            spec.name: {
                'ros__parameters': {
                    'brightness': MANUAL_BRIGHTNESS,
                    'gain': MIN_GAIN,
                    'auto_white_balance': results[
                        spec.name
                    ]['auto_white_balance'],
                    'white_balance': results[spec.name]['white_balance'],
                    'autoexposure': False,
                    'exposure': results[spec.name]['exposure'],
                },
            }
            for spec in specs
        }
        updates['usb_camera_controls'] = {
            'ros__parameters': {
                BACKLIGHT_KEYS[spec.name]: spec.backlight_compensation
                for spec in specs
            }
        }
        if args.dry_run:
            print(f'dry-run: YAML not changed: {args.config}')
        else:
            update_yaml_values(args.config, updates)
            print(f'updated YAML: {args.config}')
        print(f'analysis output: {output_dir}')
        return 0
    finally:
        if node is not None:
            for name in reversed(started_by_script):
                try:
                    node.set_capture(name, False)
                    print(f'{name}: stopped capture')
                except Exception as error:
                    print(f'warning: failed to stop {name}: {error}')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
