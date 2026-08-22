#!/usr/bin/env python3
"""Collect full-resolution color images for YOLO training."""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any


WORKSPACE = Path('/home/rc4/fruit_ws')
CONFIG_PATH = WORKSPACE / 'src/fruit_perception/config/fruit_perception.yaml'
OUTPUT_ROOT = WORKSPACE / 'img_output'
TMP_DIR = WORKSPACE / 'tmp'
DEFAULT_DISPLAY = ':0'


def configure_desktop_display() -> None:
    runtime_dir = Path(f'/run/user/{os.getuid()}')
    xauthority = runtime_dir / 'gdm' / 'Xauthority'
    os.environ['DISPLAY'] = DEFAULT_DISPLAY
    os.environ['XDG_RUNTIME_DIR'] = str(runtime_dir)
    os.environ['DBUS_SESSION_BUS_ADDRESS'] = f'unix:path={runtime_dir}/bus'
    if xauthority.is_file():
        os.environ['XAUTHORITY'] = str(xauthority)


configure_desktop_display()

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
import tkinter as tk  # noqa: E402
import yaml  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from tkinter import messagebox, ttk  # noqa: E402

from camera_control_gui import run_v4l2_control  # noqa: E402


@dataclass(frozen=True)
class CameraSelection:
    number: int
    name: str
    kind: str
    topic: str
    parameters: dict[str, Any]
    controls: dict[str, int] | None = None


class FrameSubscriber(Node):
    def __init__(self, topic: str) -> None:
        super().__init__('collect_cam_img')
        self._bridge = CvBridge()
        self._latest_frame: np.ndarray | None = None
        self._subscriptions = [
            self.create_subscription(
                Image,
                topic,
                self._on_image,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
            ),
            self.create_subscription(
                Image,
                topic,
                self._on_image,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            ),
        ]

    def _on_image(self, message: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(
                message,
                desired_encoding='rgb8',
            )
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f'Frame conversion failed: {error}')
            return
        self._latest_frame = np.ascontiguousarray(frame)

    def take_latest_frame(self) -> np.ndarray | None:
        frame = self._latest_frame
        self._latest_frame = None
        return frame


class CaptureGui:
    def __init__(self, selection: CameraSelection, output_dir: Path) -> None:
        self._selection = selection
        self._output_dir = output_dir
        self._latest_frame: np.ndarray | None = None
        self._photo: tk.PhotoImage | None = None
        self._open = True

        width = int(selection.parameters.get('image_width', 1280))
        height = int(selection.parameters.get('image_height', 720))

        self._root = tk.Tk()
        self._root.title(f'{selection.name} YOLO training image collector')
        self._root.resizable(False, False)
        self._root.protocol('WM_DELETE_WINDOW', self.close)

        frame = ttk.Frame(self._root, padding=10)
        frame.grid(sticky='nsew')

        self._canvas = tk.Canvas(
            frame,
            width=width,
            height=height,
            background='black',
            highlightthickness=0,
        )
        self._canvas.grid(row=0, column=0, sticky='nsew')
        self._image_item = self._canvas.create_image(0, 0, anchor='nw')

        controls = ttk.Frame(frame, padding=(12, 0, 0, 0))
        controls.grid(row=0, column=1, sticky='ns')
        controls.rowconfigure(0, weight=1)
        controls.rowconfigure(2, weight=1)

        self._capture_button = ttk.Button(
            controls,
            text='截图',
            command=self._capture,
            state=tk.DISABLED,
            width=12,
        )
        self._capture_button.grid(row=1, column=0, sticky='ew')

        self._status = tk.StringVar(value='等待画面')
        ttk.Label(
            controls,
            textvariable=self._status,
            anchor='center',
            justify='center',
            wraplength=160,
        ).grid(row=3, column=0, pady=(12, 0), sticky='ew')

        self._root.update_idletasks()
        self._root.geometry('+20+60')

    def show_frame(self, frame_rgb: np.ndarray) -> None:
        height, width = frame_rgb.shape[:2]
        ppm_header = f'P6\n{width} {height}\n255\n'.encode('ascii')
        self._photo = tk.PhotoImage(
            data=ppm_header + frame_rgb.tobytes(),
            format='PPM',
        )
        self._canvas.configure(width=width, height=height)
        self._canvas.itemconfigure(self._image_item, image=self._photo)
        self._latest_frame = frame_rgb
        if str(self._capture_button['state']) == tk.DISABLED:
            self._capture_button.configure(state=tk.NORMAL)
            self._status.set('画面已就绪')

    def _capture(self) -> None:
        if self._latest_frame is None:
            return

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        output_path = self._output_dir / f'{timestamp}.png'
        frame_bgr = cv2.cvtColor(self._latest_frame.copy(), cv2.COLOR_RGB2BGR)
        if not cv2.imwrite(str(output_path), frame_bgr):
            error = f'无法保存截图：{output_path}'
            self._status.set('保存失败')
            messagebox.showerror('截图失败', error, parent=self._root)
            print(f'error: {error}', file=sys.stderr, flush=True)
            return

        self._status.set(f'已保存\n{output_path.name}')
        print(f'Saved {output_path}', flush=True)

    def update(self) -> None:
        if not self._open:
            return
        try:
            self._root.update_idletasks()
            self._root.update()
        except tk.TclError:
            self._open = False

    def is_open(self) -> bool:
        return self._open

    def close(self) -> None:
        if not self._open:
            return
        self._open = False
        try:
            self._root.destroy()
        except tk.TclError:
            pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Preview a configured camera and save YOLO training images.',
    )
    parser.add_argument(
        '--cam',
        type=int,
        choices=(1, 2, 3),
        required=True,
        help='camera number: 1=left USB, 2=right USB, 3=D435i color',
    )
    return parser.parse_args()


def load_config() -> dict[str, Any]:
    with CONFIG_PATH.open(encoding='utf-8') as config_file:
        return yaml.safe_load(config_file)


def select_camera(cam: int, config: dict[str, Any]) -> CameraSelection:
    perception = config['fruit_perception_node']['ros__parameters']
    controls = config['usb_camera_controls']['ros__parameters']

    if cam == 1:
        return CameraSelection(
            number=cam,
            name='left_camera',
            kind='usb',
            topic=str(perception['left_input_topic']),
            parameters=dict(config['left_camera']['ros__parameters']),
            controls={
                'backlight_compensation': int(
                    controls['left_backlight_compensation'],
                ),
                'gamma': int(controls['left_gamma']),
                'power_line_frequency': int(
                    controls['left_power_line_frequency'],
                ),
            },
        )
    if cam == 2:
        return CameraSelection(
            number=cam,
            name='right_camera',
            kind='usb',
            topic=str(perception['right_input_topic']),
            parameters=dict(config['right_camera']['ros__parameters']),
            controls={
                'backlight_compensation': int(
                    controls['right_backlight_compensation'],
                ),
                'gamma': int(controls['right_gamma']),
                'power_line_frequency': int(
                    controls['right_power_line_frequency'],
                ),
            },
        )

    return CameraSelection(
        number=cam,
        name='d435i',
        kind='d435i',
        topic='/camera/camera/color/image_raw',
        parameters=dict(config['d435i_camera']['ros__parameters']),
    )


def apply_usb_controls(selection: CameraSelection) -> None:
    parameters = selection.parameters
    device = str(parameters['video_device'])

    for name, value in (selection.controls or {}).items():
        run_v4l2_control(device, name, int(value))
    run_v4l2_control(
        device,
        'white_balance_automatic',
        int(bool(parameters['auto_white_balance'])),
    )
    if not bool(parameters['auto_white_balance']):
        run_v4l2_control(
            device,
            'white_balance_temperature',
            int(parameters['white_balance']),
        )

    auto_exposure = bool(parameters['autoexposure'])
    run_v4l2_control(device, 'auto_exposure', 3 if auto_exposure else 1)
    if not auto_exposure:
        run_v4l2_control(
            device,
            'exposure_time_absolute',
            int(parameters['exposure']),
        )


def write_ros_params_file(selection: CameraSelection) -> Path:
    TMP_DIR.mkdir(parents=True, exist_ok=True)
    params_path = TMP_DIR / f'collect_cam_img_{selection.name}_params.yaml'
    node_key = '/camera/camera' if selection.kind == 'd435i' else '/**'
    document = {node_key: {'ros__parameters': selection.parameters}}
    with params_path.open('w', encoding='utf-8') as params_file:
        yaml.safe_dump(document, params_file, sort_keys=False)
    return params_path


def ros_param_value(value: Any) -> str:
    text = yaml.safe_dump(value, default_flow_style=True).strip()
    if text.endswith('\n...'):
        text = text[:-4].strip()
    return text


def append_component_params(command: list[str], parameters: dict[str, Any]) -> None:
    for key, value in parameters.items():
        command.extend(['-p', f'{key}:={ros_param_value(value)}'])


def start_camera(selection: CameraSelection) -> tuple[subprocess.Popen, Path]:
    TMP_DIR.mkdir(parents=True, exist_ok=True)
    log_path = TMP_DIR / f'collect_cam_img_{selection.name}.log'
    params_path = write_ros_params_file(selection)

    if selection.kind == 'usb':
        apply_usb_controls(selection)
        command = [
            'ros2',
            'component',
            'standalone',
            'usb_cam',
            'usb_cam::UsbCamNode',
            '-n',
            selection.name,
            '-c',
            f'collect_cam_img_{selection.name}_container',
            '-r',
            f'__node:={selection.name}',
            '-r',
            f'image_raw:={selection.topic}',
            '-r',
            f'camera_info:=/{selection.name}/camera_info',
            '-r',
            f'/set_capture:=/{selection.name}/set_capture',
        ]
    else:
        command = [
            'ros2',
            'component',
            'standalone',
            'realsense2_camera',
            'realsense2_camera::RealSenseNodeFactory',
            '-n',
            'camera',
            '--node-namespace',
            '/camera',
            '-c',
            'collect_cam_img_d435i_container',
        ]

    append_component_params(command, selection.parameters)
    with log_path.open('w', encoding='utf-8') as log_file:
        log_file.write('$ ' + ' '.join(command) + '\n')
        log_file.write(f'# params snapshot: {params_path}\n')
        log_file.flush()
        process = subprocess.Popen(
            command,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
    return process, log_path


def stop_process(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        process.wait(timeout=5.0)
    except Exception:  # noqa: BLE001
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=2.0)
        except Exception:  # noqa: BLE001
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except Exception:  # noqa: BLE001
                pass
            process.wait(timeout=1.0)


def check_camera_process(
    process: subprocess.Popen | None,
    log_path: Path,
) -> None:
    if process is None:
        return
    return_code = process.poll()
    if return_code is None:
        return

    tail = ''
    try:
        lines = log_path.read_text(encoding='utf-8', errors='replace').splitlines()
        tail = '\n'.join(lines[-20:])
    except OSError:
        pass
    raise RuntimeError(
        f'Camera process exited with code {return_code}. '
        f'Log: {log_path}\n{tail}',
    )


def wait_for_publisher(node: Node, topic: str, timeout_seconds: float) -> bool:
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        if node.count_publishers(topic) > 0:
            return True
        rclpy.spin_once(node, timeout_sec=0.1)
    return node.count_publishers(topic) > 0


def run() -> int:
    args = parse_args()
    selection = select_camera(args.cam, load_config())
    output_dir = OUTPUT_ROOT / f'cam{selection.number}'
    output_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = FrameSubscriber(selection.topic)
    camera_process: subprocess.Popen | None = None
    log_path = TMP_DIR / f'collect_cam_img_{selection.name}.log'
    gui: CaptureGui | None = None

    try:
        if wait_for_publisher(node, selection.topic, 1.0):
            print(f'Using existing publisher on {selection.topic}', flush=True)
        else:
            camera_process, log_path = start_camera(selection)
            print(
                f'Started {selection.name}; image topic {selection.topic}; '
                f'log {log_path}',
                flush=True,
            )

        gui = CaptureGui(selection, output_dir)
        first_frame_deadline = (
            time.monotonic() + 10.0
            if selection.kind == 'd435i' and camera_process is not None
            else None
        )
        announced_first_frame = False
        last_wait_message = 0.0

        while rclpy.ok() and gui.is_open():
            check_camera_process(camera_process, log_path)
            rclpy.spin_once(node, timeout_sec=0.01)
            frame = node.take_latest_frame()
            if frame is not None:
                if not announced_first_frame:
                    print(
                        f'Received first frame from {selection.topic}: '
                        f'{frame.shape[1]}x{frame.shape[0]}',
                        flush=True,
                    )
                    announced_first_frame = True
                gui.show_frame(frame)
            elif not announced_first_frame:
                now = time.monotonic()
                if now - last_wait_message > 2.0:
                    print(f'Waiting for frames on {selection.topic}...', flush=True)
                    last_wait_message = now
                if first_frame_deadline is not None and now >= first_frame_deadline:
                    check_camera_process(camera_process, log_path)
                    raise RuntimeError(
                        f'Timed out waiting for D435i frames on {selection.topic}',
                    )
            gui.update()
    finally:
        if gui is not None:
            gui.close()
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
        stop_process(camera_process)

    return 0


def main() -> None:
    try:
        raise SystemExit(run())
    except KeyboardInterrupt:
        raise SystemExit(130) from None
    except Exception as error:  # noqa: BLE001
        print(f'error: {error}', file=sys.stderr)
        raise SystemExit(1) from None


if __name__ == '__main__':
    main()
