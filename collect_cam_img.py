#!/usr/bin/env python3
"""Interactive single-camera YOLO training image collector."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import signal
import subprocess
import time
from typing import Any


def configure_desktop_display() -> None:
    current_display = os.environ.get('DISPLAY', '').strip()
    if not current_display:
        os.environ['DISPLAY'] = ':0'

    uid = str(os.getuid())
    os.environ.setdefault('XDG_RUNTIME_DIR', f'/run/user/{uid}')

    gdm_xauthority = Path(f'/run/user/{uid}/gdm/Xauthority')
    home_xauthority = Path.home() / '.Xauthority'
    if 'XAUTHORITY' not in os.environ:
        if gdm_xauthority.is_file():
            os.environ['XAUTHORITY'] = str(gdm_xauthority)
        elif home_xauthority.is_file():
            os.environ['XAUTHORITY'] = str(home_xauthority)

    user_bus = Path(f'/run/user/{uid}/bus')
    if 'DBUS_SESSION_BUS_ADDRESS' not in os.environ and user_bus.is_socket():
        os.environ['DBUS_SESSION_BUS_ADDRESS'] = f'unix:path={user_bus}'


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

WORKSPACE = Path('/home/rc4/fruit_ws')
CONFIG_PATH = WORKSPACE / 'src/fruit_perception/config/fruit_perception.yaml'
TMP_DIR = WORKSPACE / 'tmp'
OUTPUT_ROOT = WORKSPACE / 'data/yolo_images/camera'


@dataclass(frozen=True)
class CameraSelection:
    name: str
    topic: str
    parameters: dict[str, Any]


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

        controls = ttk.Frame(frame, padding=(0, 10, 0, 0))
        controls.grid(row=1, column=0, sticky='ew')
        controls.columnconfigure(2, weight=1)

        self._save_button = ttk.Button(
            controls,
            text='Save Frame (Space)',
            command=self.save_current_frame,
        )
        self._save_button.grid(row=0, column=0, padx=(0, 10))

        self._counter_var = tk.StringVar()
        self._counter_label = ttk.Label(
            controls,
            textvariable=self._counter_var,
        )
        self._counter_label.grid(row=0, column=1, padx=(0, 10))

        self._status_var = tk.StringVar(
            value=f'Saving to: {self._output_dir}',
        )
        self._status_label = ttk.Label(
            controls,
            textvariable=self._status_var,
        )
        self._status_label.grid(row=0, column=2, sticky='w')

        self._root.bind('<space>', lambda _event: self.save_current_frame())
        self._root.bind('<Escape>', lambda _event: self.close())
        self._root.bind('q', lambda _event: self.close())
        self._root.bind('Q', lambda _event: self.close())

        self.refresh_counter()

    @property
    def is_open(self) -> bool:
        return self._open

    def update_frame(self, frame: np.ndarray | None) -> None:
        if not self._open:
            return
        if frame is not None:
            self._latest_frame = frame
            header = f'P6\n{frame.shape[1]} {frame.shape[0]}\n255\n'.encode('ascii')
            self._photo = tk.PhotoImage(
                data=header + frame.tobytes(),
                format='PPM',
            )
            self._canvas.itemconfigure(self._image_item, image=self._photo)
        self._root.update_idletasks()
        self._root.update()

    def count_saved_images(self) -> int:
        return len(
            list(self._output_dir.glob('*.jpg'))
            + list(self._output_dir.glob('*.png')),
        )

    def refresh_counter(self) -> None:
        self._counter_var.set(f'Saved: {self.count_saved_images()}')

    def next_image_path(self) -> Path:
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        millis = int((time.time() % 1.0) * 1000)
        filename = f'frame_{timestamp}_{millis:03d}.jpg'
        return self._output_dir / filename

    def save_current_frame(self) -> None:
        if not self._open:
            return
        if self._latest_frame is None:
            self._status_var.set('No frame available to save yet.')
            return

        bgr_frame = cv2.cvtColor(self._latest_frame, cv2.COLOR_RGB2BGR)
        destination = self.next_image_path()
        try:
            if not cv2.imwrite(str(destination), bgr_frame):
                raise OSError(f'Could not save image: {destination}')
        except Exception as error:  # noqa: BLE001
            messagebox.showerror('Save Failed', str(error))
            self._status_var.set(f'Save failed: {error}')
            return

        self.refresh_counter()
        self._status_var.set(f'Saved: {destination.name}')

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
        description='Preview the camera and save YOLO training images.',
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=OUTPUT_ROOT,
        help=f'directory to save captured images (default: {OUTPUT_ROOT})',
    )
    return parser.parse_args()


def load_config() -> dict[str, Any]:
    with CONFIG_PATH.open(encoding='utf-8') as config_file:
        return yaml.safe_load(config_file)


def select_camera(config: dict[str, Any]) -> CameraSelection:
    return CameraSelection(
        name='camera',
        topic='/camera/color/image_raw',
        parameters=dict(config['camera']['ros__parameters']),
    )


def write_ros_params_file(selection: CameraSelection) -> Path:
    TMP_DIR.mkdir(parents=True, exist_ok=True)
    params_path = TMP_DIR / f'collect_cam_img_{selection.name}_params.yaml'
    document = {'/camera': {'ros__parameters': selection.parameters}}
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

    command = [
        'ros2',
        'component',
        'standalone',
        'realsense2_camera',
        'realsense2_camera::RealSenseNodeFactory',
        '-n',
        'camera',
        '--node-namespace',
        '/',
        '-c',
        'collect_cam_img_camera_container',
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
    selection = select_camera(load_config())
    output_dir = args.output_dir
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
            start_deadline = time.monotonic() + 10.0
            while time.monotonic() < start_deadline:
                check_camera_process(camera_process, log_path)
                if wait_for_publisher(node, selection.topic, 0.2):
                    break
            else:
                check_camera_process(camera_process, log_path)
                raise TimeoutError(
                    f'Timed out waiting for publisher on {selection.topic}',
                )

        gui = CaptureGui(selection, output_dir)
        while rclpy.ok() and gui.is_open:
            check_camera_process(camera_process, log_path)
            rclpy.spin_once(node, timeout_sec=0.03)
            gui.update_frame(node.take_latest_frame())
    except KeyboardInterrupt:
        pass
    finally:
        if gui is not None:
            gui.close()
        node.destroy_node()
        rclpy.shutdown()
        stop_process(camera_process)

    return 0


if __name__ == '__main__':
    raise SystemExit(run())
