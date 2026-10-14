#!/usr/bin/env python3
"""OpenCV GUI camera detector and 3D depth inspection tool for single RealSense camera."""

from __future__ import annotations

import argparse
import ast
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Callable


DEFAULT_DISPLAY = ':0'


def display_from_argv(argv: list[str]) -> str:
    for index, argument in enumerate(argv):
        if argument in ('--x11-display', '--x-display') and index + 1 < len(argv):
            return argv[index + 1]
        if argument.startswith(('--x11-display=', '--x-display=')):
            return argument.split('=', maxsplit=1)[1]
        if argument == '--display' and index + 1 < len(argv) and argv[index + 1].startswith(':'):
            return argv[index + 1]
        if argument.startswith('--display=:'):
            return argument.split('=', maxsplit=1)[1]
    return os.environ.get('DISPLAY', DEFAULT_DISPLAY)


def configure_desktop_display(display: str) -> None:
    runtime_dir = Path(f'/run/user/{os.getuid()}')
    xauthority = runtime_dir / 'gdm' / 'Xauthority'
    os.environ['DISPLAY'] = display
    os.environ.setdefault('XDG_RUNTIME_DIR', str(runtime_dir))
    os.environ.setdefault(
        'DBUS_SESSION_BUS_ADDRESS',
        f'unix:path={runtime_dir}/bus',
    )
    if xauthority.is_file():
        os.environ['XAUTHORITY'] = str(xauthority)


configure_desktop_display(display_from_argv(sys.argv[1:]))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import openvino as ov  # noqa: E402
import rclpy  # noqa: E402
import tkinter as tk  # noqa: E402
import yaml  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from fruit_interfaces.msg import ArmTelemetry  # noqa: E402
from fruit_arm_planner.joint_transformer import JointTransformer  # noqa: E402
from fruit_perception_geometry.camera_target_transform import (  # noqa: E402
    load_camera_extrinsic,
    transform_camera_to_end_effector_mm,
    transform_end_to_base_mm,
)
from openvino.preprocess import PrePostProcessor  # noqa: E402
from rcl_interfaces.msg import ParameterType  # noqa: E402
from rcl_interfaces.srv import DescribeParameters, GetParameters, SetParameters  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
from tkinter import ttk  # noqa: E402


WORKSPACE = Path('/home/rc4/fruit_ws')
CONFIG_PATH = WORKSPACE / 'src/fruit_perception/config/fruit_perception.yaml'
EXTRINSIC_CONFIG_PATH = WORKSPACE / 'src/fruit_mission/config/camera_extrinsic.yaml'
TMP_DIR = WORKSPACE / 'tmp'
MODEL_INPUT_SIZE = 640
PAD_COLOR = 114
_OWNED_PROCESS_GROUPS: dict[int, int] = {}
_SHUTDOWN_SIGNALS = tuple(
    signum for signum in (signal.SIGINT, signal.SIGTERM, getattr(signal, 'SIGHUP', None))
    if signum is not None
)


@dataclass(frozen=True)
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    score: float
    label: str


@dataclass(frozen=True)
class LetterboxTransform:
    scale: float
    pad_x: int
    pad_y: int
    original_width: int
    original_height: int


@dataclass(frozen=True)
class CameraSelection:
    name: str
    kind: str
    color_topic: str
    depth_topic: str
    raw_depth_topic: str
    info_topic: str
    parameters: dict[str, Any]


class FrameSubscriber(Node):
    """Subscribes to color image, aligned depth, raw depth, camera intrinsics, and arm telemetry."""

    def __init__(
        self,
        color_topic: str,
        depth_topic: str,
        raw_depth_topic: str,
        info_topic: str,
        joint_state_topic: str = '/arm_planner_node/joint_states',
        arm_config_path: Path | str | None = None,
    ) -> None:
        super().__init__('test_camera_detector')
        self._bridge = CvBridge()
        self._latest_color: np.ndarray | None = None
        self._latest_color_header: Any | None = None
        self._latest_depth: np.ndarray | None = None
        self._latest_raw_depth: np.ndarray | None = None
        self.camera_info: CameraInfo | None = None

        self._tcp_pose: tuple[float, float, float, float, float] | None = None
        self._tcp_received_at = 0.0
        self.status_max_age_sec = 0.25
        self.transformer: JointTransformer | None = None
        self.joint_state_topic = joint_state_topic
        self.telemetry_conflict = False

        if arm_config_path is not None and Path(arm_config_path).is_file():
            try:
                with Path(arm_config_path).open(encoding='utf-8') as stream:
                    arm_params = yaml.safe_load(stream)['arm_planner_node']['ros__parameters']
                self.transformer = JointTransformer.from_config_dict(arm_params['joint_mapping'])
                self.status_max_age_sec = float(arm_params['planning']['status_max_age_sec'])
            except Exception as error:  # noqa: BLE001
                self.get_logger().warning(f'Failed to load arm configuration: {error}')

        qos_best_effort = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_reliable = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        qos_telemetry = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(Image, color_topic, self._on_color, qos_best_effort)
        self.create_subscription(Image, depth_topic, self._on_depth, qos_best_effort)
        self.create_subscription(Image, raw_depth_topic, self._on_raw_depth, qos_best_effort)
        self.create_subscription(CameraInfo, info_topic, self._on_info, qos_reliable)
        self.create_subscription(
            ArmTelemetry, joint_state_topic, self._on_telemetry, qos_telemetry,
        )

        self.annotated_topic = '/fruit_perception/camera/annotated_image'
        self._annotated_pub = self.create_publisher(
            Image, self.annotated_topic, qos_best_effort,
        )

    @property
    def latest_header(self) -> Any | None:
        return self._latest_color_header

    def publish_annotated(self, frame_rgb: np.ndarray, header: Any | None = None) -> None:
        try:
            msg = self._bridge.cv2_to_imgmsg(frame_rgb, encoding='rgb8')
            if header is not None:
                msg.header = header
            else:
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_color_optical_frame'
            self._annotated_pub.publish(msg)
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f'Failed to publish annotated image: {error}')

    def set_telemetry_conflict(self, conflict: bool) -> None:
        self.telemetry_conflict = conflict
        if conflict:
            self._tcp_pose = None

    def _on_color(self, message: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(message, desired_encoding='rgb8')
            self._latest_color = np.ascontiguousarray(frame)
            self._latest_color_header = message.header
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f'Color frame conversion failed: {error}')

    def _on_depth(self, message: Image) -> None:
        try:
            depth = self._bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
            self._latest_depth = np.ascontiguousarray(depth)
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f'Depth frame conversion failed: {error}')

    def _on_raw_depth(self, message: Image) -> None:
        try:
            raw_depth = self._bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
            if raw_depth.ndim == 2 and raw_depth.dtype == np.uint16:
                self._latest_raw_depth = np.ascontiguousarray(raw_depth)
            else:
                self.get_logger().warning(
                    f'Unexpected raw depth frame format: {raw_depth.dtype} {raw_depth.shape}',
                )
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f'Raw depth frame conversion failed: {error}')

    def _on_info(self, message: CameraInfo) -> None:
        self.camera_info = message

    def _on_telemetry(self, message: ArmTelemetry) -> None:
        if self.telemetry_conflict:
            return
        received_at = time.monotonic()
        if message.header.frame_id != 'arm_base_link':
            return
        names = [f'joint_{i}' for i in range(1, 5)]
        hardware = message.hardware_joint_state
        model = message.model_joint_state
        for state in (hardware, model):
            if (len(state.name) != 4 or set(state.name) != set(names)
                    or len(state.position) != 4
                    or not all(math.isfinite(q) for q in state.position)):
                return
        joints = tuple(hardware.position[hardware.name.index(name)] for name in names)
        tcp = message.tcp_position
        if 'joint_1' not in model.name:
            return
        yaw = float(model.position[model.name.index('joint_1')])
        pose = (float(tcp.x), float(tcp.y), float(tcp.z), float(message.tcp_pitch_deg), yaw)
        if (self.transformer is not None and not self.transformer.check_hardware_limits(joints)[0]
                or not all(math.isfinite(v) for v in pose)):
            return
        self._tcp_pose = pose
        self._tcp_received_at = received_at

    def get_fresh_tcp_pose(self) -> tuple[float, float, float, float, float] | None:
        if self.telemetry_conflict or self._tcp_pose is None:
            return None
        if time.monotonic() - self._tcp_received_at > self.status_max_age_sec:
            return None
        return self._tcp_pose

    def take_latest(self) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None]:
        color = self._latest_color
        depth = self._latest_depth
        raw_depth = self._latest_raw_depth
        self._latest_color = None
        return color, depth, raw_depth


def normalize_display_mode(val: str) -> str:
    cleaned = str(val).strip().lower()
    if cleaned in ('false', '0', 'none', 'off', 'no', 'headless'):
        return 'false'
    if cleaned in ('true', '1', 'on', 'yes', 'gui'):
        return 'rgb'
    if cleaned in ('rgb', 'depth', 'both'):
        return cleaned
    raise argparse.ArgumentTypeError(
        f"Invalid display mode: '{val}'. Choose from: false, true, rgb, depth, both."
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='RealSense single camera object detection and 3D depth debug viewer.',
    )
    parser.add_argument(
        '--model',
        '-m',
        default='A',
        choices=('A', 'B', 'C', 'D', 'a', 'b', 'c', 'd'),
        help='area detector: A/C vegetables, B/D fruits (default: A)',
    )
    parser.add_argument(
        '--conf',
        type=float,
        default=None,
        metavar='THRESHOLD',
        help='confidence threshold override (default: from fruit_perception.yaml)',
    )
    parser.add_argument(
        '--iou',
        type=float,
        default=None,
        metavar='THRESHOLD',
        help='NMS IoU threshold override (default: from fruit_perception.yaml)',
    )
    parser.add_argument(
        '--display',
        type=normalize_display_mode,
        default='false',
        help='preview window display mode: false (default: headless), true/rgb, depth, or both',
    )
    parser.add_argument(
        '--scale',
        type=float,
        default=0.5,
        metavar='SCALE',
        help='preview window display scaling factor (default: 0.5 for lightweight view)',
    )
    parser.add_argument(
        '--gui-fps',
        type=float,
        default=15.0,
        metavar='FPS',
        help='maximum GUI rendering refresh rate in FPS (default: 15.0)',
    )
    parser.add_argument(
        '--x11-display',
        default=os.environ.get('DISPLAY', DEFAULT_DISPLAY),
        help=f'desktop X11 display for GUI window (default: {DEFAULT_DISPLAY})',
    )
    parser.add_argument(
        '--controls',
        action='store_true',
        default=None,
        help='open the camera parameters adjustment panel (default: on in GUI, off in headless)',
    )
    parser.add_argument(
        '--no-controls',
        dest='controls',
        action='store_false',
        help='do not open the camera parameters adjustment panel',
    )
    parsed = parser.parse_args()
    if parsed.controls is None:
        parsed.controls = (parsed.display != 'false')
    elif parsed.display == 'false' and parsed.controls:
        print(
            'Warning: --controls is not supported in headless mode (--display false); '
            'disabling control panel.',
            file=sys.stderr,
            flush=True,
        )
        parsed.controls = False
    return parsed


def load_config() -> dict[str, Any]:
    with CONFIG_PATH.open(encoding='utf-8') as config_file:
        return yaml.safe_load(config_file)


def select_camera(config: dict[str, Any]) -> CameraSelection:
    parameters = dict(config['camera']['ros__parameters'])
    return CameraSelection(
        name='camera',
        kind='camera',
        color_topic='/camera/color/image_raw',
        depth_topic='/camera/aligned_depth_to_color/image_raw',
        raw_depth_topic='/camera/depth/image_rect_raw',
        info_topic='/camera/color/camera_info',
        parameters=parameters,
    )


def model_path_for(letter: str, config: dict[str, Any]) -> Path:
    perception = config['fruit_perception_node']['ros__parameters']
    model_root = Path(perception['model_root'])
    if not model_root.is_absolute():
        raise ValueError('model_root must be an absolute source-directory path')
    detector = 'fruit_type' if letter.upper() in ('B', 'D') else 'vegetable_type'
    return model_root / detector / 'best.onnx'


def load_extrinsic_transform() -> tuple[np.ndarray, np.ndarray] | None:
    """Load R_E_C and t_E_C (mm) from camera_extrinsic.yaml using shared validation."""
    try:
        r, t = load_camera_extrinsic(EXTRINSIC_CONFIG_PATH)
        return np.array(r, dtype=float), np.array(t, dtype=float)
    except Exception as error:  # noqa: BLE001
        print(f'Warning: failed to load camera extrinsic: {error}', file=sys.stderr)
        return None


def get_depth_at(
    depth_image: np.ndarray | None, u: int, v: int, window_radius: int = 3,
) -> float | None:
    """Sample median depth in a small window around (u, v) in meters."""
    if depth_image is None:
        return None
    h, w = depth_image.shape[:2]
    if not (0 <= u < w and 0 <= v < h):
        return None
    u1, u2 = max(0, u - window_radius), min(w, u + window_radius + 1)
    v1, v2 = max(0, v - window_radius), min(h, v + window_radius + 1)
    window = depth_image[v1:v2, u1:u2]
    valid = window[window > 0]
    if len(valid) == 0:
        return None
    return float(np.median(valid)) * 0.001


def pixel_to_camera_point(
    u: float, v: float, depth_m: float,
    fx: float, fy: float, cx: float, cy: float,
) -> tuple[float, float, float]:
    """Project 2D pixel + depth to 3D point in camera optical frame (m)."""
    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy
    z = depth_m
    return x, y, z


def camera_to_base_point(
    point_cam_m: tuple[float, float, float],
    r_e_c: np.ndarray | list[list[float]],
    t_e_c_mm: np.ndarray | list[float],
    tcp_pose: tuple[float, float, float, float, float],
) -> tuple[float, float, float]:
    """Transform camera optical 3D point (m) to arm base frame coordinates (m).

    tcp_pose: (tcp_x_m, tcp_y_m, tcp_z_m, pitch_deg, yaw_rad)
    """
    cam_mm = tuple(float(v) * 1000.0 for v in point_cam_m)
    r_rows = tuple(tuple(float(v) for v in row) for row in r_e_c)
    t_vals = tuple(float(v) for v in t_e_c_mm)
    end_mm = transform_camera_to_end_effector_mm(cam_mm, r_rows, t_vals)
    origin_mm = (tcp_pose[0] * 1000.0, tcp_pose[1] * 1000.0, tcp_pose[2] * 1000.0)
    pitch_rad = math.radians(tcp_pose[3])
    yaw_rad = float(tcp_pose[4])
    base_mm = transform_end_to_base_mm(end_mm, origin_mm, yaw_rad, pitch_rad)
    return tuple(float(v) / 1000.0 for v in base_mm)


def validate_camera_info(
    info: CameraInfo | None,
    image_width: int,
    image_height: int,
) -> tuple[float, float, float, float] | None:
    """Validate camera info matrix; return (fx, fy, cx, cy) or None if invalid."""
    if info is None:
        return None
    if info.width > 0 and info.height > 0:
        if info.width != image_width or info.height != image_height:
            return None
    k = info.k
    if len(k) < 9:
        return None
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    if not (math.isfinite(fx) and math.isfinite(fy) and math.isfinite(cx) and math.isfinite(cy)):
        return None
    if fx <= 0.0 or fy <= 0.0 or cx < 0.0 or cy < 0.0 or cx > image_width or cy > image_height:
        return None
    return fx, fy, cx, cy


def launch_background_process(
    command: list[str], log_filename: str,
) -> tuple[subprocess.Popen, Path]:
    TMP_DIR.mkdir(parents=True, exist_ok=True)
    log_path = TMP_DIR / log_filename
    with log_path.open('w', encoding='utf-8') as log_file:
        log_file.write('$ ' + ' '.join(command) + '\n')
        log_file.flush()
        process = subprocess.Popen(
            command,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
    _register_process_group(process)
    return process, log_path


def ensure_arm_communication(
    node: FrameSubscriber,
) -> tuple[subprocess.Popen | None, subprocess.Popen | None, subprocess.Popen | None]:
    """Check for existing serial bridge and planner; launch only missing nodes.

    Returns (arm_debug_process, bridge_process, planner_process).
    """
    deadline = time.monotonic() + 0.6
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    node_names = set(node.get_node_names())
    status_topic = '/fruit_hardware_bridge/arm/status'
    joint_state_topic = getattr(node, 'joint_state_topic', '/arm_planner_node/joint_states')

    planner_pubs = node.get_publishers_info_by_topic(joint_state_topic)
    bridge_pubs = node.get_publishers_info_by_topic(status_topic)

    expected_telemetry_type = 'fruit_interfaces/msg/ArmTelemetry'
    expected_status_type = 'fruit_interfaces/msg/ArmStatus'

    # Check for telemetry publisher conflicts or type mismatches
    if len(planner_pubs) > 1:
        pub_nodes = [f'{p.node_namespace}/{p.node_name}' for p in planner_pubs]
        print(
            f'Warning: {joint_state_topic} has {len(planner_pubs)} publishers '
            f'(duplicate conflict: {pub_nodes}); cannot use telemetry, '
            'Base coordinates will be N/A',
            file=sys.stderr,
            flush=True,
        )
        node.set_telemetry_conflict(True)
        return None, None, None

    if len(planner_pubs) == 1 and planner_pubs[0].topic_type != expected_telemetry_type:
        print(
            f'Warning: {joint_state_topic} publisher has unexpected type '
            f"'{planner_pubs[0].topic_type}'; expected '{expected_telemetry_type}'. "
            'Base coordinates will be N/A',
            file=sys.stderr,
            flush=True,
        )
        node.set_telemetry_conflict(True)
        return None, None, None

    # Check for bridge publisher conflicts or type mismatches
    if len(bridge_pubs) > 1:
        pub_nodes = [f'{p.node_namespace}/{p.node_name}' for p in bridge_pubs]
        print(
            f'Warning: {status_topic} has {len(bridge_pubs)} publishers '
            f'(duplicate conflict: {pub_nodes}); cannot reuse bridge, '
            'Base coordinates will be N/A',
            file=sys.stderr,
            flush=True,
        )
        node.set_telemetry_conflict(True)
        return None, None, None

    if len(bridge_pubs) == 1 and bridge_pubs[0].topic_type != expected_status_type:
        print(
            f'Warning: {status_topic} publisher has unexpected type '
            f"'{bridge_pubs[0].topic_type}'; expected '{expected_status_type}'. "
            'Base coordinates will be N/A',
            file=sys.stderr,
            flush=True,
        )
        node.set_telemetry_conflict(True)
        return None, None, None

    has_planner = ('arm_planner_node' in node_names) or (len(planner_pubs) == 1)
    has_bridge = ('serial_controller' in node_names) or (len(bridge_pubs) == 1)

    debug_proc: subprocess.Popen | None = None
    bridge_proc: subprocess.Popen | None = None
    planner_proc: subprocess.Popen | None = None

    try:
        if has_bridge and has_planner:
            print('Reusing existing serial bridge and arm planner', flush=True)
        elif not has_bridge and not has_planner:
            print(
                'Neither serial bridge nor arm planner detected; '
                'launching fruit_arm_debug.launch.py...',
                flush=True,
            )
            debug_proc, log_p = launch_background_process(
                ['ros2', 'launch', 'fruit_arm_planner', 'fruit_arm_debug.launch.py',
                 'log_level:=info'],
                'test_camera_detector_arm_debug.log',
            )
            print(f'Started fruit_arm_debug launch; log={log_p}', flush=True)
        elif has_bridge and not has_planner:
            print(
                'Serial bridge detected, arm planner missing; '
                'launching fruit_arm_planner.launch.py...',
                flush=True,
            )
            planner_proc, log_p = launch_background_process(
                ['ros2', 'launch', 'fruit_arm_planner', 'fruit_arm_planner.launch.py',
                 'log_level:=info'],
                'test_camera_detector_planner.log',
            )
            print(f'Started fruit_arm_planner launch; log={log_p}', flush=True)
        else:  # not has_bridge and has_planner
            print(
                'Arm planner detected, serial bridge missing; '
                'launching auto_serial_bridge.launch.py...',
                flush=True,
            )
            bridge_proc, log_p = launch_background_process(
                ['ros2', 'launch', 'auto_serial_bridge', 'auto_serial_bridge.launch.py',
                 'log_level:=info'],
                'test_camera_detector_serial.log',
            )
            print(f'Started auto_serial_bridge launch; log={log_p}', flush=True)

        if debug_proc is not None or planner_proc is not None or bridge_proc is not None:
            print('Waiting up to 10s for arm telemetry...', flush=True)
            wait_deadline = time.monotonic() + 10.0
            while time.monotonic() < wait_deadline:
                if node.get_fresh_tcp_pose() is not None:
                    print('Arm telemetry received successfully', flush=True)
                    break
                for p, name in (
                    (debug_proc, 'arm_debug'),
                    (bridge_proc, 'serial_bridge'),
                    (planner_proc, 'arm_planner'),
                ):
                    if p is not None and p.poll() is not None:
                        print(
                            f'Warning: {name} exited unexpectedly: code {p.returncode}',
                            file=sys.stderr,
                            flush=True,
                        )
                rclpy.spin_once(node, timeout_sec=0.1)
            else:
                print(
                    'Warning: arm telemetry not received within 10s; Base coordinates will be N/A',
                    file=sys.stderr,
                    flush=True,
                )
    except BaseException:
        if planner_proc is not None:
            stop_process(planner_proc)
        if bridge_proc is not None:
            stop_process(bridge_proc)
        if debug_proc is not None:
            stop_process(debug_proc)
        raise

    return debug_proc, bridge_proc, planner_proc


def ensure_robviz_bridge(node: FrameSubscriber) -> subprocess.Popen | None:
    """Check if RobViz video gateway is running; launch if missing."""
    node_names = set(node.get_node_names())
    if 'fruit_robviz_image_bridge' in node_names:
        print('Reusing existing RobViz video gateway (/fruit_robviz_image_bridge)', flush=True)
        return None

    print(
        'RobViz video gateway (/fruit_robviz_image_bridge) not detected; '
        'launching fruit_robviz_bridge.launch.py...',
        flush=True,
    )
    bridge_proc, log_p = launch_background_process(
        ['ros2', 'launch', 'fruit_robviz_bridge', 'fruit_robviz_bridge.launch.py'],
        'test_camera_detector_robviz_bridge.log',
    )
    print(f'Started RobViz video gateway; log={log_p}', flush=True)
    return bridge_proc


def write_ros_params_file(selection: CameraSelection) -> Path:
    TMP_DIR.mkdir(parents=True, exist_ok=True)
    params_path = TMP_DIR / f'test_camera_detector_{selection.name}_params.yaml'
    document = {
        '/camera': {
            'ros__parameters': selection.parameters,
        },
    }
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
    log_path = TMP_DIR / f'test_camera_detector_{selection.name}.log'
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
        'test_camera_detector_camera_container',
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
    _register_process_group(process)
    return process, log_path


def _register_process_group(process: subprocess.Popen) -> None:
    try:
        process_group_id = os.getpgid(process.pid)
    except ProcessLookupError:
        process_group_id = process.pid
    _OWNED_PROCESS_GROUPS[process.pid] = process_group_id


def _process_group_exists(process_group_id: int) -> bool:
    try:
        os.killpg(process_group_id, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def stop_process(process: subprocess.Popen | None) -> None:
    if process is None:
        return

    process_group_id = _OWNED_PROCESS_GROUPS.get(process.pid)
    if process_group_id is None:
        try:
            process_group_id = os.getpgid(process.pid)
        except ProcessLookupError:
            return

    for stop_signal, wait_timeout in (
        (signal.SIGINT, 5.0),
        (signal.SIGTERM, 2.0),
        (signal.SIGKILL, 1.0),
    ):
        if not _process_group_exists(process_group_id):
            break
        try:
            os.killpg(process_group_id, stop_signal)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=wait_timeout)
        except subprocess.TimeoutExpired:
            continue
        if not _process_group_exists(process_group_id):
            break

    _OWNED_PROCESS_GROUPS.pop(process.pid, None)


def _raise_shutdown_signal(_signum: int, _frame: Any) -> None:
    raise KeyboardInterrupt


def _install_shutdown_handlers() -> None:
    for signum in _SHUTDOWN_SIGNALS:
        signal.signal(signum, _raise_shutdown_signal)


def _ignore_shutdown_signals() -> None:
    for signum in _SHUTDOWN_SIGNALS:
        signal.signal(signum, signal.SIG_IGN)


def parse_nms_output(
    output: np.ndarray,
    confidence_threshold: float,
    labels: list[str],
) -> list[Detection]:
    rows = output.reshape(300, 6)
    detections: list[Detection] = []
    for row in rows:
        score = float(row[4])
        if not math.isfinite(score) or score < confidence_threshold:
            continue
        class_value = float(row[5])
        if not math.isfinite(class_value):
            continue
        class_id = int(round(class_value))
        if class_id < 0 or class_id >= len(labels):
            continue
        if not all(math.isfinite(float(value)) for value in row[:4]):
            continue
        detections.append(
            Detection(
                float(row[0]),
                float(row[1]),
                float(row[2]),
                float(row[3]),
                score,
                labels[class_id],
            ),
        )
    return detections


def parse_raw_yolo_output(
    output: np.ndarray,
    confidence_threshold: float,
    nms_iou_threshold: float,
    labels: list[str],
) -> list[Detection]:
    if output.ndim != 3 or output.shape[0] != 1 or output.shape[1] < 5:
        raise RuntimeError(f'Unsupported model output shape {output.shape}')

    class_count = int(output.shape[1] - 4)
    candidate_count = int(output.shape[2])
    if class_count != len(labels):
        raise RuntimeError('Raw YOLO class count does not match model labels')

    data = output[0]
    boxes: list[list[int]] = []
    scores: list[float] = []
    class_ids: list[int] = []

    for column in range(candidate_count):
        x = float(data[0, column])
        y = float(data[1, column])
        width = float(data[2, column])
        height = float(data[3, column])
        if (
            not math.isfinite(x)
            or not math.isfinite(y)
            or not math.isfinite(width)
            or not math.isfinite(height)
            or width <= 0.0
            or height <= 0.0
        ):
            continue

        class_scores = data[4:, column]
        best_class = int(np.argmax(class_scores))
        best_score = float(class_scores[best_class])
        if not math.isfinite(best_score) or best_score < confidence_threshold:
            continue

        x1 = x - width / 2.0
        y1 = y - height / 2.0
        boxes.append([
            int(round(x1)),
            int(round(y1)),
            max(1, int(round(width))),
            max(1, int(round(height))),
        ])
        scores.append(best_score)
        class_ids.append(best_class)

    kept = cv2.dnn.NMSBoxes(
        boxes,
        scores,
        confidence_threshold,
        nms_iou_threshold,
    )
    if len(kept) == 0:
        return []

    detections: list[Detection] = []
    for kept_index in np.array(kept).reshape(-1):
        index = int(kept_index)
        if index < 0 or index >= len(boxes):
            continue
        x, y, width, height = boxes[index]
        detections.append(
            Detection(
                float(x),
                float(y),
                float(x + width),
                float(y + height),
                scores[index],
                labels[class_ids[index]],
            ),
        )
    return detections


def parse_detections(
    output: np.ndarray,
    confidence_threshold: float,
    nms_iou_threshold: float,
    labels: list[str],
) -> list[Detection]:
    if output.shape == (1, 300, 6):
        detections = parse_nms_output(output, confidence_threshold, labels)
    else:
        detections = parse_raw_yolo_output(
            output,
            confidence_threshold,
            nms_iou_threshold,
            labels,
        )
    detections.sort(key=lambda detection: detection.score, reverse=True)
    return detections


def letterbox_image(frame_rgb: np.ndarray) -> tuple[np.ndarray, LetterboxTransform]:
    if frame_rgb.ndim != 3 or frame_rgb.shape[2] != 3:
        raise RuntimeError(f'Expected RGB image, got shape {frame_rgb.shape}')
    if frame_rgb.dtype != np.uint8:
        raise RuntimeError(f'Expected uint8 image, got {frame_rgb.dtype}')

    height, width = frame_rgb.shape[:2]
    scale = min(MODEL_INPUT_SIZE / width, MODEL_INPUT_SIZE / height)
    resized_width = int(round(width * scale))
    resized_height = int(round(height * scale))
    pad_x = (MODEL_INPUT_SIZE - resized_width) // 2
    pad_y = (MODEL_INPUT_SIZE - resized_height) // 2

    letterboxed = np.full(
        (MODEL_INPUT_SIZE, MODEL_INPUT_SIZE, 3),
        PAD_COLOR,
        dtype=np.uint8,
    )
    resized = cv2.resize(
        frame_rgb,
        (resized_width, resized_height),
        interpolation=cv2.INTER_LINEAR,
    )
    letterboxed[
        pad_y:pad_y + resized_height,
        pad_x:pad_x + resized_width,
    ] = resized
    return (
        np.expand_dims(letterboxed, axis=0),
        LetterboxTransform(scale, pad_x, pad_y, width, height),
    )


def to_original_box(
    detection: Detection,
    transform: LetterboxTransform,
) -> tuple[int, int, int, int]:
    x1 = int(math.floor((detection.x1 - transform.pad_x) / transform.scale))
    y1 = int(math.floor((detection.y1 - transform.pad_y) / transform.scale))
    x2 = int(math.ceil((detection.x2 - transform.pad_x) / transform.scale))
    y2 = int(math.ceil((detection.y2 - transform.pad_y) / transform.scale))

    x1 = min(max(x1, 0), transform.original_width)
    y1 = min(max(y1, 0), transform.original_height)
    x2 = min(max(x2, 0), transform.original_width)
    y2 = min(max(y2, 0), transform.original_height)
    return x1, y1, max(0, x2 - x1), max(0, y2 - y1)


def draw_annotations(
    frame_rgb: np.ndarray,
    depth_frame: np.ndarray | None,
    detections: list[Detection],
    transform: LetterboxTransform,
    intrinsics: tuple[float, float, float, float] | None,
    extrinsic: tuple[np.ndarray, np.ndarray] | None,
    tcp_pose: tuple[float, float, float, float, float] | None,
    mouse_u: int,
    mouse_v: int,
    fps: float,
    model_name: str,
    ripeness_classifier: RipenessClassifier | None = None,
    scale: float = 1.0,
) -> np.ndarray:
    annotated = frame_rgb.copy()
    h, w = annotated.shape[:2]

    center_color = (0, 220, 255)
    f_main = 0.55 if scale < 0.75 else 0.48
    f_sub = 0.48 if scale < 0.75 else 0.42
    f_banner = 0.60 if scale < 0.75 else 0.55
    f_mouse = 0.48 if scale < 0.75 else 0.44

    for detection in detections:
        bx, by, bw, bh = to_original_box(detection, transform)
        if bw <= 0 or bh <= 0:
            continue

        # Ripeness classification
        is_bad = detection.label.lower() == 'bad'
        mat_text = ''
        box_color = (0, 255, 0)

        if is_bad:
            box_color = (0, 0, 255)  # Red for bad produce
            mat_text = '[BAD]'
        elif ripeness_classifier is not None:
            rgb_roi = frame_rgb[by:by + bh, bx:bx + bw]
            mat_result = ripeness_classifier.classify(rgb_roi, detection.label)
            if mat_result is not None:
                mat_status, mat_conf = mat_result
                if mat_status == 'ripe':
                    box_color = (0, 255, 0)  # Green for ripe
                    mat_text = f'[{mat_status.upper()} {mat_conf * 100:.0f}%]'
                else:
                    box_color = (0, 165, 255)  # Orange for unripe
                    mat_text = f'[{mat_status.upper()} {mat_conf * 100:.0f}%]'

        cv2.rectangle(annotated, (bx, by), (bx + bw, by + bh), box_color, 2)

        # Center point
        uc = bx + bw // 2
        vc = by + bh // 2
        depth_m = get_depth_at(depth_frame, uc, vc, window_radius=3)

        # Draw center point crosshair
        cv2.circle(annotated, (uc, vc), 4, center_color, -1)
        cv2.line(annotated, (uc - 8, vc), (uc + 8, vc), center_color, 1)
        cv2.line(annotated, (uc, vc - 8), (uc, vc + 8), center_color, 1)

        mat_suffix = f' {mat_text}' if mat_text else ''
        if depth_m is None:
            main_label = f'{detection.label}{mat_suffix} {detection.score:.2f} | D: NoDepth'
            sub_label = 'Depth missing/blocked at center'
        else:
            main_label = f'{detection.label}{mat_suffix} {detection.score:.2f} | D={depth_m:.3f}m'
            if intrinsics is None:
                sub_label = 'Cam: N/A (no CameraInfo)  Base: N/A'
            else:
                fx, fy, cx, cy = intrinsics
                xc, yc, zc = pixel_to_camera_point(uc, vc, depth_m, fx, fy, cx, cy)
                if extrinsic is None:
                    sub_label = f'Cam:({xc:+.2f}, {yc:+.2f}, {zc:.2f})m  Base: N/A (no extrinsic)'
                elif tcp_pose is None:
                    sub_label = f'Cam:({xc:+.2f}, {yc:+.2f}, {zc:.2f})m  Base: N/A (no TCP)'
                else:
                    r_e_c, t_e_c = extrinsic
                    xb, yb, zb = camera_to_base_point((xc, yc, zc), r_e_c, t_e_c, tcp_pose)
                    sub_label = (
                        f'Cam:({xc:+.2f}, {yc:+.2f}, {zc:.2f})  '
                        f'Base:({xb:+.2f}, {yb:+.2f}, {zb:+.2f})m'
                    )

        # Main label
        ts_main, bl_main = cv2.getTextSize(main_label, cv2.FONT_HERSHEY_SIMPLEX, f_main, 1)
        tx = min(max(bx, 0), max(0, w - ts_main[0] - 6))
        ty = max(by - 6, ts_main[1] + 4)
        cv2.rectangle(
            annotated,
            (tx, ty - ts_main[1] - 4),
            (tx + ts_main[0] + 6, ty + bl_main + 2),
            box_color,
            cv2.FILLED,
        )
        text_color = (255, 255, 255) if is_bad else (0, 0, 0)
        cv2.putText(
            annotated, main_label, (tx + 3, ty),
            cv2.FONT_HERSHEY_SIMPLEX, f_main, text_color, 1, cv2.LINE_AA,
        )

        # Sub label below box
        ts_sub, bl_sub = cv2.getTextSize(sub_label, cv2.FONT_HERSHEY_SIMPLEX, f_sub, 1)
        sub_y = min(by + bh + ts_sub[1] + 6, h - 35)
        sub_x = min(max(bx, 0), max(0, w - ts_sub[0] - 6))
        cv2.rectangle(
            annotated,
            (sub_x, sub_y - ts_sub[1] - 2),
            (sub_x + ts_sub[0] + 4, sub_y + bl_sub + 2),
            (30, 30, 30),
            cv2.FILLED,
        )
        cv2.putText(
            annotated, sub_label, (sub_x + 2, sub_y),
            cv2.FONT_HERSHEY_SIMPLEX, f_sub, (0, 255, 255), 1, cv2.LINE_AA,
        )

    # Mouse inspection crosshair & coordinates
    mouse_text = f'Mouse: ({mouse_u}, {mouse_v}) | Out of bounds'
    if 0 <= mouse_u < w and 0 <= mouse_v < h:
        cv2.circle(annotated, (mouse_u, mouse_v), 5, (255, 255, 0), 1)
        m_depth = get_depth_at(depth_frame, mouse_u, mouse_v, window_radius=2)
        if m_depth is None:
            mouse_text = f'Mouse ({mouse_u}, {mouse_v}): Depth=None (0 mm / invalid)'
        elif intrinsics is None:
            mouse_text = f'Mouse ({mouse_u}, {mouse_v}): Depth={m_depth:.3f}m | Cam=N/A | Base=N/A'
        else:
            fx, fy, cx, cy = intrinsics
            m_xc, m_yc, m_zc = pixel_to_camera_point(mouse_u, mouse_v, m_depth, fx, fy, cx, cy)
            if extrinsic is None:
                mouse_text = (
                    f'Mouse ({mouse_u}, {mouse_v}): Depth={m_depth:.3f}m | '
                    f'Cam=({m_xc:+.3f}, {m_yc:+.3f}, {m_zc:.3f})m | Base=N/A (no extrinsic)'
                )
            elif tcp_pose is None:
                mouse_text = (
                    f'Mouse ({mouse_u}, {mouse_v}): Depth={m_depth:.3f}m | '
                    f'Cam=({m_xc:+.3f}, {m_yc:+.3f}, {m_zc:.3f})m | Base=N/A (no TCP)'
                )
            else:
                r_e_c, t_e_c = extrinsic
                m_xb, m_yb, m_zb = camera_to_base_point((m_xc, m_yc, m_zc), r_e_c, t_e_c, tcp_pose)
                mouse_text = (
                    f'Mouse ({mouse_u}, {mouse_v}): Depth={m_depth:.3f}m | '
                    f'Cam=({m_xc:+.3f}, {m_yc:+.3f}, {m_zc:.3f})m | '
                    f'Base=({m_xb:+.3f}, {m_yb:+.3f}, {m_zb:+.3f})m'
                )

    # Top-left info banner
    if tcp_pose is not None:
        tcp_status = (
            f'TCP: [{tcp_pose[0]:.3f}, {tcp_pose[1]:.3f}, {tcp_pose[2]:.3f}]m '
            f'p={tcp_pose[3]:.1f}°'
        )
    else:
        tcp_status = 'TCP: N/A'
    top_banner = (
        f'Model: {model_name} | {tcp_status} | FPS: {fps:.1f} | Detections: {len(detections)}'
    )
    cv2.rectangle(annotated, (0, 0), (w, 30), (20, 20, 20), cv2.FILLED)
    cv2.putText(
        annotated, top_banner, (10, 21),
        cv2.FONT_HERSHEY_SIMPLEX, f_banner, (0, 255, 120), 1, cv2.LINE_AA,
    )

    # Bottom status bar (mouse probe + hotkey hints)
    cv2.rectangle(annotated, (0, h - 32), (w, h), (15, 15, 15), cv2.FILLED)
    cv2.putText(
        annotated, mouse_text, (10, h - 18),
        cv2.FONT_HERSHEY_SIMPLEX, f_mouse, (255, 255, 255), 1, cv2.LINE_AA,
    )
    hints = '[s]Save [m]Model [q]Quit'
    th, _ = cv2.getTextSize(hints, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
    cv2.putText(
        annotated, hints, (w - th[0] - 10, h - 18),
        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 180), 1, cv2.LINE_AA,
    )

    return annotated


def colorize_depth_image(depth: np.ndarray) -> np.ndarray:
    """Colorize a 2D uint16 mm depth image into a 3-channel uint8 visualization copy.

    Zero depth remains black [0, 0, 0].
    Does not mutate the input array.
    """
    if depth.ndim != 2:
        raise ValueError(f'Expected 2D depth array, got shape {depth.shape}')
    h, w = depth.shape
    colored = np.zeros((h, w, 3), dtype=np.uint8)
    valid_mask = depth > 0
    if not np.any(valid_mask):
        return colored
    valid_depth = depth[valid_mask]
    min_d = float(np.min(valid_depth))
    max_d = float(np.max(valid_depth))
    if max_d == min_d:
        norm = np.zeros((h, w), dtype=np.uint8)
        norm[valid_mask] = 128
    else:
        norm = np.zeros((h, w), dtype=np.uint8)
        scaled = 1.0 + 254.0 * ((depth.astype(np.float32) - min_d) / (max_d - min_d))
        norm[valid_mask] = np.clip(scaled[valid_mask], 1, 255).astype(np.uint8)
    colormap_result = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
    colored[valid_mask] = colormap_result[valid_mask]
    return colored


def draw_depth_view(
    raw_depth: np.ndarray | None,
    fps: float,
    downsample: int = 1,
) -> np.ndarray:
    """Render the native depth visualization with distance legend and FPS."""
    if raw_depth is None:
        ph_w = 848 // downsample if downsample > 1 else 848
        ph_h = 480 // downsample if downsample > 1 else 480
        view = np.zeros((ph_h, ph_w, 3), dtype=np.uint8)
        cv2.putText(
            view,
            f'Waiting for depth frames ({ph_w}x{ph_h})...',
            (20, ph_h // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5 if downsample > 1 else 0.7,
            (0, 255, 255),
            1 if downsample > 1 else 2,
            cv2.LINE_AA,
        )
        return view

    if downsample > 1:
        sampled_depth = raw_depth[::downsample, ::downsample]
    else:
        sampled_depth = raw_depth

    h, w = sampled_depth.shape[:2]
    colored = colorize_depth_image(sampled_depth)
    valid = sampled_depth[sampled_depth > 0]
    if len(valid) == 0:
        cv2.putText(
            colored,
            'No valid depth',
            (max(10, w // 2 - 80), h // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6 if downsample > 1 else 0.8,
            (0, 0, 255),
            1 if downsample > 1 else 2,
            cv2.LINE_AA,
        )
        legend_text = f'Depth ({w}x{h}) | No valid depth pixels'
    else:
        min_m = float(np.min(valid)) * 0.001
        max_m = float(np.max(valid)) * 0.001
        coverage = (len(valid) / (h * w)) * 100.0
        legend_text = (
            f'Depth ({w}x{h}) | {min_m:.2f}~{max_m:.2f}m | '
            f'{coverage:.0f}% | FPS: {fps:.1f}'
        )

    banner_h = 24 if downsample > 1 else 30
    cv2.rectangle(colored, (0, 0), (w, banner_h), (20, 20, 20), cv2.FILLED)
    font_sc = 0.42 if downsample > 1 else 0.52
    text_y = 17 if downsample > 1 else 21
    cv2.putText(
        colored,
        legend_text,
        (8, text_y),
        cv2.FONT_HERSHEY_SIMPLEX,
        font_sc,
        (0, 255, 120),
        1,
        cv2.LINE_AA,
    )
    return colored


class RipenessClassifier:
    """Two-stage crop ripeness classifier using OpenVINO edge models."""

    CROPS = {'apple', 'pear', 'pumpkin', 'tomato', 'onion', 'pepper'}

    def __init__(self, model_root: Path, core: ov.Core, device: str = 'GPU') -> None:
        self.model_root = model_root / 'ripeness_edge'
        self.core = core
        self.device = device
        self._requests: dict[str, tuple[Any, Any, Any]] = {}

    def classify(self, rgb_roi: np.ndarray, label: str) -> tuple[str, float] | None:
        if rgb_roi.shape[0] < 8 or rgb_roi.shape[1] < 8:
            return None
        crop = 'pepper' if label.lower() == 'chili' else label.lower()
        if crop not in self.CROPS:
            return None
        if crop not in self._requests:
            p = self.model_root / crop / 'best.onnx'
            if not p.is_file():
                return None
            try:
                m = self.core.read_model(str(p))
                m.reshape([1, 3, 224, 224])
                compiled = self.core.compile_model(
                    m, self.device, {'PERFORMANCE_HINT': 'LATENCY'},
                )
                self._requests[crop] = (
                    compiled.create_infer_request(),
                    compiled.input(0),
                    compiled.output(0),
                )
            except Exception as err:
                print(
                    f'Warning: failed to compile ripeness model for {crop}: {err}',
                    file=sys.stderr,
                )
                return None

        req, inp, outp = self._requests[crop]
        try:
            resized = cv2.resize(rgb_roi, (224, 224), interpolation=cv2.INTER_LINEAR)
            mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
            std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
            norm = (resized.astype(np.float32) / 255.0 - mean) / std
            blob = np.transpose(norm, (2, 0, 1))[np.newaxis, ...]
            res = req.infer({inp: blob})
            logits = res[outp][0]
            exps = np.exp(logits - np.max(logits))
            probs = exps / np.sum(exps)
            p_ripe = float(probs[0])
            p_unripe = float(probs[1])
            if p_ripe >= p_unripe:
                return 'ripe', p_ripe
            return 'unripe', p_unripe
        except Exception as err:
            print(f'Warning: ripeness inference failed for {crop}: {err}', file=sys.stderr)
            return None


class Detector:
    def __init__(self, model_path: Path) -> None:
        if not model_path.exists():
            raise RuntimeError(f'Model file does not exist: {model_path}')

        self._core = ov.Core()
        devices = self._core.available_devices
        self._device = 'GPU' if 'GPU' in devices else 'CPU'

        model = self._core.read_model(str(model_path))
        if not model.has_rt_info(['framework', 'names']):
            raise RuntimeError('Detector is missing class-name metadata')
        names = ast.literal_eval(model.get_rt_info(['framework', 'names']).value)
        if (
            not isinstance(names, dict)
            or not names
            or any(type(key) is not int for key in names)
            or set(names) != set(range(len(names)))
            or any(not isinstance(name, str) or not name for name in names.values())
        ):
            raise RuntimeError('Invalid detector class-name metadata')
        self.labels = [names[index] for index in range(len(names))]
        input_port = model.input()
        if (
            input_port.get_element_type() != ov.Type.f32
            or list(input_port.get_shape()) != [1, 3, 640, 640]
        ):
            raise RuntimeError(
                f'Expected model input f32 [1,3,640,640], got '
                f'{input_port.get_element_type()} {input_port.get_shape()}',
            )

        preprocessor = PrePostProcessor(model)
        preprocessor.input().tensor().set_element_type(
            ov.Type.u8,
        ).set_layout(ov.Layout('NHWC'))
        preprocessor.input().preprocess().convert_element_type(
            ov.Type.f32,
        ).scale(255.0)
        preprocessor.input().model().set_layout(ov.Layout('NCHW'))
        model = preprocessor.build()

        self._compiled = self._core.compile_model(
            model,
            self._device,
            {'PERFORMANCE_HINT': 'THROUGHPUT'},
        )
        self._input_port = self._compiled.input(0)
        self._output_port = self._compiled.output(0)
        self._request = self._compiled.create_infer_request()

        input_shape = list(self._input_port.get_shape())
        output_shape = list(self._output_port.get_shape())
        if (
            self._input_port.get_element_type() != ov.Type.u8
            or input_shape != [1, 640, 640, 3]
        ):
            raise RuntimeError(
                f'Expected compiled input u8 [1,640,640,3], got '
                f'{self._input_port.get_element_type()} {input_shape}',
            )
        if not (
            output_shape == [1, 300, 6]
            or (
                len(output_shape) == 3
                and output_shape[0] == 1
                and output_shape[1] == 4 + len(self.labels)
                and output_shape[2] > 0
            )
        ):
            raise RuntimeError(f'Unsupported model output shape {output_shape}')

    @property
    def core(self) -> ov.Core:
        return self._core

    @property
    def device(self) -> str:
        return self._device

    def infer(self, frame_rgb: np.ndarray) -> tuple[np.ndarray, LetterboxTransform]:
        input_blob, transform = letterbox_image(frame_rgb)
        outputs = self._request.infer({self._input_port: input_blob})
        output = outputs[self._output_port]
        return np.asarray(output), transform


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


CAMERA_CONTROL_LABELS = {
    'rgb_camera.enable_auto_exposure': 'Auto exposure',
    'rgb_camera.exposure': 'Exposure',
    'rgb_camera.gain': 'Gain',
    'rgb_camera.enable_auto_white_balance': 'Auto white balance',
    'rgb_camera.white_balance': 'White balance',
    'rgb_camera.brightness': 'Brightness',
    'rgb_camera.contrast': 'Contrast',
    'rgb_camera.saturation': 'Saturation',
    'rgb_camera.sharpness': 'Sharpness',
    'rgb_camera.gamma': 'Gamma',
    'rgb_camera.backlight_compensation': 'Backlight compensation',
    'rgb_camera.power_line_frequency': 'Power line frequency',
}

AUTO_CONTROL_FOR = {
    'rgb_camera.exposure': 'rgb_camera.enable_auto_exposure',
    'rgb_camera.white_balance': 'rgb_camera.enable_auto_white_balance',
}


@dataclass(frozen=True)
class CameraControl:
    name: str
    label: str
    minimum: float
    maximum: float
    step: float
    value: bool | int | float
    value_type: type


RGB_CONTROL_LABELS = {
    'rgb_camera.enable_auto_exposure': 'Auto exposure',
    'rgb_camera.exposure': 'Exposure',
    'rgb_camera.gain': 'Gain',
    'rgb_camera.enable_auto_white_balance': 'Auto white balance',
    'rgb_camera.white_balance': 'White balance',
    'rgb_camera.brightness': 'Brightness',
    'rgb_camera.contrast': 'Contrast',
    'rgb_camera.saturation': 'Saturation',
    'rgb_camera.sharpness': 'Sharpness',
    'rgb_camera.gamma': 'Gamma',
    'rgb_camera.backlight_compensation': 'Backlight compensation',
    'rgb_camera.power_line_frequency': 'Power line frequency',
}

DEPTH_CONTROL_LABELS = {
    'depth_module.exposure': 'Depth Exposure (us)',
    'depth_module.gain': 'Depth Gain',
}

ALL_CAMERA_CONTROL_LABELS = {**RGB_CONTROL_LABELS, **DEPTH_CONTROL_LABELS}


class CameraControlPanel:
    def __init__(
        self,
        camera_name: str,
        controls: list[CameraControl],
        set_control: Callable[[str, bool | int | float], None],
        save_controls: Callable[[dict[str, bool | int | float]], Path],
    ) -> None:
        if not controls:
            raise RuntimeError(f'No adjustable controls found for {camera_name}')

        self._controls = {control.name: control for control in controls}
        self._values = {control.name: control.value for control in controls}
        self._pending: dict[str, bool | int | float] = {}
        self._set_control = set_control
        self._save_controls = save_controls
        self._save_requested = False
        self._open = True
        self._variables: dict[str, tk.Variable] = {}

        self._root = tk.Tk()
        self._root.title('RealSense Camera Controls')
        self._root.resizable(False, True)
        self._root.protocol('WM_DELETE_WINDOW', self.close)

        main_frame = ttk.Frame(self._root, padding=10)
        main_frame.grid(sticky='nsew')
        main_frame.columnconfigure(0, weight=1)

        # Depth controls group
        depth_controls = [c for c in controls if c.name in DEPTH_CONTROL_LABELS]
        if depth_controls:
            depth_group = ttk.LabelFrame(main_frame, text='Depth Controls', padding=10)
            depth_group.grid(row=0, column=0, sticky='ew', pady=(0, 8))
            depth_group.columnconfigure(1, weight=1)
            for row, control in enumerate(depth_controls):
                ttk.Label(depth_group, text=control.label).grid(
                    row=row,
                    column=0,
                    padx=(0, 12),
                    pady=3,
                    sticky='w',
                )
                variable = self._create_variable(control)
                self._variables[control.name] = variable
                widget = self._create_widget(depth_group, control, variable)
                widget.grid(row=row, column=1, pady=3, sticky='ew')

        # RGB controls group
        rgb_controls = [c for c in controls if c.name in RGB_CONTROL_LABELS]
        if rgb_controls:
            rgb_group = ttk.LabelFrame(main_frame, text='RGB Controls', padding=10)
            rgb_group.grid(row=1, column=0, sticky='ew', pady=(0, 8))
            rgb_group.columnconfigure(1, weight=1)
            for row, control in enumerate(rgb_controls):
                ttk.Label(rgb_group, text=control.label).grid(
                    row=row,
                    column=0,
                    padx=(0, 12),
                    pady=3,
                    sticky='w',
                )
                variable = self._create_variable(control)
                self._variables[control.name] = variable
                widget = self._create_widget(rgb_group, control, variable)
                widget.grid(row=row, column=1, pady=3, sticky='ew')

        # Status & Save
        self._status = tk.StringVar(value='Ready')
        ttk.Separator(main_frame).grid(row=2, column=0, pady=(4, 6), sticky='ew')
        ttk.Label(main_frame, textvariable=self._status).grid(
            row=3,
            column=0,
            sticky='w',
        )
        ttk.Button(main_frame, text='Save', command=self._request_save).grid(
            row=4,
            column=0,
            pady=(8, 0),
            sticky='ew',
        )

        self._root.update_idletasks()
        x = max(0, self._root.winfo_screenwidth() - self._root.winfo_reqwidth() - 20)
        self._root.geometry(f'+{x}+40')

    def _create_widget(
        self,
        parent: ttk.Frame | ttk.LabelFrame,
        control: CameraControl,
        variable: tk.Variable,
    ) -> tk.Widget:
        if control.value_type is bool:
            return ttk.Checkbutton(
                parent,
                variable=variable,
                command=lambda: self._queue(control.name),
            )
        if control.name.endswith('power_line_frequency'):
            values = range(int(control.minimum), int(control.maximum) + 1)
            widget = ttk.Combobox(
                parent,
                textvariable=variable,
                values=list(values),
                state='readonly',
                width=12,
            )
            widget.bind(
                '<<ComboboxSelected>>',
                lambda _event: self._queue(control.name),
            )
            return widget
        return tk.Scale(
            parent,
            variable=variable,
            from_=control.minimum,
            to=control.maximum,
            resolution=control.step,
            orient=tk.HORIZONTAL,
            length=330,
            command=lambda _value: self._queue(control.name),
        )

    @staticmethod
    def _create_variable(control: CameraControl) -> tk.Variable:
        if control.value_type is bool:
            return tk.BooleanVar(value=control.value)
        if control.value_type is int:
            return tk.IntVar(value=control.value)
        return tk.DoubleVar(value=control.value)

    def _queue(self, name: str) -> None:
        if name in DEPTH_CONTROL_LABELS:
            # Depth manual controls require depth auto exposure to be disabled
            try:
                self._set_control('depth_module.enable_auto_exposure', False)
            except Exception as error:  # noqa: BLE001
                message = f'Failed to disable depth auto exposure: {error}'
                self._status.set(message)
                print(message, file=sys.stderr, flush=True)
                return
        else:
            auto_name = AUTO_CONTROL_FOR.get(name)
            if auto_name in self._variables and self._variables[auto_name].get():
                self._variables[auto_name].set(False)
                self._pending[auto_name] = False
        value = self._variables[name].get()
        self._pending[name] = self._controls[name].value_type(value)

    def _request_save(self) -> None:
        self._save_requested = True

    def apply_pending(self) -> None:
        if self._open:
            try:
                self._root.update_idletasks()
                self._root.update()
            except tk.TclError:
                self._open = False

        pending = self._pending
        self._pending = {}
        for name, value in pending.items():
            if value == self._values[name]:
                continue
            try:
                self._set_control(name, value)
            except Exception as error:  # noqa: BLE001
                message = f'Failed: {self._controls[name].label}: {error}'
                self._status.set(message)
                self._variables[name].set(self._values[name])
                print(message, file=sys.stderr, flush=True)
            else:
                self._values[name] = value
                self._status.set(f'Applied: {self._controls[name].label}')
                print(
                    f'{self._controls[name].label} = '
                    f'{self._format_value(value)}',
                    flush=True,
                )
        if self._save_requested:
            self._save()

    def _save(self) -> None:
        self._save_requested = False
        try:
            backup_path = self._save_controls(dict(self._values))
        except Exception as error:  # noqa: BLE001
            message = f'Save failed: {error}'
            self._status.set(message)
            print(message, file=sys.stderr, flush=True)
            return
        self._status.set(f'Saved; backup: {backup_path.name}')
        print(f'Saved camera controls; backup: {backup_path}', flush=True)

    @staticmethod
    def _format_value(value: bool | int | float) -> str:
        if isinstance(value, bool):
            return 'On' if value else 'Off'
        if isinstance(value, float):
            return f'{value:.4g}'
        return str(value)

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


class CameraControlService:
    NODE_NAME = '/camera'

    def __init__(self, node: Node | None = None) -> None:
        self._node = Node('camera_control_client')
        self._get_client = self._node.create_client(
            GetParameters,
            f'{self.NODE_NAME}/get_parameters',
        )
        self._describe_client = self._node.create_client(
            DescribeParameters,
            f'{self.NODE_NAME}/describe_parameters',
        )
        self._set_client = self._node.create_client(
            SetParameters,
            f'{self.NODE_NAME}/set_parameters',
        )
        self._descriptors: dict[str, Any] = {}
        self._unsupported: set[str] = set()

    def destroy(self) -> None:
        try:
            self._node.destroy_node()
        except Exception:
            pass

    def ensure_descriptors(self, names: list[str], timeout_sec: float = 10.0) -> None:
        missing = [n for n in names if n not in self._descriptors and n not in self._unsupported]
        if not missing:
            return
        for client in (self._get_client, self._describe_client, self._set_client):
            if not client.wait_for_service(timeout_sec=timeout_sec):
                raise RuntimeError(
                    f'Parameter service is unavailable for {self.NODE_NAME}',
                )
        describe_response = self._call(
            self._describe_client,
            DescribeParameters.Request(names=missing),
        )
        for name, desc in zip(missing, describe_response.descriptors):
            if desc.type != ParameterType.PARAMETER_NOT_SET:
                self._descriptors[name] = desc
            else:
                self._unsupported.add(name)

    def has_descriptor(self, name: str) -> bool:
        return name in self._descriptors

    def get_parameter_values(self, names: list[str]) -> dict[str, Any]:
        self.ensure_descriptors(names)
        get_response = self._call(
            self._get_client,
            GetParameters.Request(names=names),
        )
        results: dict[str, Any] = {}
        for name, val in zip(names, get_response.values):
            if val.type == ParameterType.PARAMETER_BOOL:
                results[name] = val.bool_value
            elif val.type == ParameterType.PARAMETER_INTEGER:
                results[name] = val.integer_value
            elif val.type == ParameterType.PARAMETER_DOUBLE:
                results[name] = val.double_value
            elif val.type == ParameterType.PARAMETER_STRING:
                results[name] = val.string_value
            elif val.type != ParameterType.PARAMETER_NOT_SET:
                results[name] = val
        return results

    def discover(self) -> list[CameraControl]:
        names = list(ALL_CAMERA_CONTROL_LABELS)
        self.ensure_descriptors(names)

        get_response = self._call(
            self._get_client,
            GetParameters.Request(names=names),
        )

        controls: list[CameraControl] = []
        discovered_names: set[str] = set()
        for name, value in zip(names, get_response.values):
            descriptor = self._descriptors.get(name)
            if descriptor is not None:
                control = self._control_from_parameter(name, value, descriptor)
                if control is not None:
                    controls.append(control)
                    discovered_names.add(name)

        # Enforce required depth controls
        required_depth = {'depth_module.exposure', 'depth_module.gain'}
        missing = required_depth - discovered_names
        if missing:
            raise RuntimeError(
                f'Required depth controls missing from {self.NODE_NAME}: {missing}',
            )

        return controls

    def set(self, name: str, value: bool | int | float | str) -> None:
        self.ensure_descriptors([name])
        desc = self._descriptors.get(name)
        param_value = value
        if desc is not None:
            if desc.type == ParameterType.PARAMETER_INTEGER:
                param_value = int(round(float(value)))
            elif desc.type == ParameterType.PARAMETER_DOUBLE:
                param_value = float(value)
            elif desc.type == ParameterType.PARAMETER_BOOL:
                param_value = bool(value)
            elif desc.type == ParameterType.PARAMETER_STRING:
                param_value = str(value)
        elif isinstance(value, float) and value.is_integer() and not isinstance(value, bool):
            param_value = int(value)

        request = SetParameters.Request(
            parameters=[Parameter(name, value=param_value).to_parameter_msg()],
        )
        response = self._call(self._set_client, request)
        result = response.results[0]
        if not result.successful:
            raise RuntimeError(result.reason or 'parameter was rejected')

    def _call(self, client: Any, request: Any) -> Any:
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
        if not future.done():
            raise RuntimeError(
                f'ROS parameter request timed out on {getattr(client, "srv_name", "unknown")}: {request}',
            )
        if future.exception() is not None:
            raise RuntimeError(str(future.exception()))
        return future.result()

    @staticmethod
    def _control_from_parameter(
        name: str,
        value: Any,
        descriptor: Any,
    ) -> CameraControl | None:
        label = ALL_CAMERA_CONTROL_LABELS.get(name, name)
        if value.type == ParameterType.PARAMETER_BOOL:
            return CameraControl(
                name,
                label,
                0.0,
                1.0,
                1.0,
                value.bool_value,
                bool,
            )
        if (
            value.type == ParameterType.PARAMETER_INTEGER
            and descriptor.integer_range
        ):
            value_range = descriptor.integer_range[0]
            return CameraControl(
                name,
                label,
                float(value_range.from_value),
                float(value_range.to_value),
                float(max(1, value_range.step)),
                value.integer_value,
                int,
            )
        if (
            value.type == ParameterType.PARAMETER_DOUBLE
            and descriptor.floating_point_range
        ):
            value_range = descriptor.floating_point_range[0]
            if value_range.step <= 0.0:
                return None
            return CameraControl(
                name,
                label,
                value_range.from_value,
                value_range.to_value,
                value_range.step,
                value.double_value,
                float,
            )
        return None


def save_camera_controls(
    camera_name: str,
    values: dict[str, bool | int | float],
    config_path: Path,
    tmp_dir: Path,
) -> Path:
    tmp_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    backup_path = tmp_dir / f'fruit_perception_{timestamp}.yaml.bak'
    shutil.copy2(config_path, backup_path)

    with config_path.open('r', encoding='utf-8') as f:
        doc = yaml.safe_load(f)
    if 'camera' not in doc or 'ros__parameters' not in doc['camera']:
        raise KeyError('Invalid config: missing camera.ros__parameters')
    doc['camera']['ros__parameters'].update(values)

    temp_path = config_path.with_suffix(f'.tmp_{timestamp}')
    with temp_path.open('w', encoding='utf-8') as f:
        yaml.safe_dump(doc, f, sort_keys=False)
    temp_path.replace(config_path)

    return backup_path


def apply_camera_yaml_configuration(
    service: CameraControlService,
    config: dict[str, Any],
    subscriber: FrameSubscriber,
    timeout_sec: float = 10.0,
) -> None:
    """Apply YAML configuration to /camera and verify stream resolution and parameters."""
    params = config['camera']['ros__parameters']

    all_target_names = [
        'enable_depth',
        'enable_color',
        'depth_module.depth_profile',
        'rgb_camera.color_profile',
        'depth_module.visual_preset',
        'depth_module.enable_auto_exposure',
        'depth_module.exposure',
        'depth_module.gain',
        'depth_module.emitter_enabled',
        'depth_module.laser_power',
        'rgb_camera.enable_auto_exposure',
        'rgb_camera.exposure',
        'rgb_camera.gain',
        'rgb_camera.enable_auto_white_balance',
        'rgb_camera.white_balance',
        'rgb_camera.brightness',
        'rgb_camera.contrast',
        'rgb_camera.saturation',
        'rgb_camera.sharpness',
        'rgb_camera.gamma',
        'rgb_camera.backlight_compensation',
        'rgb_camera.power_line_frequency',
        'enable_sync',
        'align_depth.enable',
        'enable_rgbd',
        'spatial_filter.enable',
        'temporal_filter.enable',
    ]
    service.ensure_descriptors(all_target_names)

    # 1. Video stream profiles
    current_values = service.get_parameter_values([
        'depth_module.depth_profile',
        'rgb_camera.color_profile',
    ])
    target_depth_profile = str(params.get('depth_module.depth_profile', '848x480x30'))
    target_color_profile = str(params.get('rgb_camera.color_profile', '1280x720x30'))

    cur_depth_prof = current_values.get('depth_module.depth_profile')
    if cur_depth_prof is not None and cur_depth_prof != target_depth_profile:
        print(
            f'Applying depth profile: {cur_depth_prof} -> {target_depth_profile}',
            flush=True,
        )
        depth_was_enabled = False
        if service.has_descriptor('enable_depth'):
            cur_enable = service.get_parameter_values(['enable_depth']).get('enable_depth')
            if cur_enable is True or cur_enable is None:
                service.set('enable_depth', False)
                depth_was_enabled = True
        try:
            service.set('depth_module.depth_profile', target_depth_profile)
        finally:
            if depth_was_enabled and service.has_descriptor('enable_depth'):
                try:
                    service.set('enable_depth', True)
                except Exception as restore_err:
                    print(
                        f'Warning: failed to restore enable_depth=True: {restore_err}',
                        file=sys.stderr,
                        flush=True,
                    )

    cur_color_prof = current_values.get('rgb_camera.color_profile')
    if cur_color_prof is not None and cur_color_prof != target_color_profile:
        print(
            f'Applying color profile: {cur_color_prof} -> {target_color_profile}',
            flush=True,
        )
        color_was_enabled = False
        if service.has_descriptor('enable_color'):
            cur_enable = service.get_parameter_values(['enable_color']).get('enable_color')
            if cur_enable is True or cur_enable is None:
                service.set('enable_color', False)
                color_was_enabled = True
        try:
            service.set('rgb_camera.color_profile', target_color_profile)
        finally:
            if color_was_enabled and service.has_descriptor('enable_color'):
                try:
                    service.set('enable_color', True)
                except Exception as restore_err:
                    print(
                        f'Warning: failed to restore enable_color=True: {restore_err}',
                        file=sys.stderr,
                        flush=True,
                    )

    def set_if_diff(name: str, target: Any, required: bool = False) -> None:
        if not service.has_descriptor(name):
            if required:
                raise RuntimeError(
                    f"Required parameter '{name}' is not supported by {service.NODE_NAME}",
                )
            return
        cur = service.get_parameter_values([name]).get(name)
        if cur == target:
            return
        if isinstance(target, float) and cur is not None and abs(float(cur) - float(target)) < 1e-3:
            return
        print(f'Setting camera parameter {name}: {cur} -> {target}', flush=True)
        service.set(name, target)

    # 2. Sync, aligned depth, rgbd, and filters
    for key in (
        'enable_sync',
        'align_depth.enable',
        'enable_rgbd',
    ):
        if key in params:
            set_if_diff(key, bool(params[key]), required=False)

    for filter_key in (
        'spatial_filter.enable',
        'temporal_filter.enable',
    ):
        if filter_key in params:
            set_if_diff(filter_key, bool(params[filter_key]), required=True)

    # 3. Depth settings: strict order (preset -> auto-exposure off -> manual controls)
    set_if_diff(
        'depth_module.visual_preset',
        int(params.get('depth_module.visual_preset', 3)),
        required=True,
    )
    set_if_diff(
        'depth_module.enable_auto_exposure',
        bool(params.get('depth_module.enable_auto_exposure', False)),
        required=True,
    )
    if 'depth_module.exposure' in params:
        set_if_diff('depth_module.exposure', int(params['depth_module.exposure']), required=True)
    if 'depth_module.gain' in params:
        set_if_diff('depth_module.gain', int(params['depth_module.gain']), required=True)
    if 'depth_module.emitter_enabled' in params:
        set_if_diff(
            'depth_module.emitter_enabled',
            int(params['depth_module.emitter_enabled']),
            required=True,
        )
    if 'depth_module.laser_power' in params:
        set_if_diff(
            'depth_module.laser_power',
            float(params['depth_module.laser_power']),
            required=True,
        )

    # 4. RGB settings
    set_if_diff(
        'rgb_camera.enable_auto_exposure',
        bool(params.get('rgb_camera.enable_auto_exposure', False)),
    )
    if 'rgb_camera.exposure' in params:
        set_if_diff('rgb_camera.exposure', int(params['rgb_camera.exposure']))
    if 'rgb_camera.gain' in params:
        set_if_diff('rgb_camera.gain', int(params['rgb_camera.gain']))
    if 'rgb_camera.enable_auto_white_balance' in params:
        set_if_diff(
            'rgb_camera.enable_auto_white_balance',
            bool(params['rgb_camera.enable_auto_white_balance']),
        )
    if 'rgb_camera.white_balance' in params:
        set_if_diff('rgb_camera.white_balance', float(params['rgb_camera.white_balance']))

    for rgb_key in (
        'rgb_camera.brightness',
        'rgb_camera.contrast',
        'rgb_camera.saturation',
        'rgb_camera.sharpness',
        'rgb_camera.gamma',
        'rgb_camera.backlight_compensation',
        'rgb_camera.power_line_frequency',
    ):
        if rgb_key in params and service.has_descriptor(rgb_key):
            set_if_diff(rgb_key, params[rgb_key])

    # 5. Readback verification
    verify_names = [
        'depth_module.visual_preset',
        'depth_module.enable_auto_exposure',
        'depth_module.exposure',
        'depth_module.gain',
        'depth_module.emitter_enabled',
        'depth_module.laser_power',
        'spatial_filter.enable',
        'temporal_filter.enable',
    ]
    readback = service.get_parameter_values(verify_names)
    expected_preset = int(params.get('depth_module.visual_preset', 3))
    if readback.get('depth_module.visual_preset') != expected_preset:
        raise RuntimeError(
            f'Visual preset verification failed: expected {expected_preset}, '
            f'got {readback.get("depth_module.visual_preset")}',
        )
    if readback.get('depth_module.enable_auto_exposure') is not False:
        raise RuntimeError(
            f'Depth auto-exposure verification failed: expected False, '
            f'got {readback.get("depth_module.enable_auto_exposure")}',
        )
    if 'depth_module.exposure' in params:
        expected_exp = int(params['depth_module.exposure'])
        if readback.get('depth_module.exposure') != expected_exp:
            raise RuntimeError(
                f'Depth exposure verification failed: expected {expected_exp}, '
                f'got {readback.get("depth_module.exposure")}',
            )
    if 'depth_module.gain' in params:
        expected_gain = int(params['depth_module.gain'])
        if readback.get('depth_module.gain') != expected_gain:
            raise RuntimeError(
                f'Depth gain verification failed: expected {expected_gain}, '
                f'got {readback.get("depth_module.gain")}',
            )
    if 'depth_module.emitter_enabled' in params:
        expected_emitter = int(params['depth_module.emitter_enabled'])
        if readback.get('depth_module.emitter_enabled') != expected_emitter:
            raise RuntimeError(
                f'Depth emitter verification failed: expected {expected_emitter}, '
                f'got {readback.get("depth_module.emitter_enabled")}',
            )
    if 'depth_module.laser_power' in params:
        expected_laser = float(params['depth_module.laser_power'])
        actual_laser = readback.get('depth_module.laser_power')
        if actual_laser is None or abs(float(actual_laser) - expected_laser) > 1e-2:
            raise RuntimeError(
                f'Depth laser power verification failed: expected {expected_laser}, '
                f'got {actual_laser}',
            )
    if 'spatial_filter.enable' in params:
        expected_spatial = bool(params['spatial_filter.enable'])
        if readback.get('spatial_filter.enable') != expected_spatial:
            raise RuntimeError(
                f'Spatial filter verification failed: expected {expected_spatial}, '
                f'got {readback.get("spatial_filter.enable")}',
            )
    if 'temporal_filter.enable' in params:
        expected_temporal = bool(params['temporal_filter.enable'])
        if readback.get('temporal_filter.enable') != expected_temporal:
            raise RuntimeError(
                f'Temporal filter verification failed: expected {expected_temporal}, '
                f'got {readback.get("temporal_filter.enable")}',
            )
    print(f'Verified depth parameters on /camera: {readback}', flush=True)

    # 6. Wait for confirmed frame dimensions
    deadline = time.monotonic() + timeout_sec
    confirmed_color = False
    confirmed_depth = False
    while time.monotonic() < deadline:
        rclpy.spin_once(subscriber, timeout_sec=0.1)
        c, d, raw_d = subscriber.take_latest()
        if c is not None and c.shape[:2] == (720, 1280):
            confirmed_color = True
        if raw_d is not None and raw_d.shape[:2] == (480, 848):
            confirmed_depth = True
        if confirmed_color and confirmed_depth:
            print(
                'Camera stream dimensions confirmed: color 1280x720, native depth 848x480',
                flush=True,
            )
            break
    else:
        raise RuntimeError(
            'Timeout waiting for confirmed frame dimensions '
            f'(color confirmed: {confirmed_color}, native depth confirmed: {confirmed_depth})',
        )


def is_window_open(name: str) -> bool:
    try:
        val = cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE)
        return val >= 1.0
    except Exception:
        return False


def run() -> int:
    args = parse_args()
    config = load_config()
    selection = select_camera(config)

    current_model = args.model.upper()
    model_path = model_path_for(current_model, config)

    perception = config['fruit_perception_node']['ros__parameters']
    confidence_threshold = float(
        perception['confidence_threshold'] if args.conf is None else args.conf,
    )
    nms_iou_threshold = float(
        perception['nms_iou_threshold'] if args.iou is None else args.iou,
    )
    for name, value in (
        ('confidence threshold', confidence_threshold),
        ('NMS IoU threshold', nms_iou_threshold),
    ):
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            raise ValueError(f'{name} must be between 0 and 1, got {value}')

    if args.scale <= 0.0 or args.scale > 2.0:
        raise ValueError(f'scale must be between 0.0 and 2.0, got {args.scale}')
    if args.gui_fps <= 0.0:
        raise ValueError(f'gui_fps must be positive, got {args.gui_fps}')

    detector = Detector(model_path)
    extrinsic = load_extrinsic_transform()
    model_root = Path(perception['model_root'])
    ripeness_classifier = RipenessClassifier(model_root, detector.core, detector.device)

    rclpy.init()

    # Pre-flight check: reject running if /fruit_mission is active
    temp_node = Node('test_camera_preflight_check')
    try:
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            rclpy.spin_once(temp_node, timeout_sec=0.1)
        node_names = set(temp_node.get_node_names())
        if '/fruit_mission' in node_names or any('fruit_mission' in name for name in node_names):
            print(
                'Error: /fruit_mission is currently active. '
                'Camera tuning is prohibited during an active mission.',
                file=sys.stderr,
                flush=True,
            )
            return 1
    finally:
        temp_node.destroy_node()

    joint_state_topic = '/arm_planner_node/joint_states'
    arm_config_path = WORKSPACE / 'src/fruit_arm_planner/config/arm_presets.yaml'
    try:
        with EXTRINSIC_CONFIG_PATH.open(encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        if isinstance(cfg, dict):
            joint_state_topic = str(cfg.get('joint_state_topic', joint_state_topic))
            if 'arm_config_path' in cfg:
                arm_config_path = Path(str(cfg['arm_config_path']))
    except Exception:  # noqa: BLE001
        pass

    node = FrameSubscriber(
        selection.color_topic,
        selection.depth_topic,
        selection.raw_depth_topic,
        selection.info_topic,
        joint_state_topic,
        arm_config_path,
    )

    owned_arm_debug: subprocess.Popen | None = None
    owned_bridge: subprocess.Popen | None = None
    owned_planner: subprocess.Popen | None = None
    owned_robviz_bridge: subprocess.Popen | None = None
    camera_process: subprocess.Popen | None = None
    log_path = TMP_DIR / f'test_camera_detector_{selection.name}.log'
    control_panel = None
    control_service = None

    try:
        owned_arm_debug, owned_bridge, owned_planner = ensure_arm_communication(node)
        owned_robviz_bridge = ensure_robviz_bridge(node)

        if wait_for_publisher(node, selection.color_topic, 1.0):
            print(
                f'Using existing camera stream on {selection.color_topic}; '
                f'model={model_path.name}; conf={confidence_threshold:.2f}',
                flush=True,
            )
        else:
            camera_process, log_path = start_camera(selection)
            print(
                f'Started RealSense camera; color={selection.color_topic}; '
                f'depth={selection.depth_topic}; raw_depth={selection.raw_depth_topic}; log={log_path}',
                flush=True,
            )
            print('Waiting for camera node to initialize publishers...', flush=True)
            if not wait_for_publisher(node, selection.color_topic, 10.0):
                check_camera_process(camera_process, log_path)
                raise RuntimeError(
                    f'Camera process failed to publish on {selection.color_topic} within 10s',
                )

        # Apply YAML configuration to /camera (handles both newly started and reused camera)
        control_service = CameraControlService(node)
        apply_camera_yaml_configuration(control_service, config, node)

        show_rgb = args.display in ('rgb', 'both')
        show_depth = args.display in ('depth', 'both')
        scale = float(args.scale)
        gui_fps = float(args.gui_fps)
        min_gui_interval = 1.0 / gui_fps if gui_fps > 0 else 0.0
        depth_downsample = max(1, int(round(1.0 / scale))) if scale < 1.0 else 1

        window_rgb_name = 'RealSense 3D Detector & Depth Inspector'
        window_depth_name = 'RealSense Native Depth'

        active_windows: list[str] = []
        rgb_w = max(160, int(round(1280 * scale)))
        rgb_h = max(90, int(round(720 * scale)))
        depth_w = max(160, int(round(848 * scale)))
        depth_h = max(90, int(round(480 * scale)))

        if show_rgb:
            active_windows.append(window_rgb_name)
            cv2.namedWindow(window_rgb_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_rgb_name, rgb_w, rgb_h)
            cv2.moveWindow(window_rgb_name, 20, 40)

        if show_depth:
            active_windows.append(window_depth_name)
            cv2.namedWindow(window_depth_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_depth_name, depth_w, depth_h)
            if show_rgb:
                cv2.moveWindow(window_depth_name, 20 + rgb_w + 10, 40)
            else:
                cv2.moveWindow(window_depth_name, 20, 40)

        mouse_coords = [-1, -1]

        def on_mouse(event: int, x: int, y: int, _flags: int, _param: Any) -> None:
            if scale > 0.0 and scale != 1.0:
                mouse_coords[0] = int(round(x / scale))
                mouse_coords[1] = int(round(y / scale))
            else:
                mouse_coords[0] = x
                mouse_coords[1] = y

        if show_rgb:
            cv2.setMouseCallback(window_rgb_name, on_mouse)

        # Show initial placeholder frames on active windows
        if show_rgb:
            init_rgb = np.zeros((720, 1280, 3), dtype=np.uint8)
            cv2.putText(
                init_rgb,
                'Waiting for camera frames...',
                (40, 360),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            disp_init = (
                cv2.resize(init_rgb, (rgb_w, rgb_h), interpolation=cv2.INTER_AREA)
                if scale != 1.0
                else init_rgb
            )
            cv2.imshow(window_rgb_name, disp_init)

        if show_depth:
            init_depth = draw_depth_view(None, 0.0, downsample=depth_downsample)
            if init_depth.shape[:2] != (depth_h, depth_w):
                init_depth = cv2.resize(
                    init_depth,
                    (depth_w, depth_h),
                    interpolation=cv2.INTER_AREA,
                )
            cv2.imshow(window_depth_name, init_depth)

        if active_windows:
            cv2.waitKey(1)

        if args.controls:
            control_panel = CameraControlPanel(
                selection.name,
                control_service.discover(),
                control_service.set,
                lambda values: save_camera_controls(
                    selection.name, values, CONFIG_PATH, TMP_DIR,
                ),
            )

        last_render_time = 0.0
        last_frame_time = 0.0
        displayed_fps = 0.0
        last_wait_message = 0.0
        last_headless_log = 0.0
        announced_first_frame = False
        current_raw_depth: np.ndarray | None = None
        latest_annotated_bgr: np.ndarray | None = None
        latest_depth_view: np.ndarray | None = None

        print('\n=== RealSense 3D Inspector Controls ===')
        if active_windows:
            print(
                f'  Mode: --display {args.display} | Scale: {scale:.2f} ({rgb_w}x{rgb_h}) | '
                f'Max GUI FPS: {gui_fps:.0f}'
            )
            print('  [Hover Mouse] : Probe depth and 3D camera / arm base coordinates')
            print('  [Key s]       : Save current annotated snapshot image to disk')
            print('  [Key m]       : Toggle model between A (vegetables) and B (fruits)')
            print('  [Key q / ESC] : Quit viewer\n')
        else:
            print(
                f'  Mode: Headless (--display false) | '
                f'RobViz streaming: {node.annotated_topic}'
            )
            print('  [Ctrl-C]      : Stop detector\n')

        window_opened = {wname: False for wname in active_windows}

        while rclpy.ok():
            check_camera_process(camera_process, log_path)
            if control_panel is not None:
                control_panel.apply_pending()
            rclpy.spin_once(node, timeout_sec=0.01)

            # Check if any OpenCV window was mapped and then closed by user
            window_closed = False
            for wname in active_windows:
                if is_window_open(wname):
                    window_opened[wname] = True
                elif window_opened[wname]:
                    print(f'{wname} closed by user; exiting.', flush=True)
                    window_closed = True
                    break
            if window_closed:
                break

            if control_panel is not None and not control_panel.is_open():
                print('Camera controls panel closed by user; exiting.', flush=True)
                break

            now = time.monotonic()
            if min_gui_interval > 0.0 and (now - last_render_time) < min_gui_interval:
                if active_windows:
                    key = cv2.waitKey(1)
                    if key in (27, ord('q'), ord('Q')):
                        break
                    elif key in (ord('s'), ord('S')):
                        target_img = (
                            latest_annotated_bgr
                            if latest_annotated_bgr is not None
                            else latest_depth_view
                        )
                        if target_img is not None:
                            timestamp = time.strftime('%Y%m%d_%H%M%S')
                            save_path = WORKSPACE / f'snapshot_{timestamp}.jpg'
                            cv2.imwrite(str(save_path), target_img)
                            print(f'>>> Saved snapshot to {save_path}', flush=True)
                    elif key in (ord('m'), ord('M')):
                        current_model = 'B' if current_model in ('A', 'C') else 'A'
                        model_path = model_path_for(current_model, config)
                        detector = Detector(model_path)
                        print(
                            f'>>> Switched to Model {current_model}: {model_path.name}',
                            flush=True,
                        )
                else:
                    time.sleep(0.005)
                continue

            frame, depth_frame, raw_depth = node.take_latest()
            if raw_depth is not None:
                current_raw_depth = raw_depth

            if frame is None:
                now = time.monotonic()
                if not announced_first_frame and now - last_wait_message > 2.0:
                    print(f'Waiting for frames on {selection.color_topic}...', flush=True)
                    last_wait_message = now
                if active_windows:
                    key = cv2.waitKey(1)
                    if key in (27, ord('q'), ord('Q')):
                        break
                else:
                    time.sleep(0.01)
                continue

            last_render_time = now

            if not announced_first_frame:
                print(
                    f'Received first frame: {frame.shape[1]}x{frame.shape[0]} '
                    f'(aligned depth: {depth_frame is not None}, '
                    f'native depth: {current_raw_depth is not None})',
                    flush=True,
                )
                announced_first_frame = True

            output, transform = detector.infer(frame)
            detections = parse_detections(
                output,
                confidence_threshold,
                nms_iou_threshold,
                detector.labels,
            )

            now_frame = time.monotonic()
            if last_frame_time > 0.0:
                interval = now_frame - last_frame_time
                instantaneous_fps = 1.0 / interval if interval > 0.0 else 0.0
                displayed_fps = (
                    instantaneous_fps
                    if displayed_fps == 0.0
                    else 0.8 * displayed_fps + 0.2 * instantaneous_fps
                )
            last_frame_time = now_frame

            h_img, w_img = frame.shape[:2]
            if depth_frame is not None and depth_frame.shape[:2] != (h_img, w_img):
                depth_frame = None

            intrinsics = validate_camera_info(node.camera_info, w_img, h_img)
            tcp_pose = node.get_fresh_tcp_pose()

            annotated_rgb = draw_annotations(
                frame,
                depth_frame,
                detections,
                transform,
                intrinsics,
                extrinsic,
                tcp_pose,
                mouse_coords[0],
                mouse_coords[1],
                displayed_fps,
                f'{current_model} ({"fruit" if current_model in ("B", "D") else "vegetable"})',
                ripeness_classifier=ripeness_classifier,
                scale=scale,
            )

            node.publish_annotated(annotated_rgb, header=node.latest_header)

            if show_rgb:
                latest_annotated_bgr = cv2.cvtColor(annotated_rgb, cv2.COLOR_RGB2BGR)
                if scale != 1.0:
                    disp_bgr = cv2.resize(
                        latest_annotated_bgr,
                        (rgb_w, rgb_h),
                        interpolation=cv2.INTER_AREA,
                    )
                else:
                    disp_bgr = latest_annotated_bgr
                cv2.imshow(window_rgb_name, disp_bgr)

            # Native depth preview window
            if show_depth:
                latest_depth_view = draw_depth_view(
                    current_raw_depth,
                    displayed_fps,
                    downsample=depth_downsample,
                )
                if latest_depth_view.shape[:2] != (depth_h, depth_w):
                    depth_disp = cv2.resize(
                        latest_depth_view,
                        (depth_w, depth_h),
                        interpolation=cv2.INTER_AREA,
                    )
                else:
                    depth_disp = latest_depth_view
                cv2.imshow(window_depth_name, depth_disp)

            if active_windows:
                key = cv2.waitKey(1)
                if key in (27, ord('q'), ord('Q')):
                    break
                elif key in (ord('s'), ord('S')):
                    target_img = (
                        latest_annotated_bgr
                        if latest_annotated_bgr is not None
                        else latest_depth_view
                    )
                    if target_img is not None:
                        timestamp = time.strftime('%Y%m%d_%H%M%S')
                        save_path = WORKSPACE / f'snapshot_{timestamp}.jpg'
                        cv2.imwrite(str(save_path), target_img)
                        print(f'>>> Saved snapshot to {save_path}', flush=True)
                elif key in (ord('m'), ord('M')):
                    current_model = 'B' if current_model in ('A', 'C') else 'A'
                    model_path = model_path_for(current_model, config)
                    detector = Detector(model_path)
                    print(
                        f'>>> Switched to Model {current_model}: {model_path.name}',
                        flush=True,
                    )
            else:
                now_log = time.monotonic()
                if now_log - last_headless_log >= 2.0:
                    last_headless_log = now_log
                    det_summary = ', '.join(f'{d.label}:{d.score:.2f}' for d in detections[:3])
                    if len(detections) > 3:
                        det_summary += f', ... (+{len(detections) - 3})'
                    print(
                        f'[detector] Model: {current_model} | FPS: {displayed_fps:.1f} '
                        f'| Detections: {len(detections)} ({det_summary or "none"}) | '
                        f'RobViz: {node.annotated_topic}',
                        flush=True,
                    )

    finally:
        _ignore_shutdown_signals()
        if control_panel is not None:
            control_panel.close()
        try:
            cv2.destroyAllWindows()
        except Exception:  # noqa: BLE001
            pass
        if control_service is not None:
            control_service.destroy()
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
        if owned_robviz_bridge is not None:
            stop_process(owned_robviz_bridge)
        if owned_planner is not None:
            stop_process(owned_planner)
        if owned_bridge is not None:
            stop_process(owned_bridge)
        if owned_arm_debug is not None:
            stop_process(owned_arm_debug)
        stop_process(camera_process)

    return 0


def main() -> None:
    _install_shutdown_handlers()
    try:
        raise SystemExit(run())
    except KeyboardInterrupt:
        raise SystemExit(130) from None
    except Exception as error:  # noqa: BLE001
        print(f'error: {error}', file=sys.stderr)
        raise SystemExit(1) from None


if __name__ == '__main__':
    main()
