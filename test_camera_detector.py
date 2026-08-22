#!/usr/bin/env python3
"""OpenCV GUI camera detector smoke-test tool."""

from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from urllib.parse import unquote, urlparse


DEFAULT_DISPLAY = ':0'


def display_from_argv(argv: list[str]) -> str:
    for index, argument in enumerate(argv):
        if argument == '--display' and index + 1 < len(argv):
            return argv[index + 1]
        if argument.startswith('--display='):
            return argument.split('=', maxsplit=1)[1]
    return DEFAULT_DISPLAY


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
import yaml  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from openvino.preprocess import PrePostProcessor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

from camera_control_gui import (  # noqa: E402
    CameraControlPanel,
    D435iControlService,
    discover_usb_controls,
    run_v4l2_control,
    save_camera_controls,
    set_usb_control,
)


WORKSPACE = Path('/home/rc4/fruit_ws')
CONFIG_PATH = WORKSPACE / 'src/fruit_perception/config/fruit_perception.yaml'
MODEL_DIR = WORKSPACE / 'src/fruit_perception/model'
TMP_DIR = WORKSPACE / 'tmp'
MODEL_INPUT_SIZE = 640
PAD_COLOR = 114

FRUIT_TYPES = [
    'apple_ripe', 'apple_unripe', 'chili_ripe', 'chili_unripe',
    'pumpkin_ripe', 'pumpkin_unripe', 'onion_ripe', 'onion_unripe',
    'pear_ripe', 'pear_unripe', 'tomato_ripe', 'tomato_unripe',
]
MATURITY_TYPES = ['ripe', 'unripe']
TREE_FRUIT_TYPES = [
    'apple_ripe', 'apple_unripe', 'pear_ripe', 'pear_unripe',
]
SIDE_FRUIT_TYPES = [
    'chili_ripe', 'chili_unripe', 'pumpkin_ripe', 'pumpkin_unripe',
    'onion_ripe', 'onion_unripe', 'tomato_ripe', 'tomato_unripe',
]


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
    topic: str
    parameters: dict[str, Any]
    controls: dict[str, int] | None = None
    calibration: dict[str, Any] | None = None


class FrameSubscriber(Node):
    def __init__(self, topic: str) -> None:
        super().__init__('test_camera_detector')
        self._bridge = CvBridge()
        self._latest_frame: np.ndarray | None = None
        self._latest_encoding = ''
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
        self._latest_encoding = message.encoding

    def take_latest_frame(self) -> np.ndarray | None:
        frame = self._latest_frame
        self._latest_frame = None
        return frame


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Open a configured camera and show OpenVINO detections.',
    )
    camera_group = parser.add_mutually_exclusive_group(required=True)
    camera_group.add_argument(
        '--cam1',
        action='store_true',
        help='open the configured left USB camera',
    )
    camera_group.add_argument(
        '--cam2',
        action='store_true',
        help='open the configured right USB camera',
    )
    camera_group.add_argument(
        '--cam3',
        action='store_true',
        help='open the configured D435i color camera',
    )
    parser.add_argument(
        '--model',
        required=True,
        choices=('A', 'B', 'C'),
        help='model letter to load from src/fruit_perception/model',
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
        default=DEFAULT_DISPLAY,
        help=f'desktop display for the GUI window (default: {DEFAULT_DISPLAY})',
    )
    return parser.parse_args()


def load_config() -> dict[str, Any]:
    with CONFIG_PATH.open(encoding='utf-8') as config_file:
        return yaml.safe_load(config_file)


def load_calibration(camera_parameters: dict[str, Any]) -> dict[str, Any]:
    url = str(camera_parameters.get('camera_info_url', ''))
    parsed = urlparse(url)
    if parsed.scheme != 'file':
        raise RuntimeError(f'Expected file:// camera_info_url, got {url!r}')

    calibration_path = Path(unquote(parsed.path))
    with calibration_path.open(encoding='utf-8') as calibration_file:
        calibration = yaml.safe_load(calibration_file)

    expected_width = int(camera_parameters['image_width'])
    expected_height = int(camera_parameters['image_height'])
    actual_width = int(calibration['image_width'])
    actual_height = int(calibration['image_height'])
    if (actual_width, actual_height) != (expected_width, expected_height):
        raise RuntimeError(
            f'Calibration {calibration_path} is {actual_width}x'
            f'{actual_height}, expected {expected_width}x{expected_height}',
        )

    for key in (
        'camera_matrix',
        'distortion_coefficients',
        'rectification_matrix',
        'projection_matrix',
    ):
        if key not in calibration or 'data' not in calibration[key]:
            raise RuntimeError(f'Calibration {calibration_path} missing {key}')

    return calibration


def select_camera(
    args: argparse.Namespace,
    config: dict[str, Any],
) -> CameraSelection:
    perception = config['fruit_perception_node']['ros__parameters']
    controls = config['usb_camera_controls']['ros__parameters']

    if args.cam1:
        parameters = dict(config['left_camera']['ros__parameters'])
        return CameraSelection(
            name='left_camera',
            kind='usb',
            topic=perception['left_input_topic'],
            parameters=parameters,
            controls={
                'backlight_compensation': int(
                    controls['left_backlight_compensation'],
                ),
                'gamma': int(controls['left_gamma']),
                'power_line_frequency': int(
                    controls['left_power_line_frequency'],
                ),
            },
            calibration=load_calibration(parameters),
        )
    if args.cam2:
        parameters = dict(config['right_camera']['ros__parameters'])
        return CameraSelection(
            name='right_camera',
            kind='usb',
            topic=perception['right_input_topic'],
            parameters=parameters,
            controls={
                'backlight_compensation': int(
                    controls['right_backlight_compensation'],
                ),
                'gamma': int(controls['right_gamma']),
                'power_line_frequency': int(
                    controls['right_power_line_frequency'],
                ),
            },
            calibration=load_calibration(parameters),
        )

    parameters = dict(config['d435i_camera']['ros__parameters'])
    return CameraSelection(
        name='d435i',
        kind='d435i',
        topic='/camera/camera/color/image_raw',
        parameters=parameters,
    )


def model_path_for(letter: str, config: dict[str, Any]) -> Path:
    perception = config['fruit_perception_node']['ros__parameters']
    if letter == 'B':
        return Path(perception['b_model_path'])
    if letter == 'C':
        return Path(perception['c_model_path'])
    return MODEL_DIR / 'A.onnx'


def apply_usb_controls(selection: CameraSelection) -> None:
    parameters = selection.parameters
    controls = selection.controls or {}
    device = str(parameters['video_device'])

    for name, value in controls.items():
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
    params_path = TMP_DIR / f'test_camera_detector_{selection.name}_params.yaml'
    if selection.kind == 'd435i':
        node_key = '/camera/camera'
    else:
        node_key = '/**'
    document = {
        node_key: {
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
    log_file = log_path.open('w', encoding='utf-8')
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
            f'test_camera_detector_{selection.name}_container',
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
            'test_camera_detector_d435i_container',
        ]

    append_component_params(command, selection.parameters)
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


def labels_for_class_count(class_count: int) -> list[str]:
    if class_count == len(MATURITY_TYPES):
        return MATURITY_TYPES
    if class_count == len(TREE_FRUIT_TYPES):
        return TREE_FRUIT_TYPES
    if class_count == len(SIDE_FRUIT_TYPES):
        return SIDE_FRUIT_TYPES
    if class_count == len(FRUIT_TYPES):
        return FRUIT_TYPES
    return []


def parse_nms_output(
    output: np.ndarray,
    confidence_threshold: float,
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
        if class_id < 0 or class_id >= len(FRUIT_TYPES):
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
                FRUIT_TYPES[class_id],
            ),
        )
    return detections


def parse_raw_yolo_output(
    output: np.ndarray,
    confidence_threshold: float,
    nms_iou_threshold: float,
) -> list[Detection]:
    if output.ndim != 3 or output.shape[0] != 1 or output.shape[1] < 5:
        raise RuntimeError(f'Unsupported model output shape {output.shape}')

    class_count = int(output.shape[1] - 4)
    candidate_count = int(output.shape[2])
    labels = labels_for_class_count(class_count)
    if not labels:
        raise RuntimeError(f'Unsupported raw YOLO class count {class_count}')

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
        if (
            not math.isfinite(best_score)
            or best_score < confidence_threshold
        ):
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
) -> list[Detection]:
    if output.shape == (1, 300, 6):
        detections = parse_nms_output(output, confidence_threshold)
    else:
        detections = parse_raw_yolo_output(
            output,
            confidence_threshold,
            nms_iou_threshold,
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
    detections: list[Detection],
    transform: LetterboxTransform,
    fps: float,
) -> np.ndarray:
    annotated = frame_rgb.copy()
    color = (0, 255, 0)

    for detection in detections:
        x, y, width, height = to_original_box(detection, transform)
        if width <= 0 or height <= 0:
            continue
        cv2.rectangle(annotated, (x, y), (x + width, y + height), color, 2)

        label = f'{detection.label} {detection.score:.2f}'
        text_size, baseline = cv2.getTextSize(
            label,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            1,
        )
        text_x = min(max(x, 0), max(0, annotated.shape[1] - text_size[0] - 6))
        text_y = max(y - 6, text_size[1] + 4)
        cv2.rectangle(
            annotated,
            (
                text_x,
                text_y - text_size[1] - 4,
                text_size[0] + 6,
                text_size[1] + baseline + 6,
            ),
            color,
            cv2.FILLED,
        )
        cv2.putText(
            annotated,
            label,
            (text_x + 3, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )

    fps_label = f'FPS: {fps:.1f}'
    cv2.putText(
        annotated,
        fps_label,
        (10, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 0, 0),
        4,
        cv2.LINE_AA,
    )
    cv2.putText(
        annotated,
        fps_label,
        (10, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        color,
        2,
        cv2.LINE_AA,
    )
    return annotated


class Detector:
    def __init__(self, model_path: Path) -> None:
        if not model_path.exists():
            raise RuntimeError(f'Model file does not exist: {model_path}')

        self._core = ov.Core()
        devices = self._core.available_devices
        if 'GPU' not in devices:
            raise RuntimeError(
                f'OpenVINO GPU device is not available; devices: {devices}',
            )

        model = self._core.read_model(str(model_path))
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
            'GPU',
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
                and output_shape[1] >= 5
                and output_shape[2] > 0
            )
        ):
            raise RuntimeError(f'Unsupported model output shape {output_shape}')

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


def run() -> int:
    args = parse_args()
    config = load_config()
    selection = select_camera(args, config)
    model_path = model_path_for(args.model, config)
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

    detector = Detector(model_path)
    rclpy.init()
    node = FrameSubscriber(selection.topic)
    camera_process: subprocess.Popen | None = None
    log_path = TMP_DIR / f'test_camera_detector_{selection.name}.log'
    if wait_for_publisher(node, selection.topic, 1.0):
        print(
            f'Using existing publisher on {selection.topic}; '
            f'model {model_path}; conf {confidence_threshold:.3f}; '
            f'iou {nms_iou_threshold:.3f}',
            flush=True,
        )
    else:
        camera_process, log_path = start_camera(selection)
        print(
            f'Started {selection.name}; image topic {selection.topic}; '
            f'model {model_path}; conf {confidence_threshold:.3f}; '
            f'iou {nms_iou_threshold:.3f}; log {log_path}',
            flush=True,
        )
    window_name = f'{selection.name} model {args.model}'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.moveWindow(window_name, 20, 60)
    try:
        if selection.kind == 'd435i' and not wait_for_publisher(
            node,
            selection.topic,
            10.0,
        ):
            check_camera_process(camera_process, log_path)
            raise RuntimeError(
                f'Timed out waiting for D435i frames on {selection.topic}',
            )
        if selection.kind == 'usb':
            device = str(selection.parameters['video_device'])
            controls = discover_usb_controls(device)
            control_panel = CameraControlPanel(
                selection.name,
                controls,
                lambda name, value: set_usb_control(device, name, value),
                lambda values: save_camera_controls(
                    selection.name,
                    values,
                    CONFIG_PATH,
                    TMP_DIR,
                ),
            )
        else:
            control_service = D435iControlService(node)
            control_panel = CameraControlPanel(
                selection.name,
                control_service.discover(),
                control_service.set,
                lambda values: save_camera_controls(
                    selection.name,
                    values,
                    CONFIG_PATH,
                    TMP_DIR,
                ),
            )
    except BaseException:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        stop_process(camera_process)
        cv2.destroyAllWindows()
        raise

    last_frame_time = 0.0
    displayed_fps = 0.0
    last_wait_message = 0.0
    announced_first_frame = False

    try:
        while rclpy.ok():
            check_camera_process(camera_process, log_path)
            control_panel.apply_pending()
            rclpy.spin_once(node, timeout_sec=0.01)
            frame = node.take_latest_frame()
            if frame is None:
                now = time.monotonic()
                if not announced_first_frame and now - last_wait_message > 2.0:
                    print(f'Waiting for frames on {selection.topic}...', flush=True)
                    last_wait_message = now
                if (
                    cv2.waitKey(1) in (27, ord('q'))
                    or not control_panel.is_open()
                ):
                    break
                continue
            if not announced_first_frame:
                print(
                    f'Received first frame from {selection.topic}: '
                    f'{frame.shape[1]}x{frame.shape[0]}',
                    flush=True,
                )
                announced_first_frame = True

            output, transform = detector.infer(frame)
            detections = parse_detections(
                output,
                confidence_threshold,
                nms_iou_threshold,
            )

            now = time.monotonic()
            if last_frame_time > 0.0:
                interval = now - last_frame_time
                instantaneous_fps = 1.0 / interval if interval > 0.0 else 0.0
                displayed_fps = (
                    instantaneous_fps
                    if displayed_fps == 0.0
                    else 0.8 * displayed_fps + 0.2 * instantaneous_fps
                )
            last_frame_time = now

            annotated_rgb = draw_annotations(
                frame,
                detections,
                transform,
                displayed_fps,
            )
            annotated_bgr = cv2.cvtColor(annotated_rgb, cv2.COLOR_RGB2BGR)
            cv2.imshow(window_name, annotated_bgr)

            key = cv2.waitKey(1)
            if key in (27, ord('q')) or not control_panel.is_open():
                break
    finally:
        control_panel.close()
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
        cv2.destroyAllWindows()

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
