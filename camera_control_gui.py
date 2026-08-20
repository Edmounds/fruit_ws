#!/usr/bin/env python3
"""Live camera controls shared by the detector GUI."""

from __future__ import annotations

import re
import shutil
import subprocess
import sys
import tkinter as tk
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from tkinter import ttk
from typing import Any, Callable

import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import DescribeParameters, GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter

from auto_tune_usb_cameras import update_yaml_values


USB_CONTROL_LABELS = {
    'auto_exposure': 'Auto exposure',
    'exposure_time_absolute': 'Exposure',
    'gain': 'Gain',
    'white_balance_automatic': 'Auto white balance',
    'white_balance_temperature': 'White balance',
    'brightness': 'Brightness',
    'contrast': 'Contrast',
    'saturation': 'Saturation',
    'sharpness': 'Sharpness',
    'gamma': 'Gamma',
    'backlight_compensation': 'Backlight compensation',
    'power_line_frequency': 'Power line frequency',
}

D435I_CONTROL_LABELS = {
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

USB_CAMERA_PARAMETER_KEYS = {
    'auto_exposure': 'autoexposure',
    'exposure_time_absolute': 'exposure',
    'gain': 'gain',
    'white_balance_automatic': 'auto_white_balance',
    'white_balance_temperature': 'white_balance',
    'brightness': 'brightness',
    'contrast': 'contrast',
    'saturation': 'saturation',
    'sharpness': 'sharpness',
}

AUTO_CONTROL_FOR = {
    'exposure_time_absolute': 'auto_exposure',
    'white_balance_temperature': 'white_balance_automatic',
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
        self._root.title(f'{camera_name} camera controls')
        self._root.resizable(False, True)
        self._root.protocol('WM_DELETE_WINDOW', self.close)
        frame = ttk.Frame(self._root, padding=10)
        frame.grid(sticky='nsew')
        frame.columnconfigure(1, weight=1)

        for row, control in enumerate(controls):
            ttk.Label(frame, text=control.label).grid(
                row=row,
                column=0,
                padx=(0, 12),
                pady=3,
                sticky='w',
            )
            variable = self._create_variable(control)
            self._variables[control.name] = variable
            widget = self._create_widget(frame, control, variable)
            widget.grid(row=row, column=1, pady=3, sticky='ew')

        self._status = tk.StringVar(value='Ready')
        ttk.Separator(frame).grid(
            row=len(controls),
            column=0,
            columnspan=2,
            pady=(8, 6),
            sticky='ew',
        )
        ttk.Label(frame, textvariable=self._status).grid(
            row=len(controls) + 1,
            column=0,
            columnspan=2,
            sticky='w',
        )
        ttk.Button(frame, text='Save', command=self._request_save).grid(
            row=len(controls) + 2,
            column=0,
            columnspan=2,
            pady=(8, 0),
            sticky='ew',
        )
        self._root.update_idletasks()
        x = max(0, self._root.winfo_screenwidth() - self._root.winfo_reqwidth() - 20)
        self._root.geometry(f'+{x}+60')

    def _create_widget(
        self,
        parent: ttk.Frame,
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


class D435iControlService:
    NODE_NAME = '/camera/camera'

    def __init__(self, node: Node) -> None:
        self._node = node
        self._get_client = node.create_client(
            GetParameters,
            f'{self.NODE_NAME}/get_parameters',
        )
        self._describe_client = node.create_client(
            DescribeParameters,
            f'{self.NODE_NAME}/describe_parameters',
        )
        self._set_client = node.create_client(
            SetParameters,
            f'{self.NODE_NAME}/set_parameters',
        )

    def discover(self) -> list[CameraControl]:
        for client in (
            self._get_client,
            self._describe_client,
            self._set_client,
        ):
            if not client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(
                    f'Parameter service is unavailable for {self.NODE_NAME}',
                )

        names = list(D435I_CONTROL_LABELS)
        get_response = self._call(
            self._get_client,
            GetParameters.Request(names=names),
        )
        describe_response = self._call(
            self._describe_client,
            DescribeParameters.Request(names=names),
        )

        controls: list[CameraControl] = []
        for name, value, descriptor in zip(
            names,
            get_response.values,
            describe_response.descriptors,
        ):
            control = self._control_from_parameter(name, value, descriptor)
            if control is not None:
                controls.append(control)
        return controls

    def set(self, name: str, value: bool | int | float) -> None:
        request = SetParameters.Request(
            parameters=[Parameter(name, value=value).to_parameter_msg()],
        )
        response = self._call(self._set_client, request)
        result = response.results[0]
        if not result.successful:
            raise RuntimeError(result.reason or 'parameter was rejected')

    def _call(self, client: Any, request: Any) -> Any:
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
        if not future.done():
            raise RuntimeError('ROS parameter request timed out')
        if future.exception() is not None:
            raise RuntimeError(str(future.exception()))
        return future.result()

    @staticmethod
    def _control_from_parameter(
        name: str,
        value: Any,
        descriptor: Any,
    ) -> CameraControl | None:
        if value.type == ParameterType.PARAMETER_BOOL:
            return CameraControl(
                name,
                D435I_CONTROL_LABELS[name],
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
                D435I_CONTROL_LABELS[name],
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
                D435I_CONTROL_LABELS[name],
                value_range.from_value,
                value_range.to_value,
                value_range.step,
                value.double_value,
                float,
            )
        return None


def run_v4l2_control(device: str, control: str, value: int) -> None:
    result = subprocess.run(
        [
            'v4l2-ctl',
            '--device',
            device,
            '--set-ctrl',
            f'{control}={value}',
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        details = (result.stderr or result.stdout).strip()
        raise RuntimeError(
            f'Failed to set {control}={value} on {device}: {details}',
        )


def discover_usb_controls(device: str) -> list[CameraControl]:
    result = subprocess.run(
        ['v4l2-ctl', '--device', device, '--list-ctrls'],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        details = (result.stderr or result.stdout).strip()
        raise RuntimeError(f'Failed to query controls on {device}: {details}')

    discovered: dict[str, tuple[str, dict[str, int]]] = {}
    pattern = re.compile(r'^\s*(\w+)\s+0x[0-9a-f]+ \(([^)]+)\)\s*:\s*(.*)$')
    for line in result.stdout.splitlines():
        match = pattern.match(line)
        if match is None:
            continue
        name, control_type, details = match.groups()
        fields = {
            key: int(value)
            for key, value in re.findall(r'(\w+)=(-?\d+)', details)
        }
        if {'default', 'value'} <= fields.keys():
            discovered[name] = (control_type, fields)

    controls: list[CameraControl] = []
    for name, label in USB_CONTROL_LABELS.items():
        if name not in discovered:
            continue
        control_type, fields = discovered[name]
        if name == 'auto_exposure':
            controls.append(
                CameraControl(
                    name,
                    label,
                    0.0,
                    1.0,
                    1.0,
                    fields['value'] == 3,
                    bool,
                ),
            )
            continue
        if control_type == 'bool':
            controls.append(
                CameraControl(
                    name,
                    label,
                    0.0,
                    1.0,
                    1.0,
                    bool(fields['value']),
                    bool,
                ),
            )
            continue
        controls.append(
            CameraControl(
                name,
                label,
                float(fields['min']),
                float(fields['max']),
                float(max(1, fields.get('step', 1))),
                fields['value'],
                int,
            ),
        )
    return controls


def set_usb_control(
    device: str,
    name: str,
    value: bool | int | float,
) -> None:
    hardware_value = int(value)
    if name == 'auto_exposure':
        hardware_value = 3 if bool(value) else 1
    run_v4l2_control(device, name, hardware_value)


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

    if camera_name == 'd435i':
        updates = {'d435i_camera': {'ros__parameters': values}}
    else:
        camera_parameters = {
            USB_CAMERA_PARAMETER_KEYS[name]: value
            for name, value in values.items()
            if name in USB_CAMERA_PARAMETER_KEYS
        }
        side = 'left' if camera_name == 'left_camera' else 'right'
        v4l2_controls = {
            f'{side}_backlight_compensation': values[
                'backlight_compensation'
            ],
            f'{side}_gamma': values['gamma'],
            f'{side}_power_line_frequency': values[
                'power_line_frequency'
            ],
        }
        updates = {
            camera_name: {'ros__parameters': camera_parameters},
            'usb_camera_controls': {'ros__parameters': v4l2_controls},
        }

    update_yaml_values(config_path, updates)
    return backup_path
