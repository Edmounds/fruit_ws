#!/usr/bin/env python3
"""Interactive manual arm motion client for a running arm debug launch."""

import argparse
import curses
import math
import os
import readline
import shlex
import sys
import tempfile
import threading
import time
from pathlib import Path

import yaml

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger

from fruit_arm_interfaces.srv import ExecuteArmPreset, MoveArmCartesian, MoveArmJoints
from fruit_arm_planner.arm_planner_node import validate_presets
from fruit_arm_planner.joint_transformer import JointTransformer
from fruit_arm_planner.kinematics import RobotKinematics
from fruit_hardware_interfaces.msg import ArmControlMode, ArmStatus, TrajectoryHeader, TrajectoryPoint


STATUS_NAMES = {
    ArmStatus.STATUS_IDLE: 'IDLE',
    ArmStatus.STATUS_RUNNING: 'RUNNING',
    ArmStatus.STATUS_COMPLETED: 'COMPLETED',
    ArmStatus.STATUS_FAILED: 'FAILED',
    ArmStatus.STATUS_EMERGENCY: 'EMERGENCY',
}
GRIPPER_ACTIONS = {'hold': 0, 'open': 1, 'close': 2}
LOG_LEVELS = {'info': 0, 'debug': 1, 'trace': 2}
MODE_NAMES = {
    ArmControlMode.MODE_TRAJECTORY: 'TRAJECTORY',
    ArmControlMode.MODE_STOP: 'STOP',
    ArmControlMode.MODE_ESTOP: 'ESTOP',
}
HELP = (
    'Commands:\n'
    'move X Y Z PITCH_DEG [DURATION_MS] [ptp|lin] [hold|open|close]\n'
    'joints Q1 Q2 Q3 Q4 [DURATION_MS] [hold|open|close]\n'
    'motors Q1 Q2 Q3 Q4 [DURATION_MS] [hold|open|close]  Raw hardware degrees\n'
    'preset NAME   Execute a configured preset (zero, home, retract, pick_sequence)\n'
    'jog       Keyboard jog/teach: arrows=XY, a/z=Z, s/x=pitch, c/o/h=gripper, Space=save\n'
    'reload    Reload all presets in the running planner (only when not moving)\n'
    'status    Show the latest controller status and measured joint angles\n'
    'stop      Request a controlled stop\n'
    'estop     Request an emergency stop\n'
    'level [info|debug|trace]  Show or change console log detail\n'
    'help      Show this command list\n'
    'quit      Exit when no move is pending (Ctrl+C sends ESTOP)\n'
    'Coordinates are metres in the fixed arm-base frame. '
    'PITCH_DEG is absolute; DURATION_MS=0 uses the planner default.\n'
    'joints: Q2=0 arm upright, Q3=0 forearm level, Q4=0 end effector level.\n'
    'joints applies the YAML joint mapping; motors bypasses it.\n'
    'Example: move 0.20 0.0 0.25 0 2500 ptp hold\n'
    'Example: joints 0 20 0 0 2500 hold; preset home\n'
    'debug prints every 20 Hz status and ROS Header/Mode; '
    'trace also prints every ROS trajectory point.'
)


class ArmConsole(Node):
    def __init__(self, log_level, config_path, params):
        super().__init__('arm_cartesian_debug_console')
        self.log_level = log_level
        self.config_path = config_path
        self.transformer = JointTransformer.from_config_dict(params['joint_mapping'])
        self.kinematics = RobotKinematics.from_geometry_dict(params['geometry'])
        self.planning = params['planning']
        self._output_lock = threading.Lock()
        self._terminal_ui = False
        self._ui_message = ''
        self.move_client = self.create_client(MoveArmCartesian, '/arm_planner_node/move_cartesian')
        self.joints_client = self.create_client(MoveArmJoints, '/arm_planner_node/move_joints')
        self.preset_client = self.create_client(ExecuteArmPreset, '/arm_planner_node/execute_preset')
        self.stop_client = self.create_client(Trigger, '/arm_planner_node/stop')
        self.estop_client = self.create_client(Trigger, '/arm_planner_node/emergency_stop')
        self.reload_client = self.create_client(Trigger, '/arm_planner_node/reload_presets')
        self.create_subscription(ArmStatus, '/fruit_hardware_bridge/arm/status', self._on_status, 10)
        tx_qos = QoSProfile(depth=512, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            TrajectoryHeader, '/fruit_hardware_bridge/arm/trajectory_header',
            self._on_header, tx_qos,
        )
        self.create_subscription(
            TrajectoryPoint, '/fruit_hardware_bridge/arm/trajectory_point',
            self._on_point, tx_qos,
        )
        self.create_subscription(
            ArmControlMode, '/fruit_hardware_bridge/arm/control_mode',
            self._on_mode, tx_qos,
        )
        self._lock = threading.Lock()
        self._status = None
        self._status_at = 0.0
        self._status_key = None
        self._move_future = None

    def _emit(self, label, message):
        now = time.time()
        stamp = time.strftime('%H:%M:%S', time.localtime(now))
        with self._output_lock:
            if self._terminal_ui:
                self._ui_message = f'{label}: {message}'
                return
            sys.stdout.write(f'\r\033[K[{stamp}.{int(now % 1 * 1000):03d}] {label} {message}\n')
            sys.stdout.flush()
            readline.redisplay()

    def _on_status(self, msg):
        key = (msg.trajectory_id, msg.task_status, msg.error_code)
        with self._lock:
            self._status = msg
            self._status_at = time.monotonic()
            changed = key != self._status_key
            self._status_key = key
        if self._terminal_ui:
            return  # The curses screen displays telemetry without overwriting operation results.
        if self.log_level == 'info' and not changed:
            return
        joints_deg = ', '.join(
            f'{math.degrees(q):.1f}' for q in
            (msg.joint_1, msg.joint_2, msg.joint_3, msg.joint_4)
        )
        self._emit(
            'RX STATUS',
            f'id={msg.trajectory_id} state={STATUS_NAMES.get(msg.task_status, msg.task_status)} '
            f'progress={msg.progress:.1f}% joints_deg=[{joints_deg}] '
            f'gripper={msg.gripper_state} error={msg.error_code}',
        )

    def _on_header(self, msg):
        if LOG_LEVELS[self.log_level] >= LOG_LEVELS['debug']:
            self._emit('ROS TX HEADER', f'id={msg.trajectory_id} point_num={msg.point_num}')

    def _on_point(self, msg):
        if self.log_level == 'trace':
            joints = ', '.join(
                f'{q:.5f}' for q in
                (msg.joint_1, msg.joint_2, msg.joint_3, msg.joint_4)
            )
            self._emit(
                'ROS TX POINT',
                f'id={msg.trajectory_id} point={msg.point_id} t={msg.time_from_start}ms '
                f'joints_rad=[{joints}] gripper_action={msg.gripper_action}',
            )

    def _on_mode(self, msg):
        if LOG_LEVELS[self.log_level] >= LOG_LEVELS['debug']:
            self._emit('ROS TX MODE', f'mode={MODE_NAMES.get(msg.control_mode, msg.control_mode)}')

    def set_level(self, args):
        if not args:
            print(f'Current console log level: {self.log_level}')
        elif len(args) == 1 and args[0].lower() in LOG_LEVELS:
            self.log_level = args[0].lower()
            print(f'Console log level: {self.log_level}')
        else:
            print('Usage: level info|debug|trace')

    def show_status(self):
        with self._lock:
            msg = self._status
            age = time.monotonic() - self._status_at
        if msg is None:
            print('No arm status received. Check the serial bridge and lower controller.')
            return
        joints = (msg.joint_1, msg.joint_2, msg.joint_3, msg.joint_4)
        angles = ', '.join(f'{math.degrees(q):.1f}' for q in joints)
        q_algo = self.transformer.hardware_to_algo(joints)
        link_angles = (q_algo[0], q_algo[1] - math.pi / 2,
                       q_algo[1] + q_algo[2], q_algo[1] + q_algo[2] + q_algo[3])
        print(f'Status: {STATUS_NAMES.get(msg.task_status, msg.task_status)}, '
              f'trajectory={msg.trajectory_id}, progress={msg.progress:.2f}, '
              f'error={msg.error_code}, age={age:.2f}s')
        print(f'Motors (deg): [{angles}], gripper={msg.gripper_state}')
        print('Link angles (deg): [' + ', '.join(f'{math.degrees(q):.1f}' for q in link_angles) + ']')

    def move(self, args):
        if not 4 <= len(args) <= 7:
            print(HELP)
            return
        try:
            values = [float(value) for value in args[:4]]
            if not all(math.isfinite(value) for value in values):
                raise ValueError('coordinates and pitch must be finite')
            duration = int(args[4]) if len(args) >= 5 else 0
            if not 0 <= duration <= 0xFFFFFFFF:
                raise ValueError('duration must be 0..4294967295 ms')
            mode = args[5].lower() if len(args) >= 6 else 'ptp'
            if mode not in ('ptp', 'lin'):
                raise ValueError('mode must be ptp or lin')
            gripper = args[6].lower() if len(args) >= 7 else 'hold'
            if gripper not in GRIPPER_ACTIONS:
                raise ValueError('gripper must be hold, open or close')
        except ValueError as exc:
            print(f'Invalid move command: {exc}')
            return

        request = MoveArmCartesian.Request()
        request.x, request.y, request.z, request.pitch_deg = values
        request.duration_ms = duration
        request.linear_path = mode == 'lin'
        request.gripper_action = GRIPPER_ACTIONS[gripper]
        self._submit_motion(
            self.move_client, 'move_cartesian', request,
            f'Submitted {mode.upper()} target {values[:3]} m, pitch={values[3]} deg, '
            f'duration={duration or "default"} ms, gripper={gripper}.',
        )

    def joints(self, args, raw=False):
        if not 4 <= len(args) <= 6:
            name = 'motors' if raw else 'joints'
            print(f'Usage: {name} Q1 Q2 Q3 Q4 [DURATION_MS] [hold|open|close]')
            return
        try:
            angles = [float(value) for value in args[:4]]
            if not all(math.isfinite(angle) for angle in angles):
                raise ValueError('joint angles must be finite')
            duration = int(args[4]) if len(args) >= 5 else 0
            if not 0 <= duration <= 0xFFFFFFFF:
                raise ValueError('duration must be 0..4294967295 ms')
            gripper = args[5].lower() if len(args) >= 6 else 'hold'
            if gripper not in GRIPPER_ACTIONS:
                raise ValueError('gripper must be hold, open or close')
        except ValueError as exc:
            print(f'Invalid joints command: {exc}')
            return

        if raw:
            target_hw_deg = angles
        else:
            target_hw_deg = [math.degrees(q) for q in self.transformer.link_angles_to_hardware(angles)]

        request = MoveArmJoints.Request()
        request.joints_deg = target_hw_deg
        request.duration_ms = duration
        request.gripper_action = GRIPPER_ACTIONS[gripper]
        self._submit_motion(
            self.joints_client, 'move_joints', request,
            f'Submitted {"motor" if raw else "link"} target {angles} deg '
            f'-> hardware {[round(q, 2) for q in target_hw_deg]} deg, '
            f'duration={duration or "default"} ms, gripper={gripper}.',
        )

    def preset(self, args):
        if len(args) != 1 or not args[0].strip():
            print('Usage: preset NAME (zero, home, retract, pick_sequence)')
            return
        request = ExecuteArmPreset.Request()
        request.preset_name = args[0]
        self._submit_motion(
            self.preset_client, 'execute_preset', request,
            f'Submitted preset {request.preset_name}.',
        )

    def _submit_motion(self, client, name, request, message):
        if not client.service_is_ready():
            self._emit('ERROR', f'Planner {name} service is unavailable.')
            return
        with self._lock:
            if self._move_future is not None:
                self._emit('BUSY', 'A move is still pending. Use stop or estop to interrupt it.')
                return
            future = client.call_async(request)
            self._move_future = future
        future.add_done_callback(self._on_move_done)
        self._emit('MOVE', message)
        return future

    def _on_move_done(self, future):
        try:
            result = future.result()
            self._emit('RESULT', f'success={result.success}, '
                       f'trajectory_id={result.trajectory_id}, message={result.message}')
        except Exception as exc:
            self._emit('ERROR', f'Move service failed: {exc}')
        finally:
            with self._lock:
                if self._move_future is future:
                    self._move_future = None

    def control(self, name):
        client = self.stop_client if name == 'stop' else self.estop_client
        if not client.service_is_ready():
            self._emit('ERROR', f'{name} service is unavailable.')
            return None
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda done: self._on_control_done(name, done))
        self._emit('CONTROL', f'{name.upper()} requested; wait for hardware status.')
        return future

    def _on_control_done(self, name, future):
        try:
            result = future.result()
            self._emit(name.upper(), f'success={result.success}, message={result.message}')
        except Exception as exc:
            self._emit('ERROR', f'{name.upper()} service failed: {exc}')

    def move_pending(self):
        with self._lock:
            return self._move_future is not None

    def reload_presets(self):
        if self.move_pending():
            self._emit('ERROR', 'Wait for the move to finish before reloading presets.')
            return None
        if not self.reload_client.service_is_ready():
            self._emit('ERROR', 'reload_presets unavailable; rebuild and restart the planner.')
            return None
        future = self.reload_client.call_async(Trigger.Request())
        future.add_done_callback(lambda done: self._on_control_done('reload_presets', done))
        return future

    def measured_pose(self):
        with self._lock:
            status = self._status
            age = time.monotonic() - self._status_at
        if status is None or age > float(self.planning['status_max_age_sec']):
            raise ValueError('Fresh four-joint feedback is required')
        joints = (status.joint_1, status.joint_2, status.joint_3, status.joint_4)
        if not all(math.isfinite(q) for q in joints):
            raise ValueError('Invalid joint feedback (NaN/Inf)')
        ok, message = self.transformer.check_hardware_limits(joints)
        if not ok:
            raise ValueError(message)
        algo = self.transformer.hardware_to_algo(joints)
        x, y, z, pitch = self.kinematics.forward_kinematics(*algo)['end_effector']
        return status, joints, algo, [x, y, z, math.degrees(pitch)]

    def save_point(self, name, snapshot, gripper_action=0):
        """Insert only the new waypoint/preset, preserving the rest of the YAML text."""
        if (not isinstance(gripper_action, int) or isinstance(gripper_action, bool)
                or gripper_action not in GRIPPER_ACTIONS.values()):
            raise ValueError('Invalid recorded gripper action')
        name = name.strip()
        if not name or any(ord(char) < 32 for char in name):
            raise ValueError('Preset name must be nonempty and contain no control characters')
        original = self.config_path.read_text(encoding='utf-8')
        data = yaml.safe_load(original)
        params = data['arm_planner_node']['ros__parameters']
        presets = params['presets']
        validate_presets(presets)
        _, joints, algo, pose = snapshot
        frame = presets[name]['coordinate_frame'] if name in presets else 'cartesian'
        target = (pose if frame == 'cartesian' else
                  [math.degrees(q) for q in joints] if frame == 'joints' else
                  [math.degrees(algo[0]), math.degrees(algo[1]) - 90,
                   math.degrees(algo[1] + algo[2]), pose[3]])
        duration = params['planning']['default_duration_ms']
        waypoint = {'target': target, 'duration_ms': duration, 'gripper': gripper_action}

        def child(node, key):
            return next(value for item, value in node.value if item.value == key)

        tree = yaml.compose(original)
        preset_node = child(child(child(tree, 'arm_planner_node'), 'ros__parameters'), 'presets')
        entry = yaml.safe_dump(waypoint, default_flow_style=True, width=1000).strip()
        if name in presets:
            sequence = child(child(preset_node, name), 'waypoints')
            presets[name]['waypoints'].append(waypoint)
            if sequence.flow_style:
                position = sequence.end_mark.index - 1
                insertion = ', ' + entry
            else:
                last = sequence.value[-1].end_mark
                if sequence.value[-1].flow_style:
                    newline = original.find('\n', last.index)
                    position = len(original) if newline == -1 else newline + 1
                else:
                    position = last.index if last.index == len(original) else last.index - last.column
                insertion = ('\n' if position and original[position - 1] != '\n' else '')
                insertion += ' ' * sequence.start_mark.column + '- ' + entry + '\n'
        else:
            preset = {'description': 'Recorded from keyboard jog mode',
                      'coordinate_frame': 'cartesian', 'waypoints': [waypoint]}
            presets[name] = preset
            if preset_node.flow_style:
                position = preset_node.end_mark.index - 1
                block = yaml.safe_dump({name: preset}, default_flow_style=True,
                                       allow_unicode=True, width=1000).strip()
                insertion = (', ' if preset_node.value else '') + block[1:-1]
            else:
                end = preset_node.end_mark
                position = end.index if end.index == len(original) else end.index - end.column
                block = yaml.safe_dump({name: preset}, sort_keys=False, allow_unicode=True, width=1000)
                indent = preset_node.start_mark.column
                insertion = ('\n' if position and original[position - 1] != '\n' else '')
                insertion += ''.join(' ' * indent + line + '\n' for line in block.splitlines())
        validate_presets(presets)
        updated = original[:position] + insertion + original[position:]
        if yaml.safe_load(updated) != data:
            raise ValueError('Cannot safely insert into this YAML layout; file left unchanged')
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                                             dir=self.config_path.parent, delete=False) as stream:
                temporary = Path(stream.name)
                stream.write(updated)
                stream.flush()
                os.fsync(stream.fileno())
            os.chmod(temporary, self.config_path.stat().st_mode & 0o777)
            if self.config_path.read_text(encoding='utf-8') != original:
                raise ValueError('Configuration changed while saving; retry')
            os.replace(temporary, self.config_path)
        finally:
            if temporary is not None and temporary.exists():
                temporary.unlink()
        self._emit('SAVED', f'{name}: appended waypoint, duration={duration} ms, '
                   f'gripper={gripper_action}; reloading presets')
        return self.reload_presets()

    def jog(self):
        if not sys.stdin.isatty() or not sys.stdout.isatty():
            self._emit('ERROR', 'jog requires an interactive terminal')
            return
        if self.move_pending():
            self._emit('ERROR', 'Wait for the current move before entering jog')
            return
        try:
            status, _, _, pose = self.measured_pose()
            if status.task_status not in (ArmStatus.STATUS_IDLE, ArmStatus.STATUS_COMPLETED):
                raise ValueError('Arm must be idle before entering jog')
            with self._output_lock:
                self._terminal_ui = True
                self._ui_message = 'Ready. Keys move the arm immediately; Space saves only when settled.'
            curses.wrapper(self._jog_screen, pose)
        except Exception as exc:
            if self.move_pending():
                self.control('stop')
            with self._output_lock:
                self._terminal_ui = False
            self._emit('ERROR', str(exc))
        finally:
            with self._output_lock:
                self._terminal_ui = False

    def _jog_screen(self, screen, pose):
        screen.keypad(True)
        screen.timeout(50)
        curses.set_escdelay(25)
        target = list(pose)
        sent = list(pose)
        active = None
        active_gripper = 0
        pending_gripper = None
        recorded_gripper = 0
        gripper_names = {value: name.upper() for name, value in GRIPPER_ACTIONS.items()}
        halted = False
        notice = ''
        reload_future = None
        steps = {curses.KEY_UP: (0, .01), curses.KEY_DOWN: (0, -.01),
                 curses.KEY_LEFT: (1, .01), curses.KEY_RIGHT: (1, -.01),
                 ord('a'): (2, .01), ord('z'): (2, -.01),
                 ord('s'): (3, 5.), ord('x'): (3, -5.)}
        while True:
            if active is not None and active.done() and not self.move_pending():
                try:
                    result = active.result()
                    if not result.success:
                        raise ValueError(result.message)
                except Exception as exc:
                    halted = True
                    target = list(sent)
                    pending_gripper = None
                    recorded_gripper = 0
                    notice = f'Action failed; pending actions cleared. Exit/re-enter jog: {exc}'
                active = None
                active_gripper = 0
            if (not halted and active is None and (pending_gripper is not None or target != sent)
                    and (reload_future is None or reload_future.done())):
                # A queued gripper action runs in place before any further accumulated move.
                if pending_gripper is not None:
                    try:
                        status, joints, _, _ = self.measured_pose()
                        if status.task_status not in (ArmStatus.STATUS_IDLE, ArmStatus.STATUS_COMPLETED):
                            raise ValueError('Arm must be idle before operating the gripper')
                    except ValueError as exc:
                        halted = True
                        target = list(sent)
                        pending_gripper = None
                        recorded_gripper = 0
                        notice = f'Gripper not submitted; pending actions cleared: {exc}'
                        continue
                    request = MoveArmJoints.Request()
                    request.joints_deg = [math.degrees(q) for q in joints]
                    request.duration_ms = 0
                    active_gripper = pending_gripper
                    request.gripper_action = active_gripper
                    pending_gripper = None
                    active = self._submit_motion(self.joints_client, 'move_joints', request,
                                                 f'In-place gripper {gripper_names[active_gripper]}')
                else:
                    request = MoveArmCartesian.Request()
                    request.x, request.y, request.z = [
                        value - delta for value, delta in zip(target[:3], self.planning['cartesian_offset_m'])]
                    request.pitch_deg = target[3]
                    request.duration_ms = 0  # The running planner's configured default.
                    request.gripper_action = 0
                    request.linear_path = False
                    active_gripper = 0
                    recorded_gripper = 0
                    active = self._submit_motion(self.move_client, 'move_cartesian', request,
                                                 f'Jog target {target}')
                    if active is not None:
                        sent = list(target)
                if active is None:
                    halted = True
                    notice = 'Submission failed; pending actions cleared. Exit/re-enter jog.'
                    target = list(sent)
                    pending_gripper = None
                    recorded_gripper = 0
            try:
                status, _, _, measured = self.measured_pose()
                measured_text = (str([round(v, 4) for v in measured])
                                 + f'  {STATUS_NAMES.get(status.task_status, status.task_status)}'
                                 + f'  error={status.error_code}  gripper_state={status.gripper_state}')
                if status.task_status in (ArmStatus.STATUS_FAILED, ArmStatus.STATUS_EMERGENCY):
                    halted = True
                    target = list(sent)
                    pending_gripper = None
                    recorded_gripper = 0
                    notice = 'Hardware failure/emergency; pending actions cleared. Exit/re-enter jog.'
            except ValueError as exc:
                measured_text = str(exc)
            with self._output_lock:
                message = self._ui_message
            executing = '-'
            if active is not None:
                executing = ('In-place ' + gripper_names[active_gripper] if active_gripper
                             else str([round(v, 4) for v in sent]))
            lines = ['KEYBOARD JOG / TEACH (PTP, metres / degrees)',
                     'Up/Down: X +/-1cm   Left/Right: Y +/-1cm   a/z: Z +/-1cm',
                     's/x: pitch +/-5deg   Space: save   r: reload   t: STOP   e: ESTOP   Esc: exit',
                     'c: CLOSE in place   o: OPEN in place   h: HOLD/cancel queued gripper',
                     'Measured: ' + measured_text,
                     'Accumulated target: ' + str([round(v, 4) for v in target]),
                     'Executing: ' + executing,
                     'Waypoint gripper: ' + gripper_names[recorded_gripper]
                     + '   Queued gripper: ' + gripper_names.get(pending_gripper, '-'),
                     'State: ' + ('HALTED (exit/re-enter)' if halted else 'MOVING' if active else 'READY'),
                     notice, message]
            screen.erase()
            height, width = screen.getmaxyx()
            for row, line in enumerate(lines[:max(0, height - 1)]):
                screen.addnstr(row, 0, line, max(0, width - 1))
            screen.refresh()
            key = screen.getch()
            if key == 27:
                target = list(sent)
                pending_gripper = None
                if self.move_pending():
                    self.control('stop')
                curses.flushinp()
                return
            if key == 3:
                raise KeyboardInterrupt
            if key in (ord('t'), ord('e')):
                halted = True
                target = list(sent)
                pending_gripper = None
                recorded_gripper = 0
                self.control('stop' if key == ord('t') else 'estop')
                notice = 'Pending actions cleared. Exit/re-enter after hardware returns to idle.'
            elif key in steps and not halted:
                axis, delta = steps[key]
                target[axis] += delta
                recorded_gripper = 0
                notice = ''
            elif key in (ord('c'), ord('o'), ord('h')) and not halted:
                recorded_gripper = {ord('c'): 2, ord('o'): 1, ord('h'): 0}[key]
                pending_gripper = recorded_gripper or None
                notice = ('HOLD selected; queued gripper cancelled (submitted actions are not undone).'
                          if key == ord('h') else '')
            elif key in (ord(' '), ord('r')):
                if (halted or active is not None or self.move_pending() or target != sent
                        or pending_gripper is not None
                        or (reload_future is not None and not reload_future.done())):
                    notice = 'Wait for all movement/gripper/reload to finish before saving or reloading.'
                    continue
                try:
                    if key == ord('r'):
                        reload_future = self.reload_presets()
                    else:
                        snapshot = self.measured_pose()
                        if snapshot[0].task_status not in (ArmStatus.STATUS_IDLE, ArmStatus.STATUS_COMPLETED):
                            raise ValueError('Arm must be idle before recording')
                        name = self._preset_dialog(screen)
                        curses.flushinp()
                        if name:
                            reload_future = self.save_point(name, snapshot, recorded_gripper)
                except (ValueError, KeyError, TypeError, OSError, yaml.YAMLError, StopIteration) as exc:
                    notice = f'Not saved/reloaded: {exc}'

    @staticmethod
    def _preset_dialog(screen):
        name = ''
        while True:
            height, width = screen.getmaxyx()
            if height < 6 or width < 30:
                raise ValueError('Terminal too small for the preset dialog')
            columns = min(width - 2, 76)
            dialog = curses.newwin(5, columns, (height - 5) // 2, (width - columns) // 2)
            dialog.keypad(True)
            dialog.border()
            dialog.addnstr(1, 2, 'Preset name (Enter saves, Esc cancels)', columns - 4)
            dialog.addnstr(2, 2, name[-(columns - 4):], columns - 4)
            dialog.move(2, min(columns - 2, 2 + len(name)))
            dialog.refresh()
            key = dialog.get_wch()
            if key in ('\n', '\r', curses.KEY_ENTER):
                return name.strip()
            if key == '\x1b':
                return None
            if key == '\x03':
                raise KeyboardInterrupt
            if key in ('\x7f', '\b', curses.KEY_BACKSPACE):
                name = name[:-1]
            elif isinstance(key, str) and key.isprintable() and len(name) < 128:
                name += key


def main():
    parser = argparse.ArgumentParser(description='Manual arm debug console')
    parser.add_argument('--log-level', choices=LOG_LEVELS, default='info',
                        help='Initial console log detail (default: info)')
    options = parser.parse_args()
    config_path = Path(__file__).resolve().parents[1] / 'config' / 'arm_presets.yaml'
    with config_path.open(encoding='utf-8') as config_file:
        params = yaml.safe_load(config_file)['arm_planner_node']['ros__parameters']
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = ArmConsole(options.log_level, config_path, params)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        print('Waiting up to 5 s for the planner service...')
        if not node.move_client.wait_for_service(timeout_sec=5.0):
            print('Planner service unavailable. Start fruit_arm_debug.launch.py first.')
            return
        print(HELP)
        node.show_status()
        while True:
            try:
                parts = shlex.split(input('arm> '))
            except EOFError:
                raise KeyboardInterrupt from None
            except ValueError as exc:
                print(f'Invalid command: {exc}')
                continue
            if not parts:
                continue
            command, *args = parts
            command = command.lower()
            if command == 'move':
                node.move(args)
            elif command == 'joints':
                node.joints(args)
            elif command == 'motors':
                node.joints(args, raw=True)
            elif command == 'preset':
                node.preset(args)
            elif command == 'jog' and not args:
                node.jog()
            elif command == 'reload' and not args:
                node.reload_presets()
            elif command == 'status':
                node.show_status()
            elif command in ('stop', 'estop'):
                node.control(command)
            elif command == 'level':
                node.set_level(args)
            elif command in ('help', '?'):
                print(HELP)
            elif command in ('quit', 'exit'):
                if node.move_pending():
                    print('Move still pending. Send stop or estop and wait for its result before quitting.')
                else:
                    break
            else:
                print('Unknown command. Enter help for available commands.')
    except KeyboardInterrupt:
        print('\nCtrl+C: requesting EMERGENCY STOP before exit.')
        future = node.control('estop')
        if future is not None:
            deadline = time.monotonic() + 2.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)
            if not future.done():
                print('ESTOP service did not respond; hardware state is unconfirmed.')
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
