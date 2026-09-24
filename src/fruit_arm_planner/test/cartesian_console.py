#!/usr/bin/env python3
"""Interactive manual Cartesian motion client for a running arm debug launch."""

import argparse
import math
import readline
import shlex
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger

from fruit_arm_interfaces.srv import MoveArmCartesian
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
    'status    Show the latest controller status and measured joint angles\n'
    'stop      Request a controlled stop\n'
    'estop     Request an emergency stop\n'
    'level [info|debug|trace]  Show or change console log detail\n'
    'help      Show this command list\n'
    'quit      Exit when no move is pending (Ctrl+C sends ESTOP)\n'
    'Coordinates are metres in the fixed arm-base frame. '
    'PITCH_DEG is absolute; DURATION_MS=0 uses the planner default.\n'
    'Example: move 0.20 0.0 0.25 0 2500 ptp hold\n'
    'debug prints every 20 Hz status and ROS Header/Mode; '
    'trace also prints every ROS trajectory point.'
)


class ArmConsole(Node):
    def __init__(self, log_level):
        super().__init__('arm_cartesian_debug_console')
        self.log_level = log_level
        self._output_lock = threading.Lock()
        self.move_client = self.create_client(MoveArmCartesian, '/arm_planner_node/move_cartesian')
        self.stop_client = self.create_client(Trigger, '/arm_planner_node/stop')
        self.estop_client = self.create_client(Trigger, '/arm_planner_node/emergency_stop')
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
        print(f'Status: {STATUS_NAMES.get(msg.task_status, msg.task_status)}, '
              f'trajectory={msg.trajectory_id}, progress={msg.progress:.2f}, '
              f'error={msg.error_code}, age={age:.2f}s')
        print(f'Joints (deg): [{angles}], gripper={msg.gripper_state}')

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

        if not self.move_client.service_is_ready():
            print('Planner move_cartesian service is unavailable.')
            return
        with self._lock:
            if self._move_future is not None:
                print('A move is still pending. Use stop or estop to interrupt it.')
                return
            request = MoveArmCartesian.Request()
            request.x, request.y, request.z, request.pitch_deg = values
            request.duration_ms = duration
            request.linear_path = mode == 'lin'
            request.gripper_action = GRIPPER_ACTIONS[gripper]
            future = self.move_client.call_async(request)
            self._move_future = future
        future.add_done_callback(self._on_move_done)
        print(f'Submitted {mode.upper()} target {values[:3]} m, pitch={values[3]} deg, '
              f'duration={duration or "default"} ms, gripper={gripper}.')

    def _on_move_done(self, future):
        try:
            result = future.result()
            print(f'\nMove result: success={result.success}, '
                  f'trajectory_id={result.trajectory_id}, message={result.message}', flush=True)
        except Exception as exc:
            print(f'\nMove service failed: {exc}', flush=True)
        finally:
            with self._lock:
                if self._move_future is future:
                    self._move_future = None

    def control(self, name):
        client = self.stop_client if name == 'stop' else self.estop_client
        if not client.service_is_ready():
            print(f'{name} service is unavailable.')
            return None
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda done: self._on_control_done(name, done))
        print(f'{name.upper()} requested; wait for status to confirm the hardware state.')
        return future

    @staticmethod
    def _on_control_done(name, future):
        try:
            result = future.result()
            print(f'\n{name.upper()} service: success={result.success}, '
                  f'message={result.message}', flush=True)
        except Exception as exc:
            print(f'\n{name.upper()} service failed: {exc}', flush=True)

    def move_pending(self):
        with self._lock:
            return self._move_future is not None


def main():
    parser = argparse.ArgumentParser(description='Manual Cartesian arm debug console')
    parser.add_argument('--log-level', choices=LOG_LEVELS, default='info',
                        help='Initial console log detail (default: info)')
    options = parser.parse_args()
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = ArmConsole(options.log_level)
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
