"""Own a prepared competition session and its single localization retry."""

import argparse
import fcntl
import json
import os
from pathlib import Path
import shlex
import subprocess
import time
import uuid

from fruit_interfaces.msg import ArmStatus, ArmTelemetry
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from tf2_ros import TransformException

from fruit_mission.camera_target_planner_client import CameraTargetPlannerClient
from fruit_mission.competition_protocol import localization_sample_ready, module_health
from fruit_mission.script_hardware_client import latched_qos
from fruit_mission.stack_readiness_client import StackReadinessClient
from fruit_mission.waypoint_route import MissionWaypointRoute
from fruit_robviz_bridge.process_groups import process_groups_alive, stop_process_groups


WORKSPACE = Path('/home/rc4/fruit_ws')
STATUS = WORKSPACE / 'tmp/fruit_competition/status.json'
START_SERVICE = '/fruit_competition/start_localization'
NAV_LAUNCH = WORKSPACE / 'src/navigation/fruit_nav_bringup/launch/pointlio_odometry.launch.py'


class CompetitionRunner(StackReadinessClient):
    def __init__(self, options):
        super().__init__()
        self.options = options
        self.processes = {}
        self.process_groups = {}
        self.last_data = {}
        self.stream_counts = {}
        self.stream_stamps = {}
        self.stream_log_at = time.monotonic()
        self.odometry = None
        self.started_ns = 0
        self.arm_fault = ''
        self.recording_checked = False
        self.requested = options.phase == 'all'
        self.state = {'run_id': str(uuid.uuid4()), 'pid': os.getpid(), 'phase': 'PREPARING',
                      'parameters': {'flow': options.flow, 'open_loop': options.open_loop,
                                     'debug': options.debug}, 'home': None, 'error': '',
                      'execution': {}, 'modules': {}}
        self.log_dir = Path(os.environ['FRUIT_RUNTIME_SESSION_DIR'])
        self.create_service(Trigger, START_SERVICE, self.request_localization)
        self.create_service(Trigger, f'{START_SERVICE}/{options.flow}', self.request_localization)
        for key, topic, message_type in (
            ('camera', '/camera/color/image_raw', Image),
            ('annotated', '/fruit_perception/camera/annotated_image', Image),
            ('depth', '/camera/depth/image_rect_raw', Image),
            ('perception', self.perception_heartbeat_topic, Header),
            ('radar', '/livox/imu', Imu),
            ('pointlio', '/aft_mapped_to_init', Odometry),
        ):
            self.create_subscription(message_type, topic,
                                     lambda msg, k=key: self.touch(k, msg),
                                     qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry', self.on_odometry, 10)
        self.create_subscription(ArmTelemetry, '/arm_planner_node/joint_states',
                                 lambda msg: self.touch('arm'), 10)
        self.create_subscription(ArmStatus, '/fruit_hardware_bridge/arm/status',
                                 self.on_arm_status, 10)
        self.create_subscription(String, '/fruit_mission/status', self.on_execution, latched_qos())
        self.create_timer(1.0, self.snapshot)

    def wait_until(self, predicate, timeout_sec=None):
        timeout_sec = 10.0 if timeout_sec is None else min(timeout_sec, 10.0)
        return super().wait_until(predicate, timeout_sec)

    def touch(self, key, message=None):
        self.last_data[key] = time.time()
        if self.options.debug:
            self.stream_counts[key] = self.stream_counts.get(key, 0) + 1
            if message is not None:
                stamp = message.stamp if isinstance(message, Header) else message.header.stamp
                self.stream_stamps[key] = stamp.sec * 1_000_000_000 + stamp.nanosec

    def on_odometry(self, message):
        self.odometry = message
        self.touch('odometry')

    def on_arm_status(self, message):
        self.touch('serial')
        self.arm_fault = (f'task_status={message.task_status}, error_code={message.error_code}'
                          if message.error_code or message.task_status in (
                              ArmStatus.STATUS_FAILED, ArmStatus.STATUS_EMERGENCY) else '')

    def on_execution(self, message):
        # Ignore transient-local status left by a previous mission.
        if 'mission' not in self.processes:
            return
        try:
            data = json.loads(message.data)
            if isinstance(data, dict):
                self.state['execution'] = data
        except (ValueError, TypeError) as error:
            self.get_logger().warning(f'Invalid mission status: {error}')

    def request_localization(self, request, response):
        if self.state['phase'] not in ('PREPARING', 'PREPARED', 'LOCALIZING', 'RUNNING'):
            response.success = False
            response.message = '本轮已结束或定位失败，请 Kill 后重新准备'
        else:
            self.requested = True
            response.success = True
            response.message = '已请求定位，沿用准备阶段参数；重复请求不会重启定位'
        return response

    def start(self, name, command):
        self.get_logger().info(f'START {name}: {command}')
        env = dict(os.environ)
        env.update(FRUIT_RECORDING_WORKER='1', FRUIT_RECORDING_PROFILE='mission')
        filter_args = ['--output-file', str(self.log_dir / f'{name}.log'), '--component', name]
        if self.options.debug:
            filter_args.append('--debug')
        if name in ('mission', 'prepared_stack', 'pointlio', 'loam'):
            filter_args.extend(['--stack-log-dir', str(self.log_dir)])
        # Fixed pipeline, command tokens are arguments rather than browser shell input.
        filter_command = [
            '/usr/bin/python3', str(WORKSPACE / 'scripts/runtime_log_filter.py'), *filter_args]
        self.processes[name] = subprocess.Popen(
            ['bash', '-c', f'set -o pipefail; {shlex.join(command)} 2>&1 | '
             f'{shlex.join(filter_command)}'],
            env=env, start_new_session=True,
            stdout=subprocess.DEVNULL if name in (
                'system_load', 'process_load', 'usb_topology') else None)
        self.process_groups[name] = {self.processes[name].pid}

    def stop(self, name):
        process = self.processes.get(name)
        if process is None:
            return
        groups = self.process_groups.setdefault(name, {process.pid})
        if not stop_process_groups(groups):
            raise RuntimeError(f'{name} process group did not exit; restart is blocked')
        process.wait(timeout=1.0)
        self.processes.pop(name, None)
        self.process_groups.pop(name, None)

    def start_localization_process(self, name):
        self.start(name, ['ros2', 'launch', str(NAV_LAUNCH), 'start_lidar:=false',
                          f'start_pointlio:={str(name == "pointlio").lower()}',
                          'start_loam:=true' if name == 'loam' else 'start_loam:=false',
                          'start_sensor_frames:=false',
                          'initial_stability_timeout_sec:=10.0'])

    def localization_ready(self):
        message = self.odometry
        if (message is None or message.header.frame_id != 'odom'
                or message.child_frame_id != 'base_link'):
            return False
        pose = message.pose.pose
        stamp = message.header.stamp
        now = self.get_clock().now().nanoseconds
        values = (pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        if not localization_sample_ready(stamp.sec * 1_000_000_000 + stamp.nanosec,
                                         now, self.started_ns, values):
            return False
        try:
            transform = self._tf_buffer.lookup_transform('odom', 'base_link', Time())
        except TransformException:
            return False
        stamp = transform.header.stamp
        tf_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        return tf_ns >= self.started_ns and 0 <= now - tf_ns <= 1_000_000_000

    def localize(self):
        self.state['phase'] = 'LOCALIZING'
        for attempt in (1, 2):
            if not rclpy.ok(context=self.context):
                return
            self.state['attempt'] = attempt
            self.started_ns = self.get_clock().now().nanoseconds
            self.odometry = None
            self.start_localization_process('pointlio')
            if self.wait_until(self.localization_ready, 10.0):
                self.state['phase'] = 'RUNNING'
                self.start('mission', [
                    'ros2', 'run', 'fruit_mission', 'fruit_mission_node',
                    '--ros-args', '-p', f'mission_flow:={self.options.flow}',
                    '-p', f'waypoint_file:={self.options.waypoint_file}',
                    '-p', f'open_loop:={self.options.open_loop}'])
                return
            if not rclpy.ok(context=self.context):
                return
            self.get_logger().error(f'定位初始化失败或超时，attempt={attempt}')
            self.stop('pointlio')
            if attempt == 1:
                self.stop('loam')
                self.start_localization_process('loam')
        self.state.update(phase='ERROR', error='定位初始化失败，已重启一次，未启动 mission')

    def snapshot(self):
        now = time.time()
        if self.options.debug:
            steady_now = time.monotonic()
            interval = max(steady_now - self.stream_log_at, 1e-6)
            ros_now = self.get_clock().now().nanoseconds
            streams = {}
            for key in ('camera', 'depth', 'annotated', 'perception', 'radar', 'serial'):
                received = self.last_data.get(key)
                stamp = self.stream_stamps.get(key)
                count = self.stream_counts.get(key, 0)
                streams[key] = {
                    'messages': count, 'received_hz': round(count / interval, 3),
                    'last_received_age_s': None if received is None else round(now - received, 3),
                    'header_age_s': None if not stamp else round((ros_now - stamp) / 1e9, 3),
                }
            try:
                with (self.log_dir / 'camera_health.log').open('a', encoding='utf-8') as stream:
                    stream.write(json.dumps({
                        'time': now, 'interval_s': round(interval, 3), 'streams': streams,
                    }) + '\n')
            except OSError as error:
                self.get_logger().warning(f'Cannot write camera health: {error}')
            self.stream_counts.clear()
            self.stream_log_at = steady_now
        nodes = self.node_names()
        services = {name for name, types in self.get_service_names_and_types()}
        modules = {}
        for group, key, label, node_candidates, stream, enabled in (
            ('perception', 'camera', '相机 RGB', ('/camera',), True, True),
            ('perception', 'depth', '相机深度', ('/camera',), True, True),
            ('perception', 'annotated', '标注画面',
             ('/fruit_perception_node', '/fruit_perception_container'), True, True),
            ('perception', 'perception', '感知 RGBD 心跳',
             ('/fruit_perception_node', '/fruit_perception_container'), True, True),
            ('perception', 'qr', '二维码', ('/fruit_qr_planner',), False, True),
            ('arm', 'serial', '串口桥',
             ('/serial_controller', '/auto_serial_bridge_container'), True, True),
            ('arm', 'arm', '机械臂 planner', ('/arm_planner_node',), True, True),
            ('navigation', 'radar', '雷达 IMU', ('/livox_ros_driver2',), True, True),
            ('navigation', 'pointlio', 'Point-LIO',
             ('/laserMapping',), True, bool(self.started_ns)),
            ('navigation', 'odometry', '里程计', ('/loam_interface',), True, bool(self.started_ns)),
            ('navigation', 'navigation', '航点导航', ('/axis_waypoint_action_server',), False, True),
            ('mission', 'mission', 'fruit_mission', ('/fruit_mission',), False,
             self.state['phase'] == 'RUNNING'),
        ):
            status = module_health(any(node in nodes for node in node_candidates),
                                   self.last_data.get(key), now, stream, enabled)
            detail = {'OK': '正常', 'MISSING': '节点不在运行', 'NO_DATA': '超过 3 秒无新数据',
                      'DISABLED': '尚未启用或本轮已结束'}[status]
            if key == 'serial' and status == 'OK' and not self._bridge_ready:
                status, detail = 'ERROR', '串口桥 ready=false'
            if key == 'arm' and status == 'OK':
                if self.arm_fault:
                    status, detail = 'ERROR', self.arm_fault
                elif '/arm_planner_node/execute_preset' not in services:
                    status, detail = 'ERROR', 'planner 服务缺失'
            modules.setdefault(group, []).append({
                'id': key, 'label': label, 'state': status,
                'detail': detail, 'last_received': self.last_data.get(key)})
        try:
            transform = self._tf_buffer.lookup_transform('odom', 'base_link', Time())
            stamp = transform.header.stamp
            age = (self.get_clock().now().nanoseconds
                   - stamp.sec * 1_000_000_000 - stamp.nanosec) / 1e9
            tf_state = 'OK' if 0 <= age <= 3 else 'NO_DATA'
        except TransformException:
            tf_state = 'NO_DATA'
        if not self.started_ns:
            tf_state = 'DISABLED'
        modules['navigation'].append({
            'id': 'tf', 'label': 'odom → base_link TF', 'state': tf_state,
            'detail': '新鲜 TF' if tf_state == 'OK' else '无本轮有效 TF',
            'last_received': None})
        mission = self.processes.get('mission')
        if (mission and mission.poll() is not None
                and not process_groups_alive(self.process_groups.get('mission', {mission.pid}))
                and self.state['phase'] == 'RUNNING'):
            self.state.update(
                phase='FINISHED' if mission.returncode == 0 else 'ERROR',
                error='' if mission.returncode == 0 else f'mission 退出码 {mission.returncode}')
        if (self.options.debug and '/camera' in nodes
                and self.state['phase'] not in ('ERROR', 'FINISHED', 'STOPPED')):
            if not self.recording_checked:
                self.recording_checked = True
                if '/rosbag2_recorder' not in nodes:
                    self.start('rosbag', [str(WORKSPACE / 'record_rosbag.sh')])
            for name, ready in (
                ('camera_params', True),
                ('camera_params_streaming',
                 'camera' in self.last_data or 'depth' in self.last_data),
            ):
                if ready and name not in self.processes:
                    self.start(name, [
                        'bash', '-c',
                        'if timeout --kill-after=1s 9s '
                        'ros2 param dump /camera --no-daemon > "$1"; '
                        'then printf "[camera_params] saved: %s\\n" "$1"; else rm -f -- "$1"; '
                        'printf "[camera_params] WARNING: snapshot failed\\n" >&2; fi',
                        'camera-params', str(self.log_dir / f'{name}.yaml')])
        self.state.update(updated_at=now, modules=modules)
        self.write_snapshot()

    def write_snapshot(self):
        STATUS.parent.mkdir(parents=True, exist_ok=True)
        temporary = STATUS.with_suffix('.tmp')
        try:
            temporary.write_text(json.dumps(self.state, ensure_ascii=False), encoding='utf-8')
            temporary.replace(STATUS)
        except OSError as error:
            self.get_logger().error(f'Cannot write competition status: {error}')

    def camera_ready(self):
        now = time.time()
        camera_time = self.last_data.get('camera')
        return camera_time is not None and (now - camera_time <= 3.0)

    def radar_ready(self):
        now = time.time()
        radar_time = self.last_data.get('radar')
        return (
            radar_time is not None
            and (now - radar_time <= 3.0)
            and any(node in self.node_names() for node in ('/livox_ros_driver2',))
        )

    def wait_for_camera(self):
        self.get_logger().info('等待摄像头启动完成...')
        last_log = time.monotonic()
        while rclpy.ok(context=self.context):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.camera_ready():
                self.snapshot()
                self.get_logger().info('摄像头已启动完成，请手动打开雷达电源...')
                return True
            if time.monotonic() - last_log >= 3.0:
                self.get_logger().info('正在等待摄像头启动...')
                last_log = time.monotonic()
        return False

    def wait_for_radar(self):
        self.get_logger().info('正在持续观察雷达，等待雷达启动并显示正常...')
        last_log = time.monotonic()
        while rclpy.ok(context=self.context):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.radar_ready():
                self.snapshot()
                self.get_logger().info('雷达已启动并显示正常！')
                return True
            if time.monotonic() - last_log >= 3.0:
                self.get_logger().info('持续等待雷达启动中（请手动开启雷达电源）...')
                last_log = time.monotonic()
        return False

    def prepare(self):
        start_bridge = self.robviz_bridge_start_required()
        launch_name = ('fruit_mission_acb.launch.py' if self.options.flow == 'acb'
                       else 'fruit_mission.launch.py')
        self.start('prepared_stack', [
            'ros2', 'launch', str(WORKSPACE / 'src/fruit_mission/launch' / launch_name),
            'prepare_only:=true', 'start_lidar:=true', 'start_pointlio:=false',
            'start_loam:=false', 'start_sensor_frames:=true',
            f'mission_flow:={self.options.flow}', f'open_loop:={self.options.open_loop}',
            f'waypoint_file:={self.options.waypoint_file}',
            f'start_robviz_bridge:={str(start_bridge).lower()}',
            f'serial_log_level:={"debug" if self.options.debug else "info"}'])
        self.start_localization_process('loam')
        if self.options.debug:
            self.start('udev', ['udevadm', 'monitor', '--kernel', '--udev'])
            self.start('kernel', ['journalctl', '-k', '-f', '-o', 'short-iso'])
            self.start('system_load', ['vmstat', '-w', '-t', '1'])
            self.start('process_load', [
                'pidstat', '-h', '-u', '-r', '-d', '-C',
                'component_cont|livox|pointlio|python3|serial_node', '1'])
            self.start('usb_topology', ['lsusb', '-t'])
        if not self.wait_for_camera():
            return
        if not self.wait_for_radar():
            return
        try:
            planner = CameraTargetPlannerClient(self)
            # Wait only during preparation, never repeat this gate on the field.
            self.wait_until(lambda: planner._preset_client.service_is_ready()
                            and planner._fresh_pose() is not None, 10.0)
            completed, reason = planner.execute_preset('home', self.wait_until)
            self.state['home'] = {'success': completed, 'reason': reason}
        except Exception as error:
            self.state['home'] = {'success': False, 'reason': str(error)}
            self.get_logger().error(f'Preparation Home failed: {error}')
        self.state['phase'] = 'PREPARED'
        self.snapshot()

    def close(self):
        # Request STOP, but never claim it proves hardware standstill.
        try:
            if rclpy.ok(context=self.context):
                stop = self.create_client(Trigger, '/arm_planner_node/stop')
                if stop.service_is_ready():
                    future = stop.call_async(Trigger.Request())
                    self.wait_until(future.done, 2.0)
            else:
                self.get_logger().warning('ROS context 已关闭，STOP 未发送，硬件停稳未确认')
        except Exception as error:
            self.get_logger().warning(f'STOP unconfirmed: {error}')
        finally:
            # Cleanup must not depend on a valid ROS context or a pipeline leader.
            groups = {process.pid for process in self.processes.values()}
            for owned in self.process_groups.values():
                groups.update(owned)
            stopped = False
            try:
                stopped = stop_process_groups(groups) if groups else True
                for process in self.processes.values():
                    if stopped:
                        process.wait(timeout=1.0)
                    else:
                        process.poll()
            except Exception as error:
                stopped = False
                self.get_logger().warning(f'Process cleanup unconfirmed: {error}')
            if stopped:
                self.processes.clear()
                self.process_groups.clear()
            self.state['phase'] = 'STOPPED' if stopped else 'ERROR'
            if not stopped:
                self.state['error'] = '子进程停止未确认，请检查进程和硬件状态'
            self.state['updated_at'] = time.time()
            for modules in self.state['modules'].values():
                for module in modules:
                    module.update(state=self.state['phase'], detail='停稳状态未确认')
            self.write_snapshot()


def request_start(expected_flow=None):
    node = Node('competition_start_client')
    try:
        service = START_SERVICE if expected_flow is None else f'{START_SERVICE}/{expected_flow}'
        client = node.create_client(Trigger, service)
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError(f'准备会话不存在或流程不匹配: {expected_flow or "当前会话"}')
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        if not future.done():
            raise RuntimeError('定位启动请求超时，结果未知，请查看比赛状态')
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(response.message if response else '定位启动无应答')
        node.get_logger().info(response.message)
    finally:
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--phase', choices=('all', 'prepare', 'localize'), default='all')
    parser.add_argument('--flow', choices=('full', 'ac', 'acb', 'a', 'b', 'c'))
    parser.add_argument('--open-loop', choices=('A', 'C', 'AC', 'none'), default='A')
    parser.add_argument('--debug', action='store_true')
    options = parser.parse_args()
    if options.phase == 'localize':
        rclpy.init()
        try:
            request_start(options.flow)
        finally:
            rclpy.try_shutdown()
        return
    options.flow = options.flow or 'full'
    if options.flow == 'acb':
        options.open_loop = 'none'
    filename = 'odom_waypoints_acb.yaml' if options.flow == 'acb' else 'odom_waypoints.yaml'
    options.waypoint_file = WORKSPACE / 'src/navigation/fruit_nav_bringup/waypoints' / filename
    STATUS.parent.mkdir(parents=True, exist_ok=True)
    with (STATUS.parent / 'session.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('比赛会话已存在，请使用 --phase localize 或 Kill')
        MissionWaypointRoute.from_file(options.waypoint_file,
                                       options.flow, 'odom')
        subprocess.run(['bash', str(WORKSPACE / 'kill_fruit_processes.sh'), '--competition'],
                       timeout=10, check=True)
        rclpy.init()
        runner = CompetitionRunner(options)
        try:
            runner.snapshot()
            if not runner.nodes_are_absent([
                '/fruit_mission', '/laserMapping', '/loam_interface',
                '/serial_controller', '/auto_serial_bridge_container',
                '/arm_planner_node', '/camera', '/fruit_perception_container',
                '/fruit_perception_node', '/fruit_qr_planner', '/livox_ros_driver2',
                '/axis_waypoint_action_server', '/twist_mux',
            ]):
                raise RuntimeError('已有硬件节点，请先清理旧任务')
            runner.prepare()
            while rclpy.ok():
                if runner.requested and runner.state['phase'] == 'PREPARED':
                    runner.localize()
                rclpy.spin_once(runner, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        finally:
            try:
                runner.close()
            finally:
                runner.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()


if __name__ == '__main__':
    main()
