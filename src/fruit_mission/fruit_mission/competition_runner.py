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

from composition_interfaces.srv import ListNodes, LoadNode, UnloadNode
from fruit_interfaces.msg import ArmStatus, ArmTelemetry
from nav_msgs.msg import Odometry
import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from tf2_ros import TransformException
import yaml

from fruit_mission.camera_target_planner_client import CameraTargetPlannerClient
from fruit_mission.competition_protocol import localization_sample_ready, module_health
from fruit_mission.script_hardware_client import latched_qos
from fruit_mission.stack_readiness_client import StackReadinessClient
from fruit_mission.waypoint_route import MissionWaypointRoute
from fruit_robviz_bridge.process_groups import process_groups_alive, stop_process_groups


WORKSPACE = Path('/home/rc4/fruit_ws')
STATUS = WORKSPACE / 'tmp/fruit_competition/status.json'
CLIENT_CONFIG = WORKSPACE / 'src/fruit_mission/config/script_ros_clients.yaml'
START_SERVICE = '/fruit_competition/start_localization'
NAV_LAUNCH = WORKSPACE / 'src/navigation/fruit_nav_bringup/launch/pointlio_odometry.launch.py'


class CompetitionRunner(StackReadinessClient):
    def __init__(self, options):
        super().__init__()
        self.options = options
        self.processes = {}
        self.process_groups = {}
        self.last_data = {}
        self.last_data_monotonic = {}
        self.odometry = None
        self.started_ns = 0
        self.arm_fault = ''
        self.recording_checked = False
        self.readiness_config = self._load_readiness_config()
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
                                     lambda msg, k=key: self.touch(k), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry', self.on_odometry, 10)
        self.create_subscription(ArmTelemetry, '/arm_planner_node/joint_states',
                                 lambda msg: self.touch('arm'), 10)
        self.create_subscription(ArmStatus, '/fruit_hardware_bridge/arm/status',
                                 self.on_arm_status, 10)
        self.create_subscription(String, '/fruit_mission/status', self.on_execution, latched_qos())
        self.create_timer(1.0, self.snapshot)

    def wait_until(self, predicate, timeout_sec=None):
        timeout_sec = 10.0 if timeout_sec is None else min(timeout_sec, 30.0)
        return super().wait_until(predicate, timeout_sec)

    def touch(self, key):
        self.last_data[key] = time.time()
        self.last_data_monotonic[key] = time.monotonic()

    @staticmethod
    def _load_readiness_config():
        document = yaml.safe_load(CLIENT_CONFIG.read_text(encoding='utf-8')) or {}
        return dict(document.get('/**', {}).get('ros__parameters', {}))

    def wait_for_camera_stream(self):
        timeout_sec = float(self.readiness_config.get(
            'perception_readiness_timeout_sec', 30.0))
        stable_sec = float(self.readiness_config.get(
            'perception_stable_duration_sec', 3.0))
        max_gap_sec = float(self.readiness_config.get(
            'perception_stream_max_gap_sec', 1.0))
        attempt_started = time.monotonic()
        stable_started = None

        def camera_is_stable():
            nonlocal stable_started
            now = time.monotonic()
            if not self.camera_streams_fresh(max_gap_sec, now):
                stable_started = None
                return False
            stamps = [self.last_data_monotonic.get(key, 0.0)
                      for key in ('camera', 'depth', 'perception')]
            if any(stamp < attempt_started
                   for stamp in stamps):
                stable_started = None
                return False
            if stable_started is None:
                stable_started = now
            return now - stable_started >= stable_sec

        ready = self.wait_until(camera_is_stable, timeout_sec)
        if not ready:
            self.get_logger().error(
                f'Camera streams were not stable within {timeout_sec:.0f}s')
        return ready

    def camera_streams_fresh(self, max_gap_sec, now=None):
        now = time.monotonic() if now is None else now
        if not {'/camera', '/fruit_perception_node'}.issubset(self.node_names()):
            return False
        return all(
            now - self.last_data_monotonic.get(key, 0.0) <= max_gap_sec
            for key in ('camera', 'depth', 'perception')
        )

    def _component_camera_parameters(self, initial_reset):
        document = yaml.safe_load(
            (WORKSPACE / 'src/fruit_perception/config/fruit_perception.yaml').read_text(
                encoding='utf-8'))
        values = dict(document['camera']['ros__parameters'])
        values['initial_reset'] = initial_reset
        values['use_sim_time'] = False
        return [Parameter(name=name, value=value).to_parameter_msg()
                for name, value in values.items()]

    def restart_camera_component(self):
        container = '/fruit_perception_container'
        list_client = self.create_client(ListNodes, f'{container}/_container/list_nodes')
        unload_client = self.create_client(
            UnloadNode, f'{container}/_container/unload_node')
        load_client = self.create_client(LoadNode, f'{container}/_container/load_node')
        try:
            if not list_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('Camera component list service is unavailable')
                return False
            list_future = list_client.call_async(ListNodes.Request())
            if not self.wait_until(list_future.done, 5.0):
                self.get_logger().error('Camera component list request timed out')
                return False
            listed = list_future.result()
            camera_ids = [uid for name, uid in zip(
                listed.full_node_names, listed.unique_ids) if name == '/camera']
            if len(camera_ids) != 1:
                self.get_logger().error(
                    f'Expected one /camera component, found {len(camera_ids)}')
                return False
            if not unload_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('Camera component unload service is unavailable')
                return False
            unload_future = unload_client.call_async(
                UnloadNode.Request(unique_id=camera_ids[0]))
            if not self.wait_until(unload_future.done, 5.0):
                self.get_logger().error('Camera component unload timed out')
                return False
            if not unload_future.result().success:
                self.get_logger().error(
                    f'Camera component unload failed: {unload_future.result().error_message}')
                return False
            if not load_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('Camera component load service is unavailable')
                return False
            request = LoadNode.Request(
                package_name='realsense2_camera',
                plugin_name='realsense2_camera::RealSenseNodeFactory',
                node_name='camera',
                node_namespace='/',
                parameters=self._component_camera_parameters(initial_reset=True),
                extra_arguments=[Parameter(
                    name='use_intra_process_comms', value=True).to_parameter_msg()],
            )
            load_future = load_client.call_async(request)
            if not self.wait_until(load_future.done, 10.0):
                self.get_logger().error('Camera component load timed out')
                return False
            response = load_future.result()
            if not response.success:
                self.get_logger().error(
                    f'Camera component load failed: {response.error_message}')
                return False
            self.get_logger().warning('Camera component restarted with one hardware reset')
            return True
        finally:
            self.destroy_client(list_client)
            self.destroy_client(unload_client)
            self.destroy_client(load_client)

    def wait_for_prepare_modules(self):
        required_nodes = [
            '/camera', '/fruit_perception_node', '/fruit_qr_planner',
            '/serial_controller', '/arm_planner_node', '/livox_ros_driver2',
            '/axis_waypoint_action_server', '/twist_mux',
        ]
        if not self.wait_for_nodes(required_nodes, 10.0):
            return False
        if not self.wait_for_action(self.follow_waypoints_action, 10.0):
            return False
        if not self.wait_for_planner(10.0):
            return False
        if not self.wait_for_bridge(10.0):
            return False
        for key in ('radar', 'serial', 'arm'):
            if not self.wait_until(
                    lambda key=key: key in self.last_data_monotonic, 10.0):
                self.get_logger().error(f'No fresh prepare stream: {key}')
                return False
        return True

    def wait_for_loam(self):
        if not self.wait_for_nodes(['/loam_interface'], 10.0):
            return False
        if not self.wait_until(lambda: 'odometry' in self.last_data_monotonic, 10.0):
            self.get_logger().error('No fresh prepare stream: odometry')
            return False
        if not self.wait_for_transform('odom', 'base_link', 5.0, 1.0):
            return False
        return True

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
            env=env, start_new_session=True)
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
        if (self.options.debug and not self.recording_checked and self.last_data.get('perception')
                and self.state['phase'] not in ('ERROR', 'FINISHED', 'STOPPED')):
            self.recording_checked = True
            if '/rosbag2_recorder' not in nodes:
                self.start('rosbag', [str(WORKSPACE / 'record_rosbag.sh')])
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
        camera_retry_used = False
        if not self.wait_for_camera_stream():
            camera_retry_used = True
            self.get_logger().warning(
                'Camera startup timed out; restarting the camera component once')
            if not self.restart_camera_component() or not self.wait_for_camera_stream():
                self.state.update(phase='ERROR', error='camera readiness failed after one retry')
                self.write_snapshot()
                raise RuntimeError('camera readiness failed after one retry')
        if not self.wait_for_loam():
            self.get_logger().warning('Loam readiness failed; restarting the Loam process once')
            self.stop('loam')
            self.start_localization_process('loam')
            if not self.wait_for_loam():
                self.state.update(phase='ERROR', error='Loam readiness failed after one retry')
                self.write_snapshot()
                raise RuntimeError('Loam readiness failed after one retry')
        if not self.wait_for_prepare_modules():
            self.state.update(phase='ERROR', error='prepare module readiness failed')
            self.write_snapshot()
            raise RuntimeError('prepare module readiness failed')
        if not self.camera_streams_fresh(
                float(self.readiness_config.get('perception_stream_max_gap_sec', 1.0))):
            if camera_retry_used:
                self.state.update(phase='ERROR', error='camera stream stopped during prepare')
                self.write_snapshot()
                raise RuntimeError('camera stream stopped during prepare')
            camera_retry_used = True
            if (not self.restart_camera_component()
                    or not self.wait_for_camera_stream()
                    or not self.wait_for_prepare_modules()):
                self.state.update(phase='ERROR', error='camera readiness failed after one retry')
                self.write_snapshot()
                raise RuntimeError('camera readiness failed after one retry')
        try:
            planner = CameraTargetPlannerClient(self)
            if not self.wait_until(lambda: planner._preset_client.service_is_ready()
                                   and planner._fresh_pose() is not None, 10.0):
                raise RuntimeError('planner readiness failed during prepare')
            completed, reason = planner.execute_preset('home', self.wait_until)
            self.state['home'] = {'success': completed, 'reason': reason}
            if not completed:
                raise RuntimeError(f'Preparation Home failed: {reason}')
        except Exception as error:
            self.state['home'] = {'success': False, 'reason': str(error)}
            self.get_logger().error(f'Preparation Home failed: {error}')
            self.state.update(phase='ERROR', error=f'prepare failed: {error}')
            self.write_snapshot()
            raise RuntimeError(f'prepare failed: {error}') from error
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
