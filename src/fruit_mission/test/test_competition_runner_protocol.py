"""Localhost DDS and process protocol checks. Never start hardware or motion requests."""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import yaml

from fruit_mission import competition_runner
from fruit_robviz_bridge.process_groups import process_groups_alive, stop_process_groups


class CompetitionRunnerProtocolTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        self.status_before = competition_runner.STATUS
        competition_runner.STATUS = self.root / 'status.json'
        self.service_before = competition_runner.START_SERVICE
        competition_runner.START_SERVICE = f'/offline_{self.root.name}/start_localization'
        self.session_before = os.environ.get('FRUIT_RUNTIME_SESSION_DIR')
        os.environ['FRUIT_RUNTIME_SESSION_DIR'] = str(self.root)
        self.localhost_before = os.environ.get('ROS_LOCALHOST_ONLY')
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        rclpy.init(domain_id=201, signal_handler_options=SignalHandlerOptions.NO)
        options = SimpleNamespace(phase='prepare', flow='acb', open_loop='none', debug=False)
        self.runner = competition_runner.CompetitionRunner(options)
        self.processes = []

    def tearDown(self):
        if self.processes:
            stop_process_groups({process.pid for process in self.processes}, 0, 0, 1)
        for process in self.processes:
            process.wait(timeout=2)
        self.runner.destroy_node()
        rclpy.try_shutdown()
        competition_runner.STATUS = self.status_before
        competition_runner.START_SERVICE = self.service_before
        for name, value in [('FRUIT_RUNTIME_SESSION_DIR', self.session_before),
                            ('ROS_LOCALHOST_ONLY', self.localhost_before)]:
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value
        self.directory.cleanup()

    def test_close_after_ros_shutdown_still_cleans_and_writes_status(self):
        process = subprocess.Popen(['/bin/sleep', '60'], start_new_session=True)
        self.processes.append(process)
        self.runner.processes['fixture'] = process
        rclpy.shutdown()
        self.runner.close()
        self.assertFalse(process_groups_alive({process.pid}))
        self.assertFalse(self.runner.processes)
        self.assertEqual(json.loads(competition_runner.STATUS.read_text())['phase'], 'STOPPED')

    def test_default_response_wait_has_ten_second_deadline(self):
        future = Future()
        started = time.monotonic()
        self.assertFalse(self.runner.wait_until(future.done))
        elapsed = time.monotonic() - started
        self.assertGreaterEqual(elapsed, 9.9)
        self.assertLess(elapsed, 11.5)
        self.assertFalse(self.runner.wait_until(future.done, 0))

    def test_shutdown_does_not_start_or_retry_localization(self):
        rclpy.shutdown()
        self.runner.localize()
        self.assertFalse(self.runner.processes)
        self.runner.close()
        self.assertEqual(self.runner.state['phase'], 'STOPPED')

    def test_camera_snapshot_without_rgbd_or_duplicate_recorder(self):
        camera = rclpy.create_node('camera')
        recorder = rclpy.create_node('rosbag2_recorder')
        camera.declare_parameter('enable_depth', True)
        camera.declare_parameter('depth_module.depth_profile', '848x480x30')
        image_publisher = camera.create_publisher(Image, '/camera/color/image_raw', 10)
        heartbeat_publisher = camera.create_publisher(
            Header, self.runner.perception_heartbeat_topic, 10)
        executor = SingleThreadedExecutor()
        executor.add_node(camera)
        executor.add_node(recorder)
        thread = threading.Thread(target=executor.spin)
        thread.start()
        domain_before = os.environ.get('ROS_DOMAIN_ID')
        try:
            self.assertTrue(self.runner.wait_until(
                lambda: {'/camera', '/rosbag2_recorder'} <= self.runner.node_names(), 3))
            os.environ['ROS_DOMAIN_ID'] = '201'
            self.runner.options.debug = True
            self.runner.snapshot()
            self.assertNotIn('perception', self.runner.last_data)
            self.assertNotIn('rosbag', self.runner.processes)
            process = self.runner.processes['camera_params']
            self.processes.append(process)
            self.assertEqual(process.wait(timeout=10), 0)
            parameters = yaml.safe_load(
                (self.root / 'camera_params.yaml').read_text())['/camera']['ros__parameters']
            self.assertTrue(parameters['enable_depth'])
            self.assertEqual(parameters['depth_module']['depth_profile'], '848x480x30')
            self.runner.snapshot()
            self.assertIs(self.runner.processes['camera_params'], process)
            self.assertNotIn('camera_params_streaming', self.runner.processes)
            camera.declare_parameter('align_depth.enable', True)
            self.assertTrue(self.runner.wait_until(
                lambda: image_publisher.get_subscription_count() > 0
                and heartbeat_publisher.get_subscription_count() > 0, 3))
            image = Image()
            image.header.stamp = camera.get_clock().now().to_msg()
            image.height, image.width, image.step = 1, 1, 3
            image.encoding = 'rgb8'
            image.data = [0, 0, 0]
            image_publisher.publish(image)
            heartbeat_publisher.publish(image.header)
            self.assertTrue(self.runner.wait_until(
                lambda: {'camera', 'perception'} <= self.runner.last_data.keys(), 3))
            self.runner.snapshot()
            streaming = self.runner.processes['camera_params_streaming']
            self.processes.append(streaming)
            self.assertEqual(streaming.wait(timeout=10), 0)
            parameters = yaml.safe_load((self.root / 'camera_params_streaming.yaml').read_text())
            self.assertTrue(parameters['/camera']['ros__parameters']['align_depth']['enable'])
            records = [json.loads(line) for line in
                       (self.root / 'camera_health.log').read_text().splitlines()]
            self.assertEqual(sum(row['streams']['camera']['messages'] for row in records), 1)
            self.assertEqual(sum(row['streams']['perception']['messages'] for row in records), 1)
            self.assertIsNone(records[-1]['streams']['depth']['last_received_age_s'])
            self.assertIsNotNone(records[-1]['streams']['camera']['header_age_s'])
            self.runner.snapshot()
            self.assertIs(self.runner.processes['camera_params_streaming'], streaming)
        finally:
            if domain_before is None:
                os.environ.pop('ROS_DOMAIN_ID', None)
            else:
                os.environ['ROS_DOMAIN_ID'] = domain_before
            executor.shutdown(timeout_sec=2)
            thread.join(timeout=2)
            camera.destroy_node()
            recorder.destroy_node()

    def test_module_output_reaches_existing_subsystem_logs(self):
        for name, label, logfile in [('prepared_stack', 'arm_planner_node', 'arm_planner.log'),
                                     ('pointlio', 'laserMapping', 'navigation.log'),
                                     ('prepared_stack', 'camera', 'perception.log')]:
            self.runner.start(name, [sys.executable, '-c', f'print("[INFO] [{label}]: fixture")'])
            process = self.runner.processes[name]
            self.processes.append(process)
            self.assertEqual(process.wait(timeout=3), 0)
            self.assertIn('fixture', (self.root / logfile).read_text())

    def test_camera_ready_and_radar_ready(self):
        self.assertFalse(self.runner.camera_ready())
        self.assertFalse(self.runner.radar_ready())

        self.runner.touch('camera')
        self.assertTrue(self.runner.camera_ready())

        self.runner.last_data['camera'] = time.time() - 4.0
        self.assertFalse(self.runner.camera_ready())

        self.runner.touch('radar')
        self.assertFalse(self.runner.radar_ready())

        livox = rclpy.create_node('livox_ros_driver2')
        executor = SingleThreadedExecutor()
        executor.add_node(livox)
        thread = threading.Thread(target=executor.spin)
        thread.start()
        try:
            self.assertTrue(self.runner.wait_until(
                lambda: '/livox_ros_driver2' in self.runner.node_names(), 3))
            self.runner.touch('radar')
            self.assertTrue(self.runner.radar_ready())

            self.runner.last_data['radar'] = time.time() - 4.0
            self.assertFalse(self.runner.radar_ready())
        finally:
            executor.shutdown(timeout_sec=2)
            thread.join(timeout=2)
            livox.destroy_node()

    def test_named_flow_cannot_trigger_different_prepared_flow(self):
        executor = SingleThreadedExecutor()

        def serve():
            try:
                rclpy.spin(self.runner, executor=executor)
            except ExternalShutdownException:
                pass

        thread = threading.Thread(target=serve)
        thread.start()
        try:
            with self.assertRaises(RuntimeError):
                competition_runner.request_start('full')
            with self.assertRaises(RuntimeError):
                competition_runner.request_start('acbd')
            self.assertFalse(self.runner.requested)
            competition_runner.request_start('acb')
            self.assertTrue(self.runner.requested)
            self.runner.requested = False
            competition_runner.request_start()
            self.assertTrue(self.runner.requested)
            self.assertEqual(self.runner.state['parameters']['flow'], 'acb')
        finally:
            rclpy.shutdown()
            thread.join(timeout=2)
            executor.shutdown()


if __name__ == '__main__':
    unittest.main()
