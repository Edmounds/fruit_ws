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

    def test_module_output_reaches_existing_subsystem_logs(self):
        for name, label, logfile in [('prepared_stack', 'arm_planner_node', 'arm_planner.log'),
                                     ('pointlio', 'laserMapping', 'navigation.log'),
                                     ('prepared_stack', 'camera', 'perception.log')]:
            self.runner.start(name, [sys.executable, '-c', f'print("[INFO] [{label}]: fixture")'])
            process = self.runner.processes[name]
            self.processes.append(process)
            self.assertEqual(process.wait(timeout=3), 0)
            self.assertIn('fixture', (self.root / logfile).read_text())

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
