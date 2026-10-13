"""Run against temporary scripts only. No ROS graph or hardware is required."""

import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from urllib.parse import urlencode

import yaml

from fruit_robviz_bridge.runtime_manager import RuntimeManager
from fruit_robviz_bridge.runtime_server import RuntimeHTTPServer


class RuntimeProtocolTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        setup = self.root / 'setup.bash'
        setup.write_text('', encoding='utf-8')
        (self.root / 'sample.sh').write_text(
            "printf '\\033[32m中文测试\\033[0m\\n'\n"
            'printf "%s\\n" "$@"\n', encoding='utf-8')
        (self.root / 'kill_fruit_processes.sh').write_text(
            'echo "safe temporary cleanup"\nexit 0\n', encoding='utf-8')
        config = self.root / 'tasks.yaml'
        config.write_text(yaml.safe_dump({'tasks': {'sample': {
            'command': '/bin/false', 'script': 'sample.sh',
            'parameters': {
                'mode': {'flag': '--mode', 'choices': ['one', 'two'], 'required': True},
                'debug': {'flag': '--debug', 'default': False},
                'model': {'flag': 'model:=', 'choices': ['edge', 'yolo'], 'default': 'edge'},
            },
        }}}), encoding='utf-8')
        self.manager = RuntimeManager(
            config, setup, self.root, self.root / 'logs', self.root / 'control.lock')
        self.server = RuntimeHTTPServer('127.0.0.1', 0, self.manager)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.base = f'http://127.0.0.1:{self.server.server_port}/api/runtime/'

    def tearDown(self):
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            current = self.manager.scripts()['current']
            if current is None or current['state'] == 'EXITED':
                break
            time.sleep(0.01)
        if current and current['state'] == 'RUNNING':
            self.manager.kill_scripts()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()
        self.directory.cleanup()

    def request(self, path, body=None, origin=None):
        headers = {'Content-Type': 'application/json'}
        if origin:
            headers['Origin'] = origin
        request = Request(self.base + path, headers=headers,
                          data=json.dumps(body).encode() if body is not None else None)
        try:
            with urlopen(request, timeout=3) as response:
                return response.status, json.load(response)
        except HTTPError as error:
            return error.code, json.load(error)

    def test_metadata_parameters_and_colored_current_log(self):
        status, data = self.request('scripts')
        self.assertEqual(status, 200)
        self.assertIsNone(data['current'])
        self.assertEqual(data['scripts'][0]['parameters']['mode']['choices'], ['one', 'two'])
        status, first = self.request('scripts/sample/start', {
            'parameters': {'mode': 'two', 'debug': True}})
        self.assertEqual(status, 200)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            status, log = self.request('scripts/log')
            if log['current']['state'] == 'EXITED':
                break
            time.sleep(0.01)
        self.assertEqual(log['current']['exit_code'], 0)
        self.assertTrue(log['reset'])
        self.assertIn('\x1b[32m中文测试\x1b[0m', log['data'])
        self.assertIn('--mode\ntwo\n', log['data'])
        self.assertIn('--debug\n', log['data'])
        self.assertIn('model:=edge\n', log['data'])
        run_id = log['run_id']
        query = urlencode({'run_id': run_id, 'cursor': log['cursor']})
        status, empty = self.request(f'scripts/log?{query}')
        self.assertEqual(empty['data'], '')
        self.assertFalse(empty['reset'])
        # A second HTTP client can restore the same current run after browser refresh.
        self.assertEqual(self.request('scripts')[1]['current']['run_id'], run_id)
        status, second = self.request('scripts/sample/start', {'parameters': {'mode': 'one'}})
        self.assertEqual(status, 200)
        self.assertNotEqual(first['current']['run_id'], second['current']['run_id'])
        query = urlencode({'run_id': run_id, 'cursor': 999999})
        status, reset = self.request(f'scripts/log?{query}')
        self.assertTrue(reset['reset'])
        self.assertNotIn('--mode\ntwo\n', reset['data'])

    def test_reject_invalid_input_before_cleanup(self):
        (self.root / 'kill_fruit_processes.sh').write_text('touch cleanup-ran\n')
        for parameters in ({}, {'mode': '$(touch injected)'}, {'mode': 'one', 'debug': 'true'},
                           {'mode': 'one', 'debug': None},
                           {'mode': 'one', 'command': 'echo unsafe'}, []):
            status, _ = self.request('scripts/sample/start', {'parameters': parameters})
            self.assertEqual(status, 400)
        self.assertFalse((self.root / 'cleanup-ran').exists())
        self.assertEqual(self.request('scripts/log?cursor=-1')[0], 400)
        self.assertEqual(self.request('scripts/log?cursor=wrong')[0], 400)
        self.assertEqual(self.request('scripts/unknown/start', {})[0], 404)

    def test_start_script_without_cleanup_and_failed_cleanup_aborts_kill(self):
        (self.root / 'sample.sh').write_text('touch script-ran\n')
        (self.root / 'kill_fruit_processes.sh').write_text('echo cleanup-failed\nexit 1\n')
        status, reply = self.request('scripts/sample/start', {'parameters': {'mode': 'one'}})
        self.assertEqual(status, 200)
        deadline = time.monotonic() + 2
        while not (self.root / 'script-ran').exists() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue((self.root / 'script-ran').exists())
        status, reply = self.request('scripts/kill', {})
        self.assertEqual(status, 409)
        self.assertIn('Kill', reply['error'])
        self.assertIn('cleanup-failed', self.request('scripts/log')[1]['data'])

    def test_switch_and_kill_running_temporary_script(self):
        (self.root / 'sample.sh').write_text(
            'echo $$ > script.pid\ntrap "exit 0" TERM\n'
            'while true; do sleep 0.1; done\n')
        (self.root / 'kill_fruit_processes.sh').write_text(
            'if [[ -f script.pid ]]; then\n'
            '  kill -TERM -- -"$(cat script.pid)" 2>/dev/null || true\n'
            '  sleep 0.1\nfi\n')
        try:
            status, first = self.request('scripts/sample/start', {'parameters': {'mode': 'one'}})
            self.assertEqual(status, 200)
            deadline = time.monotonic() + 2
            while not (self.root / 'script.pid').exists() and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertTrue((self.root / 'script.pid').exists())
            status, second = self.request('scripts/sample/start', {'parameters': {'mode': 'two'}})
            self.assertEqual(status, 200)
            self.assertNotEqual(first['current']['pid'], second['current']['pid'])
            self.assertEqual(second['current']['state'], 'RUNNING')
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                if (self.root / 'script.pid').read_text().strip() == str(second['current']['pid']):
                    break
                time.sleep(0.01)
            status, stopped = self.request('scripts/kill', {})
            self.assertEqual(status, 200)
            self.assertEqual(stopped['current']['state'], 'EXITED')
            self.assertIn('[Kill] Cleanup completed.', self.request('scripts/log')[1]['data'])
        finally:
            self.request('scripts/kill', {})

    def test_origin_and_body_validation(self):
        self.assertEqual(self.request('scripts/kill', {}, 'http://evil.example:8000')[0], 403)
        self.assertEqual(self.request('scripts/kill', {}, 'http://127.0.0.1:broken')[0], 403)
        self.assertEqual(self.request('scripts/kill', {}, 'http://localhost:8000')[0], 200)
        self.assertEqual(self.request('scripts/kill', {'command': 'arbitrary'})[0], 400)
        status, _ = self.request('scripts/sample/start', {
            'parameters': {'mode': 'one'}, 'command': 'arbitrary'})
        self.assertEqual(status, 400)

    def test_utf8_split_at_chunk_boundary(self):
        self.request('scripts/kill', {})
        log_path = self.root / 'logs' / 'kill.log'
        log_path.write_bytes(b'x' * 65535 + '中文'.encode('utf-8'))
        status, first = self.request('scripts/log')
        self.assertEqual(first['cursor'], 65535)
        self.assertNotIn('\ufffd', first['data'])
        query = urlencode({'run_id': first['run_id'], 'cursor': first['cursor']})
        status, second = self.request(f'scripts/log?{query}')
        self.assertEqual(second['data'], '中文')
        self.assertEqual(second['cursor'], log_path.stat().st_size)

    def test_utf8_tail_restore_and_incomplete_final_byte(self):
        self.request('scripts/kill', {})
        log_path = self.root / 'logs' / 'kill.log'
        log_path.write_bytes('中'.encode('utf-8') * 100000)
        status, tail = self.request('scripts/log')
        self.assertTrue(tail['reset'])
        self.assertNotIn('\ufffd', tail['data'])
        self.assertGreater(tail['cursor'], 300000 - 256 * 1024)
        log_path.write_bytes(b'\xe4')
        status, ended = self.request('scripts/log')
        self.assertEqual(ended['data'], '\ufffd')
        self.assertEqual(ended['cursor'], 1)


if __name__ == '__main__':
    unittest.main()
