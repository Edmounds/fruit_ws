"""Run against temporary scripts only. No ROS graph or hardware is required."""

import json
import os
from pathlib import Path
import signal
import tempfile
import threading
import time
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from urllib.parse import quote, urlencode

import yaml

from fruit_robviz_bridge.runtime_manager import RuntimeManager
from fruit_robviz_bridge.runtime_server import RuntimeHTTPServer
from fruit_robviz_bridge import runtime_server
from fruit_robviz_bridge.process_groups import process_groups_alive


class RuntimeProtocolTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.process_ids = set()
        self.root = Path(self.directory.name)
        self.status_before = runtime_server.COMPETITION_STATUS
        runtime_server.COMPETITION_STATUS = self.root / 'competition.json'
        setup = self.root / 'setup.bash'
        setup.write_text('', encoding='utf-8')
        (self.root / 'sample.sh').write_text(
            "printf '\\033[32m中文测试\\033[0m\\n'\n"
            'printf "%s\\n" "$@"\n', encoding='utf-8')
        (self.root / 'sample2.sh').write_text(
            'trap "exit 0" INT TERM\n'
            'printf "second script running\\n"\n'
            'while true; do sleep 0.05; done\n', encoding='utf-8')
        (self.root / 'kill_fruit_processes.sh').write_text(
            'echo "safe temporary cleanup"\nexit 0\n', encoding='utf-8')
        config = self.root / 'tasks.yaml'
        config.write_text(yaml.safe_dump({'tasks': {
            'sample': {
                'command': '/bin/false', 'script': 'sample.sh',
                'parameters': {
                    'mode': {'flag': '--mode', 'choices': ['one', 'two'], 'required': True},
                    'debug': {'flag': '--debug', 'default': False},
                    'model': {'flag': 'model:=', 'choices': ['edge', 'yolo'], 'default': 'edge'},
                },
            },
            'sample2': {
                'command': '/bin/false', 'script': 'sample2.sh',
                'parameters': {
                    'debug': {'flag': '--debug', 'default': False},
                },
            },
        }}), encoding='utf-8')
        self.manager = RuntimeManager(
            config, setup, self.root, self.root / 'logs', self.root / 'control.lock')
        self.server = RuntimeHTTPServer('127.0.0.1', 0, self.manager)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.base = f'http://127.0.0.1:{self.server.server_port}/api/runtime/'

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()
        for pid in self.process_ids:
            try:
                os.killpg(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            runs = self.manager.scripts()['runs']
            if all(run['state'] == 'EXITED' for run in runs.values()):
                break
            time.sleep(0.01)
        self.directory.cleanup()
        runtime_server.COMPETITION_STATUS = self.status_before

    def request(self, path, body=None, origin=None):
        headers = {'Content-Type': 'application/json'}
        if origin:
            headers['Origin'] = origin
        request = Request(self.base + path, headers=headers,
                          data=json.dumps(body).encode() if body is not None else None)
        try:
            with urlopen(request, timeout=12) as response:
                payload = json.load(response)
                if path.endswith('/start') and payload.get('run', {}).get('pid'):
                    self.process_ids.add(payload['run']['pid'])
                return response.status, payload
        except HTTPError as error:
            return error.code, json.load(error)

    def wait_for_log(self, script_id, text):
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            status, log = self.request(f'scripts/log?script_id={script_id}')
            if status == 200 and text in log['data']:
                return
            time.sleep(0.01)
        self.fail(f'{script_id} did not log {text!r}')

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

    def test_registered_acb_script_parameters_and_log(self):
        config = Path(__file__).resolve().parents[1] / 'config/runtime_tasks.yaml'
        self.manager = RuntimeManager(
            config, self.root / 'setup.bash', self.root,
            self.root / 'logs', self.root / 'control.lock')
        self.server.manager = self.manager
        (self.root / 'start_fruit_mission_acb.sh').write_text(
            'printf "%s\\n" "$@"\n', encoding='utf-8')
        status, data = self.request('scripts')
        self.assertEqual(status, 200)
        scripts = {item['id']: item for item in data['scripts']}
        self.assertEqual(scripts['mission_acb']['script'], 'start_fruit_mission_acb.sh')
        self.assertEqual(set(scripts['mission_acb']['parameters']), {'phase', 'debug'})
        self.assertEqual(self.manager.script_arguments('mission_acb', {}), ['--phase', 'all'])
        self.assertEqual(self.manager.script_arguments('mission', {}),
                         ['--phase', 'all', '--flow', 'full', '--open-loop', 'A'])
        for parameters in ({'flow': 'full'}, {'open_loop': 'A'}, {'phase': 'invalid'},
                           {'debug': 'true'}, {'phase': '$(touch injected)'}):
            self.assertEqual(self.request('scripts/mission_acb/start', {
                'parameters': parameters})[0], 400)
        status, reply = self.request('scripts/mission_acb/start', {
            'parameters': {'phase': 'prepare', 'debug': True}})
        self.assertEqual(status, 200)
        self.assertEqual(reply['run']['id'], 'mission_acb')
        self.wait_for_log('mission_acb', '--phase\nprepare\n--debug')
        self.assertFalse((self.root / 'injected').exists())

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
            status, _ = self.request('scripts/sample/start', {'parameters': {'mode': 'two'}})
            self.assertEqual(status, 409)
            current = self.request('scripts')[1]['runs']['sample']
            self.assertEqual(current['pid'], first['run']['pid'])
            self.assertEqual(current['run_id'], first['run']['run_id'])
            self.assertEqual(self.request('scripts/sample/stop', {})[0], 200)
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

    def test_concurrent_scripts_individual_stop_and_logs(self):
        (self.root / 'sample.sh').write_text(
            'trap "exit 0" INT TERM\necho 中文测试\n'
            'while true; do sleep 0.05; done\n', encoding='utf-8')
        try:
            status, start1 = self.request('scripts/sample/start', {'parameters': {'mode': 'one'}})
            self.assertEqual(status, 200)
            status, start2 = self.request('scripts/sample2/start', {'parameters': {'debug': True}})
            self.assertEqual(status, 200)

            self.wait_for_log('sample', '中文测试')
            self.wait_for_log('sample2', 'second script running')
            status, state = self.request('scripts')
            self.assertIn('runs', state)
            self.assertEqual(state['runs']['sample2']['state'], 'RUNNING')
            self.assertEqual(state['runs']['sample']['state'], 'RUNNING')

            # Fetch log for sample2 specifically
            status, log2 = self.request('scripts/log?script_id=sample2')
            self.assertEqual(status, 200)
            self.assertIn('second script running', log2['data'])

            # Fetch log for sample specifically
            status, log1 = self.request('scripts/log?script_id=sample')
            self.assertEqual(status, 200)
            self.assertIn('中文测试', log1['data'])

            # Stop sample2 individually
            status, stop_res = self.request('scripts/sample2/stop', {})
            self.assertEqual(status, 200)
            self.assertEqual(stop_res['run']['state'], 'EXITED')
            state = self.request('scripts')[1]['runs']
            self.assertEqual(state['sample']['pid'], start1['run']['pid'])
            self.assertEqual(state['sample']['state'], 'RUNNING')

            # Idempotent stop on already stopped script
            status, stop_again = self.request('scripts/sample2/stop', {})
            self.assertEqual(status, 200)
            self.assertEqual(stop_again['run']['state'], 'EXITED')

            # Stop unknown script -> 404
            status, _ = self.request('scripts/nonexistent/stop', {})
            self.assertEqual(status, 404)
        finally:
            self.request('scripts/kill', {})

    def test_kill_retains_tracking_for_surviving_scripts(self):
        for exit_code in (0, 1):
            with self.subTest(exit_code=exit_code):
                (self.root / 'kill_fruit_processes.sh').write_text(f'exit {exit_code}\n')
                status, started = self.request('scripts/sample2/start', {})
                self.assertEqual(status, 200)
                self.wait_for_log('sample2', 'second script running')
                status, _ = self.request('scripts/kill', {})
                self.assertEqual(status, 200 if exit_code == 0 else 409)
                current = self.request('scripts')[1]['runs']['sample2']
                self.assertEqual(current['state'], 'RUNNING')
                self.assertEqual(current['pid'], started['run']['pid'])
                self.assertEqual(self.request('scripts/sample2/start', {})[0], 409)
                status, stopped = self.request('scripts/sample2/stop', {})
                self.assertEqual(status, 200)
                self.assertEqual(stopped['run']['state'], 'EXITED')

    def test_stop_waits_for_sigterm_exit(self):
        (self.root / 'sample2.sh').write_text(
            'trap "" INT\ntrap "sleep 0.2; exit 0" TERM\necho ready\n'
            'while true; do sleep 0.05; done\n')
        self.assertEqual(self.request('scripts/sample2/start', {})[0], 200)
        self.wait_for_log('sample2', 'ready')
        status, stopped = self.request('scripts/sample2/stop', {})
        self.assertEqual(status, 200)
        self.assertEqual(stopped['run']['state'], 'EXITED')
        self.assertEqual(stopped['run']['exit_code'], 0)
        self.assertEqual(self.request('scripts/sample2/start', {})[0], 200)

    def test_stop_timeout_retains_tracking_and_rejects_restart(self):
        (self.root / 'sample2.sh').write_text(
            'trap "if [[ -f allow-exit ]]; then exit 0; fi" INT TERM\necho ready\n'
            'while true; do sleep 0.05; done\n')
        status, started = self.request('scripts/sample2/start', {})
        self.assertEqual(status, 200)
        self.wait_for_log('sample2', 'ready')
        self.assertEqual(self.request('scripts/sample2/stop', {})[0], 409)
        current = self.request('scripts')[1]['runs']['sample2']
        self.assertEqual(current['state'], 'RUNNING')
        self.assertEqual(current['pid'], started['run']['pid'])
        self.assertEqual(self.request('scripts/sample2/start', {})[0], 409)
        (self.root / 'allow-exit').touch()
        status, stopped = self.request('scripts/sample2/stop', {})
        self.assertEqual(status, 200)
        self.assertEqual(stopped['run']['state'], 'EXITED')

    def test_exited_leader_remains_running_until_descendant_stops(self):
        (self.root / 'sample2.sh').write_text(
            'trap "" INT\n'
            '/bin/sleep 60 &\necho descendant-ready\nexit 0\n')
        status, started = self.request('scripts/sample2/start', {})
        self.assertEqual(status, 200)
        self.wait_for_log('sample2', 'descendant-ready')
        time.sleep(0.1)
        self.assertEqual(self.request('scripts')[1]['runs']['sample2']['state'], 'RUNNING')
        self.assertEqual(self.request('scripts/sample2/start', {})[0], 409)
        status, stopped = self.request('scripts/sample2/stop', {})
        self.assertEqual(status, 200)
        self.assertEqual(stopped['run']['state'], 'EXITED')
        self.assertFalse(process_groups_alive({started['run']['pid']}))

    def test_all_registered_cards_stop_owned_nested_groups_only(self):
        config = Path(__file__).resolve().parents[1] / 'config/runtime_tasks.yaml'
        manager = RuntimeManager(
            config, self.root / 'setup.bash', self.root,
            self.root / 'logs', self.root / 'cards.lock')
        (self.root / 'worker.py').write_text(
            'import os,signal,sys,time\n'
            'from pathlib import Path\n'
            'signal.signal(signal.SIGINT, lambda *args: sys.exit(0))\n'
            'signal.signal(signal.SIGTERM, lambda *args: sys.exit(0))\n'
            'Path(sys.argv[1]).write_text(str(os.getpid()))\n'
            'print("ready",flush=True)\ntime.sleep(60)\n')
        # These are fixture programs named after the cards, never the real robot scripts.
        for card in manager.scripts()['scripts']:
            with self.subTest(card=card['id']):
                (self.root / card['script']).write_text(
                    'trap "exit 0" INT TERM\n'
                    'setsid /usr/bin/python3 worker.py nested.pid &\n'
                    'while true; do sleep 0.05; done\n')
                (self.root / 'nested.pid').unlink(missing_ok=True)
                parameters = {
                    name: spec['choices'][0] for name, spec in card['parameters'].items()
                    if spec.get('required')}
                started = manager.start_script(card['id'], parameters)['run']
                self.process_ids.add(started['pid'])
                deadline = time.monotonic() + 2
                while not (self.root / 'nested.pid').exists() and time.monotonic() < deadline:
                    time.sleep(0.01)
                nested_pid = int((self.root / 'nested.pid').read_text())
                self.process_ids.add(nested_pid)
                # Start an independently owned script through the original HTTP manager.
                self.assertEqual(self.request('scripts/sample2/start', {})[0], 200)
                self.wait_for_log('sample2', 'second script running')
                stopped = manager.stop_script(card['id'])['run']
                self.assertEqual(stopped['state'], 'EXITED')
                self.assertFalse(process_groups_alive({started['pid'], nested_pid}))
                self.assertEqual(self.request('scripts')[1]['runs']['sample2']['state'], 'RUNNING')
                self.assertEqual(self.request('scripts/sample2/stop', {})[0], 200)

    def test_origin_and_body_validation(self):
        self.assertEqual(self.request('scripts/kill', {}, 'http://evil.example:8000')[0], 403)
        self.assertEqual(self.request('scripts/kill', {}, 'http://127.0.0.1:broken')[0], 403)
        self.assertEqual(self.request('scripts/kill', {}, 'http://localhost:8000')[0], 200)
        self.assertEqual(self.request('scripts/kill', {'command': 'arbitrary'})[0], 400)
        status, _ = self.request('scripts/sample/start', {
            'parameters': {'mode': 'one'}, 'command': 'arbitrary'})
        self.assertEqual(status, 400)

    def test_localization_endpoint_rejects_parameter_changes(self):
        for body in ({'flow': 'acb'}, {'parameters': {'flow': 'full'}}, []):
            self.assertEqual(self.request('competition/localize', body)[0], 400)
        self.assertEqual(self.request(
            'competition/localize', {}, 'http://evil.example:8000')[0], 403)

    def test_named_localization_rejects_flow_mismatch_without_launching(self):
        status_path = runtime_server.COMPETITION_STATUS
        for phase in ('PREPARED', 'LOCALIZING', 'RUNNING'):
            status_path.write_text(json.dumps({'phase': phase, 'parameters': {'flow': 'full'}}))
            self.assertEqual(self.request('competition/localize/acb', {})[0], 409)
        status_path.write_text(json.dumps({'phase': 'RUNNING', 'parameters': {'flow': 'acb'}}))
        code, payload = self.request('competition/localize/acb', {})
        self.assertEqual(code, 200)
        self.assertFalse(payload['accepted'])
        self.assertEqual(self.request('competition/localize', {})[0], 200)
        for flow in ('invalid', 'acb/extra', '$(touch injected)'):
            self.assertEqual(self.request(f'competition/localize/{quote(flow, safe="")}', {})[0],
                             400)
        self.assertEqual(self.request('competition/localize/acb', {'flow': 'acb'})[0], 400)
        self.assertFalse((self.root / 'logs/competition-localize.log').exists())

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
