"""Exercise Unix signal protocols using temporary processes, never robot commands."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

from fruit_robviz_bridge.process_groups import (
    owned_process_groups, process_groups_alive, stop_process_groups,
)
from fruit_robviz_bridge.runtime_manager import ManagedProcess


class ProcessGroupsTest(unittest.TestCase):
    def setUp(self):
        self.processes = []
        self.groups = set()

    def tearDown(self):
        if self.groups:
            stop_process_groups(self.groups, 0, 0, 1)
        for process in self.processes:
            process.wait(timeout=2)
            if process.stdout:
                process.stdout.close()

    def start(self, code, *args):
        process = subprocess.Popen(
            [sys.executable, '-u', '-c', code, *args], start_new_session=True,
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        self.processes.append(process)
        self.groups.add(process.pid)
        return process

    def test_leader_exit_does_not_hide_live_descendant(self):
        child = ('import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN);'
                 'signal.signal(signal.SIGTERM,signal.SIG_IGN);'
                 'print("ready",flush=True); time.sleep(60)')
        parent = self.start(
            'import subprocess,sys,time\n'
            'child=subprocess.Popen([sys.executable,"-u","-c",sys.argv[1]],'
            'stdout=subprocess.PIPE,text=True)\n'
            'assert child.stdout.readline().strip()=="ready"\n'
            'print(child.pid,flush=True)\ntime.sleep(60)\n', child)
        parent.stdout.readline()
        os.kill(parent.pid, signal.SIGTERM)
        parent.wait(timeout=2)
        self.assertTrue(process_groups_alive({parent.pid}))
        groups = {parent.pid}
        self.assertTrue(stop_process_groups(groups, 0.1, 0.1, 1))
        self.assertFalse(process_groups_alive({parent.pid}))

    def test_nested_session_cleanup_preserves_unrelated_process(self):
        unrelated = self.start('import time; print("ready",flush=True); time.sleep(60)')
        self.assertEqual(unrelated.stdout.readline().strip(), 'ready')
        parent = self.start(
            'import subprocess,sys,time\n'
            'child=subprocess.Popen([sys.executable,"-u","-c",sys.argv[1]],'
            'start_new_session=True,stdout=subprocess.PIPE,text=True)\n'
            'assert child.stdout.readline().strip()=="ready"\n'
            'print(child.pid,flush=True)\ntime.sleep(60)\n',
            'import time; print("ready",flush=True); time.sleep(60)')
        child_pid = int(parent.stdout.readline())
        self.groups.add(child_pid)
        groups = owned_process_groups({parent.pid})
        self.assertIn(child_pid, groups)
        self.assertNotIn(unrelated.pid, groups)
        self.assertTrue(stop_process_groups(groups, 0.1, 0.1, 1))
        self.assertIsNone(unrelated.poll())
        self.assertFalse(process_groups_alive(groups))

    def test_no_kill_retains_unresponsive_group(self):
        parent = self.start(
            'import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN);'
            'signal.signal(signal.SIGTERM,signal.SIG_IGN);'
            'print("ready",flush=True); time.sleep(60)')
        self.assertEqual(parent.stdout.readline().strip(), 'ready')
        self.assertFalse(stop_process_groups({parent.pid}, 0.1, 0.1, None))
        self.assertIsNone(parent.poll())

    def test_failed_stop_keeps_discovered_orphan_session_for_next_stop(self):
        parent = self.start(
            'import subprocess,sys,time\n'
            'child=subprocess.Popen([sys.executable,"-u","-c",sys.argv[1]],'
            'start_new_session=True,stdout=subprocess.PIPE,text=True)\n'
            'assert child.stdout.readline().strip()=="ready"\n'
            'print(child.pid,flush=True)\ntime.sleep(60)\n',
            'import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN);'
            'signal.signal(signal.SIGTERM,signal.SIG_IGN);'
            'print("ready",flush=True); time.sleep(60)')
        child_pid = int(parent.stdout.readline())
        self.groups.add(child_pid)
        groups = {parent.pid}
        self.assertFalse(stop_process_groups(groups, 0.1, 0.1, None))
        self.assertIsNotNone(parent.poll())
        self.assertIn(child_pid, groups)
        self.assertTrue(process_groups_alive(groups))
        self.assertTrue(stop_process_groups(groups, 0.1, 0.1, 1))

    def test_zombie_main_thread_does_not_hide_live_worker(self):
        parent = self.start(
            'import ctypes,threading,time\n'
            'threading.Thread(target=lambda: time.sleep(60)).start()\n'
            'print("ready",flush=True)\nctypes.CDLL(None).pthread_exit(None)\n')
        self.assertEqual(parent.stdout.readline().strip(), 'ready')
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            fields = Path(f'/proc/{parent.pid}/stat').read_text().rsplit(')', 1)[1].split()
            if fields[0] == 'Z':
                break
            time.sleep(0.01)
        self.assertEqual(fields[0], 'Z')
        self.assertGreater(int(fields[17]), 1)
        self.assertTrue(process_groups_alive({parent.pid}))
        self.assertTrue(stop_process_groups({parent.pid}, 0.1, 0.1, 1))

    def test_rejects_caller_group_and_invalid_wait_before_signaling(self):
        with self.assertRaises(ValueError):
            stop_process_groups({os.getpgrp()}, 0, 0, 0)
        with self.assertRaises(ValueError):
            stop_process_groups({0}, 0, 0, 0)
        with self.assertRaises(ValueError):
            stop_process_groups(set(), float('nan'), 0, 0)

    def test_finished_tracking_cannot_reactivate_reused_group_ids(self):
        parent = self.start('pass')
        parent.wait(timeout=2)
        tracked = ManagedProcess(parent, Path('/unused-fixture-log'))
        self.assertFalse(tracked.is_running())
        unrelated = self.start('import time; print("ready",flush=True); time.sleep(60)')
        self.assertEqual(unrelated.stdout.readline().strip(), 'ready')
        # Model a recorded group ID resolving to a new, independently owned process.
        tracked.groups = {unrelated.pid}
        self.assertFalse(tracked.is_running())
        self.assertIsNone(unrelated.poll())

    def test_shell_helper_handles_an_exited_leader(self):
        parent = self.start(
            'import subprocess,sys,time\n'
            'child=subprocess.Popen([sys.executable,"-u","-c",sys.argv[1]],'
            'stdout=subprocess.PIPE,text=True)\n'
            'assert child.stdout.readline().strip()=="ready"\n'
            'print(child.pid,flush=True)\ntime.sleep(60)\n',
            'import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN);'
            'print("ready",flush=True); time.sleep(60)')
        parent.stdout.readline()
        os.kill(parent.pid, signal.SIGTERM)
        parent.wait(timeout=2)
        workspace = Path(__file__).resolve().parents[3]
        subprocess.run([
            '/bin/bash', '-c', 'source "$1"; fruit_stop_process_group "$2" fixture 0.1 0.5',
            'fixture', str(workspace / 'scripts/fruit_runtime_logging.bash'), str(parent.pid),
        ], check=True, timeout=4)
        self.assertFalse(process_groups_alive({parent.pid}))

    def test_voice_exec_stops_fake_player_without_audio(self):
        workspace = Path(__file__).resolve().parents[3]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'sound.wav').write_bytes(b'fixture only, not audio')
            player = root / 'paplay'
            player.write_text('#!/bin/bash\necho ready\nexec /bin/sleep 60\n')
            player.chmod(0o755)
            process = subprocess.Popen([
                '/bin/bash', str(workspace / 'test_voice.sh'), str(root / 'sound.wav')],
                env={**os.environ, 'PATH': f'{root}:/usr/bin:/bin'},
                stdout=subprocess.PIPE, text=True, start_new_session=True)
            self.processes.append(process)
            self.groups.add(process.pid)
            process.stdout.readline()
            self.assertEqual(process.stdout.readline().strip(), 'ready')
            self.assertTrue(stop_process_groups({process.pid}, 0.1, 0.1, 1))


if __name__ == '__main__':
    unittest.main()
