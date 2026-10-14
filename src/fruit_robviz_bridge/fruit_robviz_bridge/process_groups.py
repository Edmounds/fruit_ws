"""Bounded Unix signal cleanup for owned Linux process groups."""

import argparse
import math
import os
from pathlib import Path
import signal
import time


def _processes():
    rows = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            # comm can contain spaces and parentheses. Fields after it are stable.
            fields = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
            # A zombie main thread can still have live worker threads.
            live = fields[0] not in ('Z', 'X') or int(fields[17]) > 1
            rows[int(entry.name)] = (int(fields[1]), int(fields[2]), live)
        except (OSError, ValueError, IndexError):
            continue
    return rows


def owned_process_groups(groups):
    """Include descendant setsid groups, but never unrelated reused services."""
    rows = _processes()
    owned = {pid for pid, (_, group, _) in rows.items() if group in groups}
    while True:
        children = {pid for pid, (parent, _, _) in rows.items() if parent in owned}
        if children <= owned:
            break
        owned.update(children)
    return set(groups) | {rows[pid][1] for pid in owned}


def process_groups_alive(groups):
    """Reap leaders separately. A zombie cannot execute or hold a hardware device."""
    return any(group in groups and live for _, group, live in _processes().values())


def stop_process_groups(groups, int_timeout=5.0, term_timeout=2.0, kill_timeout=2.0):
    """Mutate the owned group set and return true only when all live members exit."""
    phases = [(signal.SIGINT, int_timeout), (signal.SIGTERM, term_timeout)]
    if kill_timeout is not None:
        phases.append((signal.SIGKILL, kill_timeout))
    if any(not math.isfinite(timeout) or not 0 <= timeout <= 10 for _, timeout in phases):
        raise ValueError('Signal wait must be finite and between 0 and 10 seconds')
    for sig, timeout in phases:
        groups.update(owned_process_groups(groups))
        targets = tuple(groups)
        if any(group <= 1 or group == os.getpgrp() for group in targets):
            raise ValueError('Refusing to signal an invalid or caller process group')
        for group in targets:
            try:
                os.killpg(group, sig)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + timeout
        while process_groups_alive(groups):
            if time.monotonic() >= deadline:
                break
            time.sleep(0.05)
        if not process_groups_alive(groups):
            return True
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--int-timeout', type=float, default=5.0)
    parser.add_argument('--term-timeout', type=float, default=2.0)
    parser.add_argument('groups', type=int, nargs='+')
    options = parser.parse_args()
    if not stop_process_groups(set(options.groups), options.int_timeout, options.term_timeout):
        parser.exit(1, 'Owned process groups did not exit; stop remains unconfirmed\n')


if __name__ == '__main__':
    main()
