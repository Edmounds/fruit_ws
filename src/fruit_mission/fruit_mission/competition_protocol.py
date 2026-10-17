"""Pure competition snapshot calculations, independent of ROS and hardware."""

import math

from fruit_mission.decisions import PRODUCE_PREFIX_NAMES


def produce_type(model_type):
    """Group ripe/unripe model classes under one produce type."""
    for prefix, name in PRODUCE_PREFIX_NAMES.items():
        if model_type in (name, prefix + '_ripe', prefix + '_unripe'):
            return name
    return 'unknown'


def count_result(statistics, area, model_type, outcome):
    """Count one actual opportunity, never infer an unseen fruit quantity."""
    if area not in ('A', 'B', 'C', 'D') or outcome not in ('completed', 'failed', 'skipped'):
        raise ValueError('Invalid competition result')
    counts = statistics.setdefault(area, {}).setdefault(
        produce_type(model_type), {'completed': 0, 'failed': 0, 'skipped': 0})
    counts[outcome] += 1


def localization_sample_ready(stamp_ns, now_ns, started_ns, values):
    """Require a finite new pose from this attempt, with a valid quaternion."""
    return (stamp_ns > 0 and stamp_ns >= started_ns and 0 <= now_ns - stamp_ns <= 1_000_000_000
            and len(values) == 7 and all(math.isfinite(v) for v in values)
            and sum(v * v for v in values[3:]) > 0)


def module_health(node_present, last_received, now, expects_stream=False, enabled=True):
    """Do not confuse node existence with active data or on-demand results."""
    if not enabled:
        return 'DISABLED'
    if not node_present:
        return 'MISSING'
    if expects_stream and (last_received is None or now - last_received > 3):
        return 'NO_DATA'
    return 'OK'
