#!/usr/bin/env python3
"""Offline mathematical and document checks. Never connects to hardware."""

import argparse
import copy
import math
from pathlib import Path
import tempfile

import yaml

from fruit_arm_planner.joint_transformer import JointTransformer
from fruit_arm_planner.kinematics import RobotKinematics
from fruit_arm_planner.planning_api import configuration, revision
from fruit_arm_planner.trajectory_planner import TrajectoryPlanner
from fruit_robviz_bridge.arm_editor import calculate, document, save_presets
from fruit_robviz_bridge.robviz_web_server import _document_lock, VersionConflict


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path(
        '/home/rc4/fruit_ws/src/fruit_arm_planner/config/arm_presets.yaml'))
    args = parser.parse_args()
    params = configuration(yaml.safe_load(args.config.read_text())[
        'arm_planner_node']['ros__parameters'])
    transformer = JointTransformer.from_config_dict(params['joint_mapping'])
    start = tuple((ch.min_limit_rad + ch.max_limit_rad) / 2 for ch in transformer.channels)
    target = [value * 180 / 3.141592653589793 for value in start]
    request = dict(configuration=params, frame='joints', target=target,
                   start_hw_rad=list(start), duration_ms=1)
    response = calculate(request, preview=True)
    assert response['point_count'] == 2
    assert response['points'][0]['hardware_rad'] == start
    expected = RobotKinematics.from_geometry_dict(params['geometry']).forward_kinematics(
        *transformer.hardware_to_algo(start))
    assert all(math.isclose(a, b, abs_tol=1e-12) for actual, reference in
               zip(response['points'][-1]['nodes'], expected['points'])
               for a, b in zip(actual, reference))
    assert revision({'x': 1}) == revision({'x': 1.0})
    assert revision({'x': -0.0}) == revision({'x': 0})
    for change in ({'target': [float('nan')] * 4}, {'duration_ms': 0xFFFFFFFF},
                   {'duration_ms': False}, {'duration_ms': None}, {'gripper': 1.0},
                   {'gripper': True}, {'frame': 'joints', 'linear': True}):
        try:
            calculate(dict(request, **change), preview=True)
        except ValueError:
            pass
        else:
            raise AssertionError(f'Invalid request accepted: {change}')
    math_params = copy.deepcopy(params)
    math_params['planning'].update(interpolation_step_ms=4, default_dwell_ms=7)
    sequence = dict(coordinate_frame='joints', waypoints=[
        dict(target=target, duration_ms=13, gripper=1, is_via_point=True),
        dict(target=target, duration_ms=17, gripper=2, dwell_ms=0)])
    legacy = TrajectoryPlanner(RobotKinematics.from_geometry_dict(params['geometry']))
    ok, points, message = legacy.plan_multi_waypoint_sequence(
        start, sequence['waypoints'], transformer, 'joints', 4, 7)
    assert ok, message
    preview = calculate(dict(configuration=math_params, start_hw_rad=list(start),
                             preset=sequence), preview=True)
    assert len(points) == preview['point_count']
    for original, serialized in zip(points, preview['points']):
        assert original.joints == serialized['hardware_rad']
        assert original.time_from_start == serialized['time_ms']
        assert original.gripper_action == serialized['gripper']
        assert original.is_via_point == serialized['is_via_point']
    with tempfile.TemporaryDirectory() as temporary:
        path = Path(temporary) / 'arm.yaml'
        path.write_bytes(args.config.read_bytes())
        original = document(path)
        presets = copy.deepcopy(params['presets'])
        # A mathematical fixture, not a frozen competition preset.
        presets['offline_preview_check'] = dict(
            coordinate_frame='joints', description='offline fixture',
            waypoints=[dict(target=target, duration_ms=1)])
        result = save_presets(path, Path(temporary) / 'backups',
                              dict(version=original['version'], presets=presets),
                              _document_lock, VersionConflict)
        assert Path(result['backup']).read_bytes() == args.config.read_bytes()
        for key in ('geometry', 'joint_mapping', 'planning'):
            assert result['configuration'][key] == params[key]
        try:
            save_presets(path, Path(temporary) / 'backups',
                         dict(version=original['version'], presets=presets),
                         _document_lock, VersionConflict)
        except VersionConflict:
            pass
        else:
            raise AssertionError('Stale file version accepted')
        # YAML data fixtures check comments, unknown values and numeric sequence types.
        fixture = copy.deepcopy(params)
        fixture['presets'] = {'fixture': dict(
            coordinate_frame='joints', description='# description data',
            custom={'untouched': True}, waypoints=[dict(target=target, duration_ms=1)])}
        text = yaml.safe_dump({'arm_planner_node': {'ros__parameters': fixture}}, sort_keys=False)
        text = text.replace('coordinate_frame: joints', 'coordinate_frame: joints # frame note')
        text = text.replace(
            'waypoints:', '# repeated note\n        # repeated note\n        waypoints:')
        path.write_text(text)
        for change in ('target', 'append', 'rename'):
            before = document(path)
            presets = copy.deepcopy(before['configuration']['presets'])
            name = next(iter(presets))
            if change == 'target':
                presets[name]['waypoints'][0]['target'][0] += 0.001
            elif change == 'append':
                presets[name]['waypoints'].append(dict(target=[0, 0.5, 0, 0], duration_ms=1))
            else:
                presets['renamed'] = presets.pop(name)
            saved = save_presets(path, Path(temporary) / 'backups',
                                 dict(version=before['version'], presets=presets),
                                 _document_lock, VersionConflict)
            text = path.read_text()
            assert text.count('# frame note') == 1
            assert text.count('# repeated note') == 2
            for item in saved['configuration']['presets'].values():
                assert item['custom'] == {'untouched': True}
                assert item['description'] == '# description data'
                for point in item['waypoints']:
                    assert all(isinstance(value, float) for value in point['target'])
    print('Arm FK, shared trajectories, validation, YAML, backup and conflict checks passed.')
    print('No motion.')


if __name__ == '__main__':
    main()
