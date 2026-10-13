#!/usr/bin/env python3
"""Offline mathematical and document checks. Never connects to hardware."""

import argparse
import copy
import hashlib
import math
from pathlib import Path
import tempfile

import yaml

from fruit_arm_planner.joint_transformer import JointTransformer
from fruit_arm_planner.kinematics import RobotKinematics
from fruit_arm_planner.planning_api import configuration, revision
from fruit_arm_planner.trajectory_planner import TrajectoryPlanner
from fruit_robviz_bridge.arm_editor import calculate, document, save_presets
from fruit_robviz_bridge.waypoint_files import _document_lock, VersionConflict


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
        # Presets order and block reordering offline checks
        order_fixture = copy.deepcopy(params)
        order_fixture['presets'] = {
            '10': dict(coordinate_frame='joints', description='# 10 fixture',
                       waypoints=[dict(target=list(target), duration_ms=1)]),
            '2': dict(coordinate_frame='joints', description='# 2 fixture',
                      waypoints=[dict(target=list(target), duration_ms=2)]),
            'alpha': dict(coordinate_frame='joints', description='# alpha note',
                          waypoints=[dict(target=list(target), duration_ms=3)]),
            'beta': dict(coordinate_frame='joints', description='# beta note',
                         waypoints=[dict(target=list(target), duration_ms=4)]),
        }
        order_text = yaml.safe_dump(
            {'arm_planner_node': {'ros__parameters': order_fixture}}, sort_keys=False)
        order_text = order_text.replace(
            'presets:\n',
            'presets:\n      # header comment before first preset\n'
        )
        order_path = Path(temporary) / 'order.yaml'
        order_path.write_text(order_text)

        doc_before = document(order_path)
        assert doc_before['preset_order'] == ['10', '2', 'alpha', 'beta']

        # 1. Pure reorder: semantic revisions unchanged, byte version changes,
        # header comment preserved
        new_order = ['beta', '10', 'alpha', '2']
        res_reorder = save_presets(order_path, Path(temporary) / 'backups',
                                   dict(version=doc_before['version'],
                                        presets=doc_before['configuration']['presets'],
                                        preset_order=new_order),
                                   _document_lock, VersionConflict)
        assert res_reorder['preset_order'] == new_order
        assert res_reorder['revision'] == doc_before['revision']
        assert res_reorder['presets_revision'] == doc_before['presets_revision']
        assert res_reorder['version'] != doc_before['version']
        saved_text = order_path.read_text()
        assert '# header comment before first preset' in saved_text
        assert (saved_text.index('beta:') < saved_text.index("'10':")
                < saved_text.index('alpha:') < saved_text.index("'2':"))
        assert '# beta note' in saved_text
        assert '# alpha note' in saved_text

        # 2. Reorder with content change
        doc_mid = document(order_path)
        updated_presets = copy.deepcopy(doc_mid['configuration']['presets'])
        updated_presets['alpha']['waypoints'][0]['duration_ms'] = 50
        content_order = ['alpha', 'beta', '2', '10']
        res_content = save_presets(order_path, Path(temporary) / 'backups',
                                   dict(version=doc_mid['version'],
                                        presets=updated_presets,
                                        preset_order=content_order),
                                   _document_lock, VersionConflict)
        assert res_content['preset_order'] == content_order
        assert (res_content['configuration']['presets']['alpha']['waypoints'][0]['duration_ms']
                == 50)
        assert res_content['revision'] != doc_mid['revision']

        # 3. Backward compatibility: save without preset_order
        doc_legacy = document(order_path)
        legacy_presets = copy.deepcopy(doc_legacy['configuration']['presets'])
        legacy_presets['beta']['waypoints'][0]['duration_ms'] = 80
        res_legacy = save_presets(order_path, Path(temporary) / 'backups',
                                  dict(version=doc_legacy['version'],
                                       presets=legacy_presets),
                                  _document_lock, VersionConflict)
        assert res_legacy['configuration']['presets']['beta']['waypoints'][0]['duration_ms'] == 80

        # 4. Invalid preset_order rejected and file untouched
        doc_check = document(order_path)
        content_before_err = order_path.read_bytes()
        for bad_order in (
            ['alpha'],                          # missing
            ['alpha', 'beta', '2', '10', 'x'],  # extra
            ['alpha', 'beta', '2', '2'],        # duplicate
            ['10', '2', 'alpha', 'wrong'],      # unknown
            'not_a_list',                       # invalid type
        ):
            try:
                save_presets(order_path, Path(temporary) / 'backups',
                             dict(version=doc_check['version'],
                                  presets=doc_check['configuration']['presets'],
                                  preset_order=bad_order),
                             _document_lock, VersionConflict)
            except ValueError:
                pass
            else:
                raise AssertionError(f'Invalid preset_order accepted: {bad_order}')
            assert order_path.read_bytes() == content_before_err

        # 5. Unsupported YAML structure (merge key <<) rejected safely
        merge_path = Path(temporary) / 'merge.yaml'
        merge_text = order_text.replace('presets:\n', 'presets:\n      <<: {}\n')
        merge_path.write_text(merge_text)
        try:
            save_presets(merge_path, Path(temporary) / 'backups',
                         dict(version=hashlib.sha256(merge_text.encode()).hexdigest(),
                              presets=order_fixture['presets'],
                              preset_order=['10', '2', 'alpha', 'beta']),
                         _document_lock, VersionConflict)
        except ValueError:
            pass
        else:
            raise AssertionError('Merge key in YAML accepted')
        assert merge_path.read_text() == merge_text

        # 6. Reorder with trailing sections and unknown fields after presets
        trailing_fixture = {
            'geometry': copy.deepcopy(params['geometry']),
            'joint_mapping': copy.deepcopy(params['joint_mapping']),
            'presets': {
                'first': dict(coordinate_frame='joints', description='# first block',
                              waypoints=[dict(target=list(target), duration_ms=1)]),
                'last': dict(coordinate_frame='joints', description='# last block',
                             waypoints=[dict(target=list(target), duration_ms=2)]),
            },
            'planning': copy.deepcopy(params['planning']),
            'unknown_field': {'custom_setting': 42},
        }
        trailing_text = yaml.safe_dump(
            {'arm_planner_node': {'ros__parameters': trailing_fixture}}, sort_keys=False)
        trailing_path = Path(temporary) / 'trailing.yaml'
        trailing_path.write_text(trailing_text)

        doc_trailing = document(trailing_path)
        assert doc_trailing['preset_order'] == ['first', 'last']

        res_trailing = save_presets(trailing_path, Path(temporary) / 'backups',
                                    dict(version=doc_trailing['version'],
                                         presets=doc_trailing['configuration']['presets'],
                                         preset_order=['last', 'first']),
                                    _document_lock, VersionConflict)
        assert res_trailing['preset_order'] == ['last', 'first']
        assert res_trailing['revision'] == doc_trailing['revision']
        saved_trailing_text = trailing_path.read_text()
        assert saved_trailing_text.index('last:') < saved_trailing_text.index('first:')
        assert (saved_trailing_text.index('first:') < saved_trailing_text.index('planning:')
                < saved_trailing_text.index('unknown_field:'))
        expected_trailing_doc = copy.deepcopy(trailing_fixture)
        expected_trailing_doc['presets'] = {
            'last': trailing_fixture['presets']['last'],
            'first': trailing_fixture['presets']['first'],
        }
        assert (yaml.safe_load(saved_trailing_text)['arm_planner_node']['ros__parameters']
                == expected_trailing_doc)
        fixture = copy.deepcopy(params)
        fixture['presets'] = {'fixture': dict(
            coordinate_frame='joints', description='# description data',
            custom={'untouched': True}, waypoints=[dict(target=target, duration_ms=1)])}
        text = yaml.safe_dump({'arm_planner_node': {'ros__parameters': fixture}}, sort_keys=False)
        text = text.replace('coordinate_frame: joints', 'coordinate_frame: joints # frame note')
        text = text.replace(
            'waypoints:', '# repeated note\n        # repeated note\n        waypoints:')
        path.write_text(text)
        for change in ('target', 'append', 'delete', 'rename'):
            before = document(path)
            presets = copy.deepcopy(before['configuration']['presets'])
            name = next(iter(presets))
            if change == 'target':
                presets[name]['waypoints'][0]['target'][0] += 0.001
            elif change == 'append':
                presets[name]['waypoints'].append(dict(target=[0, 0.5, 0, 0], duration_ms=1))
            elif change == 'delete':
                presets[name]['waypoints'].pop()
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
