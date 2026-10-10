"""Allowlisted arm document editing and side-effect-free mathematical preview."""

import copy
from datetime import datetime
import hashlib
import math
import os
from pathlib import Path
import shutil
import stat
import tempfile

import yaml

from fruit_arm_planner.joint_transformer import JointTransformer
from fruit_arm_planner.kinematics import RobotKinematics
from fruit_arm_planner.planning_api import (
    configuration, plan_preset, plan_target, revision, validate_presets, validate_trajectory,
)
from fruit_arm_planner.trajectory_planner import TrajectoryPlanner


def document(path):
    content = path.read_bytes()
    params = configuration(yaml.safe_load(content)['arm_planner_node']['ros__parameters'])
    return {'version': hashlib.sha256(content).hexdigest(),
            'revision': revision(params), 'configuration': params,
            'presets_revision': revision(params['presets']), 'file': str(path)}


def _comments(text, node):
    """Keep comments when a changed YAML collection needs reserialization."""
    comments = {}
    strings = []

    def visit(item):
        if isinstance(item, yaml.ScalarNode):
            if item.style in ('|', '>', '"', "'"):
                strings.append((item.start_mark.index, item.end_mark.index))
            tail = text[item.end_mark.index:].split('\n', 1)[0]
            position = item.end_mark.index + len(tail) - len(tail.lstrip())
            if tail.lstrip().startswith('#') and position < node.end_mark.index:
                comments[position] = tail.lstrip()
        elif isinstance(item, yaml.MappingNode):
            for key, value in item.value:
                visit(key)
                visit(value)
        elif isinstance(item, yaml.SequenceNode):
            for value in item.value:
                visit(value)
    visit(node)
    position = node.start_mark.index
    for line in text[node.start_mark.index:node.end_mark.index].splitlines(keepends=True):
        start = position + len(line) - len(line.lstrip())
        if (line.lstrip().startswith('#')
                and not any(begin <= start < end for begin, end in strings)):
            comments[start] = line.strip()
        position += len(line)
    return [value for _, value in sorted(comments.items())]


def patch_yaml(text, node, old, new):
    """Edit only changed value spans, keeping unrelated bytes and comments."""
    edits = []

    def replace(item, value):
        flow = not isinstance(item, (yaml.MappingNode, yaml.SequenceNode)) or item.flow_style
        rendered = yaml.safe_dump(value, allow_unicode=True, sort_keys=False,
                                  default_flow_style=flow).rstrip('\n')
        if rendered.endswith('\n...'):
            rendered = rendered[:-4]
        indent = ' ' * item.start_mark.column
        comments = _comments(text, item)
        if comments:
            # A block value can retain comments before its serialized replacement.
            if flow:
                rendered = rendered + ' ' + ' '.join(comments)
            else:
                rendered = '\n'.join(comments) + '\n' + rendered
        rendered = rendered.replace('\n', '\n' + indent)
        if isinstance(item, (yaml.MappingNode, yaml.SequenceNode)) and not item.flow_style:
            rendered += '\n'
        edits.append((item.start_mark.index, item.end_mark.index, rendered))

    def walk(item, before, after):
        if before == after:
            return
        if (isinstance(item, yaml.MappingNode) and not item.flow_style
                and isinstance(after, dict) and set(before) == set(after)):
            for key, value in item.value:
                walk(value, before[key.value], after[key.value])
        elif (isinstance(item, yaml.SequenceNode) and not item.flow_style
              and isinstance(after, list) and len(before) == len(after)):
            for value, previous, following in zip(item.value, before, after):
                walk(value, previous, following)
        else:
            replace(item, after)
    walk(node, old, new)
    for start, end, replacement in sorted(edits, reverse=True):
        text = text[:start] + replacement + text[end:]
    return text


def save_presets(path, backup_dir, payload, lock, conflict):
    expected = payload.get('version')
    if not isinstance(expected, str) or not expected:
        raise ValueError('Missing arm file version')
    presets = copy.deepcopy(payload.get('presets'))
    validate_presets(presets)
    for preset in presets.values():
        for waypoint in preset['waypoints']:
            waypoint['target'] = [float(v) for v in waypoint['target']]
    with lock(path):
        content = path.read_bytes()
        if hashlib.sha256(content).hexdigest() != expected:
            raise conflict('机械臂文件已被其他工具修改，请核对草稿后重新加载')
        text = content.decode('utf-8')
        root = yaml.compose(text)
        node = root
        for key in ('arm_planner_node', 'ros__parameters', 'presets'):
            node = next(value for name, value in node.value if name.value == key)
        original = yaml.safe_load(text)
        old = original['arm_planner_node']['ros__parameters']['presets']
        updated = patch_yaml(text, node, old, presets)
        parsed = yaml.safe_load(updated)
        expected_document = copy.deepcopy(original)
        expected_document['arm_planner_node']['ros__parameters']['presets'] = presets
        if parsed != expected_document:
            raise ValueError('Cannot preserve YAML structure safely; file unchanged')
        configuration(parsed['arm_planner_node']['ros__parameters'])
        backup_dir.mkdir(parents=True, exist_ok=True)
        backup = backup_dir / (path.name + '.' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        shutil.copy2(path, backup)
        descriptor, temporary = tempfile.mkstemp(dir=path.parent, prefix='.' + path.name)
        try:
            with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
                stream.write(updated)
                stream.flush()
                os.fsync(stream.fileno())
            os.chmod(temporary, stat.S_IMODE(path.stat().st_mode))
            # Detect non-cooperating writers immediately before the atomic replacement.
            if path.read_bytes() != content:
                raise conflict('机械臂文件保存期间发生外部修改，未覆盖文件')
            os.replace(temporary, path)
            directory = os.open(path.parent, os.O_RDONLY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        finally:
            Path(temporary).unlink(missing_ok=True)
        result = document(path)
        result.update(saved=True, backup=str(backup))
        return result


def vector(value):
    if (not isinstance(value, list) or len(value) != 4
            or not all(isinstance(v, (float, int)) and not isinstance(v, bool)
                       and math.isfinite(v) for v in value)):
        raise ValueError('Expected four finite numeric values')
    return tuple(value)


def pose(kinematics, transformer, target, frame):
    target = vector(target)
    if frame == 'cartesian':
        ok, algo, message = kinematics.inverse_kinematics(*target[:3], pitch_deg=target[3])
        if not ok:
            raise ValueError(message)
        hardware = transformer.algo_to_hardware(algo)
    elif frame == 'link_angles':
        hardware = transformer.link_angles_to_hardware(target)
        algo = transformer.hardware_to_algo(hardware)
    elif frame == 'joints':
        hardware = tuple(math.radians(v) for v in target)
        algo = transformer.hardware_to_algo(hardware)
    elif frame == 'hardware_rad':
        hardware = target
        algo = transformer.hardware_to_algo(hardware)
    elif frame == 'model_joints':
        algo = target
        hardware = transformer.algo_to_hardware(algo)
    else:
        raise ValueError('Unknown coordinate frame')
    ok, message = transformer.check_hardware_limits(hardware)
    if not ok:
        raise ValueError(message)
    fk = kinematics.forward_kinematics(*algo)
    links = [math.degrees(algo[0]), math.degrees(algo[1]) - 90,
             math.degrees(algo[1] + algo[2]), math.degrees(sum(algo[1:]))]
    return {'hardware_rad': hardware, 'hardware_deg': [math.degrees(v) for v in hardware],
            'model_rad': algo, 'link_deg': links,
            'tcp': list(fk['end_effector'][:3]) + [math.degrees(fk['end_effector'][3])],
            'nodes': fk['points']}


def calculate(payload, preview=False):
    raw = payload.get('configuration')
    if isinstance(raw, str):
        raw = yaml.safe_load(raw)
    params = configuration(raw)
    kinematics = RobotKinematics.from_geometry_dict(params['geometry'])
    transformer = JointTransformer.from_config_dict(params['joint_mapping'])
    result = {'configuration': params, 'revision': revision(params),
              'presets_revision': revision(params['presets'])}
    if not preview:
        if 'target' in payload:
            result['pose'] = pose(
                kinematics, transformer, payload.get('target'), payload.get('frame'))
        return result
    start = vector(payload.get('start_hw_rad'))
    ok, message = transformer.check_hardware_limits(start)
    if not ok:
        raise ValueError(message)
    planner = TrajectoryPlanner(kinematics)
    if 'preset' in payload:
        ok, points, message = plan_preset(planner, transformer, params['planning'], start,
                                          payload['preset'])
    else:
        ok, points, message = plan_target(
            planner, transformer, params['planning'], start, vector(payload.get('target')),
            payload.get('frame'), payload.get('duration_ms', 0), payload.get('gripper', 0),
            payload.get('linear', False))
    if not ok:
        raise ValueError(message)
    error = validate_trajectory(points, start, transformer)
    if error:
        raise ValueError(error)
    result['points'] = [dict(pose(kinematics, transformer, list(point.joints), 'hardware_rad'),
                             time_ms=point.time_from_start, gripper=point.gripper_action,
                             is_via_point=point.is_via_point) for point in points]
    result['duration_ms'] = points[-1].time_from_start
    result['point_count'] = len(points)
    return result
