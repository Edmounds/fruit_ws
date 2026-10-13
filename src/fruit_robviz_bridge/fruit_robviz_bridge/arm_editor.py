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


def parse_preset_order(source):
    text = source.decode('utf-8') if isinstance(source, bytes) else source
    root = yaml.compose(text)
    if not isinstance(root, yaml.MappingNode):
        return []
    node = root
    for key in ('arm_planner_node', 'ros__parameters', 'presets'):
        node = next(
            (v for k, v in node.value if isinstance(k, yaml.ScalarNode) and k.value == key),
            None,
        )
        if node is None or not isinstance(node, yaml.MappingNode):
            return []
    return [k.value for k, v in node.value if isinstance(k, yaml.ScalarNode)]


def reorder_presets_blocks(text, preset_order):
    root = yaml.compose(text)
    node = root
    presets_key_node = None
    for name_key in ('arm_planner_node', 'ros__parameters', 'presets'):
        for k, v in node.value:
            if isinstance(k, yaml.ScalarNode) and k.value == name_key:
                presets_key_node = k
                node = v
                break
        else:
            raise ValueError(f'Cannot find node {name_key}')
    presets_node = node
    if not isinstance(presets_node, yaml.MappingNode) or presets_node.flow_style:
        raise ValueError('presets is not a block mapping')
    visited_ids = set()
    for k, v in presets_node.value:
        if id(k) in visited_ids or id(v) in visited_ids:
            raise ValueError('Aliases not supported')
        visited_ids.add(id(k))
        visited_ids.add(id(v))
        if not isinstance(k, yaml.ScalarNode):
            raise ValueError('Non-scalar key in presets')
        if k.value == '<<':
            raise ValueError('Merge key << not supported')

    keys = [k.value for k, v in presets_node.value]
    if len(keys) != len(set(keys)):
        raise ValueError('Duplicate keys in presets')
    if set(keys) != set(preset_order) or len(keys) != len(preset_order):
        raise ValueError('preset_order mismatch')

    if keys == preset_order or not presets_node.value:
        return text

    preset_indent = presets_node.value[0][0].start_mark.column
    if preset_indent <= presets_key_node.start_mark.column:
        raise ValueError('Invalid preset indentation')

    start_positions = {}
    for k, v in presets_node.value:
        if k.start_mark.column != preset_indent:
            raise ValueError(f'Inconsistent indentation for preset {k.value}')
        line_start = text.rfind('\n', 0, k.start_mark.index) + 1
        prefix = text[line_start:k.start_mark.index]
        if prefix != ' ' * k.start_mark.column:
            raise ValueError(f'Preset key {k.value} not at beginning of line after indentation')
        start_positions[k.value] = line_start

    end_positions = {}
    for i in range(len(keys) - 1):
        end_positions[keys[i]] = start_positions[keys[i + 1]]

    last_val = presets_node.value[-1][1]
    idx = last_val.end_mark.index
    curr = text.rfind('\n', 0, idx) + 1
    while curr < len(text):
        nl = text.find('\n', curr)
        line = text[curr:] if nl == -1 else text[curr:nl]
        stripped = line.strip()
        if stripped and not stripped.startswith('#'):
            indent = len(line) - len(line.lstrip(' '))
            if indent <= presets_key_node.start_mark.column or stripped in ('...', '---'):
                last_end = curr
                break
        curr = len(text) if nl == -1 else nl + 1
    else:
        last_end = len(text)
    end_positions[keys[-1]] = last_end

    blocks = {k: text[start_positions[k]:end_positions[k]] for k in keys}
    orig_slice = text[start_positions[keys[0]]:end_positions[keys[-1]]]
    if ''.join(blocks[k] for k in keys) != orig_slice:
        raise ValueError('Block slice reconstruction failed')

    new_middle = ''.join(blocks[k] for k in preset_order)
    return text[:start_positions[keys[0]]] + new_middle + text[end_positions[keys[-1]]:]


def document(path):
    content = path.read_bytes()
    params = configuration(yaml.safe_load(content)['arm_planner_node']['ros__parameters'])
    preset_order = parse_preset_order(content)
    return {'version': hashlib.sha256(content).hexdigest(),
            'revision': revision(params), 'configuration': params,
            'preset_order': preset_order,
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
        end_idx = item.end_mark.index
        if not flow and item.end_mark.column > 0:
            prefix = text[item.end_mark.index - item.end_mark.column:item.end_mark.index]
            if prefix == ' ' * item.end_mark.column:
                end_idx -= item.end_mark.column
        edits.append((item.start_mark.index, end_idx, rendered))

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
    preset_order = None
    if 'preset_order' in payload:
        order = payload['preset_order']
        if (not isinstance(order, list)
                or not all(isinstance(k, str) and k for k in order)
                or len(order) != len(set(order))
                or set(order) != set(presets.keys())):
            raise ValueError(
                'Invalid preset_order: must match preset keys exactly without duplicates'
            )
        preset_order = order
        presets = {k: presets[k] for k in preset_order}
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
        if preset_order is not None and parse_preset_order(updated) != preset_order:
            updated = reorder_presets_blocks(updated, preset_order)
        parsed = yaml.safe_load(updated)
        expected_document = copy.deepcopy(original)
        expected_document['arm_planner_node']['ros__parameters']['presets'] = presets
        if parsed != expected_document:
            raise ValueError('Cannot preserve YAML structure safely; file unchanged')
        if preset_order is not None and parse_preset_order(updated) != preset_order:
            raise ValueError('Preset order in saved file does not match requested order')
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
