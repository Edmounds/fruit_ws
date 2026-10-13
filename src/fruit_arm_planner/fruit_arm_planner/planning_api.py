"""Pure configuration and planning entry points shared by ROS and web preview."""

import copy
import hashlib
import json
import math

from fruit_arm_planner.joint_transformer import JointTransformer
from fruit_arm_planner.kinematics import RobotKinematics
from fruit_arm_planner.trajectory_planner import GripperAction

MAX_TRAJECTORY_POINTS = 64


def validate_presets(presets):
    """Validate a complete preset replacement before installing or saving it."""
    if not isinstance(presets, dict):
        raise ValueError('presets must be a mapping')
    for name, preset in presets.items():
        if (not isinstance(name, str) or not name.strip() or not isinstance(preset, dict)
                or preset.get('coordinate_frame') not in ('joints', 'cartesian', 'link_angles')
                or not isinstance(preset.get('waypoints'), list) or not preset['waypoints']):
            raise ValueError(f'Invalid preset {name}')
        for wp in preset['waypoints']:
            if not isinstance(wp, dict):
                raise ValueError(f'Invalid waypoint in preset {name}')
            target = wp.get('target')
            gripper = wp.get('gripper', 0)
            dwell_ms = wp.get('dwell_ms', 0)
            if (not isinstance(target, list) or len(target) < 4
                    or not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                               and math.isfinite(v) for v in target)
                    or not isinstance(wp.get('duration_ms'), int)
                    or isinstance(wp['duration_ms'], bool)
                    or not 0 < wp['duration_ms'] <= 0xFFFFFFFF
                    or not isinstance(dwell_ms, int) or isinstance(dwell_ms, bool)
                    or not 0 <= dwell_ms <= 0xFFFFFFFF
                    or isinstance(gripper, bool) or not isinstance(gripper, int)
                    or gripper not in (0, 1, 2)
                    or not isinstance(wp.get('linear', False), bool)
                    or not isinstance(wp.get('is_via_point', False), bool)):
                raise ValueError(f'Invalid waypoint in preset {name}')
    revision(presets)


def configuration(params):
    """Validate and normalize the four configuration sections without ROS state."""
    if not isinstance(params, dict):
        raise ValueError('Configuration must be a mapping')
    result = copy.deepcopy({key: params[key] for key in
                            ('geometry', 'joint_mapping', 'planning', 'presets')})
    RobotKinematics.from_geometry_dict(result['geometry'])
    for key in ('q1', 'q2', 'q3', 'q4'):
        channel = result['joint_mapping'][key]
        limits = channel['limits_deg']
        if (len(limits) != 2 or not all(math.isfinite(float(v)) for v in limits)
                or float(limits[0]) >= float(limits[1])
                or not math.isfinite(float(channel['offset_deg']))
                or float(channel['direction']) not in (-1.0, 1.0)):
            raise ValueError(f'Invalid joint_mapping.{key}')
    JointTransformer.from_config_dict(result['joint_mapping'])
    planning = result['planning']
    for key in ('default_duration_ms', 'interpolation_step_ms', 'default_dwell_ms'):
        value = planning[key]
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f'planning.{key} must be an integer')
    if (not 0 < planning['default_duration_ms'] <= 0xFFFFFFFF
            or not 0 < planning['interpolation_step_ms'] <= 0xFFFFFFFF
            or not 0 <= planning['default_dwell_ms'] <= 0xFFFFFFFF
            or not 0 < float(planning['status_max_age_sec']) < 10
            or not math.isfinite(float(planning['hover_height_m']))):
        raise ValueError('Invalid planning parameters')
    validate_presets(result['presets'])
    # Reject non-JSON YAML objects and non-finite values, including unknown fields.
    revision(result)
    return result


def revision(value):
    """Hash semantic values, treating numeric 1 and 1.0 as equivalent."""
    def normalize(item):
        if isinstance(item, dict):
            return {key: normalize(val) for key, val in item.items()}
        if isinstance(item, list):
            return [normalize(val) for val in item]
        if isinstance(item, (int, float)) and not isinstance(item, bool):
            return 0.0 if item == 0 else float(item)
        return item
    encoded = json.dumps(normalize(value), sort_keys=True, ensure_ascii=False,
                         allow_nan=False, separators=(',', ':')).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def validate_trajectory(points, start, transformer):
    """Validate the same wire point stream for preview and dispatch."""
    if not 2 <= len(points) <= MAX_TRAJECTORY_POINTS:
        return (f'Trajectory point count must be 2..{MAX_TRAJECTORY_POINTS} '
                f'(MCU capacity, got {len(points)})')
    if (points[0].joints != start or points[0].gripper_action != GripperAction.HOLD
            or points[0].is_via_point != 0):
        return 'Trajectory first point must be measured joints with HOLD'
    previous_time = -1
    for index, point in enumerate(points):
        if point.point_id != index or not previous_time < point.time_from_start <= 0xFFFFFFFF:
            return f'Invalid point ID or timestamp at point {index}'
        if not all(math.isfinite(q) for q in point.joints):
            return f'Non-finite joint angle at point {index}'
        ok, message = transformer.check_hardware_limits(point.joints)
        if not ok:
            return f'Point {index}: {message}'
        if point.gripper_action not in (0, 1, 2) or point.is_via_point not in (0, 1):
            return f'Invalid action or via-point flag at point {index}'
        previous_time = point.time_from_start
    if points[0].time_from_start != 0:
        return 'Trajectory must start at 0 ms'
    return None


def sample_count(duration, step):
    return max(2, min(duration + 1, math.ceil(duration / step) + 1))


def plan_target(planner, transformer, planning, start, target, frame,
                duration_ms=0, gripper=0, linear=False):
    """Plan one formal move request without acquiring motion ownership."""
    if (isinstance(duration_ms, bool) or not isinstance(duration_ms, int)
            or not 0 <= duration_ms <= 0xFFFFFFFF):
        return False, [], 'Invalid duration'
    duration = duration_ms or planning['default_duration_ms']
    if (isinstance(duration, bool) or not isinstance(duration, int)
            or not 0 < duration <= 0xFFFFFFFF or len(target) != 4
            or not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                       and math.isfinite(v) for v in target)
            or isinstance(gripper, bool) or not isinstance(gripper, int)
            or gripper not in (0, 1, 2)
            or not isinstance(linear, bool)):
        return False, [], 'Invalid target, duration or gripper action'
    count = sample_count(duration, planning['interpolation_step_ms'])
    if count > MAX_TRAJECTORY_POINTS:
        return False, [], f'Trajectory point count exceeds {MAX_TRAJECTORY_POINTS} (got {count})'
    if frame == 'cartesian':
        pitch = math.radians(target[3])
        if linear:
            fk = planner.kinematics.forward_kinematics(*transformer.hardware_to_algo(start))
            return planner.plan_cartesian_linear_path(
                start_xyz=fk['end_effector'][:3], target_xyz=tuple(target[:3]),
                start_pitch=fk['end_effector'][3], target_pitch=pitch,
                duration_ms=duration, step_ms=planning['interpolation_step_ms'],
                transformer=transformer, start_q_hw=start, gripper_action=gripper)
        ok, algo, message = planner.kinematics.inverse_kinematics(
            *target[:3], pitch=pitch, force_elbow_up=True)
        if not ok:
            return False, [], f'PTP IK failed: {message}'
        hardware = transformer.algo_to_hardware(algo)
    elif frame in ('joints', 'link_angles') and not linear:
        hardware = (transformer.link_angles_to_hardware(target) if frame == 'link_angles'
                    else tuple(math.radians(v) for v in target))
    else:
        return False, [], 'LIN requires Cartesian coordinates'
    ok, message = transformer.check_hardware_limits(hardware)
    if not ok:
        return False, [], message
    return True, planner.plan_joint_point_to_point(
        start_q=start, end_q=hardware, duration_ms=duration, num_points=count,
        gripper_action=gripper), ''


def plan_preset(planner, transformer, planning, start, preset):
    """Preflight capacity before allocating a potentially large point stream."""
    validate_presets({'preview': preset})
    count = 1
    overflow_at = None
    for index, wp in enumerate(preset['waypoints']):
        count += sample_count(wp['duration_ms'], planning['interpolation_step_ms']) - 1
        if wp.get('gripper', 0) != 0:
            dwell = wp.get('dwell_ms', planning['default_dwell_ms'])
            count += math.ceil(dwell / planning['interpolation_step_ms'])
        if count > MAX_TRAJECTORY_POINTS and overflow_at is None:
            overflow_at = index
    if count > MAX_TRAJECTORY_POINTS:
        return False, [], (f'Waypoint {overflow_at} exceeds capacity. Trajectory point count '
                           f'exceeds {MAX_TRAJECTORY_POINTS} (got {count})')
    return planner.plan_multi_waypoint_sequence(
        current_q_hw=start, waypoints=preset['waypoints'], transformer=transformer,
        coordinate_frame=preset['coordinate_frame'],
        step_ms=planning['interpolation_step_ms'], default_dwell_ms=planning['default_dwell_ms'])
