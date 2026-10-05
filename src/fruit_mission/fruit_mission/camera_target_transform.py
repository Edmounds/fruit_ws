"""Standalone RGB-optical -> gripper-centre transform; input/output in mm."""

from __future__ import annotations

import math
from pathlib import Path

import yaml


def _vector3(values, name):
    if (not isinstance(values, (list, tuple)) or len(values) != 3
            or not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                       and math.isfinite(v) for v in values)):
        raise ValueError(f'{name} must contain three finite numbers')
    return tuple(float(v) for v in values)


def _rotation3(values):
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise ValueError('rotation must be a 3x3 matrix')
    rows = tuple(_vector3(row, 'rotation row') for row in values)
    # Allow rounding in the stored calibration, but reject scale/reflection matrices.
    for i in range(3):
        for j in range(3):
            dot = sum(rows[i][k] * rows[j][k] for k in range(3))
            if abs(dot - (1.0 if i == j else 0.0)) > 1e-4:
                raise ValueError('rotation must be orthonormal')
    a, b, c = rows
    determinant = (a[0] * (b[1] * c[2] - b[2] * c[1])
                   - a[1] * (b[0] * c[2] - b[2] * c[0])
                   + a[2] * (b[0] * c[1] - b[1] * c[0]))
    if abs(determinant - 1.0) > 1e-4:
        raise ValueError('rotation must have determinant +1')
    return rows


def load_camera_extrinsic(config_path):
    """Return (R_E_C, t_E_C_mm) from YAML; E and screw axes must be parallel."""
    with Path(config_path).open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError('camera extrinsic config must be a mapping')
    rotation = _rotation3(config['rotation_end_from_rgb'])
    end_to_screw = _vector3(config['translation_end_to_screw_mm'], 'E -> S')
    screw_to_rgb = _vector3(config['translation_screw_to_rgb_mm'], 'S -> C')
    translation = _vector3(
        tuple(a + b for a, b in zip(end_to_screw, screw_to_rgb)), 'E -> C')
    return rotation, translation


def transform_camera_to_end_effector_mm(camera_xyz_mm, rotation, translation_mm):
    """P_E = R_E_C * P_C + t_E_C; result is NOT an arm-base coordinate.

    Parameters are explicit so calibration remains in YAML, not in source code.
    The optical origin (0, 0, 0) is allowed for geometric checks; callers must
    reject invalid/missing sensor depth before requesting a grasp.
    """
    point = _vector3(camera_xyz_mm, 'camera point (mm)')
    rows = _rotation3(rotation)
    translation = _vector3(translation_mm, 'translation (mm)')
    return _vector3(tuple(
        sum(row[k] * point[k] for k in range(3)) + offset
        for row, offset in zip(rows, translation)), 'end point (mm)')


def transform_end_to_base_mm(end_xyz_mm, end_origin_mm, yaw_rad, pitch_rad):
    """Apply Rz(yaw) * Ry(-pitch), matching the existing 4-DOF FK convention."""
    x, y, z = _vector3(end_xyz_mm, 'end point (mm)')
    origin = _vector3(end_origin_mm, 'end origin in base (mm)')
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool)
               and math.isfinite(v) for v in (yaw_rad, pitch_rad)):
        raise ValueError('yaw and pitch must be finite radians')
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    radial = cp * x - sp * z
    return _vector3((
        origin[0] + cy * radial - sy * y,
        origin[1] + sy * radial + cy * y,
        origin[2] + sp * x + cp * z,
    ), 'base point (mm)')


def apply_target_offset_mm(base_xyz_mm, offset_m, pitch_rad):
    """Apply end-effector-relative offset [dx, dy, dz] in meters to base point in mm.

    Offset is rotated by Rz(atan2(base_y, base_x)) * Ry(-pitch_rad) and added
    to base_xyz_mm. Local +X is gripper forward, +Y is gripper left, +Z is gripper up.
    """
    bx, by, bz = _vector3(base_xyz_mm, 'base point (mm)')
    dx, dy, dz = _vector3(offset_m, 'target offset (m)')
    if (not isinstance(pitch_rad, (int, float)) or isinstance(pitch_rad, bool)
            or not math.isfinite(pitch_rad)):
        raise ValueError('pitch must be a finite number of radians')
    yaw_rad = math.atan2(by, bx)
    offset_local_mm = (dx * 1000.0, dy * 1000.0, dz * 1000.0)
    return transform_end_to_base_mm(offset_local_mm, (bx, by, bz), yaw_rad, float(pitch_rad))


if __name__ == '__main__':
    import argparse
    import json

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('xyz_mm', nargs=3, type=float, metavar='MM')
    parser.add_argument(
        '--config',
        default='/home/rc4/fruit_ws/src/fruit_mission/config/camera_extrinsic.yaml')
    args = parser.parse_args()
    try:
        rotation, translation = load_camera_extrinsic(args.config)
        result = transform_camera_to_end_effector_mm(args.xyz_mm, rotation, translation)
    except (OSError, ValueError, KeyError, yaml.YAMLError) as error:
        parser.error(str(error))
    print(json.dumps({'camera_mm': args.xyz_mm, 'end_effector_mm': result}))
