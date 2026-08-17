#!/usr/bin/env python3

# Copyright 2026 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import math
from pathlib import Path

import yaml


def _normalized_quaternion(values, label):
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise ValueError(f"{label} must contain four finite values")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-12:
        raise ValueError(f"{label} must not be zero")
    return tuple(value / norm for value in values)


def _multiply_quaternions(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _rotate_vector(quaternion, vector):
    qx, qy, qz, qw = quaternion
    x, y, z = vector
    return (
        (1.0 - 2.0 * (qy * qy + qz * qz)) * x
        + 2.0 * (qx * qy - qz * qw) * y
        + 2.0 * (qx * qz + qy * qw) * z,
        2.0 * (qx * qy + qz * qw) * x
        + (1.0 - 2.0 * (qx * qx + qz * qz)) * y
        + 2.0 * (qy * qz - qx * qw) * z,
        2.0 * (qx * qz - qy * qw) * x
        + 2.0 * (qy * qz + qx * qw) * y
        + (1.0 - 2.0 * (qx * qx + qy * qy)) * z,
    )


def _quaternion_to_rpy(quaternion):
    qx, qy, qz, qw = quaternion
    roll = math.atan2(
        2.0 * (qw * qx + qy * qz),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )
    pitch_term = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(pitch_term)
    yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    return roll, pitch, yaw


def _read_first_lidar_pose(trajectory_path):
    for line in Path(trajectory_path).read_text().splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        values = [float(value) for value in line.split()]
        if len(values) != 8 or not all(math.isfinite(value) for value in values):
            raise ValueError("first trajectory pose must be timestamp x y z qx qy qz qw")
        return values[1:4], _normalized_quaternion(values[4:8], "trajectory quaternion")
    raise ValueError(f"trajectory has no poses: {trajectory_path}")


def calculate_initial_pose(trajectory_path, sensor_frames_path):
    map_lidar_translation, map_lidar_rotation = _read_first_lidar_pose(trajectory_path)
    sensor_frames = yaml.safe_load(Path(sensor_frames_path).read_text())
    transform = sensor_frames["base_to_front_mid360"]
    if transform.get("parent_frame") != "base_link":
        raise ValueError("base_to_front_mid360 parent_frame must be base_link")
    if transform.get("child_frame") != "front_mid360":
        raise ValueError("base_to_front_mid360 child_frame must be front_mid360")

    base_lidar_translation = [float(value) for value in transform["translation"]]
    if len(base_lidar_translation) != 3 or not all(
        math.isfinite(value) for value in base_lidar_translation
    ):
        raise ValueError("base_to_front_mid360 translation must contain three finite values")
    base_lidar_rotation = _normalized_quaternion(
        [float(value) for value in transform["rotation"]],
        "base_to_front_mid360 rotation",
    )

    inverse_base_lidar_rotation = (
        -base_lidar_rotation[0],
        -base_lidar_rotation[1],
        -base_lidar_rotation[2],
        base_lidar_rotation[3],
    )
    map_base_rotation = _normalized_quaternion(
        _multiply_quaternions(map_lidar_rotation, inverse_base_lidar_rotation),
        "map_to_base rotation",
    )
    rotated_offset = _rotate_vector(map_base_rotation, base_lidar_translation)
    map_base_translation = [
        map_lidar_translation[index] - rotated_offset[index] for index in range(3)
    ]
    return [*map_base_translation, *_quaternion_to_rpy(map_base_rotation)]


def write_pose_parameters(output_path, pose):
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    parameters = {
        "/**": {
            "ros__parameters": {
                "initial_map_to_odom": [round(value, 9) for value in pose]
            }
        }
    }
    output.write_text(yaml.safe_dump(parameters, sort_keys=False))


def main():
    parser = argparse.ArgumentParser(
        description="Generate the localization initial pose from a GLIM LiDAR trajectory"
    )
    parser.add_argument("--trajectory", required=True, type=Path)
    parser.add_argument("--sensor-frames", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    pose = calculate_initial_pose(args.trajectory, args.sensor_frames)
    write_pose_parameters(args.output, pose)
    print("initial_map_to_odom: [" + ", ".join(f"{value:.9f}" for value in pose) + "]")
    print(f"wrote: {args.output}")


if __name__ == "__main__":
    main()
