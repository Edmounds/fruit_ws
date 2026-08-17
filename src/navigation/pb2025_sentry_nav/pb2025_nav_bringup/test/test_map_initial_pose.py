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

import importlib.util
import math
from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = PACKAGE_ROOT / "scripts/map_initial_pose.py"


def load_module():
    spec = importlib.util.spec_from_file_location("map_initial_pose", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_computes_map_to_base_from_first_lidar_pose(tmp_path):
    trajectory = tmp_path / "traj_lidar.txt"
    trajectory.write_text(
        "1786920894.175406694 -0.009479 0.002742 0.049327 "
        "-0.265802 -0.001955 -0.001132 0.964025\n"
    )
    sensor_frames = tmp_path / "sensor_frames.yaml"
    sensor_frames.write_text(
        yaml.safe_dump(
            {
                "base_to_front_mid360": {
                    "parent_frame": "base_link",
                    "child_frame": "front_mid360",
                    "translation": [0.144629969, 0.0, 0.0],
                    "rotation": [
                        -0.183012702,
                        0.183012702,
                        -0.683012702,
                        0.683012702,
                    ],
                }
            }
        )
    )

    pose = load_module().calculate_initial_pose(trajectory, sensor_frames)

    assert pose == pytest.approx(
        [-0.009653, -0.141873, 0.051420, -0.004372, 0.014471, 1.569590],
        abs=1e-6,
    )


def test_writes_ros_parameter_override(tmp_path):
    output = tmp_path / "field.initial_pose.yaml"
    pose = [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]

    load_module().write_pose_parameters(output, pose)

    params = yaml.safe_load(output.read_text())
    saved = params["/**"]["ros__parameters"]["initial_map_to_odom"]
    assert saved == pose
    assert all(math.isfinite(value) for value in saved)
