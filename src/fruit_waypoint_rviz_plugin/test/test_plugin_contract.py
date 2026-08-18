# Copyright 2026 fruit_ws team
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

from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_plugin_description_exports_rviz_panel():
    xml_path = PACKAGE_ROOT / "plugins" / "plugin_description.xml"
    root = ET.fromstring(xml_path.read_text(encoding="utf-8"))
    panel = root.find("class")

    assert root.attrib["path"] == "fruit_waypoint_rviz_plugin"
    assert panel.attrib["name"] == "fruit_waypoint_rviz_plugin/WaypointMakerPanel"
    assert panel.attrib["type"] == "fruit_waypoint_rviz_plugin::WaypointMakerPanel"
    assert panel.attrib["base_class_type"] == "rviz_common::Panel"


def test_package_registers_plugin_and_installs_runtime_files():
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    package = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
    launch = PACKAGE_ROOT / "launch" / "waypoint_maker_rviz.launch.py"

    compile(launch.read_text(encoding="utf-8"), str(launch), "exec")
    assert "pluginlib_export_plugin_description_file" in cmake
    assert "plugins/plugin_description.xml" in cmake
    assert "<rviz_common plugin=" in package


def test_panel_has_yaml_loader_and_separate_test_record_buttons():
    source = (PACKAGE_ROOT / "src" / "waypoint_maker_panel.cpp").read_text(
        encoding="utf-8"
    )
    header = (
        PACKAGE_ROOT / "include" / "fruit_waypoint_rviz_plugin"
        / "waypoint_maker_panel.hpp"
    ).read_text(encoding="utf-8")

    assert "QFileDialog::getOpenFileName" in source
    assert '"/home/rc4/fruit_ws/src"' in source
    assert '"load_test_file"' in source
    assert '"Set Waypoint"' in source
    assert '"Set Goal"' in source
    assert "QComboBox" not in header
