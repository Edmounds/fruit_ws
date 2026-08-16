import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ElementTree


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class PackageMetadataTest(unittest.TestCase):
    def test_package_xml_declares_runtime_dependencies(self):
        root = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()
        dependencies = {element.text for element in root.findall("exec_depend")}

        self.assertTrue(
            {"rclpy", "sensor_msgs", "std_msgs", "python3-opencv", "python3-numpy"}
            <= dependencies
        )

    def test_launch_file_is_valid_python(self):
        launch_source = (PACKAGE_ROOT / "launch" / "fruit_qr_planner.launch.py").read_text()

        ast.parse(launch_source)

    def test_d435i_color_is_the_default_scan_topic(self):
        config_source = (PACKAGE_ROOT / "config" / "fruit_qr_planner.yaml").read_text()

        self.assertIn('/camera/camera/color/image_raw', config_source)

    def test_launch_loads_configuration_from_source_workspace(self):
        launch_source = (PACKAGE_ROOT / "launch" / "fruit_qr_planner.launch.py").read_text()

        self.assertIn("/home/rc4/fruit_ws/src/fruit_qr_planner/config/", launch_source)

    def test_resource_marker_exists(self):
        self.assertTrue((PACKAGE_ROOT / "resource" / "fruit_qr_planner").is_file())


if __name__ == "__main__":
    unittest.main()
