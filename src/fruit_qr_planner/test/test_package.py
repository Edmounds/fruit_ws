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
            {"rclpy", "sensor_msgs", "std_msgs", "cv_bridge", "python3-opencv"}
            <= dependencies
        )

    def test_launch_file_is_valid_python(self):
        launch_source = (PACKAGE_ROOT / "launch" / "fruit_qr_planner.launch.py").read_text()

        ast.parse(launch_source)

    def test_resource_marker_exists(self):
        self.assertTrue((PACKAGE_ROOT / "resource" / "fruit_qr_planner").is_file())


if __name__ == "__main__":
    unittest.main()
