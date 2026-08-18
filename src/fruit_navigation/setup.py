from glob import glob
import os

from setuptools import find_packages, setup

package_name = "fruit_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="fruit_ws team",
    maintainer_email="fruit-ws@example.com",
    description="Fixed-field cardinal-axis navigation and route visualization.",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "axis_waypoint_action_server = "
            "fruit_navigation.axis_waypoint_action_server:main",
            "route_marker_node = fruit_navigation.route_marker_node:main",
            "route_file_client = fruit_navigation.route_file_client:main",
        ],
    },
)
