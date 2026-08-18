from glob import glob
import os

from setuptools import find_packages, setup

package_name = "fruit_waypoint_recorder"

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
    description="Record competition and test waypoints from the RViz waypoint-maker panel.",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "waypoint_recorder_node = "
            "fruit_waypoint_recorder.waypoint_recorder_node:main",
        ],
    },
)
