from glob import glob
import os

from setuptools import find_packages, setup


package_name = "fruit_qr_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="fruit_ws team",
    maintainer_email="fruit-ws@example.com",
    description="Decode B/C competition QR codes and publish ordered JSON pick plans.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fruit_qr_planner_node = fruit_qr_planner.qr_node:main",
        ],
    },
)

