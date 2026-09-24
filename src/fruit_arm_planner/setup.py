from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'fruit_arm_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fruit_ws team',
    maintainer_email='fruit-ws@example.com',
    description='4-DOF fruit picking robot arm kinematics, interpolation and trajectory planner.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'arm_planner_node = fruit_arm_planner.arm_planner_node:main',
        ],
    },
)
