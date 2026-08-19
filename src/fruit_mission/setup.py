from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'fruit_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fruit_ws team',
    maintainer_email='fruit-ws@example.com',
    description='Minimal A/B/C fruit competition mission orchestration.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'current_area_client = fruit_mission.current_area_client:main',
            'd435i_arm_target_client = '
            'fruit_mission.d435i_arm_target_client:main',
            'fruit_mission_node = fruit_mission.mission_node:main',
        ],
    },
)
