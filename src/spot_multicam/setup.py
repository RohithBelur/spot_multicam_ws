import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'spot_multicam'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rohith Belur Ravish',
    maintainer_email='rohithbr@gmx.de',
    description=(
        'ROS2 reimplementation of the distributed IDS multi-camera capture system '
        '(MultiCamera-IDS-Capture), starting with a single-camera node.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_capture_node = spot_multicam.camera_capture_node:main',
            'capture_mission_service = spot_multicam.mission.capture_mission_service:main',
        ],
    },
)
