"""
Launch a single camera_capture_node for Phase 1 testing.

Usage:
    ros2 launch spot_multicam single_camera.launch.py use_mock:=true
    ros2 launch spot_multicam single_camera.launch.py use_mock:=false serial:=4103123456

Parameters not overridden on the command line fall back to
config/single_camera_params.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('spot_multicam')
    default_params = os.path.join(pkg_share, 'config', 'single_camera_params.yaml')

    use_mock_arg = DeclareLaunchArgument(
        'use_mock', default_value='true',
        description='If true, use synthetic frames instead of a real IDS camera.',
    )
    serial_arg = DeclareLaunchArgument(
        'serial', default_value='',
        description='IDS camera serial number (ignored when use_mock:=true).',
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Path to the YAML parameter file for this camera.',
    )

    camera_node = Node(
        package='spot_multicam',
        executable='camera_capture_node',
        name='camera_capture_node',
        namespace='cam1',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'use_mock': LaunchConfiguration('use_mock'),
                'serial': LaunchConfiguration('serial'),
                'camera_prefix': 'cam1',
            },
        ],
    )

    return LaunchDescription([
        use_mock_arg,
        serial_arg,
        params_file_arg,
        camera_node,
    ])
