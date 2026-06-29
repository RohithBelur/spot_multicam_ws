"""
full_system.launch.py

Phase 4: launches the complete system in one command —
  - cam1 and cam2 camera capture nodes (mock or real)
  - capture_action_server monitoring both cameras

This is the equivalent of the original system's startup sequence
(SSH into each Pi, start capture.py, start robot_command_mission_service.py)
but as a single ROS2 launch file.

The Spot RemoteMissionService (capture_mission_service) is kept separate
since it's a standalone gRPC server, not a ROS2 node — start it independently:
    ros2 run spot_multicam capture_mission_service local --port 24567

Usage:
    ros2 launch spot_multicam full_system.launch.py use_mock:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('spot_multicam')
    multi_cam_params = os.path.join(pkg_share, 'config', 'multi_camera_params.yaml')

    use_mock_arg = DeclareLaunchArgument(
        'use_mock', default_value='true',
        description='Use mock camera backend (no IDS hardware required)',
    )

    cam1_node = Node(
        package='spot_multicam',
        executable='camera_capture_node',
        name='camera_capture_node',
        namespace='cam1',
        output='screen',
        parameters=[
            multi_cam_params,
            {'use_mock': LaunchConfiguration('use_mock'), 'camera_prefix': 'cam1'},
        ],
    )

    cam2_node = Node(
        package='spot_multicam',
        executable='camera_capture_node',
        name='camera_capture_node',
        namespace='cam2',
        output='screen',
        parameters=[
            multi_cam_params,
            {'use_mock': LaunchConfiguration('use_mock'), 'camera_prefix': 'cam2'},
        ],
    )

    action_server_node = Node(
        package='spot_multicam',
        executable='capture_action_server',
        name='capture_action_server',
        output='screen',
    )

    tf_broadcaster_node = Node(
        package='spot_multicam',
        executable='camera_tf_broadcaster',
        name='camera_tf_broadcaster',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'camera_transforms.yaml')],
    )

    return LaunchDescription([
        use_mock_arg,
        cam1_node,
        cam2_node,
        action_server_node,
        tf_broadcaster_node,
    ])
