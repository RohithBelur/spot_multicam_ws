"""
Launch multiple camera_capture_node instances, one per namespace - the Phase 2
scale-up. This is the ROS2 equivalent of the original REMOTE_NODES_JSON list:
each entry there became an SSH target; here, each entry becomes a namespaced
ROS2 node, and DDS discovery handles the rest (no SSH/rsync needed once each
node's machine is on the same ROS_DOMAIN_ID).

Written alongside Phase 1 so the architecture is scale-up-ready from day one,
but NOT yet hardware-tested across the real Raspberry Pi nodes - that
validation is the next step once single-camera testing is solid.

Usage:
    ros2 launch spot_multicam multi_camera.launch.py
    ros2 launch spot_multicam multi_camera.launch.py use_mock:=true   # test without any hardware
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import yaml


def generate_launch_description():
    pkg_share = get_package_share_directory('spot_multicam')
    config_path = os.path.join(pkg_share, 'config', 'multi_camera_params.yaml')

    use_mock_arg = DeclareLaunchArgument(
        'use_mock', default_value='true',
        description='If true, all cameras use synthetic frames instead of real hardware.',
    )

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    nodes = []
    # multi_camera_params.yaml has one top-level key per camera (cam1, cam2, ...),
    # each with its own 'ros__parameters' block - same shape as a single-camera
    # params file, just concatenated. This mirrors how REMOTE_NODES_JSON listed
    # one block per Pi.
    for camera_namespace, camera_config in config.items():
        params = dict(camera_config.get('ros__parameters', {}))
        params['camera_prefix'] = camera_namespace
        params['use_mock'] = LaunchConfiguration('use_mock')

        nodes.append(Node(
            package='spot_multicam',
            executable='camera_capture_node',
            name='camera_capture_node',
            namespace=camera_namespace,
            output='screen',
            parameters=[params],
        ))

    return LaunchDescription([use_mock_arg, *nodes])
