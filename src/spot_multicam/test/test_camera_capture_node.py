"""
Smoke test for camera_capture_node using the mock backend.

Run with:
    colcon test --packages-select spot_multicam
or directly (after sourcing the workspace):
    python3 -m pytest src/spot_multicam/test/test_camera_capture_node.py
"""

import json
import time

import pytest
import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from spot_multicam.camera_capture_node import CameraCaptureNode


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_mock_node_publishes_image_and_metadata():
    node = CameraCaptureNode()

    received = {'image': None, 'metadata': None}

    def on_image(msg):
        received['image'] = msg

    def on_metadata(msg):
        received['metadata'] = msg

    node.create_subscription(Image, 'image_raw', on_image, 10)
    node.create_subscription(String, 'metadata', on_metadata, 10)

    # spin briefly so the timer fires and the subscriptions receive at least
    # one message - generous timeout since this may run on slow CI hardware
    deadline = time.time() + 5.0
    while time.time() < deadline and (received['image'] is None or received['metadata'] is None):
        rclpy.spin_once(node, timeout_sec=0.1)

    assert received['image'] is not None, 'No Image message received within timeout'
    assert received['metadata'] is not None, 'No metadata message received within timeout'

    assert received['image'].encoding == 'bgr8'
    assert received['image'].width > 0
    assert received['image'].height > 0

    metadata = json.loads(received['metadata'].data)
    assert metadata['camera_prefix'] == 'cam1'
    assert metadata['mock'] is True

    node.destroy_node()
