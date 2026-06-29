"""
camera_tf_broadcaster.py

Phase 6: publishes static tf2 transforms from each camera's optical frame
to Spot's body frame, so the rest of the ROS2 graph knows where each camera
is physically mounted on the robot.

ROS2 concepts introduced:
  - tf2_ros.StaticTransformBroadcaster: publishes fixed frame relationships
    on /tf_static once at startup (no timer needed - static means it doesn't
    change while the system is running)
  - geometry_msgs/TransformStamped: translation (x, y, z in metres) +
    rotation as a quaternion (x, y, z, w) - the standard ROS2 way to
    represent a 6-DOF pose between two frames
  - Frame naming convention: parent = 'spot_body', child = 'cam1_optical'

Why this matters:
  Without tf2 transforms, /cam1/image_raw and /cam1/metadata are spatially
  orphaned - the system doesn't know where cam1 is relative to the robot.
  Once this node runs, any node that needs to project a point cloud into
  camera space, or align images from multiple cameras, can use
  tf2_ros.Buffer.lookup_transform() to get the exact relationship.

  This is also the foundation for Phase 7 alignment work - registering
  LiDAR point clouds into the camera frames requires knowing exactly where
  each camera is in the robot's coordinate system.

Coordinate convention (ROS REP-103):
  x = forward, y = left, z = up (robot body frame)
  Camera optical frame: x = right, y = down, z = forward (into the scene)

Placeholder values:
  The translation and rotation values below are placeholders matching a
  reasonable front-mounted camera configuration on Spot. Replace them with
  your actual measured mounting offsets once you have them.
  Parameters are declared so they can be set from a YAML file without
  recompiling - just update config/camera_transforms.yaml with real values.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


# Placeholder mounting offsets - replace with real measured values.
# Format: [tx, ty, tz, qx, qy, qz, qw]
# Translation in metres relative to spot_body origin.
# Rotation as quaternion (identity = no rotation relative to body frame).
CAMERA_DEFAULTS = {
    'cam1': {
        'tx': 0.4,    # 40cm forward on the robot body
        'ty': -0.1,   # 10cm to the right
        'tz': 0.1,    # 10cm above body origin
        'qx': 0.0,
        'qy': 0.0,
        'qz': 0.0,
        'qw': 1.0,    # identity rotation - camera aligned with body frame axes
    },
    'cam2': {
        'tx': 0.4,
        'ty': 0.1,    # 10cm to the left (symmetric with cam1)
        'tz': 0.1,
        'qx': 0.0,
        'qy': 0.0,
        'qz': 0.0,
        'qw': 1.0,
    },
}


class CameraTfBroadcaster(Node):
    """Publishes static tf2 transforms for all camera frames on startup."""

    def __init__(self):
        super().__init__('camera_tf_broadcaster')

        self._broadcaster = StaticTransformBroadcaster(self)

        # Declare parameters for each camera's transform so real mounting
        # offsets can be set from camera_transforms.yaml without code changes.
        transforms = []
        for cam_name, defaults in CAMERA_DEFAULTS.items():
            for param, default_val in defaults.items():
                self.declare_parameter(f'{cam_name}.{param}', default_val)

            tx = self.get_parameter(f'{cam_name}.tx').value
            ty = self.get_parameter(f'{cam_name}.ty').value
            tz = self.get_parameter(f'{cam_name}.tz').value
            qx = self.get_parameter(f'{cam_name}.qx').value
            qy = self.get_parameter(f'{cam_name}.qy').value
            qz = self.get_parameter(f'{cam_name}.qz').value
            qw = self.get_parameter(f'{cam_name}.qw').value

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'spot_body'          # parent frame
            t.child_frame_id = f'{cam_name}_optical'  # child frame

            t.transform.translation.x = tx
            t.transform.translation.y = ty
            t.transform.translation.z = tz
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            transforms.append(t)
            self.get_logger().info(
                f'Broadcasting static transform: spot_body -> {cam_name}_optical '
                f'[tx={tx:.3f}, ty={ty:.3f}, tz={tz:.3f}] '
                f'[qx={qx:.3f}, qy={qy:.3f}, qz={qz:.3f}, qw={qw:.3f}]'
            )

        # Publish all transforms in a single call - StaticTransformBroadcaster
        # latches them on /tf_static so late-joining nodes get them immediately.
        self._broadcaster.sendTransform(transforms)
        self.get_logger().info(
            f'Published {len(transforms)} static transforms on /tf_static'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CameraTfBroadcaster()
    # spin() keeps the node alive so the transforms remain latched.
    # StaticTransformBroadcaster re-sends on new subscriptions automatically.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
