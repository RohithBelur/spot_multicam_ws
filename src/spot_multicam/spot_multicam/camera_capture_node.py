"""
camera_capture_node.py

Phase 3 update: metadata publisher switched from std_msgs/String (JSON) to
the proper spot_multicam_msgs/CaptureMetadata message type. Everything else
(parameters, image publisher, backends, namespace layout) is unchanged from
Phase 1/2.

Phase 3 teaches: importing a custom message, populating its fields including
the std_msgs/Header stamp, and publishing it alongside sensor_msgs/Image.
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from spot_multicam_msgs.msg import CaptureMetadata

from spot_multicam.mock_camera_backend import MockCameraBackend
from spot_multicam.ids_camera_backend import IdsCameraBackend


class CameraCaptureNode(Node):

    def __init__(self):
        super().__init__('camera_capture_node')

        # --- Parameters, mirroring the original capture.py CLI flags -------
        self.declare_parameter('use_mock', True)
        self.declare_parameter('camera_prefix', 'cam1')
        self.declare_parameter('serial', '')
        self.declare_parameter('frame_rate', 10.0)
        self.declare_parameter('exposure_us', 10000)
        self.declare_parameter('black_level', 0.0)
        self.declare_parameter('gain_db', 0.0)
        self.declare_parameter('trigger_mode', 'software')
        self.declare_parameter('buffers', 16)
        self.declare_parameter('focus_fixed', -1)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)

        self._use_mock = self.get_parameter('use_mock').value
        self._camera_prefix = self.get_parameter('camera_prefix').value
        serial = self.get_parameter('serial').value
        frame_rate = self.get_parameter('frame_rate').value
        exposure_us = self.get_parameter('exposure_us').value
        black_level = self.get_parameter('black_level').value
        gain_db = self.get_parameter('gain_db').value
        trigger_mode = self.get_parameter('trigger_mode').value
        buffers = self.get_parameter('buffers').value
        focus_fixed = self.get_parameter('focus_fixed').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value

        if frame_rate <= 0:
            raise ValueError('frame_rate parameter must be > 0')

        # --- Publishers ------------------------------------------------------
        self._image_pub = self.create_publisher(Image, 'image_raw', 10)
        # Phase 3: proper typed message instead of JSON string.
        # ros2 topic echo /cam1/metadata now shows structured fields,
        # not a raw JSON blob.
        self._metadata_pub = self.create_publisher(CaptureMetadata, 'metadata', 10)

        # --- Backend selection -----------------------------------------------
        if self._use_mock:
            self.get_logger().info(
                f'[{self._camera_prefix}] Using MockCameraBackend '
                f'(use_mock:=true) - no IDS hardware required.'
            )
            self._backend = MockCameraBackend(
                serial=serial, width=width, height=height, exposure_us=exposure_us,
            )
        else:
            self.get_logger().info(
                f'[{self._camera_prefix}] Using IdsCameraBackend - real hardware expected.'
            )
            self._backend = IdsCameraBackend(
                serial=serial, frame_rate=frame_rate, exposure_us=exposure_us,
                black_level=black_level, gain_db=gain_db, trigger_mode=trigger_mode,
                buffers=buffers, focus_fixed=(None if focus_fixed < 0 else focus_fixed),
            )

        self._backend.open()

        period_s = 1.0 / frame_rate
        self._timer = self.create_timer(period_s, self._on_timer)
        self._frames_published = 0

        self.get_logger().info(
            f'[{self._camera_prefix}] camera_capture_node started: '
            f'frame_rate={frame_rate} Hz, trigger={trigger_mode}, '
            f'exposure_us={exposure_us}, mock={self._use_mock}'
        )

    def _on_timer(self) -> None:
        try:
            frame, metadata = self._backend.grab_frame()
        except NotImplementedError as exc:
            self.get_logger().error(f'[{self._camera_prefix}] {exc}')
            self._timer.cancel()
            return
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'[{self._camera_prefix}] grab_frame() failed: {exc}')
            return

        now = self.get_clock().now().to_msg()

        img_msg = self._frame_to_image_msg(frame)
        # Image header stamp matches metadata stamp - both reference the
        # same capture moment, which matters for tf2 lookups in Phase 6.
        img_msg.header.stamp = now
        img_msg.header.frame_id = self._camera_prefix
        self._image_pub.publish(img_msg)

        meta_msg = CaptureMetadata()
        meta_msg.header.stamp = now
        meta_msg.header.frame_id = self._camera_prefix
        meta_msg.camera_prefix = self._camera_prefix
        meta_msg.serial = str(metadata.get('serial', ''))
        meta_msg.frame_id = int(metadata.get('frame_id', 0))
        meta_msg.exposure_us = int(metadata.get('exposure_us', 0))
        meta_msg.is_mock = bool(metadata.get('mock', False))
        self._metadata_pub.publish(meta_msg)

        self._frames_published += 1
        if self._frames_published % 50 == 0:
            self.get_logger().info(
                f'[{self._camera_prefix}] published {self._frames_published} frames'
            )

    @staticmethod
    def _frame_to_image_msg(frame) -> Image:
        msg = Image()
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = frame.shape[1] * frame.shape[2]
        msg.data = frame.tobytes()
        return msg

    def destroy_node(self):
        if hasattr(self, '_backend') and self._backend.is_open():
            self._backend.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

