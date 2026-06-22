"""
camera_capture_node.py

Phase 1 of the spot_multicam ROS2 port: a single-camera capture node that
declares the same tunables as the original capture.py CLI flags, but exposes
them as ROS2 parameters, and publishes frames on a topic instead of writing
files to disk.

Namespace-ready by design: launch two instances under different namespaces
(see launch/multi_camera.launch.py) and you get cam1/image_raw, cam2/image_raw,
etc. with zero code changes - that's the Phase 2 scale-up, this node doesn't
need to know how many siblings it has.
"""

import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from spot_multicam.mock_camera_backend import MockCameraBackend
from spot_multicam.ids_camera_backend import IdsCameraBackend


class CameraCaptureNode(Node):

    def __init__(self):
        super().__init__('camera_capture_node')

        # --- Parameters, mirroring the original capture.py CLI flags -------
        self.declare_parameter('use_mock', True)
        self.declare_parameter('camera_prefix', 'cam1')
        self.declare_parameter('serial', '')
        self.declare_parameter('frame_rate', 10.0)         # Hz, matches --frame-rate
        self.declare_parameter('exposure_us', 10000)        # matches --exposure-us
        self.declare_parameter('black_level', 0.0)          # matches --black-level
        self.declare_parameter('gain_db', 0.0)               # matches --gain-db
        self.declare_parameter('trigger_mode', 'software')   # matches --trigger
        self.declare_parameter('buffers', 16)                 # matches --buffers
        self.declare_parameter('focus_fixed', -1)             # matches --focus-fixed, -1 = unset
        self.declare_parameter('width', 640)                  # mock backend only
        self.declare_parameter('height', 480)                 # mock backend only

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
        # Topic names are relative (no leading '/'), so namespacing at launch
        # time (see launch/multi_camera.launch.py) naturally produces
        # /cam1/image_raw, /cam2/image_raw, etc.
        self._image_pub = self.create_publisher(Image, 'image_raw', 10)
        # Placeholder for the Phase 3 custom CaptureMetadata.msg - JSON string
        # for now so this is usable end-to-end before that message exists.
        self._metadata_pub = self.create_publisher(String, 'metadata', 10)

        # --- Backend selection -------------------------------------------------
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
            # Real backend stub not ported yet - fail loudly and clearly
            # rather than silently publishing nothing.
            self.get_logger().error(
                f'[{self._camera_prefix}] {exc}'
            )
            self._timer.cancel()
            return
        except Exception as exc:  # noqa: BLE001 - log and keep node alive
            self.get_logger().warn(f'[{self._camera_prefix}] grab_frame() failed: {exc}')
            return

        img_msg = self._frame_to_image_msg(frame)
        self._image_pub.publish(img_msg)

        metadata_msg = String()
        metadata_msg.data = json.dumps({
            'camera_prefix': self._camera_prefix,
            'timestamp': metadata.get('capture_time', time.time()),
            **metadata,
        })
        self._metadata_pub.publish(metadata_msg)

        self._frames_published += 1
        if self._frames_published % 50 == 0:
            self.get_logger().info(
                f'[{self._camera_prefix}] published {self._frames_published} frames'
            )

    @staticmethod
    def _frame_to_image_msg(frame) -> Image:
        """Manual numpy -> sensor_msgs/Image conversion (bgr8).

        Avoids a hard cv_bridge dependency for the mock-data path; once the
        real IDS backend is wired in, swapping this for cv_bridge.CvBridge()
        is a one-line change if preferred.
        """
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
