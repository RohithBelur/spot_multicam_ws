"""
capture_action_server.py

Phase 4: ROS2 action server that receives CaptureAction goals and manages
capture across the running camera nodes.

This replaces the original system's SSH-based start/stop orchestration
(camera_controller.py calling paramiko into each Raspberry Pi) with a proper
ROS2 action, so the Spot RemoteMissionService bridge can call it via an
action client instead of SSH.

ROS2 concepts introduced in this phase:
  - Action server (rclpy.action.ActionServer)
  - Goal, feedback, and result handling
  - Goal cancellation (the correct way to implement "stop capture")
  - Why action vs service: start_capture is long-running with progress
    feedback — actions are designed for exactly this pattern

Architecture:
  The action server doesn't directly control the camera nodes. Instead it
  publishes a std_msgs/Bool on /capture/active that the camera nodes can
  subscribe to if they need to gate capture (Phase 4 extension). For now it
  manages a frame counter by listening to metadata topics from all active
  camera nodes, so total_frames in the result is real, not estimated.

  This mirrors the original system's design where the controller knew about
  all nodes but each node ran independently — the controller just started
  and stopped them.
"""

import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool

from spot_multicam_msgs.action import CaptureAction
from spot_multicam_msgs.msg import CaptureMetadata

# Camera namespaces to monitor — matches multi_camera_params.yaml.
# Phase 2 scale-up: add more namespaces here to monitor more cameras.
DEFAULT_CAMERA_NAMESPACES = ['cam1', 'cam2']


class CaptureActionServer(Node):
    """Action server for CaptureAction.

    Goal:     interval_s, duration_s (0 = run until cancelled)
    Feedback: frames_so_far, elapsed_s  (published every second)
    Result:   total_frames, cancelled
    """

    def __init__(self, camera_namespaces=None):
        super().__init__('capture_action_server')

        self.declare_parameter('camera_namespaces', DEFAULT_CAMERA_NAMESPACES)
        namespaces = self.get_parameter('camera_namespaces').value

        # ReentrantCallbackGroup allows the action server callbacks and the
        # metadata subscribers to run concurrently in a MultiThreadedExecutor.
        self._cb_group = ReentrantCallbackGroup()

        # Frame counter — incremented by metadata subscriber callbacks
        self._frame_lock = threading.Lock()
        self._frame_count = 0

        # Subscribe to metadata from each camera namespace to count real frames
        self._meta_subs = []
        for ns in namespaces:
            sub = self.create_subscription(
                CaptureMetadata,
                f'/{ns}/metadata',
                self._on_metadata,
                10,
                callback_group=self._cb_group,
            )
            self._meta_subs.append(sub)
            self.get_logger().info(f'Subscribed to /{ns}/metadata')

        # Publisher: /capture/active lets camera nodes gate their capture
        # if they want to (optional integration point for Phase 4 extension)
        self._active_pub = self.create_publisher(Bool, '/capture/active', 10)

        self._action_server = ActionServer(
            self,
            CaptureAction,
            'capture',
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f'capture_action_server ready, monitoring: {namespaces}'
        )

    # --- Action callbacks ---------------------------------------------------

    def _goal_callback(self, goal_request):
        interval_s = goal_request.interval_s
        duration_s = goal_request.duration_s
        # Validate interval matches the original system's documented clamp
        if not (0.05 <= interval_s <= 5.0):
            self.get_logger().warn(
                f'Goal rejected: interval_s={interval_s} outside [0.05, 5.0]'
            )
            return GoalResponse.REJECT
        self.get_logger().info(
            f'Goal accepted: interval_s={interval_s}, duration_s={duration_s}'
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        # Always accept cancel — this is how stop_capture works
        self.get_logger().info('Cancel request received — stopping capture')
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        """Main execution loop. Runs until cancelled or duration expires."""
        interval_s = goal_handle.request.interval_s
        duration_s = goal_handle.request.duration_s

        self.get_logger().info(
            f'Starting capture: interval_s={interval_s}, '
            f'duration_s={duration_s if duration_s > 0 else "unlimited"}'
        )

        # Reset frame counter for this capture session
        with self._frame_lock:
            self._frame_count = 0

        # Signal camera nodes that capture is active
        self._active_pub.publish(Bool(data=True))

        start_time = time.time()
        feedback = CaptureAction.Feedback()

        while rclpy.ok():
            # Check for cancellation (= stop_capture from mission bridge)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._active_pub.publish(Bool(data=False))
                with self._frame_lock:
                    frames = self._frame_count
                self.get_logger().info(
                    f'Capture cancelled after {frames} frames'
                )
                result = CaptureAction.Result()
                result.total_frames = frames
                result.cancelled = True
                return result

            elapsed = time.time() - start_time

            # Check duration expiry (duration_s=0 means run until cancelled)
            if duration_s > 0 and elapsed >= duration_s:
                break

            # Publish feedback every second
            with self._frame_lock:
                frames_so_far = self._frame_count
            feedback.frames_so_far = frames_so_far
            feedback.elapsed_s = float(elapsed)
            goal_handle.publish_feedback(feedback)

            time.sleep(1.0)

        # Duration expired — complete normally
        self._active_pub.publish(Bool(data=False))
        with self._frame_lock:
            frames = self._frame_count

        self.get_logger().info(
            f'Capture complete: {frames} frames in {time.time() - start_time:.1f}s'
        )

        goal_handle.succeed()
        result = CaptureAction.Result()
        result.total_frames = frames
        result.cancelled = False
        return result

    def _on_metadata(self, msg: CaptureMetadata):
        """Count frames from all active camera nodes in real time."""
        with self._frame_lock:
            self._frame_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = CaptureActionServer()
    # MultiThreadedExecutor is required for ReentrantCallbackGroup —
    # the action execute() loop and the metadata subscribers must run
    # concurrently, which a SingleThreadedExecutor can't do.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
