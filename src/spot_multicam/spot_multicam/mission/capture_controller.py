"""
capture_controller.py

Defines the seam between the Spot-facing RemoteMissionService (this file's
neighbour, capture_mission_service.py) and whatever actually starts/stops
capture on the camera nodes.

Today, that's LoggingCaptureController - it doesn't touch ROS2 at all, so the
RemoteMissionService can be built and tested end-to-end against Boston
Dynamics' official no-robot local test mode before any ROS2 wiring exists.

Once the Phase 4 ROS2 action server (start_capture/stop_capture as an action,
interval_s as the goal) exists, Ros2ActionCaptureController below becomes the
real implementation: it creates an rclpy node internally and calls that
action. The RemoteMissionService never needs to change - it only depends on
this interface.
"""

import time
from abc import ABC, abstractmethod

from rclpy.node import Node


class CaptureController(ABC):
    """Interface the RemoteMissionService depends on. Swap implementations,
    not callers.
    """

    @abstractmethod
    def start(self, interval_s: float) -> None:
        """Begin capture at the given interval. Should return quickly -
        mirrors the original system's start_capture, which starts every
        configured node in parallel and returns without waiting for capture
        to finish.
        """

    @abstractmethod
    def stop(self) -> dict:
        """Stop capture. Returns a small stats dict, e.g. {'total_frames': N},
        for the servicer to log or report back.
        """


class LoggingCaptureController(CaptureController):
    """No-op-but-honest controller for testing the RemoteMissionService in
    isolation, before the ROS2 action bridge exists.

    Fakes a frame count based on elapsed time / interval_s, purely so the
    Stop() response has a believable number in it during testing - this
    number is NOT coming from any real camera node.
    """

    def __init__(self, logger=None):
        import logging
        self._logger = logger or logging.getLogger(__name__)
        self._active = False
        self._start_time = None
        self._interval_s = None

    def start(self, interval_s: float) -> None:
        self._logger.info(f'[LoggingCaptureController] start_capture(interval_s={interval_s})'
                          f' - no real camera nodes wired up yet, this is a stand-in.')
        self._active = True
        self._start_time = time.time()
        self._interval_s = interval_s

    def stop(self) -> dict:
        if not self._active:
            self._logger.info('[LoggingCaptureController] stop_capture() called but not active')
            return {'total_frames': 0}

        elapsed = time.time() - self._start_time
        fake_frame_count = int(elapsed / self._interval_s) if self._interval_s else 0
        self._logger.info(
            f'[LoggingCaptureController] stop_capture() - {elapsed:.2f}s elapsed, '
            f'fake_frame_count={fake_frame_count} (not real - no camera nodes wired up yet)'
        )
        self._active = False
        return {'total_frames': fake_frame_count}


class Ros2ActionCaptureController(CaptureController):
    """Calls the CaptureAction server from the Spot RemoteMissionService.

    This is the bridge between the gRPC-facing CaptureMissionServicer and
    the ROS2-facing CaptureActionServer — the seam that was left as a
    NotImplementedError stub until Phase 4 existed.

    start() sends a goal to the action server (non-blocking).
    stop() cancels the active goal and waits for the result (total_frames).
    """

    def __init__(self, ros_node: Node):
        import rclpy
        from rclpy.action import ActionClient
        from spot_multicam_msgs.action import CaptureAction

        self._node = ros_node
        self._client = ActionClient(ros_node, CaptureAction, 'capture')
        self._goal_handle = None
        self._logger = ros_node.get_logger()

    def start(self, interval_s: float) -> None:
        from spot_multicam_msgs.action import CaptureAction

        if not self._client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(
                'CaptureAction server not available after 5s timeout. '
                'Is capture_action_server running?'
            )

        goal = CaptureAction.Goal()
        goal.interval_s = float(interval_s)
        goal.duration_s = 0.0  # run until cancelled (= stop_capture tick)

        # send_goal_async is non-blocking — mirrors original start_capture
        # which returned immediately after starting all camera nodes.
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        self._logger.info(
            f'[Ros2ActionCaptureController] start_capture sent '
            f'(interval_s={interval_s})'
        )

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._logger.error('[Ros2ActionCaptureController] Goal rejected by action server')
            return
        self._goal_handle = goal_handle
        self._logger.info('[Ros2ActionCaptureController] Goal accepted')

    def stop(self) -> dict:
        if self._goal_handle is None:
            self._logger.warn(
                '[Ros2ActionCaptureController] stop() called but no active goal'
            )
            return {'total_frames': 0}

        # Cancel the goal — the action server interprets this as stop_capture
        cancel_future = self._goal_handle.cancel_goal_async()

        # Spin until cancel is acknowledged and result arrives
        import rclpy
        rclpy.spin_until_future_complete(self._node, cancel_future, timeout_sec=10.0)

        result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=10.0)

        self._goal_handle = None

        if result_future.done():
            result = result_future.result().result
            self._logger.info(
                f'[Ros2ActionCaptureController] stop_capture complete: '
                f'total_frames={result.total_frames}'
            )
            return {'total_frames': result.total_frames}

        self._logger.warn('[Ros2ActionCaptureController] stop() timed out waiting for result')
        return {'total_frames': 0}
