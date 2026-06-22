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
    """PLACEHOLDER for Phase 4 integration - not implemented yet.

    Once the ROS2 action server exists (start_capture/stop_capture action,
    interval_s as the goal), this class should:
      - create or receive an rclpy node
      - create an ActionClient for that action
      - in start(): send a goal with interval_s, don't block waiting for result
      - in stop(): cancel the active goal, await the result for total_frames

    Left unimplemented intentionally so this seam is visible rather than
    silently wrong.
    """

    def __init__(self, ros_node=None):
        self._ros_node = ros_node

    def start(self, interval_s: float) -> None:
        raise NotImplementedError(
            'Ros2ActionCaptureController.start() - implement once the Phase 4 '
            'ROS2 action server exists. Use LoggingCaptureController until then.'
        )

    def stop(self) -> dict:
        raise NotImplementedError(
            'Ros2ActionCaptureController.stop() - implement once the Phase 4 '
            'ROS2 action server exists. Use LoggingCaptureController until then.'
        )
