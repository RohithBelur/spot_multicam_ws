"""
Synthetic camera backend used for Phase 1 testing without an IDS camera attached.

This exists so the ROS2 plumbing (node lifecycle, parameters, publishers, timers,
launch files) can be validated end-to-end before any real hardware is involved -
the same "test the wiring with fake data first" approach that's worth applying to
any distributed capture system, hardware or not.

Swap to IdsCameraBackend (ids_camera_backend.py) once this is confirmed working.
"""

import time

import numpy as np


class MockCameraBackend:
    """Generates synthetic BGR frames at a configurable rate.

    Mirrors the subset of capture.py's tunables that matter for testing the
    ROS2 wiring: frame_rate, frame size, and a fake per-frame exposure value
    so downstream code that reads metadata has something realistic to log.
    """

    def __init__(self, serial: str, width: int = 640, height: int = 480,
                 exposure_us: int = 10000):
        self._serial = serial or 'MOCK-0001'
        self._width = width
        self._height = height
        self._exposure_us = exposure_us
        self._frame_count = 0
        self._open = False

    def open(self) -> None:
        """Mirrors IdsCameraBackend.open() - no real device, just flips a flag."""
        self._open = True

    def close(self) -> None:
        self._open = False

    def is_open(self) -> bool:
        return self._open

    def grab_frame(self):
        """Returns (frame, metadata_dict), matching the real backend's contract.

        The frame is a moving gradient with a frame-counter overlay region so
        it's visually obvious in rqt_image_view that frames are actually
        changing, not a static image being republished.
        """
        if not self._open:
            raise RuntimeError('grab_frame() called before open()')

        self._frame_count += 1

        # Simple animated gradient, cheap to generate, visually verifiable.
        t = self._frame_count
        x = np.linspace(0, 255, self._width, dtype=np.uint8)
        gradient = np.tile(x, (self._height, 1))
        shift = (t * 4) % 256
        gradient = np.roll(gradient, shift, axis=1)
        frame = np.stack([gradient, gradient, gradient], axis=-1).astype(np.uint8)

        # Mark a block in the corner that strobes so dropped-frame issues are
        # visible at a glance in rqt_image_view.
        block_size = 40
        frame[0:block_size, 0:block_size] = 255 if (t % 2 == 0) else 0

        metadata = {
            'frame_id': self._frame_count,
            'serial': self._serial,
            'exposure_us': self._exposure_us,
            'capture_time': time.time(),
            'mock': True,
        }
        return frame, metadata
