"""
Real IDS camera backend - PORT THE ACTUAL LOGIC FROM capture.py HERE.

I don't have the source of the original capture.py in front of me while scaffolding
this (only its documented CLI interface from the MultiCamera-IDS-Capture README), so
this file is a skeleton with the same shape as MockCameraBackend, ready for the real
`ids_peak` calls to be copied in. Every method below has a `# PORT FROM capture.py`
comment marking exactly what needs to move over.

Reference: the original script's device-check snippet from the README was:

    from ids_peak import ids_peak as peak
    peak.Library.Initialize()
    mgr = peak.DeviceManager.Instance()
    mgr.Update()
    print("Found devices:", mgr.Devices().size())
    peak.Library.Close()

That's the same Library.Initialize() / DeviceManager pattern this skeleton expects
open() to use.
"""

import time

from spot_multicam.mock_camera_backend import MockCameraBackend  # noqa: F401 (see note in open())


class IdsCameraBackend:
    """Real IDS peak camera backend. Mirrors MockCameraBackend's contract:
    open(), close(), is_open(), grab_frame() -> (frame, metadata_dict).
    """

    def __init__(self, serial: str, frame_rate: float, exposure_us: int,
                 black_level: float, gain_db: float, trigger_mode: str,
                 buffers: int, focus_fixed: int = None):
        self._serial = serial
        self._frame_rate = frame_rate
        self._exposure_us = exposure_us
        self._black_level = black_level
        self._gain_db = gain_db
        self._trigger_mode = trigger_mode
        self._buffers = buffers
        self._focus_fixed = focus_fixed

        self._device = None
        self._datastream = None
        self._node_map = None
        self._frame_count = 0
        self._open = False

    def open(self) -> None:
        """
        # PORT FROM capture.py: device discovery + open + node map configuration.
        #
        # Expected shape, based on the IDS peak API and the device-check
        # snippet in the README:
        #
        #   from ids_peak import ids_peak as peak
        #   peak.Library.Initialize()
        #   device_manager = peak.DeviceManager.Instance()
        #   device_manager.Update()
        #   devices = device_manager.Devices()
        #   # match self._serial against devices, or take devices[0] if no
        #   # serial filter was requested
        #   self._device = matched_device.OpenDevice(peak.DeviceAccessType_Control)
        #   self._node_map = self._device.RemoteDevice().NodeMaps()[0]
        #
        #   # apply the tunables this class was constructed with:
        #   self._node_map.FindNode("AcquisitionFrameRate").SetValue(self._frame_rate)
        #   self._node_map.FindNode("ExposureTime").SetValue(self._exposure_us)
        #   self._node_map.FindNode("BlackLevel").SetValue(self._black_level)
        #   self._node_map.FindNode("Gain").SetValue(self._gain_db)
        #   # trigger_mode: "software" vs "freerun" - set TriggerMode /
        #   #   TriggerSource accordingly, matching the --trigger flag's
        #   #   behaviour in the original script
        #   # buffers: allocate self._buffers DataStream buffers
        #   # focus_fixed: if set, write it to the focus node (matches --focus-fixed)
        #
        #   self._datastream = self._device.DataStreams()[0].OpenDataStream()
        #   # allocate + queue buffers, start acquisition
        #
        """
        raise NotImplementedError(
            'IdsCameraBackend.open() is a stub - port the device open/configure '
            'logic from the original capture.py. Use MockCameraBackend for now '
            'to validate the ROS2 wiring (launch with use_mock:=true).'
        )

    def close(self) -> None:
        """
        # PORT FROM capture.py: stop acquisition, flush/close the datastream,
        # close the device, and call peak.Library.Close() if this is the last
        # open camera in the process.
        """
        self._open = False

    def is_open(self) -> bool:
        return self._open

    def grab_frame(self):
        """
        # PORT FROM capture.py: the actual frame-grab loop, e.g. something like
        #
        #   buffer = self._datastream.WaitForFinishedBuffer(timeout_ms)
        #   image = peak_ipl_extension.BufferToImage(buffer)
        #   converted = image.ConvertTo(peak_ipl.PixelFormatName_BGR8)
        #   frame = converted.get_numpy_3D()
        #   self._datastream.QueueBuffer(buffer)
        #
        # and build the same metadata fields the original script wrote to its
        # per-camera CSV (timestamp, node/prefix, filename equivalent, plus
        # whatever capture.py logged - exposure, frame index, etc).
        """
        self._frame_count += 1
        metadata = {
            'frame_id': self._frame_count,
            'serial': self._serial,
            'exposure_us': self._exposure_us,
            'capture_time': time.time(),
            'mock': False,
        }
        raise NotImplementedError(
            'IdsCameraBackend.grab_frame() is a stub - port the buffer-grab '
            'and pixel-format conversion logic from the original capture.py.'
        )
