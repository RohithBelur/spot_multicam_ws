# CaptureMetadata.msg
# Replaces the JSON string previously published on <camera_prefix>/metadata.
# Each field maps directly to a key in the original CSV row that
# MultiCamera-IDS-Capture wrote per frame.

# Standard ROS header: timestamp lives here (header.stamp) and
# frame_id carries the camera's tf2 frame name (added in Phase 6).
std_msgs/Header header

# Which camera node published this frame - matches camera_prefix parameter
# e.g. "cam1", "cam2"
string camera_prefix

# IDS camera serial number (or "MOCK-xxxx" for the mock backend)
string serial

# Sequential frame counter, resets on node restart
uint64 frame_id

# Exposure time used for this frame, in microseconds
uint32 exposure_us

# True if this frame came from MockCameraBackend (no real hardware)
bool is_mock
