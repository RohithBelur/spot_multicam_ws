# spot_multicam (ROS2)

A ROS2 reimplementation of [MultiCamera-IDS-Capture](https://github.com/RohithBelur/MultiCamera-IDS-Capture) — a distributed IDS camera capture system originally orchestrated via SSH/rsync and triggered from a Boston Dynamics Spot mission via a custom `RemoteMissionService`. Built phase by phase to develop production-grade ROS2 depth on top of architecture I already designed and ran in production, rather than starting from a generic tutorial.

## Motivation

The original system works, but every coordination mechanism is hand-rolled: SSH for distribution, rsync for transport, a CSV file for metadata, SIGINT for shutdown. ROS2 has first-class equivalents for all of these. This project ports the same architecture onto ROS2 primitives so each phase replaces something I already understand with its ROS2 counterpart.

| Original mechanism | ROS2 equivalent | Phase |
|---|---|---|
| `capture.py` CLI flags | ROS2 node parameters (YAML) | 1 |
| One capture process per Pi | One namespaced ROS2 node per camera | 1–2 |
| SSH start/stop via `camera_controller.py` | DDS discovery — no manual SSH | 2 |
| `rsync` image pull | `sensor_msgs/Image` topic | 1–2 |
| `*_metadata.csv` row per frame | `spot_multicam_msgs/CaptureMetadata` message | 3 |
| `start_capture` / `stop_capture` mission commands | `CaptureAction` — ROS2 action (goal/feedback/result) | 4 |
| `robot_command_mission_service.py` over gRPC | `CaptureMissionServicer` + `Ros2ActionCaptureController` | 5 |
| Implicit camera mounting geometry | `tf2` static transforms, camera frames → `spot_body` | 6 |

## Status

All phases verified and running on Ubuntu 24.04 / ROS2 Jazzy.

| Phase | Description | Status |
|---|---|---|
| 1 | Single camera node — parameterised, mock + real backend | ✅ Verified |
| 2 | Multi-camera namespaced launch — `cam1`, `cam2` simultaneous | ✅ Verified |
| 3 | `CaptureMetadata.msg` — typed message replaces JSON string | ✅ Verified |
| 4 | `CaptureAction` server — goal/feedback/result, 100 frames/5s across both cameras | ✅ Verified |
| 5 | `RemoteMissionService` bridge — full session lifecycle against BD's no-robot mode | ✅ Verified |
| 6 | `tf2` static transforms — `spot_body` → `cam1_optical`, `cam2_optical` | ✅ Verified |
| 7 | Packaging polish, CI | ⬜ Upcoming |

## Repository layout

```
spot_multicam_ws/
└── src/
    ├── spot_multicam/                          # main ROS2 package
    │   ├── package.xml
    │   ├── setup.py
    │   ├── spot_multicam/
    │   │   ├── camera_capture_node.py          # parameterised camera node (Phases 1–3)
    │   │   ├── mock_camera_backend.py          # synthetic frames — no hardware required
    │   │   ├── ids_camera_backend.py           # real IDS peak SDK stub (port from capture.py)
    │   │   ├── capture_action_server.py        # CaptureAction server (Phase 4)
    │   │   ├── camera_tf_broadcaster.py        # tf2 static transforms (Phase 6)
    │   │   └── mission/
    │   │       ├── capture_controller.py       # CaptureController interface + implementations
    │   │       └── capture_mission_service.py  # Spot RemoteMissionService (Phase 5)
    │   ├── launch/
    │   │   ├── single_camera.launch.py         # Phase 1 testing
    │   │   ├── multi_camera.launch.py          # Phase 2: N namespaced camera nodes
    │   │   └── full_system.launch.py           # Phase 4+: cameras + action server + tf
    │   ├── config/
    │   │   ├── single_camera_params.yaml
    │   │   ├── multi_camera_params.yaml
    │   │   └── camera_transforms.yaml          # mounting offsets (placeholder — update with real values)
    │   └── test/
    │       ├── test_camera_capture_node.py
    │       └── manual_test_capture_mission_service.py
    └── spot_multicam_msgs/                     # custom message/action definitions
        ├── package.xml
        ├── CMakeLists.txt
        ├── msg/
        │   └── CaptureMetadata.msg             # typed frame metadata (Phase 3)
        └── action/
            └── CaptureAction.action            # start/stop capture action (Phase 4)
```

## Prerequisites

- **ROS2 Jazzy** on Ubuntu 24.04 (tested)
- **Python 3.12**
- **bosdyn-client / bosdyn-mission 5.1.4** (for the Spot mission bridge only)

```bash
# Install ROS2 Jazzy (if not already installed)
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions

# Install Boston Dynamics SDK (for mission bridge)
pip install bosdyn-client bosdyn-mission --break-system-packages
```

## Build

```bash
cd spot_multicam_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Note: `colcon build` must be run with ROS2 sourced and outside any conda environment.

## Running

### Single camera (Phase 1 — no hardware required)

```bash
ros2 launch spot_multicam single_camera.launch.py use_mock:=true
# verify
ros2 topic echo /cam1/metadata
```

### Multi-camera (Phase 2)

```bash
ros2 launch spot_multicam multi_camera.launch.py use_mock:=true
ros2 topic list   # /cam1/image_raw, /cam2/image_raw, /cam1/metadata, /cam2/metadata
```

### Full system — cameras + action server + tf (Phase 4+)

```bash
ros2 launch spot_multicam full_system.launch.py use_mock:=true
```

Send a capture goal (second terminal):
```bash
ros2 action send_goal /capture spot_multicam_msgs/action/CaptureAction \
  "{interval_s: 0.1, duration_s: 5.0}" --feedback
```

Expected output: feedback every second with `frames_so_far` incrementing at 20 fps (10 fps × 2 cameras), result `total_frames: 100` at 5 seconds.

### tf2 transforms (Phase 6)

```bash
ros2 run tf2_ros tf2_echo spot_body cam1_optical
# Translation: [0.400, -0.100, 0.100] (placeholder — update camera_transforms.yaml with real values)
# Rotation: [0.000, 0.000, 0.000, 1.000]
```

### Spot RemoteMissionService bridge (Phase 5 — no robot required)

```bash
# terminal 1: start the service
ros2 run spot_multicam capture_mission_service local --port 24567

# terminal 2: run the full session lifecycle test
python3 src/spot_multicam/test/manual_test_capture_mission_service.py \
    --host-ip 127.0.0.1 --port 24567 --interval-s 0.2 --capture-duration-s 2.0
```

Confirmed: `EstablishSession` → `Tick(start_capture)` → `STATUS_SUCCESS` → `Tick(stop_capture)` → `STATUS_SUCCESS` → `Stop` → `TeardownSession`. Also verified: `Stop()` safety-net path, `interval_s` clamping, invalid session ID handling.

## Porting to real IDS cameras

`ids_camera_backend.py` is a skeleton with `# PORT FROM capture.py` comments marking every location where the actual `ids_peak` SDK calls need to be copied from the original `MultiCamera-IDS-Capture` repo. Once ported, switch from mock to real hardware:

```bash
ros2 launch spot_multicam single_camera.launch.py use_mock:=false serial:=<your_camera_serial>
```

## Porting to real Spot hardware

The `RemoteMissionService` (`capture_mission_service.py`) already supports a `robot` subcommand:

```bash
ros2 run spot_multicam capture_mission_service robot \
  --hostname <spot-ip> --host-ip <this-machine-ip> --port 24567
```

This registers the service with Spot's directory so it can be called from a Spot mission. The `Ros2ActionCaptureController` in `capture_controller.py` bridges the gRPC service to the ROS2 action server — no changes needed to either side.

Update `config/camera_transforms.yaml` with real measured mounting offsets before deploying.

## Tested environment

- Ubuntu 24.04 (ST-Laptop)
- ROS2 Jazzy (ros-jazzy-desktop 0.11.0)
- Python 3.12
- bosdyn-mission 5.1.4 / bosdyn-client 5.1.4
- colcon-common-extensions 0.3.0
