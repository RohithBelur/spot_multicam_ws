# spot_multicam (ROS2)

A ROS2 reimplementation of [MultiCamera-IDS-Capture](https://github.com/RohithBelur/MultiCamera-IDS-Capture),
built as a learning project to develop production-grade ROS2 skills on top of architecture
I already designed and run in production.

## Why this exists

The original repo is a distributed IDS camera capture system: each Raspberry Pi runs a capture
script, a central orchestrator starts/stops them over SSH and pulls images back with `rsync`, and
the whole thing is triggered from a Spot mission via a `RemoteMissionService` (gRPC, via the
Boston Dynamics Spot SDK).

That system works, but it's hand-rolled: SSH for distribution, rsync for transport, a CSV file for
metadata, SIGINT for shutdown. ROS2 has first-class equivalents for every one of those pieces, so
this project ports the same architecture onto ROS2 primitives, phase by phase, so each phase
replaces something I already understand with its ROS2 counterpart instead of starting from a
generic tutorial.

| Original mechanism | ROS2 equivalent | Phase |
|---|---|---|
| `capture.py` CLI flags | ROS2 node parameters | 1 |
| One capture process per Pi | One ROS2 node per Pi, namespaced | 1–2 |
| SSH start/stop over `camera_controller.py` | DDS discovery, no manual SSH needed | 2 |
| `rsync` image pull | `sensor_msgs/Image` topic (or `image_transport`) | 1–2 |
| `*_metadata.csv` rows | Custom ROS2 message | 3 |
| `start_capture` / `stop_capture` mission commands | ROS2 action (goal/feedback/result) | 4 |
| `robot_command_mission_service.py`'s pattern, new `capture_mission_service.py` | `RemoteMissionService` calling a `CaptureController` (ROS2 action client once Phase 4 exists) | 4–5 |
| Implicit camera mounting | `tf2` static transforms, camera frame → Spot body frame | 6 |

## What's implemented so far (Phase 5: Spot-facing mission bridge)

Boston Dynamics does not ship a physics simulator for Spot - they don't provide a simulation tool, which is normally needed for testing apps before running them on the real robot. What they do provide is a no-robot local test mode built into their own `RemoteMissionService` examples, and that's what this project uses instead of a real robot.

`spot_multicam/mission/capture_mission_service.py` is a `RemoteMissionService`, modeled directly on Boston Dynamics' own `hello_world_mission_service.py` and `robot_command_mission_service.py` examples (cloned from `boston-dynamics/spot-sdk` and read directly while building this, rather than worked from memory). It implements the same `command` / `interval_s` mission interface documented in the original `MultiCamera-IDS-Capture` README:

- `command`: `start_capture`, `stop_capture`, or `noop`
- `interval_s`: clamped to `[0.05, 5.0]` seconds, exactly as in the original system

It does **not** require a Spot body lease, since it never commands robot movement - it only starts/stops capture, same as the original system did.

The actual start/stop logic is delegated to a `CaptureController` interface (`capture_controller.py`), not implemented directly in the servicer. Today that's `LoggingCaptureController`, which logs what would happen and fakes a frame count - this keeps the gRPC service itself fully real and testable without the Phase 4 ROS2 action server existing yet. `Ros2ActionCaptureController` is the documented placeholder for when that bridge gets built.

### Verified, not just written

This was tested end-to-end against Boston Dynamics' official no-robot mode - not just written and assumed correct:

```bash
pip install bosdyn-client bosdyn-mission --break-system-packages

# terminal 1: start the service, no robot involved
python3 -m spot_multicam.mission.capture_mission_service local --port 24567

# terminal 2: run the full session lifecycle against it
python3 src/spot_multicam/test/manual_test_capture_mission_service.py \
    --host-ip 127.0.0.1 --port 24567 --interval-s 0.2 --capture-duration-s 1.0
```

Confirmed working: `EstablishSession` → `Tick(start_capture)` → `STATUS_SUCCESS` → `Tick(stop_capture)` → `STATUS_SUCCESS` with a correctly-computed frame count → `Stop` → `TeardownSession`. Also verified separately: the `Stop()` safety-net path (capture left active, `Stop()` called without an explicit `stop_capture` tick - the controller's `stop()` fires automatically), parameter clamping (`interval_s=999.0` gets coerced to the valid range rather than rejected), and invalid session ID handling (server returns `STATUS_INVALID_SESSION_ID`, client library correctly raises `InvalidSessionId`).

## What's implemented so far (Phase 1)

A single-camera ROS2 node, `camera_capture_node`, that:

- Declares the same tunable parameters as the original `capture.py` CLI flags
  (`frame_rate`, `exposure_us`, `trigger_mode`, `gain_db`, `buffers`, `camera_prefix`, `serial`, etc.)
- Publishes `sensor_msgs/Image` on `<camera_prefix>/image_raw`
- Publishes per-frame metadata (timestamp, node, camera_prefix, frame_id) as JSON on
  `<camera_prefix>/metadata` — this is the placeholder for the Phase 3 custom message
- Supports a **mock backend** so the whole pipeline can be tested without an IDS camera attached,
  and a **real backend** stub where the actual IDS peak SDK calls from `capture.py` get ported in
- Is already namespace-ready: launching two instances with different namespaces gives you
  `cam1/image_raw` and `cam2/image_raw` without any code changes — this is the Phase 2 scale-up,
  it's just not exercised yet because testing started with one camera

## Repository layout

```
spot_multicam_ws/
└── src/
    └── spot_multicam/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/spot_multicam
        ├── spot_multicam/
        │   ├── __init__.py
        │   ├── camera_capture_node.py      # the ROS2 node
        │   ├── mock_camera_backend.py      # synthetic frames, no hardware needed
        │   ├── ids_camera_backend.py       # real IDS peak SDK calls go here
        │   └── mission/
        │       ├── __init__.py
        │       ├── capture_controller.py        # seam between Spot bridge and ROS2 (Phase 4 plugs in here)
        │       └── capture_mission_service.py   # Phase 5: RemoteMissionService, verified against BD's no-robot mode
        ├── launch/
        │   ├── single_camera.launch.py
        │   └── multi_camera.launch.py      # ready for Phase 2, scales to N cameras
        ├── config/
        │   ├── single_camera_params.yaml
        │   └── multi_camera_params.yaml
        └── test/
            ├── test_camera_capture_node.py
            └── manual_test_capture_mission_service.py  # verified integration test, see below
```

## Build and run

This assumes ROS2 Humble or Jazzy is already installed (this was developed and written outside a
ROS2 environment, so it has not been build-tested with `colcon` — see **Known gaps** below).

```bash
# from spot_multicam_ws/
colcon build --symlink-install
source install/setup.bash

# Phase 1: single camera, mock backend, no hardware required
ros2 launch spot_multicam single_camera.launch.py use_mock:=true

# in another terminal, check it's actually publishing
ros2 topic echo /cam1/metadata
ros2 run rqt_image_view rqt_image_view   # view /cam1/image_raw
```

Once that's confirmed working, swap to the real IDS camera by porting the actual `ids_peak` calls
from the original `capture.py` into `ids_camera_backend.py` (clearly marked with `# PORT FROM
capture.py` comments) and run with `use_mock:=false`.

### Phase 2 preview (multi-camera, not yet hardware-tested)

```bash
ros2 launch spot_multicam multi_camera.launch.py
```

This launches `cam1` and `cam2` as separate namespaced nodes from
`config/multi_camera_params.yaml`, mirroring the `REMOTE_NODES_JSON` array from the original
orchestrator — except here, ROS2's DDS discovery handles the distribution instead of SSH/rsync.
To actually run this across the real Raspberry Pi nodes, ROS2 needs to be installed on each Pi and
all nodes need to be on the same `ROS_DOMAIN_ID`; that's the next thing to validate once Phase 1 is
solid on a single machine.

## Known gaps / honest status

- **Phase 1 ROS2 code is not build-tested.** No ROS2 environment was available while scaffolding
  it, so `colcon build` has not been run against `camera_capture_node.py` or the launch files.
  Treat that part as "should be correct ROS2 Humble/Jazzy API usage," not "verified." By contrast,
  **Phase 5 (`capture_mission_service.py`) genuinely was run and tested** - real `bosdyn-mission`
  5.1.4 installed, the official `boston-dynamics/spot-sdk` examples cloned and read directly, and
  the full session lifecycle plus three edge cases (safety-net `Stop()`, parameter clamping,
  invalid session ID) exercised against it locally. That distinction matters: Phase 1 is
  best-effort, Phase 5 is confirmed working.
- **IDS peak SDK calls are stubbed**, not ported. I don't have the source of the original
  `capture.py` in front of me (only its documented CLI interface from the README), so
  `ids_camera_backend.py` is a clearly-marked skeleton — the actual `ids_peak` device open /
  configure / acquire calls need to be copied over from the real script.
- **Custom message (Phase 3) and the ROS2 action server (Phase 4) are not implemented yet.**
  Metadata is JSON-in-a-String as a placeholder until Phase 3. `Ros2ActionCaptureController` in
  `capture_controller.py` is an intentional `NotImplementedError` stub until Phase 4 exists -
  `capture_mission_service.py` currently runs against `LoggingCaptureController` instead.
- **Multi-camera launch is written but not hardware-tested** across the actual Raspberry Pi nodes.

## Roadmap

- [x] Phase 1 — single camera node, parameterized, mock + real backend
- [ ] Phase 2 — validate multi-camera launch across the real Pi nodes over DDS
- [ ] Phase 3 — custom `CaptureMetadata.msg`, replacing the JSON string
- [ ] Phase 4 — `start_capture`/`stop_capture` as a ROS2 action with `interval_s` as the goal
- [x] Phase 5 — `RemoteMissionService` bridge skeleton, verified against BD's no-robot test mode; `CaptureController.start()/stop()` is the seam Phase 4 plugs into once it exists
- [ ] Phase 6 — `tf2` static transforms, camera frame → Spot body frame
- [ ] Phase 7 — packaging polish, CI, public release

Note the order: Phase 5 was built ahead of Phase 4 because it doesn't depend on ROS2 at all - it's pure Boston Dynamics SDK, and could be fully written and tested against BD's no-robot mode in isolation. `LoggingCaptureController` stands in for the ROS2 action client until Phase 4 exists; swapping it for `Ros2ActionCaptureController` is the only change needed to wire the two phases together.
