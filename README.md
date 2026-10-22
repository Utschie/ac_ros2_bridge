# ac_ros2_bridge

Zenoh → ROS 2 relay for the AC Simulator Suite. It subscribes the data
streams published by the suite's Windows-side bridge over Zenoh and
republishes them as standard ROS 2 topics — ego pose + TF, 3D detections,
lidar point clouds, camera images, telemetry, run events — ready for RViz 2
and your autonomy stack. ROS 2 runs on this (Linux) side only.

**Full deployment & integration manual**:
[USAGE_en.md](USAGE_en.md) · [USAGE_cn.md](USAGE_cn.md) — Windows/Linux
setup, wired direct-connection topology, RViz 2 usage, acceptance checklist,
troubleshooting.

## Topics at a glance

| Group | Topics |
|---|---|
| Ego pose | `/ac/ego_pose` + TF `ac_world → base_link` |
| Detections | `/ac/detections_3d` (+ a marker array for RViz) |
| Lidar | `/ac/lidar/<name>/points` (sensor frame) |
| Cameras | `/ac/camera/<cam>/image_raw/compressed`, `image_depth`, `image_classes`, `camera_info` |
| Telemetry | `/ac/telemetry`, `/ac/timing`, `/ac/telemetry_static` (latched), `/ac/expert_ctrl` |
| State / events | `/ac/vehicle_state`, `/ac/game_state`, `/ac/run_state` |
| Control (downlink) | `/ac/vehicle_cmd`, `/ac/game_cmd` |

All data arrives in the suite-wide convention (X fwd / Y left / Z up; world
Z = height). Array orders and field semantics are documented in the suite's
data-format reference.

## Build & run

```bash
mkdir -p ~/colcon_ws/src && cd ~/colcon_ws/src
git clone https://github.com/Utschie/ac_msgs.git
git clone https://github.com/Utschie/ac_ros2_bridge.git
source /opt/ros/humble/setup.bash
cd ~/colcon_ws && colcon build --symlink-install
source install/setup.bash
ZENOH_CONFIG=~/colcon_ws/src/ac_ros2_bridge/relay.json5 \
  ros2 run ac_ros2_bridge relay
```

Dependencies: ROS 2 Humble, `eclipse-zenoh` and `lz4` (pip),
`ros-humble-vision-msgs`, `ros-humble-image-transport-plugins`.

## Package layout

`relay.py` (wiring) · `relay_context.py` (shared policies: clock stamping,
frame pairing, publish lock, TF) · `scene_relay.py` · `image_relay.py` ·
`control_relay.py` · `telemetry_relay.py` · `class_table.py` · `wire.py`
(pure payload codecs, unit-testable without ROS) · `frames.py` (coordinate
assembly). Adding a data type = writing one new domain module and wiring it
in `relay.py`.
