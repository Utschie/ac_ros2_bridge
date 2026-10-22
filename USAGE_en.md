# ROS2 Deployment & Integration Manual

Complete steps for connecting the Windows capture side with a Linux ROS2
consumer. Data flow: **Windows (game + suite) → zenoh TCP → Linux relay →
ROS2 topics / RViz2**. The bridge is the active connecting party — the only
requirement is "the bridge can reach the relay machine's port 7447".

Field baseline: five cameras, three modalities, full stream ~112 frames/s
(~17 MB/s); WiFi carries only a fraction of that — use a direct Ethernet
link for full throughput (see the "Direct Ethernet link" section).

## 0. Topics and frames

Output follows the standard ROS conventions, identical to the suite-wide
convention (X fwd / Y left / Z up; world frame `ac_world` has z = altitude).
Main topics and TF:

| Group | Topic / TF |
|---|---|
| Ego pose | `/ac/ego_pose` (PoseStamped) + TF `ac_world → base_link` |
| Sensor mounts | static TF `base_link → <lidar>_frame`, `base_link → <cam>_frame` |
| 3D boxes | `/ac/detections_3d` (Detection3DArray) + `/ac/detections_3d_markers` |
| Point clouds | `/ac/lidar/<name>/points` (sensor-local, like real hardware) |
| Cameras | `/ac/camera/<cam>/image_raw/compressed`, `image_depth`, `image_classes`, `camera_info` |
| Class table | `/ac/semantic/classes` |
| Control (subscriptions) | `/ac/vehicle_cmd`, `/ac/game_cmd` (ac_msgs) |
| Feedback (publications) | `/ac/vehicle_state`, `/ac/game_state`, `/ac/expert_ctrl`, `/ac/telemetry` (every scene frame), `/ac/timing` (every scene frame), `/ac/telemetry_static` (latched, on session change), `/ac/run_state` (event-driven) |

## 1. Linux side setup

Ubuntu 22.04 + ROS2 Humble:

```bash
sudo apt install -y software-properties-common curl
# If GitHub is unreachable: download the ros2-apt-source deb on a connected
# machine and copy it over before this step
sudo apt update && sudo apt install -y ros-humble-desktop ros-dev-tools
pip3 install eclipse-zenoh lz4
sudo apt install -y ros-humble-vision-msgs ros-humble-image-transport-plugins
```

Get the ROS relay sources provided with the suite (open-source repositories,
clone both into the workspace):

```bash
mkdir -p ~/colcon_ws/src && cd ~/colcon_ws/src
git clone https://github.com/Utschie/ac_msgs.git
git clone https://github.com/Utschie/ac_ros2_bridge.git
source /opt/ros/humble/setup.bash
cd ~/colcon_ws && colcon build --symlink-install
source install/setup.bash
```

## 2. Starting the relay

```bash
ZENOH_CONFIG=~/colcon_ws/src/ac_ros2_bridge/relay.json5 \n  ros2 run ac_ros2_bridge relay
```

When started normally you should see the log line `ac_relay ready`, and
`ss -tln | grep 7447` showing LISTEN. Stop with Ctrl+C. The relay must be
started with `ZENOH_CONFIG` — without it the relay looks ready but never
listens on 7447. Restart it after every reboot (or set up auto-start).

## 3. Windows side configuration

Edit `ac_simulator_suite/configs/bridge.json5` and point it at the relay
machine:

```
{ mode: "peer", connect: { endpoints: ["tcp/<Linux machine IP>:7447"] } }
```

The file is read once at bridge startup — restart the bridge from the
launcher after editing. Once the game enters a track, data flows
automatically.

## 4. Direct Ethernet link (recommended for full throughput)

WiFi is fine for low-rate checks only; for the full stream use a direct
Ethernet link. Steps:

1. **Set a static IP on the Linux side** (substitute your NIC name, found
   via `ip addr`):

   ```bash
   sudo ip addr add 192.168.50.2/24 dev <nic> && sudo ip link set <nic> up
   ```

2. **Set a static IP on the Windows side** (admin PowerShell; find
   `<ifIndex>` via `Get-NetAdapter`):

   ```powershell
   New-NetIPAddress -InterfaceIndex <ifIndex> -IPAddress 192.168.50.1 -PrefixLength 24
   ```

   Connect the two NICs with a cable; **set no gateway on either side** —
   the Windows machine keeps its Internet over WiFi.

3. **Verify connectivity**: `ping 192.168.50.2` from Windows (the reverse
   ping failing is just the Windows firewall blocking ICMP — harmless, the
   bridge connects outbound).

4. **Point the bridge at it and restart**: set bridge.json5 to
   `tcp/192.168.50.2:7447` and restart the bridge from the launcher.

5. **When done**: restore the Windows NIC to DHCP (remove the static IP).

## 5. Viewing color images in RViz2

The color topic is a CompressedImage; RViz2's topic picker filters by type
and won't list it — republish to decode first:

```bash
source /opt/ros/humble/setup.bash && source ~/colcon_ws/install/setup.bash
ros2 run image_transport republish compressed raw --ros-args   -r /in/compressed:=/ac/camera/front/image_raw/compressed   -r /out:=/ac/camera/front/image_raw -r __node:=republish_front
```

Then select `/ac/camera/front/image_raw` in RViz2. Stop with Ctrl+C; start
one instance per camera you want to view (adjust the topic and node names).

Camera intrinsics: `/ac/camera/<cam>/camera_info` (sensor_msgs/CameraInfo).

## 6. Acceptance

```bash
source /opt/ros/humble/setup.bash && source ~/colcon_ws/install/setup.bash
ros2 topic list | grep camera          # per camera: image_raw/compressed (+depth +classes) + camera_info
ros2 topic hz /ac/camera/front/image_depth     # ≈ that camera's HZ in streamer.ini
ros2 topic echo /ac/camera/front/camera_info --field k --once   # 9 intrinsic values
```

When healthy: each topic's rate is about the HZ configured for that camera,
and the images are continuous with no multi-second stalls.

## 7. Troubleshooting

| Symptom | Cause and fix |
|---|---|
| Relay ready but all topics empty | Started without `ZENOH_CONFIG` (see "Starting the relay"); wrong address in bridge.json5 (see "Windows side configuration"); bridge not restarted after editing |
| Bridge sends at full rate, Linux receives far less, multi-second gaps | Link bandwidth (typical on WiFi) → direct Ethernet (see the "Direct Ethernet link" section) |
| Color topic not selectable in RViz2 | CompressedImage filtered by the picker → republish (see "Viewing color images") |
| republish runs but no images | Used the `in:=` short syntax → full-name remapping (see "Viewing color images") |
| camera_info never arrives | Wait a few seconds (resent periodically); if still missing, restart the bridge |
| Linux build fails "existing path cannot be removed" | Wipe build/install/log and rebuild (see "Linux side setup") |
| Windows wired Internet dead after plugging back into the router | Leftover direct-link static IP → restore DHCP (see "Direct Ethernet link") |
