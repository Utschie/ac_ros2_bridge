# ac_ros2_bridge Deployment & Integration Manual (English)

Complete step-by-step guide for connecting the Windows capture side with the Linux ROS2
consumer side (including every pitfall hit during field testing).
Data flow: **Windows (game + ac_sim_bridge) → zenoh TCP → Linux relay → ROS2 topics / RViz2**.
The bridge is the active connecting party (it connects to the relay's port 7447), so the only
network requirement is "the bridge can reach the relay's IP:7447".

Field-tested baseline (2026-10-14): five cameras, three modalities, full stream ~112 frames/s
(~17 MB/s); a WiFi link delivers only ~8% of that — **a direct gigabit Ethernet link delivers
the full stream**, with the relay using ~6% of one core.

---

## 1. Topology options

| Option | Use case | Notes |
|---|---|---|
| A. Same router (WiFi or wired) | Low-rate joint tests, functional verification | Set the bridge endpoint to the Linux machine's LAN IP. **WiFi sustains only ~2 Hz × 5 cameras**; the full stream drops 90%+ (zenoh DROP congestion policy — symptom: multi-second frame gaps) |
| B. Direct Ethernet link (recommended) | Full-throughput testing | Windows NIC ↔ Linux USB/built-in NIC on a dedicated subnet `192.168.50.0/24`, static IPs on both ends, **no gateway** on either side; Windows Internet keeps flowing over WiFi. See §4 |

---

## 2. Linux side setup

### 2.1 OS and ROS2

Ubuntu 22.04 (the native target of ROS2 Humble) + ROS2 Humble desktop:

```bash
# Official ROS2 Humble install. The apt repo key ships in the ros-apt-source package;
# the legacy ros.key URL is dead.
sudo apt install -y software-properties-common curl
# If GitHub is unreachable: download the deb on a machine that can reach it, then copy it over:
#   https://github.com/ros-infrastructure/ros-apt-source/releases → ros2-apt-source_<ver>.jammy_all.deb
sudo dpkg -i ros2-apt-source_*.deb
sudo apt update && sudo apt install -y ros-humble-desktop ros-dev-tools
```

### 2.2 Package dependencies

```bash
pip3 install eclipse-zenoh lz4          # note: the pip package for zenoh 1.x is "eclipse-zenoh"
# If direct pypi access is poor, use a mirror:
#   pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple eclipse-zenoh lz4
sudo apt install -y ros-humble-vision-msgs                 # Detection3DArray
sudo apt install -y ros-humble-image-transport-plugins     # color images in RViz2 (JPEG decode)
```

numpy comes from the system package (`python3-numpy`); nothing else to install.

### 2.3 Get the code and build

```bash
mkdir -p ~/colcon_ws/src && cd ~/colcon_ws/src
git clone git@github.com:<your account>/ac_msgs.git
git clone git@github.com:<your account>/ac_ros2_bridge.git     # private repos: configure a GitHub ssh key first
source /opt/ros/humble/setup.bash
cd ~/colcon_ws && colcon build --symlink-install
source install/setup.bash
```

**Pitfall**: if the workspace was previously built without `--symlink-install`, the
`ament_cmake_python` symlink step fails with "existing path cannot be removed: Is a
directory" — wipe `build install log` and rebuild fresh.

### 2.4 Start the relay

```bash
# KEY POINT: the relay only reads ZENOH_CONFIG from the environment. Without it the
# relay still starts and logs "ready", but uses zenoh defaults and NEVER listens on
# 7447 — the bridge can never connect (symptom: relay looks fine, all topics empty).
ZENOH_CONFIG=~/colcon_ws/src/ac_ros2_bridge/relay.json5 \
  setsid nohup ros2 run ac_ros2_bridge relay > ~/relay.log 2>&1 &
```

Verify: `ss -tln | grep 7447` must show `0.0.0.0:7447` LISTEN; the log must show
`ac_relay ready`. Re-run this after every reboot (or set up a systemd user service for
auto-start).

---

## 3. Windows side setup

1. Install the AC suite (gamehook_ac capture engine + ac_sim_bridge); `streamer.ini`
   selects which cameras/modalities/rates to forward;
2. Edit the bridge config `ac_simulator_suite\configs\bridge.json5`
   (**note: the bridge reads the copy in the install directory, not the repo copy**):
   ```
   { mode: "peer", connect: { endpoints: ["tcp/<Linux machine IP>:7447"] } }
   ```
3. The bridge reads this file only at startup — **restart the bridge from the launcher
   after editing**;
4. Once the game enters a session the bridge publishes automatically (its log line
   `img fwd: fwd=... cams=...` shows the send rate).

---

## 4. Direct Ethernet link (recommended for full throughput)

Physical: Windows NIC ↔ Linux NIC (modern NICs need no crossover cable; USB gigabit
adapters such as RTL8153 work out of the box on Linux).

Configuration (dedicated subnet 192.168.50.0/24, **no gateway on either side** — each
machine keeps its own default route, typically WiFi, for Internet):

```bash
# Linux (example NIC name enx3c18a0d56850 — substitute yours):
sudo ip addr add 192.168.50.2/24 dev enx3c18a0d56850 && sudo ip link set enx3c18a0d56850 up
```

```powershell
# Windows (admin PowerShell; find <ifIndex> via Get-NetAdapter, e.g. 14):
Get-NetIPAddress -AddressFamily IPv4 -InterfaceIndex <ifIndex> -ErrorAction SilentlyContinue |
  Where-Object IPAddress -like '192.168.0.*' | Remove-NetIPAddress -Confirm:$false
New-NetIPAddress -InterfaceIndex <ifIndex> -IPAddress 192.168.50.1 -PrefixLength 24
```

Then point the bridge endpoint to `tcp/192.168.50.2:7447` and restart the bridge.

**Verify**: `ping 192.168.50.2` from Windows (expect 0% loss, <1 ms). The reverse ping
failing is the Windows firewall blocking inbound ICMP — harmless, since the bridge
connects outbound.

**Restoring Windows wired Internet afterwards** (the static IP sticks to the NIC, so
plugging back into the router gets no DHCP — symptom: "wired connection has no
Internet"):

```powershell
Get-NetIPAddress -AddressFamily IPv4 -InterfaceIndex <ifIndex> -ErrorAction SilentlyContinue |
  Where-Object IPAddress -like '192.168.50.*' | Remove-NetIPAddress -Confirm:$false
Set-NetIPInterface -InterfaceIndex <ifIndex> -Dhcp Enabled
Set-DnsClientServerAddress -InterfaceIndex <ifIndex> -ResetServerAddresses
```

(Leaving .50.2 on the Linux side is harmless; remove with
`sudo ip addr del 192.168.50.2/24 dev <nic>`.)

---

## 5. Using RViz2

| Image | Topic | Type | How to view |
|---|---|---|---|
| Depth | `/ac/camera/<cam>/image_depth` | Image 32FC1, meters | Select directly in the RViz2 Image display; auto-normalized |
| Semantic | `/ac/camera/<cam>/image_classes` | Image mono8 (pixel = class ID) | Same; class names come from `/ac/semantic/classes` |
| **Color** | `/ac/camera/<cam>/image_raw/compressed` | **CompressedImage** (JPEG passthrough) | **RViz2's Image display does not accept this type** (its topic picker filters by type — `ros2 topic list` shows it but the picker does not). Run republish to decode first: |

```bash
# Color decode-and-forward. Field-tested on Humble: the short "in:=" syntax silently
# subscribes to the literal /in/compressed — you MUST use full-name remapping plus a
# unique node name:
source /opt/ros/humble/setup.bash && source ~/colcon_ws/install/setup.bash
setsid nohup ros2 run image_transport republish compressed raw --ros-args \
  -r /in/compressed:=/ac/camera/front/image_raw/compressed \
  -r /out:=/ac/camera/front/image_raw -r __node:=republish_front \
  > ~/republish.log 2>&1 &
```

After that, select `/ac/camera/front/image_raw` (sensor_msgs/Image) in RViz2. Start one
instance per camera you want to view.

Intrinsics: `/ac/camera/<cam>/camera_info` (sensor_msgs/CameraInfo; K is a pinhole model
derived from fovDeg; TRANSIENT_LOCAL latched — late subscribers get it immediately).

---

## 6. Acceptance checklist

```bash
source /opt/ros/humble/setup.bash && source ~/colcon_ws/install/setup.bash
ros2 topic list | grep camera          # per camera: image_raw/compressed (+depth +classes) + camera_info
ros2 topic hz /ac/camera/front/image_depth     # ≈ that camera's HZ in streamer.ini
ros2 topic echo /ac/camera/front/camera_info --field k --once   # 9 intrinsic values
# Delivery-rate diagnosis (distinguishes link loss from slow consumers):
python3 ~/colcon_ws/src/ac_ros2_bridge/tools/zenoh_rate_probe.py --seconds 12
```

Healthy baseline (gigabit direct link, field-measured): probe rate per key equals each
camera's quota, max gap ~100 ms; relay CPU ~6%.

---

## 7. Troubleshooting table

| Symptom | Cause and fix |
|---|---|
| relay "ready" but all topics empty | relay started without `ZENOH_CONFIG` → not listening on 7447 (§2.4); or the bridge points at the wrong IP / was not restarted |
| bridge log shows full send rate, Linux probe far below quota, multi-second gaps | Link bandwidth insufficient (typical on WiFi) → switch to a direct Ethernet link (§4) |
| Color topic not findable in RViz2 | CompressedImage filtered out by RViz2's type-based picker → republish (§5) |
| republish runs but no images | Used the `in:=` short syntax (subscription dangles on /in/compressed) → full-name remapping (§5) |
| camera_info topic missing | The bridge publishes it once at ring-connect; it can be dropped in the image burst → restart the bridge (with the relay already running it usually arrives); long-term fix is periodic resend (pending) |
| RViz reports incompatible QoS, no images | Image publishers are RELIABLE; RViz2's default RELIABLE subscription is compatible — the error usually comes from some other publisher |
| Linux build fails "existing path cannot be removed: Is a directory" | Old build artifacts clash with symlink mode → `rm -rf build install log` and rebuild (§2.3) |
| Windows wired Internet dead after plugging back into the router | Leftover direct-link static IP → restore DHCP (§4 restore block) |

---

## 8. Appendix: WSL2 as a fallback test rig

With no physical Linux machine: run the relay in WSL2 Ubuntu-22.04 on the Windows box
itself; the bridge connects to `tcp/localhost:7447` (WSL2 NAT localhost forwarding covers
the host→WSL direction, zero network configuration). Functionally equivalent — but the
relay **competes with the fully loaded game for CPU**, so decompressed-modality
throughput numbers are meaningless (field test: depth dropped to 1-5 Hz in WSL vs
94-100% on the real machine under the same load) — draw throughput conclusions from
real hardware only.
