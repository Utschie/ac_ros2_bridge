# ROS2 部署与联测手册

从零把 Windows 采集端和 Linux ROS2 消费端连通的完整步骤。数据流：
**Windows（游戏 + 套件）→ zenoh TCP → Linux 中继 → ROS2 话题 / RViz2**。
桥是主动连接方——只需保证"桥能连到中继机器的 7447 端口"。

实测基线：五相机三模态全量约 112 帧/s（约 17 MB/s）；WiFi 只够低速
联调，全量吞吐请用网线直连（见"网线直连"一节）。

## 0. 话题与坐标系

输出遵循 ROS 标准约定，与套件统一约定一致（X 前 / Y 左 / Z 上；世界系
`ac_world` 的 z 为高度）。主要话题与 TF：

| 组 | 话题 / TF |
|---|---|
| 自车位姿 | `/ac/ego_pose`（PoseStamped）+ TF `ac_world → base_link` |
| 传感器安装位 | 静态 TF `base_link → <雷达>_frame`、`base_link → <相机>_frame` |
| 3D 框 | `/ac/detections_3d`（Detection3DArray）+ `/ac/detections_3d_markers` |
| 点云 | `/ac/lidar/<名字>/points`（传感器本地系，与真实硬件一致） |
| 相机 | `/ac/camera/<相机>/image_raw/compressed`、`image_depth`、`image_classes`、`camera_info` |
| 类别表 | `/ac/semantic/classes` |
| 控制（订阅） | `/ac/vehicle_cmd`、`/ac/game_cmd`（ac_msgs） |
| 反馈（发布） | `/ac/vehicle_state`、`/ac/game_state`、`/ac/expert_ctrl`、`/ac/telemetry`（每场景帧）、`/ac/timing`（每场景帧）、`/ac/telemetry_static`（latched，会话变化时）、`/ac/run_state`（事件驱动） |

---

## 1. Linux 端安装

Ubuntu 22.04 + ROS2 Humble：

```bash
sudo apt install -y software-properties-common curl
# 若 GitHub 直连不畅：在能联网的机器下载 ros2-apt-source 的 deb 后拷入安装
sudo apt update && sudo apt install -y ros-humble-desktop ros-dev-tools
pip3 install eclipse-zenoh lz4
sudo apt install -y ros-humble-vision-msgs ros-humble-image-transport-plugins
```

获取套件提供的 ROS 中继源码（开源仓库，直接 clone 到工作区）：

```bash
mkdir -p ~/colcon_ws/src && cd ~/colcon_ws/src
git clone https://github.com/Utschie/ac_msgs.git
git clone https://github.com/Utschie/ac_ros2_bridge.git
source /opt/ros/humble/setup.bash
cd ~/colcon_ws && colcon build --symlink-install
source install/setup.bash
```

## 2. 启动中继

```bash
ZENOH_CONFIG=~/colcon_ws/src/ac_ros2_bridge/relay.json5 \
  ros2 run ac_ros2_bridge relay
```

启动正常时应看到日志 `ac_relay ready`，且 `ss -tln | grep 7447` 显示
LISTEN。停止用 Ctrl+C。⚠ 必须带 `ZENOH_CONFIG` 启动——不带的话中继
看似 ready，实际没有监听 7447。每次开机后需重新启动（或配置自启）。

## 3. Windows 端配置

编辑 `ac_simulator_suite\configs\bridge.json5`，把地址指向中继机器：

```
{ mode: "peer", connect: { endpoints: ["tcp/<Linux 机器 IP>:7447"] } }
```

该文件在 bridge 启动时读取一次——改完从启动器重启 bridge。游戏进入
赛道后数据自动流出。

## 4. 网线直连（全量吞吐推荐）

WiFi 只够低速联调；全量流推荐网线直连。步骤：

1. **Linux 端设静态 IP**（网卡名换成自己的，用 `ip addr` 查）：

   ```bash
   sudo ip addr add 192.168.50.2/24 dev <网卡名> && sudo ip link set <网卡名> up
   ```

2. **Windows 端设静态 IP**（管理员 PowerShell；`<ifIndex>` 用
   `Get-NetAdapter` 查）：

   ```powershell
   New-NetIPAddress -InterfaceIndex <ifIndex> -IPAddress 192.168.50.1 -PrefixLength 24
   ```

   两台机器用网线直连网卡；两边都**不设网关**，Windows 上网走原有
   WiFi 不受影响。

3. **验证连通**：Windows 上 `ping 192.168.50.2`（反向 ping 不通是
   Windows 防火墙拦 ICMP，无碍——桥是向外连接的）。

4. **改地址并重启**：把 bridge.json5 指向 `tcp/192.168.50.2:7447`，
   从启动器重启 bridge。

5. **用完恢复**：把 Windows 网卡恢复 DHCP（删除上面的静态 IP）即可。

## 5. RViz2 使用

| 图 | 话题 | 类型 | 看法 |
|---|---|---|---|
| 深度 | `/ac/camera/<cam>/image_depth` | Image 32FC1 米制 | RViz2 Image 显示直接选，自动归一化 |
| 语义 | `/ac/camera/<cam>/image_classes` | Image mono8（像素=类别 ID） | 同上；类名查 `/ac/semantic/classes` |
| **彩色** | `/ac/camera/<cam>/image_raw/compressed` | **CompressedImage**（JPEG 透传） | **RViz2 的 Image 显示不认此类型**（话题选择器按类型过滤，`ros2 topic list` 能见但选不到）——需先 republish 解码： |

```bash
# 彩色图解码转发（注意：必须用全名重映射，不能用 in:= 简写）：
source /opt/ros/humble/setup.bash && source ~/colcon_ws/install/setup.bash
ros2 run image_transport republish compressed raw --ros-args \
  -r /in/compressed:=/ac/camera/front/image_raw/compressed \
  -r /out:=/ac/camera/front/image_raw -r __node:=republish_front
```

之后 RViz2 里选 `/ac/camera/front/image_raw`（sensor_msgs/Image）即可。停止用 Ctrl+C；每个要看的相机起一路（改对应话题名与节点名）。

相机内参：`/ac/camera/<cam>/camera_info`（sensor_msgs/CameraInfo）。

---

## 6. 验收清单

```bash
source /opt/ros/humble/setup.bash && source ~/colcon_ws/install/setup.bash
ros2 topic list | grep camera          # 每相机应有 image_raw/compressed(+depth+classes)+camera_info
ros2 topic hz /ac/camera/front/image_depth     # ≈ streamer.ini 该相机 HZ
ros2 topic echo /ac/camera/front/camera_info --field k --once   # 9 个内参
# 送达率诊断（区分链路丢 vs 消费端慢）：
python3 ~/colcon_ws/src/ac_ros2_bridge/tools/zenoh_rate_probe.py --seconds 12
```

正常情况：每个话题的速率约等于该相机配置的 HZ，画面连续、无秒级停顿。

---

## 7. 故障排查

| 症状 | 原因与处理 |
|---|---|
| 中继 ready 但话题全空 | 没带 `ZENOH_CONFIG` 启动（见"启动中继"一节）；bridge.json5 地址错（见"Windows 端配置"一节）；改完没重启 bridge |
| 桥满速发送、Linux 实收远低、有秒级空洞 | 链路带宽不足（WiFi 典型）→ 网线直连（见"网线直连"一节） |
| RViz2 选不到彩色图话题 | CompressedImage 被选择器滤掉 → republish（见"RViz2 使用"一节） |
| republish 起了但没图 | 用了 `in:=` 简写 → 全名重映射（见"RViz2 使用"一节） |
| camera_info 迟迟不来 | 稍等几秒（周期重发）；仍无则重启 bridge |
| Linux 构建报 "existing path cannot be removed" | 清掉 build/install/log 后重建（见"Linux 端安装"一节） |
| Windows 有线插回路由器无法上网 | 直连静态 IP 残留 → 网卡恢复 DHCP（见"网线直连"一节） |
