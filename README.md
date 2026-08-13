# ac_ros2_bridge

AC 仿真 ↔ ROS2 中继（Linux 端）。订阅 Windows 上 `ac_sim_bridge` 经 **Zenoh** 发出的数据，
按 `shared/zenoh_payload.h` 解码，发布标准 ROS2 话题，让你在 **RViz** 里看到车移动 + 周围车 3D OBB。

> 当前状态：**中继 M1 已端到端实测通过**（Humble + peer 直连）——RViz 中车辆与 3D OBB 随开车实时移动。
> 订阅 `ac/ego_pose` + `ac/detections_3d`，发 `/ac/ego_pose`(`PoseStamped`) + TF(`ac_world→base_link`)
> + `/ac/detections_3d`(`Detection3DArray`) + `/ac/detections_3d_markers`(`MarkerArray` 保底可视化)。
> 坐标轴重映射（AC→ROS）已内置。图像/激光雷达/控制等后续里程碑再加。**ROS2 只装在这一侧**。

## 依赖（Linux）

- **ROS2 Humble 或 Jazzy**（两者皆可；中继走 zenoh-python 直连、非 rmw_zenoh，故发行版灵活。含 `rclpy`、`geometry_msgs`、`vision_msgs`、`visualization_msgs`、`tf2_ros`。vision_msgs 字段差异已自动兼容）。
- **zenoh-python 1.x**：`pip install zenoh`（非 ROS 包，pip 装）。
- **RViz 的 Detection3DArray 显示插件**（可选，`/ac/detections_3d` 原生显示）：
  ```bash
  sudo apt install ros-$ROS_DISTRO-vision-msgs-rviz-plugins
  # Humble 实测包名：ros-humble-vision-msgs-rviz-plugins
  ```
  不装也行——relay 同时发 `/ac/detections_3d_markers`（MarkerArray），RViz2 内置显示。
- 构建工具：`sudo apt install python3-colcon-common-extensions`。
- 解码格式见 [shared/zenoh_payload.h](../shared/zenoh_payload.h)；Python 侧 [ac_ros2_bridge/wire.py](ac_ros2_bridge/wire.py)。
- 纯 Python 解码可在任意 OS 验证：`python ac_ros2_bridge/ac_ros2_bridge/test_decode.py`（应打印 `decode_check OK`）。

## 完整测试流程（已实测通过，照此复现）

### 1. 构建（Linux）

```bash
mkdir -p ~/ac_ws && cp -r ac_ros2_bridge ~/ac_ws/
cd ~/ac_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select ac_ros2_bridge
source install/setup.bash
```

### 2. 写两份 Zenoh 配置（点对点，免装路由器）

**Linux**（存成 `~/relay.json5`）：
```json5
{ mode: "peer", listen: { endpoints: ["tcp/0.0.0.0:7447"] } }
```

**Windows**（存成如 `C:\ac\bridge.json5`，`LINUX_IP` 换成 Linux 机器局域网 IP，Linux 上 `ip a` 查）：
```json5
{ mode: "peer", connect: { endpoints: ["tcp/LINUX_IP:7447"] } }
```

Linux 防火墙放行 7447（用 ufw 的话）：`sudo ufw allow 7447`。

### 3. 按顺序启动

1. **Windows：启动 AC**（带 gamehook_ac + scene_sync 插件，进有车的场景）。
2. **Windows：跑桥**（`set ZENOH_CONFIG` 与 exe 必须在**同一个** cmd 窗口）：
   ```bat
   set ZENOH_CONFIG=C:\ac\bridge.json5
   cd ac_sim_bridge\build\Release
   ac_sim_bridge.exe
   ```
   应打印 `connected to AC_SceneCache_v2` + 每秒 `frame=... detections=N ego=(...)`。
3. **Linux：跑中继**：
   ```bash
   cd ~/ac_ws && source install/setup.bash
   ZENOH_CONFIG=~/relay.json5 ros2 run ac_ros2_bridge relay
   ```
   预期日志：`[INFO] [ac_relay]: ac_relay ready: subscribing ac/ego_pose + ac/detections_3d (AC->ROS axis remap enabled)`。

### 4. 验证（从简到繁，均已实测）

```bash
ros2 topic list             # 应有 /ac/ego_pose  /ac/detections_3d  /tf  /ac/detections_3d_markers
ros2 topic hz /ac/ego_pose  # ~50Hz（≈ AC/scene_sync 帧率）
ros2 topic echo /ac/ego_pose --once   # 看 position；开车后再 echo，数值应变化
```

**RViz 终极验证**：
```bash
rviz2
```
- Fixed Frame 设 `ac_world`。
- Add → **TF**（`base_link` 随车移动）。
- Add → **Pose**，话题 `/ac/ego_pose`。
- Add → **MarkerArray**，话题 `/ac/detections_3d_markers`（内置显示，绿色半透明 OBB）。
- 或 Add → **Detection3DArray**，话题 `/ac/detections_3d`（需装 `vision-msgs-rviz-plugins`）。

开车 → 车移动 + 周围车 3D OBB 跟随 = 全链路（AC → gamehook 插件 → SHM → ac_sim_bridge → Zenoh → relay → ROS2 → RViz）打通。

### 5. 排错速查

| 症状 | 查什么 |
|---|---|
| relay 起来了但 topic 无数据 | ① Windows 窗口里 `echo %ZENOH_CONFIG%` 确认生效；② `LINUX_IP` 写对没；③ Linux 防火墙 7447；④ 两边配置 `mode` 都须 `"peer"` |
| RViz 无 Marker | 确认加的是 **MarkerArray** 显示类型、话题 `/ac/detections_3d_markers` |
| `ModuleNotFoundError: zenoh` | `pip install zenoh`，并确认装到了跑 ros2 的那个 Python |
| Detection3DArray 显示类型不存在 | 装 `ros-$ROS_DISTRO-vision-msgs-rviz-plugins`，或改用 MarkerArray |

## 坐标系（已内置转换，无需手动处理）

AC 世界系是 **Y-up 左手系**（实测：开车时 x/z 变、y=海拔不变），ROS 是 Z-up 右手系。
relay 内做轴重映射（AC 的 Z前→ROS X前、Y上→ROS Z上、X右→ROS -Y），发到 `ac_world` 帧的
数据**直接就是 ROS 约定**——车直立在网格上、朝向正确，无需静态变换。轴对齐时输出即单位姿态。
（实测验证：AC 单位矩阵 → 单位四元数；位置 (-710,138,2575)→(2575,710,138)，海拔 y 正确落 ROS Z。）

## 备选：路由器模式（多机扩展时用）

跑 `zenohd`（Zenoh 路由器）：Linux 起 `zenohd`（监听 7447）；relay 默认 client 连 `localhost:7447`；
ac_sim_bridge 的 `ZENOH_CONFIG` 指 `{ connect: { endpoints: ["tcp/LINUX_IP:7447"] } }`。

## 后续里程碑（见总体计划）
- 图像（原始/NVENC H.264）→ `sensor_msgs/Image`/`CompressedImage`。
- 激光雷达 → `sensor_msgs/PointCloud2`（需 ac_sim_bridge 先接 `AC_LidarCache_v1`）。
- 控制下行：订阅 ROS2 `/ac/vehicle_cmd` → Zenoh `ac/vehicle_cmd` → Windows 桥写 `AC_ControlCache_v1`。
