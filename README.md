# ac_ros2_bridge

AC 仿真 ↔ ROS2 中继（Linux 端）。订阅 Windows 上 `ac_sim_bridge` 经 **Zenoh** 发出的数据，
按 `shared/zenoh_payload.h` 解码，发布标准 ROS2 话题，让你在 **RViz** 里看到车移动 + 周围车 3D OBB。

> 当前状态：**中继 M1**——订阅 `ac/ego_pose` + `ac/detections_3d`，发
> `/ac/ego_pose`(`PoseStamped`) + TF(`ac_world→base_link`) + `/ac/detections_3d`(`Detection3DArray`)。
> 图像/激光雷达/控制等后续里程碑再加。**ROS2 只装在这一侧**。

## 依赖（Linux）

- **ROS2 Humble 或 Jazzy**（两者皆可；中继走 zenoh-python 直连、非 rmw_zenoh，故发行版灵活。含 `rclpy`、`geometry_msgs`、`vision_msgs`、`tf2_ros`、`std_msgs`。vision_msgs 字段差异已自动兼容）。
- **zenoh-python 1.x**：`pip install zenoh`（非 ROS 包，pip 装）。
- 解码格式见 [shared/zenoh_payload.h](../shared/zenoh_payload.h)；Python 侧 [ac_ros2_bridge/wire.py](ac_ros2_bridge/wire.py)。
- 纯 Python 解码可在任意 OS 验证：`python ac_ros2_bridge/ac_ros2_bridge/test_decode.py`（应打印 `decode_check OK`）。

## 构建（Linux，colcon）

```bash
cd <workspace>            # 包含 ac_ros2_bridge/ 的目录
source /opt/ros/setup.bash        # 或 /opt/ros/humble/setup.bash（看你装的是哪个发行版）
colcon build --packages-select ac_ros2_bridge
source install/setup.bash
```

## 运行

```bash
ros2 run ac_ros2_bridge relay
```
预期日志：`[INFO] [...] [ac_relay]: ac_relay ready: subscribing ac/ego_pose + ac/detections_3d`。
收到 Windows 的数据后会在 `/ac/ego_pose`、`/ac/detections_3d`、`/tf` 上发布。

## Zenoh 跨机连通（关键）

Windows 上的 `ac_sim_bridge` 要把数据送到 Linux 这个中继，两者必须在同一个 Zenoh 网络。两种方案（都**不用改代码**，靠 `ZENOH_CONFIG` 环境变量指一个 JSON5 配置）：

**方案 A（推荐，免装路由器，点对点）**——Linux 中继监听，Windows 桥连过来：
- Linux（relay），写 `relay.json5`：
  ```json5
  { mode: "peer", listen: { endpoints: ["tcp/0.0.0.0:7447"] } }
  ```
  `ZENOH_CONFIG=relay.json5 ros2 run ac_ros2_bridge relay`
- Windows（ac_sim_bridge），写 `bridge.json5`（把 `LINUX_IP` 换成 Linux 机器 IP）：
  ```json5
  { mode: "peer", connect: { endpoints: ["tcp/LINUX_IP:7447"] } }
  ```
  `set ZENOH_CONFIG=bridge.json5` 后跑 `ac_sim_bridge.exe`。

**方案 B（路由器）**——跑 `zenohd`（Zenoh 路由器，二者都 client 连它）：
- Linux 起 `zenohd`（监听 7447）。
- relay（Linux）默认 client 连 `localhost:7447`（不用设 ZENOH_CONFIG）。
- ac_sim_bridge（Windows）：`ZENOH_CONFIG` 指 `{ connect: { endpoints: ["tcp/LINUX_IP:7447"] } }`。

## RViz 验证

```bash
rviz2
```
加显示：
- **TF**（看 `ac_world → base_link`）。
- **Pose** / `PoseWithCovariance`，话题 `/ac/ego_pose`。
- **MarkerArray** 或 `Detection3DArray`（RViz 内置 3D 检测显示），话题 `/ac/detections_3d`。
- 固定帧设 `ac_world`。

开车：应看到 `base_link`（自车）移动，`/ac/detections_3d` 画出各车 OBB 框。

> **坐标系**：AC 世界系可能是 **Y-up**，ROS 是 **Z-up**。M1 先按 AC 原矩阵发到 `ac_world` 帧；
> 若车/OBB 在 RViz 里是侧躺的，加一个静态轴变换把 AC 系对到 ROS 系，或后续在中继里做轴重映射。
> 先确认"车会动 + OBB 数量对"，朝向细节再调。

## 后续里程碑（见总体计划）
- 图像（原始/NVENC H.264）→ `sensor_msgs/Image`/`CompressedImage`。
- 激光雷达 → `sensor_msgs/PointCloud2`（需 ac_sim_bridge 先接 `AC_LidarCache_v1`）。
- 控制下行：订阅 ROS2 `/ac/vehicle_cmd` → Zenoh `ac/vehicle_cmd` → Windows 桥写 `AC_ControlCache_v1`。
