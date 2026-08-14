# ac_ros2_bridge

AC 仿真 ↔ ROS2 中继（Linux 端）。订阅 Windows 上 `ac_sim_bridge` 经 **Zenoh** 发出的数据，
按 `shared/zenoh_payload.h` 解码，发布标准 ROS2 话题，让你在 **RViz** 里看到车移动 + 周围车 3D OBB。

> 当前状态：**中继 M1 + 激光雷达全部端到端实测通过**（Humble + peer 直连）——RViz 中车辆、
> 3D OBB、4 雷达点云均与游戏画面一致。订阅 `ac/ego_pose` + `ac/detections_3d` + `ac/lidar/*/points`，
> 发 `/ac/ego_pose`(`PoseStamped`) + TF(`ac_world→base_link`) + `/ac/detections_3d`(`Detection3DArray`)
> + `/ac/detections_3d_markers`(`MarkerArray` 保底) + `/ac/lidar/<name>/points`(`PointCloud2`)
> + TF(`base_link→<name>_frame`)。**AC→ROS 坐标转换全部集中在 [frames.py](ac_ros2_bridge/frames.py)**
> （4 个实测定案事实见其 docstring，C++ 侧对应 shared/ac_coords.h）。图像/控制等后续里程碑。
> **ROS2 只装在这一侧**。

## 坐标系（定案，2026-08 实测）

全部转换在 [frames.py](ac_ros2_bridge/frames.py)（`ac_vec_to_ros` = `body_vec_to_ros` =
唯一保定向映射 `(z,x,y)` / `ac_axes_to_ros_quat` 朝向（无取负）/ `install_pose_to_ros` 安装位姿），
离线测试 `test_compose.py` 覆盖。四个定案事实与推导历史见 frames.py docstring 和
[shared/ac_coords.h](../shared/ac_coords.h)。

> **⚠️ 验证纪律**：改任何符号前必须对照**游戏画面**在**正读视角**下重新实测（纯代数推演曾连续
> 失败三轮；后来又因镜像 RViz 相机连续误验三轮）。**正读视角 = RViz 开 TF 显示，base_link 的轴
> 红(X)朝前、绿(Y)朝左、蓝(Z)朝上**——先确认 axes 正读，再判断内容左右；axes 不正读时一切
> 内容判断都不可信（从地面下方看世界整体左右镜像，错误数据会显得正确）。调试：relay 每 2s 打印
> 一行 `egoMatrix`（BODYTR 矩阵原始三元组）——直道停车即可校验矩阵约定；每雷达首见打一行
> `lidarDiag`（名字↔安装位姿↔帧落点↔帧朝向）。

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

（旧小节已并入上方"坐标系（定案）"——转换实现与事实清单以 frames.py 为准。）

## 备选：路由器模式（多机扩展时用）

跑 `zenohd`（Zenoh 路由器）：Linux 起 `zenohd`（监听 7447）；relay 默认 client 连 `localhost:7447`；
ac_sim_bridge 的 `ZENOH_CONFIG` 指 `{ connect: { endpoints: ["tcp/LINUX_IP:7447"] } }`。

## 激光雷达（Phase 2）

前置（Windows 侧，除 AC 和 ac_sim_bridge 外）：
1. 运行 **ac_optix_renderer**（新版本已内置 `AC_LidarCache_v1` 写入器；vis 模式按 **L** 开雷达，
   headless 模式默认开；按 `custom_lidars.ini` 配置雷达）。
2. ac_sim_bridge 自动发现该 SHM 并转发（统计行出现 `lidarPts=N`）。

Linux 侧无需改配置：relay 订阅 `ac/lidar/*/points`，发布
`/ac/lidar/<name>/points`（`sensor_msgs/PointCloud2`，fields: range/x/y/z/intensity/instance_id）。
**点保持在传感器局部系**（x前 y左 z上，同真实雷达驱动），frame_id = `<name>_frame`
（front_frame / rear_frame）。TF 树完整：
```
ac_world ─(ego_pose 每帧)→ base_link ─(inv(ego)·S 每扫描)→ front_frame / rear_frame
```
RViz2 / 下游经 TF 链自动解析到任意固定帧；`ros2 run tf2_tools view_frames` 可查看。

RViz：Add → **PointCloud2**，话题 `/ac/lidar/<name>/points`，Fixed Frame `ac_world`。
开车 → 点云应贴着地物随车移动（车动、点云与车体相对关系不变）。

> 带宽提示：vis 模式每帧全速扫描（4 雷达 ≈ 数百 MB/s）。跨机测试建议 `custom_lidars.ini` 少开雷达，
> 或 ac_optix_renderer 用 `--real-hz` 按真实 Hz 限频。

## 后续里程碑（见总体计划）
- 图像（原始/NVENC H.264）→ `sensor_msgs/Image`/`CompressedImage`。
- 控制下行：订阅 ROS2 `/ac/vehicle_cmd` → Zenoh `ac/vehicle_cmd` → Windows 桥写 `AC_ControlCache_v1`。
