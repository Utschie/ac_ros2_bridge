# ac_ros2_bridge

AC 仿真 ↔ ROS2 中继（Linux 端）。订阅 Windows 上 `ac_sim_bridge` 经 **Zenoh** 发出的数据，
按 `shared/zenoh_payload.h` 解码，发布标准 ROS2 话题，让你在 **RViz** 里看到车移动 + 周围车 3D OBB。

> 当前状态：**ego_pose / detections_3d / 雷达点云 / 相机图像全部端到端实测通过**（Humble +
> peer 直连）——RViz2 中车辆、3D OBB、点云、图像均正常显示。订阅 `ac/ego_pose` +
> `ac/detections_3d` + `ac/lidar/*/points` + `ac/camera/**`，发 `/ac/ego_pose`(`PoseStamped`)
> + TF(`ac_world→base_link`) + `/ac/detections_3d`(`Detection3DArray`)
> + `/ac/detections_3d_markers`(`MarkerArray` 保底) + `/ac/lidar/<name>/points`(`PointCloud2`)
> + `/ac/camera/<cam>/<suffix>`(`Image`) + TF(`base_link→<name>_frame` 静态)。
> **AC→ROS 坐标转换全部集中在 [frames.py](ac_ros2_bridge/frames.py)**
> （4 个实测定案事实见其 docstring，C++ 侧对应 shared/ac_coords.h）。
> **WiFi 带宽注意**：实测过载即崩溃（见"相机图像"节与 ac_sim_bridge README 带宽预算），
> 联测务必控制 Windows 端发送总量 ≈8-15MB/s。压缩/控制等后续里程碑。
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

## 时间戳与 TF 策略（M3，修复 RViz2 "No Transform" 闪烁）

- **所有出站消息/TF 的 header.stamp 用 relay 本机 ROS 时钟重打**（每回调用一次、消息与 TF 同刻）。
  生产端的 Windows 墙钟时间戳**不透传**——两机时钟未同步，钟差曾让 RViz2 看到"未来时间戳"
  的消息、tf2 拒绝外推，表现为包围盒/点云间歇闪烁 "No Transform ..."。
- **帧精确配对**：bridge 对同一仿真帧的 ego/dets/lidar 共用一个生产端时间戳；relay 按它 join，
  给 dets/点云打**同帧** ego TF 的本地时刻（`_publish_paired`）。否则 zenoh 回调乱序时钳制会取到
  上一帧位姿 → 车动时包围盒相对 base_link 轴"向前窜"。配对未命中先挂起（≤0.5s），同帧 ego 到达
  即发，超时回退钳制戳。**图像不参与配对**（grab 时刻戳与 ego 无 join，且 Image 显示不查 TF）。
- **依赖 ac_world→base_link 链的消息戳钳制到最新 ego TF 时刻**（`_stamp_clamped`，配对超时的兜底）：
  按消息戳查 TF 时 buffer 必有同刻或更新的变换，与到达顺序无关，机制上杜绝该类失败。
- **TF 树**：动态仅 `ac_world → base_link`（~50Hz，随 ego）；`base_link → <lidar>_frame` 安装
  位姿恒定，走 `/tf_static`（latched）——**每次变更把全部静态变换放进一条消息发送**：
  `/tf_static` 的 latch depth=1，晚加入的订阅者只拿最后一次样本；分次单发会让后启动的
  RViz2 丢掉除最后一个外的全部静态 TF，对应雷达点云卡在 message filter 报
  "queue is full" 永不显示（实测教训）。
- **发布锁**：zenoh 回调跑在 zenoh 线程而非 rclpy executor 线程，所有 ROS publish/create_publisher/
  TF 发送经同一把 `threading.Lock` 串行化（配对状态另由 `_pair_lock` 保护）。
- **取舍**：时间轴变为"relay 接收时刻"（帧间间隔=真实+网络抖动，rosbag 录制同此）；严格仿真时间
  语义留给 `/clock + use_sim_time` 未来阶段（届时只换取时来源，架构不变）。暂停时 marker 0.5s
  过期属"只显实时数据"的既定行为。
- **帧对齐现状**：ego↔dets 严格同帧（同一场景快照+同一时间戳）；ego↔lidar ≈1–2 帧（optix 的
  scanFrameId 是自身计数器非场景帧号，线上无游戏帧号可 join——严格对齐待未来统一帧号源）；
  图像独立采样（frameId 与场景计数器不同源）。

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
2. **Windows：跑桥**——两种方式任选：
   - 命令行（`set ZENOH_CONFIG` 与 exe 在**同一个** cmd 窗口）：
     ```bat
     set ZENOH_CONFIG=C:\ac\bridge.json5
     cd ac_sim_bridge\build\Release
     ac_sim_bridge.exe
     ```
   - **双击部署**：四件套（exe/zenohc.dll/bridge.json5/bridge_images.ini）放独立目录（推荐
     `<AC安装目录>\ac_bridge\` 子目录，⚠️ 不要放 acs.exe 同目录——详见 ac_sim_bridge README），
     双击运行，配置自动从 exe 旁读取。
   应打印 `connected to AC_SceneCache_v2` + 每秒 `frame=... detections=N lidarMB/s=... ego=(...)`。
3. **Linux：跑中继**：
   ```bash
   cd ~/ac_ws && source install/setup.bash
   ZENOH_CONFIG=~/relay.json5 ros2 run ac_ros2_bridge relay
   ```
   预期日志：`[INFO] [ac_relay]: ac_relay ready: subscribing ac/ego_pose + ac/detections_3d + ac/lidar/*/points + ac/camera/** (AC->ROS axis remap in frames.py)`；收到相机数据后还会打 `image publisher '/ac/camera/<cam>/image_raw' (WxH rgba8)`。

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
| RViz 报 `offering incompatible QoS ... RELIABILITY_QOS_POLICY` 无图 | 订阅器要 RELIABLE 但发布器是 BEST_EFFORT（唯一不兼容组合）。relay 的图像发布器已是 RELIABLE——若仍报，检查是否跑的旧代码；或临时把 RViz2 显示的 Reliability 改 Best Effort |

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
（front_frame / rear_frame）。TF 树完整（M3 起 `<lidar>_frame` 走静态 TF）：
```
ac_world ─(ego_pose 每帧, /tf)→ base_link ─(安装位姿恒定, /tf_static)→ front_frame / rear_frame
```
RViz2 / 下游经 TF 链自动解析到任意固定帧；`ros2 run tf2_tools view_frames` 可查看。

RViz：Add → **PointCloud2**，话题 `/ac/lidar/<name>/points`，Fixed Frame `ac_world`。
开车 → 点云应贴着地物随车移动（车动、点云与车体相对关系不变）。

> 带宽提示：vis 模式每帧全速扫描（4 雷达 ≈ 数百 MB/s）。跨机测试建议 `custom_lidars.ini` 少开雷达，
> 或 ac_optix_renderer 用 `--real-hz` 按真实 Hz 限频。

## 相机图像（M2）

relay 订阅 `ac/camera/**`，把 `ac/camera/<cam>/<suffix>` 原样转成 ROS 话题
`/ac/camera/<cam>/<suffix>`（`sensor_msgs/Image`，**Reliable QoS（depth 5）**——RViz2 的
Image 显示默认 RELIABLE 订阅，发布器用 BEST_EFFORT 会被 DDS 拒绝配对；跨机丢帧由 Windows 端
zenoh DROP 拥塞控制兜底，relay→RViz2 是本地回环、RELIABLE 无代价）。像素字节直通不转 copy
（同点云做法），编码映射见 [wire.py](ac_ros2_bridge/wire.py)
的 `ENCODING_TO_ROS`：RGBA8/BGRA8/32FC1(R32F 深度)/mono8。`image_classes`（语义图）当前载荷
为 **RGBA8 彩色语义图**（生产端如此，mono8 class-id 为未来项）。`frame_id = <cam>_frame`
（相机 TF 属后续阶段；Image 显示不查 TF，不影响显示）。

前置（Windows 侧）：AC 带 gamehook_ac + `custom_cameras.ini`；ac_sim_bridge 用
`bridge_images.ini`（per-camera 分段，见其 README）决定转发哪些相机/模态/频率——
**WiFi 联测务必只开 1-2 路、低 HZ**。实测拥塞崩溃曲线（仅 front、雷达关）：
20Hz(84MB/s)→实收 0.6Hz、10Hz→1Hz、**2Hz(8.4MB/s)→满速 2Hz**——过载即崩溃，
可用预算 ≈8-20MB/s；`AC_BRIDGE_IMAGE_HZ` env 可快速覆盖所有相机频率。

### 验证

```bash
ros2 topic list                      # 应有 /ac/camera/front/image_raw 等
ros2 topic hz /ac/camera/front/image_raw        # ≈配置的 HZ（WiFi 联测建议 2）
ros2 topic echo /ac/camera/front/image_raw --field encoding   # rgba8（或 bgra8）
ros2 topic echo /ac/camera/front/image_raw --field width --once  # 1365（保比例裁剪）
```

RViz2：Fixed Frame `ac_world` → Add → **Image** → 话题 `/ac/camera/front/image_raw` → 开车看图。
**通道序检查**：游戏里红色的车在 Image 显示里也应是红色（若红蓝互换 = BGRA/RGBA 映射错，
报给 bridge 侧改 encOf）。深度/语义图同法（`image_depth` 显示为 32FC1 灰度浮点，
RViz2 Image 显示会自动归一化）。

## 踩坑记录（按发现顺序；详细机理见上文对应小节）

1. **时间戳跨机钟差 → "No Transform" 闪烁**：透传 Windows 墙钟，两机不同步 → RViz2 看到
   "未来时间戳"消息、tf2 拒绝外推 → 间歇闪烁。修复 = 本地重打 + 钳制（"时间戳与 TF 策略"节）。
2. **配对缺失 → 包围盒"向前窜"**：仅钳制不保证同帧——zenoh 回调乱序时 dets 绑到上一帧 ego
   位姿，车动时盒子相对 base_link 轴前偏 v×20ms（30m/s ≈0.6m，清晰可见）。修复 = 按生产端
   共享时间戳做**帧精确配对**（ego/dets/lidar 同帧同 ts）。
3. **/tf_static latch depth=1 → 部分雷达永不显示**：逐雷达分次 sendTransform，晚启动的 RViz2
   只拿到最后一个 latched 样本 → 其余雷达链路断裂，点云卡 message filter "queue is full"。
   修复 = 全部静态变换**放进一条消息发送**，每次变更重发全集。
4. **图像 QoS 不兼容 → 无图**：`qos_profile_sensor_data`(BEST_EFFORT) 发布器 + RViz2 Image
   显示默认 RELIABLE 订阅 = 唯一不兼容组合（DDS 拒绝配对）。修复 = 发布器 RELIABLE（本地回环
   无代价，跨机由 zenoh DROP 兜底）。
5. **zenoh 回调线程 ≠ rclpy executor 线程**：rclpy publish 非线程安全 → `pub_lock` 串行化全部
   ROS 调用；配对状态另用 `_pair_lock`（发布永远不在配对锁内，防死锁）。
6. **带宽拥塞崩溃**（跨机链路）：过载时好吞吐塌缩（84MB/s 需求 → 0.6Hz 实收），不是"尽力而为"
   ——Windows 端必须预算制（见相机图像节与 ac_sim_bridge README）。

## 后续计划

- **M5 模块化重构**：relay.py 拆为 RelayContext（取时/配对/锁/TF 横切策略）+ scene_relay /
  image_relay 按数据域分模块——新数据类型 = 新增一个域模块。
- **图像压缩接收**：`MODE_COMPRESSED` → `sensor_msgs/CompressedImage` 分支（线格式已预留，
  Windows 端 NVENC 就绪后联调）。
- **CameraInfo + 相机 TF**：`base_link→<cam>_frame` 静态 TF 与内参发布。
- **/clock + use_sim_time**：relay 发布 /clock，届时只替换 RelayContext 的取时来源。
- **统一帧号源**：gamehook_ac/optix 把场景帧号写进 image/lidar SHM → 严格帧对齐（当前
  ego↔dets 严格、ego↔lidar ≈1-2 帧、图像独立，见"帧对齐现状"）。
- **控制下行**：订阅 ROS2 `/ac/vehicle_cmd` → Zenoh → Windows 桥写 `AC_ControlCache_v1`。
