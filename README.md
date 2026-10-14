# ac_ros2_bridge

> **部署与联测完整说明书 / Full deployment & integration manual**：
> [USAGE_cn.md](USAGE_cn.md)（中文）· [USAGE_en.md](USAGE_en.md)（English）
> ——Windows/Linux 两端设置、网线直连拓扑、RViz2 使用、验收清单、排查表
> （2026-10-14 全链实测沉淀）。

AC 仿真 ↔ ROS2 中继（Linux 端）。订阅 Windows 上 `ac_sim_bridge` 经 **Zenoh** 发出的数据，
按 `shared/zenoh_payload.h` 解码，发布标准 ROS2 话题，让你在 **RViz** 里看到车移动 + 周围车 3D OBB。

> 当前状态：**ego_pose / detections_3d / 雷达点云 / 相机图像全部端到端实测通过且流畅**（Humble +
> peer 直连；雷达 10Hz + 图像同传，2026-08 setter 悬案结案后实测，见下方"调试记录"）。
> 订阅 `ac/ego_pose` + `ac/detections_3d` + `ac/lidar/*/points` + `ac/camera/**` +
> `ac/semantic/classes`（M3 类表），
> 发 `/ac/ego_pose`(`PoseStamped`) + TF(`ac_world→base_link`) + `/ac/detections_3d`(`Detection3DArray`)
> + `/ac/detections_3d_markers`(`MarkerArray` 保底) + `/ac/lidar/<name>/points`(`PointCloud2`)
> + `/ac/camera/<cam>/<suffix>`(`Image`) + TF(`base_link→<name>_frame` 静态)
> + `/ac/semantic/classes`(M3，`std_msgs/String` latched)。
> **AC→ROS 坐标转换全部集中在 [frames.py](ac_ros2_bridge/frames.py)**
> （4 个实测定案事实见其 docstring，C++ 侧对应 shared/ac_coords.h）。
> **带宽**：链路实测 ≈86MB/s（iperf 687Mbps，v4/v6 均稳）；zenoh 单 TCP 共用，总量留余量为宜
> （原理性队头阻塞风险仍在，见 ac_sim_bridge README）。压缩/控制等后续里程碑。
> **ROS2 只装在这一侧**。

## 模块结构（M5 起）

```
relay.py         入口/组装：节点引导 + zenoh session + 各域接线
relay_context.py RelayContext：横切策略集中地——本地取时+ego-TF 钳制、帧精确配对、
                 发布锁（zenoh 线程安全）、动态 TF + /tf_static 全集 latch。
                 未来 /clock + use_sim_time 只改这里的取时来源。
scene_relay.py   SceneRelay：ego_pose / detections_3d / lidar 数据域；detections 的
                 类名/Marker 色查 ctx.semantic_classes（M3）
image_relay.py   CameraRelay：相机图像数据域
class_table.py   SemanticClassRelay（M3）：语义类词汇表域——zenoh ac/semantic/classes
                 （JSON 文本）→ ctx.semantic_classes + latched /ac/semantic/classes；
                 纯逻辑半区（parse_class_table/builtin_classes）不 import ROS，
                 test_class_table.py 任意 OS 可离线跑
control_relay.py ControlRelay：控制下行（ROS→zenoh，首个反向模块）+ 状态反馈上行；
                 v5 起含专家操作上行（zenoh ac/expert_ctrl → /ac/expert_ctrl）；
                 顶部注释含 uint8[] setter 禁用等编码铁律（调试记录教训）
wire.py          纯负载编码/解码（+ payload_bytes），test_decode.py 守护；
                 v5 起含 ac/expert_ctrl 解码（44B：每场景帧的专家操作动作标签，
                 frameId 与 ego_pose 同源对齐——模仿学习数据）
frames.py        AC→ROS 坐标转换（全部定案事实见其 docstring）
```

**新增一种数据类型 = 写一个新的域模块**（构造接 `ctx`、`attach(session)` 声明订阅、自己发自己的
话题），在 relay.py 接线一行——横切策略零复制。

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
- **zenoh-python 1.x**：`pip install zenoh`（实际包名 `eclipse-zenoh`，pip 装）。
- **python-lz4**：`pip install lz4`（depth/semantic 压缩帧解压；非 ROS 包，pip 装）。
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
   export ZENOH_CONFIG=$HOME/relay.json5
   ros2 run ac_ros2_bridge relay
   ```
   ⚠️ **固定用 export 这一种写法**（2026-10-03 排障定案）：bash 里
   `ZENOH_CONFIG=~/relay.json5` **单独一行**是不导出的赋值，下一行的
   `ros2 run` 子进程拿不到；`VAR=x cmd` 单行前缀虽有效但容易被拆成两行误用
   （Windows cmd 的 `set` 是窗口级持久，习惯不能平移）。配置没加载时 relay
   以 zenoh 默认 **client 模式静默空跑**——不监听 7447（`listen` 字段的默认值
   是幌子，client 模式不用它），bridge 连不进、无任何数据、无任何报错。
   预期日志：`[INFO] [ac_relay]: ac_relay ready: subscribing ac/ego_pose + ac/detections_3d + ac/lidar/*/points + ac/camera/** + ac/vehicle_state + ac/expert_ctrl + ac/semantic/classes; forwarding /ac/vehicle_cmd -> zenoh ...`；收到相机数据后还会打 `image publisher '/ac/camera/<cam>/image_raw' (WxH rgba8)`。

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
- Add → **MarkerArray**，话题 `/ac/detections_3d_markers`（内置显示，OBB 按**类色**半透明，
  M3 起与语义图同色；UNKNOWN 保持绿色）。
- 或 Add → **Detection3DArray**，话题 `/ac/detections_3d`（需装 `vision-msgs-rviz-plugins`）。

开车 → 车移动 + 周围车 3D OBB 跟随 = 全链路（AC → gamehook 插件 → SHM → ac_sim_bridge → Zenoh → relay → ROS2 → RViz）打通。

## 控制下行（M4）：ROS2 开车

`/ac/vehicle_cmd`（`ac_msgs/VehicleControl`，**RELIABLE QoS、定频流式 ≥20Hz**）→ relay 打包
32B `ControlPayload` → zenoh `ac/vehicle_cmd` → Windows bridge 按其 `bridge_control.ini`
的 `MODE=shm|vjoy` 注入游戏（ROS 侧对后端无感知，切换方式只改 Windows 端 ini）。
反向：zenoh `ac/vehicle_state` → `/ac/vehicle_state`（`ac_msgs/VehicleState`：挡位/车速/GO）；
zenoh `ac/expert_ctrl` → `/ac/expert_ctrl`（`ac_msgs/ExpertCtrl`：每场景帧的专家操作动作
标签——模仿学习 (观测,动作) 配对，frame_id 与 /ac/ego_pose 一致；override=true 表示
自动驾驶接管值，训练时过滤）。
**安全语义**：停止发布 ≈0.5s（bridge `RX_TIMEOUT_MS`）自动弃权回人工驾驶。

```bash
# 依赖消息包：工作区根 colcon build --packages-select ac_msgs ac_ros2_bridge
# 开车（先按既有流程起 Windows bridge + AC；正赛会自动开赛，GO 后车才动）：
# ⚠️ 必须显式写 requested_gear_index！不写 = 默认 0 = R 挡（倒车，实测踩坑）。
#    要保持当前挡位发 -100，要起步发 2（1挡）。
ros2 topic pub -r 50 /ac/vehicle_cmd ac_msgs/VehicleControl \
  "{override_enabled: true, throttle: 0.2, steering: 0.0, requested_gear_index: 2}"
# 换挡（0=R 1=N 2..8=1..7；-100=不改）：
ros2 topic pub -r 50 /ac/vehicle_cmd ac_msgs/VehicleControl \
  "{override_enabled: true, throttle: 0.3, requested_gear_index: 3}"
# 反馈：
ros2 topic echo /ac/vehicle_state          # gear / speed(m/s) / flags(bit0=GO)
ros2 topic hz /ac/vehicle_state            # ~10Hz
# 专家操作（模仿学习动作标签，与 /ac/ego_pose 同 frame_id 配对）：
ros2 topic echo /ac/expert_ctrl            # steer/throttle/brake/... /frame_id
ros2 topic hz /ac/expert_ctrl              # ~场景帧率（30-40Hz）
```

挡位/按钮语义、shm 与 vJoy 的 steering 语义差异、单 TCP 队头阻塞余量纪律：
见 `ac_msgs/README.md` 与 `ac_sim_bridge/README.md`。联测验收项之一 =
**重载（图像/雷达全开）下控制延迟观测**。

**⚠ 测试前注意（AC 输入方式互斥，2026-08-18 实测）**：AC 输入方式为「方向盘」（vJoy）
时键盘完全无效，反之亦然。bridge `MODE=shm`（推荐）时 AC 输入方式应切回「键盘」——
弃权后键盘直接接管；`MODE=vjoy` 时 AC 须保持 vJoy 方向盘绑定，弃权后键盘开不了车。
切 MODE 时记得同步切 AC 输入方式。

### 一键全命令回归（scenario_test）

`scenario_test.py` 是 M4 全链路的脚本化回归（ROS2 → relay → zenoh → bridge → 游戏），
覆盖**所有控制字段**：等 GO → 1挡起步 → 2挡 → 相对降/升挡 → 左/右转向 → 刹车 →
离合 → 手刹 → DRS/KERS → override 关/开语义 → 弃权，结束打印阶段×预期×反馈挡位
时间线小结表。Windows 侧另有 zenoh 直发的姊妹版（`ac_sim_bridge/tools/
vehicle_cmd_scenario.py`，跳过 ROS 测 bridge 单段时用）。

```bash
ros2 run ac_ros2_bridge scenario                                  # 正赛：等 GO 自动开始
ros2 run ac_ros2_bridge scenario --ros-args -p skip_go:=true      # 练习赛
ros2 run ac_ros2_bridge scenario --ros-args -p throttle_scale:=0.5  # 油门刹车减半
```

### Ackermann 输入（M5，留作模型使用者自实现）

`/ac/vehicle_cmd` 是**底层执行器接口**（归一化油门/刹车/转向，无目标速度概念），
规划/跟踪算法栈通常输出 `ackermann_msgs/AckermannDriveStamped`（`speed` m/s +
`steering_angle` rad）。二者之间缺一层「速度 → 油门/刹车」闭环转换——**这层由具体
使用自动驾驶模型的人按需实现**（不同模型的输出语义、车辆动力学、控制策略差异很大，
不放进 relay 主链）。

接入指引（想写一个自己的 converter 节点时照此做）：
- **输入**：订阅 `/ac/ackermann_cmd`（`ackermann_msgs/AckermannDriveStamped`）；
- **反馈**：订阅 `/ac/vehicle_state`（`ac_msgs/VehicleState`）拿当前车速做闭环；
- **转换**（P 控制起步即可）：`e = target_speed - current_speed`，
  `e>0 → throttle = clamp(speed_kp·e)`，`e<0 → brake = clamp(brake_kp·|e|)`；
  `steering = steer_sign · angle / max_steer_angle`（夹到 [-1,1]）；
- **发布**：`/ac/vehicle_cmd`（`ac_msgs/VehicleControl`，RELIABLE、定频 ≥20Hz，
  **持续流式发送**——停发 ≈0.5s 自动弃权回人工）；
- 挡位可发一次 `gear_up` 起步、或直接 `requested_gear_index`（0=R 1=N 2..8=1..7）；
- **注意**：`VehicleState.speed` 单位 **m/s**（实测定案），而 Ackermann 的
  `speed` 也是 m/s，闭环可直接相减无需换算；仅 P 控制有超调/稳态误差，
  进阶可换 PID/纯跟踪模型（`pure_pursuit`/`mpc`）。

### 5. 排错速查

| 症状 | 查什么 |
|---|---|
| relay 起来了但 topic 无数据 | ① Windows 窗口里 `echo %ZENOH_CONFIG%` 确认生效；② `LINUX_IP` 写对没；③ Linux 防火墙 7447；④ 两边配置 `mode` 都须 `"peer"` |
| relay 无任何数据且 Linux `ss -tlnp \| grep 7447` 无监听 | **ZENOH_CONFIG 未生效**：分两行赋值不导出（bash 陷阱）→ relay 以 zenoh 默认 client 模式静默空跑不监听。用 `export ZENOH_CONFIG=$HOME/relay.json5` 绝对路径重启（见"按顺序启动"第 3 步说明） |
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

## 相机图像（M2；2026-10-14 压缩 wire 适配）

relay 订阅 `ac/camera/**`。**M2 起三模态全部为压缩帧**（游戏内编码、桥透传；
32B 头 `<QQIIHHI` 不变），Linux 端按下表中继：

| zenoh suffix | wire（enc/mode） | ROS 话题 | 消息/处理 |
|---|---|---|---|
| `image_raw` | 5 / 1 = 整段 JPEG（nvJPEG q90、4:2:0） | `/ac/camera/<cam>/image_raw/compressed` | `sensor_msgs/CompressedImage`（format="jpeg"）**零解码透传** |
| `image_depth` | 6 / 1 = LZ4 × 米制 u16 LE | `/ac/camera/<cam>/image_depth` | LZ4 解压 → **`Image` 32FC1 米制**（米 = u16/327.675；65535 精确解码为 200.0m = 天空/满量程） |
| `image_classes` | 3 / 1 = LZ4 × u8 类别 ID | `/ac/camera/<cam>/image_classes` | LZ4 解压 → **`Image` mono8**（像素 = 类别 ID；类名/颜色查 `/ac/semantic/classes`，数组序即 ID；ID 超出词表按 UNKNOWN 处理） |
| `camera_info` | 52B CameraInfoPayload | `/ac/camera/<cam>/camera_info` | `sensor_msgs/CameraInfo`（K 由 fovDeg 算 Pinhole，TRANSIENT_LOCAL latched） |

解码细节见 [wire.py](ac_ros2_bridge/wire.py)（`decompress_lz4` fail-loud：解压结果
必须精确等于 w×h×2 / w×h）。旧 MODE_RAW 裸像素帧（enc 0-3）仍按原路中继（兼容）。
**QoS**：Image/CompressedImage = Reliable depth 5（RViz2 默认 RELIABLE 订阅，
BEST_EFFORT 发布会被 DDS 拒绝配对）；camera_info = Reliable + TRANSIENT_LOCAL
（迟到订阅者即刻拿到）。`frame_id = <cam>_frame`（相机 TF 属后续阶段；Image 显示
不查 TF，不影响显示）。

前置（Windows 侧）：AC 带 gamehook_ac + `custom_cameras.ini`；ac_sim_bridge 用
`streamer.ini`（per-camera 分段，见其 README；M5 前名 bridge_images.ini）决定
转发哪些相机/模态/频率。
**带宽更正（2026-08）**：早前实测的"拥塞崩溃曲线"（20Hz→实收 0.6Hz 等）测于 relay 的
uint8[] setter bug 时期（见"调试记录"），瓶颈是接收端 Python 逐元素循环而非链路——iperf 实证
链路 ≈86MB/s。setter 修复后高频图像值得重测真实上限；zenoh 单 TCP 队头阻塞的原理性风险仍在，
联测仍建议按需配置相机/频率（改 `streamer.ini` 各 `[CAMERA_*]` 的 HZ；20261010.17
起桥不读任何环境变量，原 `AC_BRIDGE_IMAGE_HZ` 快速覆盖已退役）。

### 验证

```bash
ros2 topic list                      # 应有 image_raw/compressed、image_depth、image_classes、camera_info
ros2 topic hz /ac/camera/front/image_depth          # ≈配置的 HZ
ros2 topic echo /ac/camera/front/image_depth --field encoding --once   # 32FC1
ros2 topic echo /ac/camera/front/camera_info --field k --once          # 9 个内参
```

RViz2：Fixed Frame `ac_world` → Add → **Image** → 话题 `/ac/camera/front/image_depth`
（32FC1 自动归一化）或 `image_classes`（mono8 ID 图）。**color 是 CompressedImage**，
RViz2 的 Image 显示不直接认——两种看法：
```bash
# 法一：image_transport 解压转发成 Image（最常用；Humble 实测 in:= 简写会
# 把订阅落到字面量 /in/compressed 上，必须用全名重映射 + 独立节点名）
ros2 run image_transport republish compressed raw --ros-args \
  -r /in/compressed:=/ac/camera/front/image_raw/compressed \
  -r /out:=/ac/camera/front/image_raw -r __node:=republish_front
# 法二：RViz2 Image 显示里直接选 compressed 传输的话题（Best Effort 注意 QoS）
```
**通道序检查**：游戏里红色的车在解压后的 Image 显示里也应是红色（JPEG 为 RGB 编码）。

## 语义类词汇表（M3，2026-10-03）

M3 起类表用户可编辑（Windows 侧 `configs\classes.json`：自定义类、数组顺序=类 ID、颜色
`#RRGGBB`），且检测的 `classId` 从写死 1 改为透传节点真实 `semanticClass`。本侧（Linux）
靠 zenoh 拿到同一张表来解释这些 ID 和语义图颜色——**否则自定义类在 ROS 侧不可辨识**。

**链路**：ac_sim_bridge（读 classes.json，1Hz mtime 热重载）→ zenoh `ac/semantic/classes`
（载荷 = `semcls::tableToJson` 规范 JSON 文本，唯一一个非二进制 key；启动 + 表变化 +
每 2s 保活重发）→ 本侧 `class_table.py`（`SemanticClassRelay`）→
`ctx.semantic_classes`（整对象热换）+ latched 话题 `/ac/semantic/classes`
（`std_msgs/String` 装原始 JSON，TRANSIENT_LOCAL——晚加入的节点立即拿到最新表）。

**消费点**：
- `/ac/detections_3d` 的 `ObjectHypothesis.id`/`class_id` = **类名字符串**（"CAR"/"CONE"；
  ⚠️ **接口变更**：2026-10-03 前是裸数字字符串如 "1"，下游若有按数字解析的脚本需同步改）；
- `/ac/detections_3d_markers` 颜色 = 类色（与语义图像素同源；UNKNOWN 类色是白色，
  浅背景下不可见，沿用旧绿色）。

**回退铁律**（与 Windows 五端一致，见 `shared/semantic_class_config.h`）：首个有效样本前用
内置 11 类兜底（出厂表的内置前缀与之间 ID 对齐）；任何非法样本（坏 JSON / 首项非 UNKNOWN /
重名 / 超 64 类 / 坏色）**保留最后成功的表**继续工作 + WARN，不闪变。类 ID 重排的过渡窗口
（表已换、SHM 残留旧 ID）越界回退 "UNKNOWN"，1-2s 自愈。改 classes.json 后约 1-2s 全链热生效
（bridge mtime 轮询 + 本侧查表即查即用）。

### 验证

```bash
ros2 topic echo /ac/semantic/classes --once   # JSON 文本（12 类含 CONE，颜色 #RRGGBB）
ros2 topic echo /ac/detections_3d --field detections[0].results[0]   # 类名而非数字
```
热重载：改 Windows 侧 classes.json（如改 CONE 颜色/重命名）→ 1-2s 内上方两处随之变化，
Marker/语义图颜色同步。离线自测（任意 OS）：`python ac_ros2_bridge/test_class_table.py`。

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
6. **⚠️ sensor_msgs 的 uint8[] 字段（data）在大载荷下必须直写私有槽位**：Humble 生成代码的
   setter **没有 ndarray 快速通道**——numpy 数组直接被断言拒绝（ndarray 不是
   `collections.abc.Sequence`）；而 bytes/list 输入触发**逐元素 Python 断言循环**（1.85MB 雷达
   消息 ≈ 每条 1-3 秒），三线程同时陷入争 GIL，zenoh 回调被拖死 → 停读 TCP → 两端流量周期性
   归零（"relay 时不时停住"、RViz 只有 3Hz 的真凶；一度误判为 WiFi 拥塞，iperf v4/v6 实测均稳
   才翻案）。**正解：`pc._data = np.frombuffer(raw, dtype=np.uint8)`**（`_data` 就是 setter
   内部写的 `__slots__` 字段；rclpy 的 C 序列化器原生消费 ndarray，零拷贝）。升级 ROS 发行版
   时需复核。定位手段：停住时 `sudo py-spy dump --pid <真身pid>`（注意 `ros2 run` 的壳进程
   不是真身，看 `ps -fp` 区分）。
7. **带宽结论（更正）**：早前"过载即崩溃、预算 8-20MB/s"的结论被推翻——那是 setter bug 时期的
   污染数据（见"调试记录"）。链路实测 ≈86MB/s（iperf v4/v6 均稳）；"慢"**先怀疑本进程
   （py-spy），再怀疑网络**。zenoh 单 TCP 队头阻塞的原理性风险仍在，总量留余量为宜。

## 调试记录：relay 周期性停住——uint8[] setter 逐元素循环（2026-08，已结案）

### 现象

- 雷达点云在 RViz2 只有 ~3Hz（配置 10Hz，78.5K 点/扫 × 24B ≈ 1.85MB/条）；
- relay 进程**时不时停住**：两端（台式机/笔记本）网卡流量**同时**归零到几百 B/s，间歇又突发
  17.5MB/s（网卡 3 秒采样看到的是突发，10 秒采样只有 4.6MB/s——停住期占多数）；
- 更早的"消息收得很慢"、图像"20Hz 配置实收 0.6Hz"等历史症状同源。

### 诊断历程

| 阶段 | 怀疑原因 | 验证手段 | 结果 |
|------|---------|---------|------|
| 1 | WiFi 带宽不够/拥塞崩溃 | bridge 日志 `lidarPts=702K/s` 属实；对账 78.5K点/扫 → 发送端确实 ~10Hz | 发送端正常，继续查 |
| 2 | 笔记本 WiFi 省电 | `iw set power_save off` 后复测 | 仍停 ✗ |
| 3 | RViz 消费慢反压 DDS | 关 RViz 只跑 relay | 仍停 ✗ |
| 4 | IPv6 路径问题（zenoh 实际走 v6 反向连入） | `iperf3 -c`（v4）与 `iperf3 -6`（v6）各 60s | **687Mbps / v6 均稳**，链路洗清 ✗ |
| 5 | **relay 进程内部** | 停住时 `sudo py-spy dump`（注意 `ros2 run` 的壳进程≠真身，`ps -fp` 区分） | **三个线程全部卡在 `_point_cloud2.py` 的 `data` setter ✓** |

### 根因

Humble 生成的 `uint8[] data` setter：**没有 ndarray 快速通道**（numpy 数组被断言直接拒绝——
ndarray 不是 `collections.abc.Sequence`）；而 **bytes/list 输入触发逐元素 Python 断言循环**
（`all(isinstance(v,int) ... for v in value)` × 185 万元素 ≈ **每条雷达消息 1-3 秒**）。三线程
（ego 回调/lidar 回调/配对定时器）同时陷入循环争 GIL → zenoh 回调线程被拖死 → 停止从 TCP 读
数据 → TCP 窗口关死 → **两端流量同时归零**；某条循环跑完 → 突发 → 再陷 → 周期性"停住"。

### 修复（含一次弯路）

- v1：`pc.data = np.frombuffer(...)` → 运行时被 Humble 断言拒绝（`AssertionError: ... must be
  a set or sequence ...`）；
- **v2（正解）**：直写私有槽位 `pc._data = np.frombuffer(raw, dtype=np.uint8)`——`_data` 就是
  setter 内部落盘的 `__slots__` 字段，跳过全部校验；rclpy 的 C 序列化器原生消费 ndarray
  （buffer 协议零拷贝）。雷达/图像两处同改，**实测点云与图像全流畅**。升级 ROS 发行版需复核。

### 经验教训

1. **应用层"看似网络慢"，先 `py-spy dump` 看线程卡在哪，再怀疑网线**——本次五个嫌疑四个是网络侧，
   全部被排除；
2. **两台机同时看网卡计数 + 区分突发/稳态**（短窗采样会把积压清空误读成稳态吞吐）；
3. **iperf 要分 v4/v6 各测**（zenoh 实际走的族可能和直觉不同）；
4. `ros2 run` 会先起一个壳进程，py-spy/pgrep 抓错 PID 会得到"一切正常"的假象；
5. 大载荷 `uint8[]` 字段：**直写 `msg._data = np.frombuffer(...)`**，绝不走 setter。

## 工具（tools/，无 ROS 依赖）

- **zenoh_rate_probe.py**：zenoh 速率探针——按 key 统计真实送达速率与到达
  间隔，区分"链路丢帧"与"消费端处理跟不上"（对照 relay 的 ROS 发布率定责）。
  ```bash
  python3 tools/zenoh_rate_probe.py                      # ac/camera/** 10s（连本机 relay）
  python3 tools/zenoh_rate_probe.py ac/ego_pose -s 30    # 多 key 定窗
  python3 tools/zenoh_rate_probe.py --listen tcp/0.0.0.0:7447   # peer 模式等桥直连
  ```

## 后续计划

- ~~**图像压缩接收**~~：**已实现（2026-10-14）**——color CompressedImage 透传 /
  depth LZ4→32FC1 米制 / semantic LZ4→mono8 ID / camera_info 内参（见"相机图像"节）。
- **相机 TF**：`base_link→<cam>_frame` 静态 TF（内参已随 camera_info 发布）。
- **/clock + use_sim_time**：relay 发布 /clock，届时只替换 RelayContext 的取时来源。
- **统一帧号源**：gamehook_ac/optix 把场景帧号写进 image/lidar SHM → 严格帧对齐（当前
  ego↔dets 严格、ego↔lidar ≈1-2 帧、图像独立，见"帧对齐现状"）。
- ~~**控制下行**~~：**已实现（M4）**——`/ac/vehicle_cmd`（VehicleControl）驾驶 + 
  `/ac/game_cmd`（GameControl）会话控制（start/pause/resume/restart/skip）下行，
  `/ac/vehicle_state` + `/ac/game_state` 反馈上行，见"控制下行（M4）"节。
- **Ackermann 输入**：**留作模型使用者自实现**（见"控制下行"节的接入指引），不放进 relay。
