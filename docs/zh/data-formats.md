# 数据格式参考（数组顺序 · 坐标系 · 字段语义）

落盘文件与 ROS 话题中所有"按位置取值"的数据（数组、矩阵、枚举）在本文
一次讲清。总约定见 [quickstart.md](quickstart.md)：**X 前 / Y 左 / Z 上；
世界系 Z 为高度**。

## 1. 坐标系：AC 原生 → 我们的输出

游戏内部有两套原生坐标系。**桥在输出前已全部转换**，落盘与 ROS 话题
数据一致，无需再做任何换算：

| 数据 | AC 原生 | 我们的输出 |
|---|---|---|
| 世界坐标：`velocity`、`carCoordinates`、`tyreContact*` | 物理世界系（y=高度，x/z 水平） | 统一 Z-up 世界系 |
| 车体分量：`accG`、`localVelocity`、`localAngularVel` | 车体系（x=左、y=上、z=前） | 统一车体系（x=前、y=左、z=上；加速为正、左向力为正、左转偏航为正） |
| 雷达点（每点 x,y,z） | 传感器本地系（与统一约定相同） | 原样 |
| ego 矩阵、检测框、相机矩阵、点云世界位姿 | — | 直接按统一约定生成 |

角度量不做转换，读法如下：

- `heading`：表盘式顺时针（0=前、+90°=右）；统一约定的方位角 = `-heading`；
- `windDirection`：同上读法；`steerAngle`：正 = 右打；
- `pitch` / `roll`：游戏原生体轴欧拉角；角运动分析建议直接用
  `localAngularVel`。

## 2. 轮位数组：第 0..3 位 = FL、FR、RL、RR

所有 `[4]` 轮胎数组按**前左、前右、后左、后右**排列（遥测查看器界面的
[FL][FR][RL][RR] 标签即此约定）：物理遥测的 `wheelSlip` `wheelLoad`
`wheelsPressure` `wheelAngularSpeed` `tyreWear` `tyreDirtyLevel`
`tyreCoreTemperature` `camberRAD` `suspensionTravel` `brakeTemp`
`tyreTempI/M/O`，增量块（`incValid=1` 有效）的 `inc_mz`
`inc_tyreLoadedRadius` `inc_slipAngle` `inc_slipRatio`，以及静态快照的
`suspensionMaxTravel` `tyreRadius`。

`[12]` 数组（`tyreContactPoint/Normal/Heading`）是**分量在外层、轮位在
内层**：前 4 个 = x 的四轮、中 4 个 = y 的四轮、后 4 个 = z 的四轮。

三个非轮序特例：`carDamage[5]` = 前、后、左、右、全体；`rideHeight[2]`
= 前、后；`inc_caster[2]` = 前轴 左、右。

## 3. 矩阵（16 浮点，行主序 4×4）

| 字段 | 行含义 |
|---|---|
| `ego_pose.jsonl` 的 `world` | 行 0/1/2 = 车体**前 / 左 / 上**三轴；行 3 = 位置 |
| `cameras.jsonl` 的 `view` | 行 0/1/2 = 相机**右 / 上 / 后**三轴；行 3 = 相机位置 |

## 4. 检测框（`detections_3d/<帧号>.json`）

`pos[3]` = 中心（统一世界系）；`half[3]` = 半尺寸，顺序**（前，左，上）**；
`rot[9]` = 3×3 旋转（行主序，行序 前 / 左 / 上）；`classId` = 类别编号
（对应语义类表顺序）；对象标识字段跨帧同值（轨迹关联用）。图像侧
`index.jsonl` 的 2D 框为像素坐标（左上原点）。

## 5. 雷达点云（`lidar/<雷达名>/<扫描号>.pcd`）

每点 5 字段：`x` `y` `z` `intensity`（float）+ `instance_id`（整数，与
检测框同源的对象标识，未命中为 0）。点在**传感器本地系**；文件头注释
带该扫描的安装位姿（车体系）与世界位姿（统一世界系）。一帧扫描与配对
的相机/真值帧可能相差 1~2 帧。ROS 话题上的点云每点多一个 `range` 字段。

## 6. 状态与事件

- **`vehicle_state`**：`gear` 0=倒挡 1=空挡 2..8=1..7 挡；`flags` 最低位
  =已发车、次位=发车灯亮。
- **`game_state`**：`sessionType` 1..7。
- **`run_state`**（事件流）：`event` = `run_begin`/`run_end`/`flag`/
  `barrier`；run_end 的 `reason` 0=赛后菜单 1=重开 2=按键切换 3=元数据
  失效 4=场景失活；flag 的 `flagField` 0=比赛结束 1=会话结束 2=赛后菜单。
- **`expert_ctrl`**：每帧专家操作，帧号与真值严格对齐。

## 7. 落盘文件 ↔ ROS 话题对照

| 数据 | 落盘 | ROS 话题 | 频率 |
|---|---|---|---|
| 自车位姿 | `ego_pose.jsonl` | `/ac/ego_pose` | 每场景帧 |
| 3D 检测框 | `detections_3d/` | `/ac/detections_3d` | 每场景帧 |
| 相机图像 | `cameras/<相机>/<模态>/` | `/ac/camera/<相机>/image_raw` 等 | 按相机配置 HZ |
| 相机内参 | `cameras.jsonl`（每帧全量） | `/ac/camera/<相机>/camera_info`（变化自动更新） | — |
| 遥测（物理） | `telemetry.jsonl` | `/ac/telemetry` | 每场景帧 |
| 圈速/比赛状态 | `timing.jsonl` | `/ac/timing` | 每场景帧 |
| 静态快照 | `telemetry_static.jsonl` | `/ac/telemetry_static`（latched） | 会话变化时 |
| run 事件 | `run_state.jsonl` | `/ac/run_state` | 事件驱动 |
| 专家操作 | `expert_ctrl.jsonl` | `/ac/expert_ctrl` | 每场景帧 |
| 车辆状态 | `vehicle_state.jsonl` | `/ac/vehicle_state` | 10 Hz |
| 比赛状态 | `game_state.jsonl` | `/ac/game_state` | 10 Hz |
| 雷达点云 | `lidar/<名>/*.pcd` | `/ac/lidar/<名>/points` | 每扫描 |
| 驾驶段元数据 | `spans.jsonl` | —（仅落盘） | 每驾驶段 |
| 写盘报告/索引 | `manifest.json`、`index.jsonl` | —（仅落盘） | — |

## 8. 帧号（sceneFrameId）

所有按帧数据共用同一全局递增帧号（每个 run 内从 1 重新计数）。跨通道
对齐以它为锚点。
