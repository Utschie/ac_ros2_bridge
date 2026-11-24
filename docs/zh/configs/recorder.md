# recorder.ini — 录制（dump）配置

配置 dump 落盘录制：真值 + 状态 + 雷达 + 相机图像 + 专家驾驶操作。

ini 在 bridge 启动时读取一次——修改后重启 bridge 生效。

## 会话目录布局

每场比赛一个 `runs/epoch_N` 子目录；帧号（sceneFrameId）在每个 run 内从 1 重新计数。

联机（在线服务器）与单机的切分规则等价，一局一个 run：练习/排位/正赛
各自独立分段，服务器换会话（练习重开、进入下一场正赛等）即切新段；
正赛中冲过终点线后的滑行与画面渐暗仍属于该段，切回维修区画面的瞬间
关段；各段之间（等待/投票/维修区）的帧不落盘。中途加入服务器时该段
从加入时刻开始。

```
<ROOT_DIR>/<时间戳>/manifest.json        + 写盘剖析报告
<ROOT_DIR>/<时间戳>/spans.jsonl          每驾驶段一行
                                         （起止 + 赛道/车/车数）
<ROOT_DIR>/<时间戳>/vehicle_state.jsonl  会话级
<ROOT_DIR>/<时间戳>/game_state.jsonl     会话级
<ROOT_DIR>/<时间戳>/run_state.jsonl      run 事件（会话级）
<ROOT_DIR>/<时间戳>/telemetry_static.jsonl  车辆/赛道静态快照（会话级）
<ROOT_DIR>/<时间戳>/runs/epoch_N/        第 N 场：
    ego_pose.jsonl  detections_3d/  cameras.jsonl  expert_ctrl.jsonl
    telemetry.jsonl  timing.jsonl
    lidar/<雷达名>/  cameras/<相机>/<模态>/   （图像 + 带 2D 框的 index.jsonl）
```

检查工具（随包在 `tools\`）：

```bat
python tools\inspect_dump.py <会话目录>
python tools\view_image.py   <会话目录>\runs\epoch_1 --cam front
```

## 坐标约定

落盘数据使用全套统一约定（X 前 / Y 左 / Z 上；世界系 Z 为高度——见
[quickstart.md](../quickstart.md)）。数组顺序、矩阵行列、各文件字段
语义与坐标特例（遥测的世界坐标字段是游戏物理系原始值）详见
[data-formats.md](../data-formats.md)。

## [recorder] 键

| 键 | 出厂值 | 怎么改 | 说明 |
|---|---|---|---|
| `ENABLED` | `0` | 启动器勾选 | 总开关。用启动器的 **Record dump data** 复选框控制（须在启动 bridge 前勾选）。 |
| `ROOT_DIR` | `..\..\dump` | 可手动编辑 | 会话目录的父目录（`..\..` = AC 安装根）。磁盘跟不上时可改为绝对路径，指到更快的盘。 |
| `LIDAR` | `1` | 一般无需改动 | 雷达扫描落盘（`lidar/<name>/<scan#>.pcd`）。 |
| `TELEMETRY` | `1` | 一般无需改动 | 遥测通道：实时发布与落盘同开关。 |
| `EXPERT_CTRL` | `1` | 一般无需改动 | 专家驾驶操作录制（模仿学习用），按帧与真值严格对齐；每段连续驾驶自动分为一个 run。 |
| `IMAGES` | `1` | 一般无需改动 | 相机图像落盘。 |
| `IMAGE_CAMS` | `all` | 可手动编辑 | 落盘哪些相机（`all` 或逗号清单）。 |
| `IMAGE_MODALITIES` | `all` | 可手动编辑 | 落盘哪些模态（`all` 或逗号清单）。 |

> ⚠ `ROOT_DIR` 的 `=` 后只写路径，不要追加任何注释。

## 按相机覆盖

可选的 `[CAMERA_<name>]` 段只控制该相机的录制，会覆盖上面的全局默认配置（转发不受影响）：

- `MODALITIES` — 落盘哪些模态（缺省 `all`）；
- `HZ` — 该相机的录制频率（缺省回落全局 `IMAGE_HZ`）；
- `FORMAT_COLOR` / `FORMAT_SEMANTIC` / `FORMAT_DEPTH` — 见下。

## 录制频率

| 键 | 出厂值 | 说明 |
|---|---|---|
| `IMAGE_HZ` | `10` | 每相机每秒最多录多少帧（支持小数；可按相机覆盖）。 |

## 图像落盘格式

优先级：按相机 `FORMAT_*` > 全局显式键 > `IMAGE_FORMAT` 简写。

| 键 | 出厂值 | 说明 |
|---|---|---|
| `IMAGE_FORMAT` | `png` | 彩色与语义的默认格式简写。 |
| `IMAGE_COLOR_FORMAT` | `jpeg` | 彩色：`png` / `jpeg` / `raw`。 |
| `IMAGE_SEMANTIC_FORMAT` | `png` | 语义：建议 `png`（像素即类别 ID，无损且人可看）。 |
| `IMAGE_DEPTH_FORMAT` | `png` | 深度：`png` = 16 位米制（训练用）；`raw` = 原始值；`inferno` / `rainbow` = 彩色可视化。 |

## 写盘调优

| 键 | 出厂值 | 说明 |
|---|---|---|
| `WRITER_THREADS` | `6` | 写盘线程池大小（通道内保序与线程数无关）。 |
| `QUEUE_MB` | `1024` | 队列预算（MB）。占满即丢帧并计数——转发路径绝不反压。 |

丢帧持续增长时的处理顺序见 [troubleshooting.md](../troubleshooting.md)。
