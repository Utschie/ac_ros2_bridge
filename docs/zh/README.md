# AC Simulator Suite — 文档（中文）

第一次用？从 [快速上手](quickstart.md) 开始——它带你从安装包一路走到
录好并体检通过的第一段数据。

## 阅读顺序

| # | 文档 | 内容 |
|---|---|---|
| 1 | [quickstart.md](quickstart.md) | 前提、安装、激活、第一场、第一段 dump |
| 2 | [activation.md](activation.md) | 产品密钥、激活、离线宽限、换密钥 |
| 3 | [launcher.md](launcher.md) | 启动器每个按钮、状态面板、更新、launcher.ini |
| 4 | [game-control.md](game-control.md) | 控制比赛：按钮 / zenoh / ROS2 三种方式 |

## 配置参考（`configs\`）

| 文档 | 对应文件 |
|---|---|
| [configs/custom_cameras.md](configs/custom_cameras.md) | custom_cameras.ini——相机、渲染管线、效果 |
| [configs/custom_lidars.md](configs/custom_lidars.md) | custom_lidars.ini——雷达（固态与旋转）、噪声 |
| [configs/capture_engine.md](configs/capture_engine.md) | capture_engine.ini——游戏内采集引擎 |
| [configs/streamer.md](configs/streamer.md) | streamer.ini——经 zenoh 转发什么 |
| [configs/recorder.md](configs/recorder.md) | recorder.ini——dump 录制 |
| [data-formats.md](data-formats.md) | 数据格式——数组顺序/坐标系/字段语义（落盘与 ROS 通用） |
| [configs/control.md](configs/control.md) | control.ini——控制下行 |
| [configs/semantic_classes.md](configs/semantic_classes.md) | semantic_classes.ini 与按赛道标注 json 链 |

## 工具与查看器

| 文档 | 内容 |
|---|---|
| `tools\README.zh.md`（在 tools 目录旁） | Python 数据体检/标注/控制测试脚本 |
| [viewer-images.md](viewer-images.md) | 图像网格查看器（View Images） |
| [viewer-3d.md](viewer-3d.md) | 3D 点云查看器（3D View） |
| [viewer-telemetry.md](viewer-telemetry.md) | 遥测仪表盘（View Telemetry） |
| [annotator.md](annotator.md) | 语义标注与场景编辑（Annotator） |
| [annotation-files.md](annotation-files.md) | 语义规则文件（json 格式） |

## 进阶

| 文档 | 内容 |
|---|---|
| [ros2-relay.md](ros2-relay.md) | 流送到 Linux / ROS2（拓扑、RViz2、调优） |
| [troubleshooting.md](troubleshooting.md) | 症状 → 原因 → 处理，含 bridge 统计行解读 |

English docs live in `docs\en\` (same structure).
