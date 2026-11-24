# 故障排查

一切"不工作"的第一站。所有套件程序都把带时间戳的日志写进
`ac_simulator_suite\logs\`——出问题程序的最新日志永远是第一现场。
启动器状态区告诉你哪些程序活着、数据是否在流。

## 安装与启动

| 症状 | 原因与处理 |
|---|---|
| 安装器警告 NVIDIA 驱动 | 显卡需支持 CUDA 13.x；驱动过旧可能导致崩溃——到 nvidia.com 更新驱动。 |
| 安装器拒绝继续："Assetto Corsa 正在运行" | 完全退出游戏（回到桌面）后重跑安装器。 |
| 相机画面全黑，或游戏启动时某相机"报错禁用" | 相机要求**游戏内 HDR 开、运动模糊关**（选项 → 视频）。设置后重启游戏。 |
| 启动器状态一直 `waiting for game` | 确认已进入赛道 session；仍 waiting 则说明插件未加载——重装套件即可。若安装了其它画面增强类 mod（滤镜/补丁），可能与本软件冲突，需移除。 |
| 某套件程序弹窗"配置文件缺失"后退出 | `configs\` 目录不完整——重装套件。配置是严格解析的，没有静默回退。 |
| 控件全灰，"License required" | 先激活——见 [activation.md](activation.md)。 |

## 录制

| 症状 | 原因与处理 |
|---|---|
| 完全没有 dump | 点 Start Bridge **之前**勾了 **Record dump data** 吗？（`recorder.ini ENABLED` 在 bridge 启动时读取。）游戏在 session 里吗（状态 `scene OK`）？ |
| bridge 日志里丢帧计数持续增长 | 写盘跟不上——按下方"录制持续丢帧时"的顺序处理。 |
| `inspect_dump.py` 打印 `[warn]` 行 | 看行内容——它写明确切的不一致项（计数 vs manifest、帧空洞等）。图像目录里**文件号跳跃是正常的**、不是丢帧（文件名是游戏发布计数器，录制按 IMAGE_HZ 采样）。 |
| 某模态文件比另一模态多/少几个 | 文件号 ±1 属正常发布窗口容差；跨模态配对看 `index.jsonl` 里的 `sceneFrameId`，那是一一精确的。 |

## 录制持续丢帧时

bridge 每秒在启动器日志区打印一行统计，其中 `rec[...]` 里的 `d` 是丢帧
计数。若持续大于 0，按顺序尝试：

1. 彩色格式改 `jpeg`（`recorder.ini` 的 `IMAGE_COLOR_FORMAT`）；
2. 深度格式用 `png` 或 `inferno`（别用 `raw`）；
3. 调低 `IMAGE_HZ`，或减少录制的相机/模态数量；
4. 把 `ROOT_DIR` 指到更快的磁盘（固态盘）；
5. 加大 `WRITER_THREADS`。

仍不改善时，收集启动器状态区截图与 `ac_simulator_suite\logs\` 里最新
的日志联系供应商。

## 流送到 Linux / ROS2

| 症状 | 原因与处理 |
|---|---|
| 中继看着 ready 但话题全空 | 中继没带 `ZENOH_CONFIG` 启动（没监听 7447）、bridge.json5 指错 IP、或改完没重启 bridge。完整清单：[ros2-relay.md](ros2-relay.md)。 |
| bridge 满速发送、Linux 实收远低、有数秒空洞 | 链路带宽不足（WiFi 典型——只够承载零头）。改用 [ros2-relay.md](ros2-relay.md) 的千兆直连方案。 |
| RViz2 里选不了彩色图像话题 | 它是 CompressedImage，RViz2 的话题选择器按类型过滤——先跑 republish（见 [ros2-relay.md](ros2-relay.md)）。 |

## 查看器

| 症状 | 原因与处理 |
|---|---|
| 图像查看器提示"请先启动游戏" | 先开游戏进 session；查看器是只读消费端。 |
| 3D 查看器只有一个空网格 | 没有雷达数据：雷达渲染器开了吗（**Start Lidar**）、游戏进 session 了吗？首帧数据到达时相机会自动取景。 |
| 遥测查看器一直显示等待提示 | 游戏遥测开关未开——安装器会自动设置；若挪动过游戏目录，重装套件即可。 |
| 标注工具窗口灰色/空白 | 还没进赛道——进去后约 0.5 秒内自动重连；或显卡未指定/指定错误——在启动器选定正确显卡后重开。 |

## 游戏控制

见 [game-control.md](game-control.md) 的故障排查表（按钮置灰、指令
不到、该看哪些日志行）。

## 许可证与更新

| 症状 | 原因与处理 |
|---|---|
| 激活失败 | 复制粘贴密钥；检查到许可证服务器的网络；确切 HTTP 错误在 `logs\license_dbg.log`。密钥已绑其它机器会被拒——找供应商解绑。见 [activation.md](activation.md)。 |
| 更新检查显示错误行 | 更新服务器不可达；不影响套件其它功能。稍后再试或手动安装。 |

## 还是解决不了？

按顺序收集：(1) 启动器状态区截图，(2) `ac_simulator_suite\logs\*.log` 里
最新的几个日志，(3) 出问题前你做了什么，(4) 若与录制数据有关，
`python tools\inspect_dump.py <会话>` 的输出——然后联系供应商。
