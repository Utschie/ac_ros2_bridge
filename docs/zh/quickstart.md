# 快速上手：从安装包到录出第一段数据

本指南带你在一台全新机器上从零走到"录好一段数据并通过体检"。每一步
都附对应主题详细文档的链接。

## 1. 前提条件

| 要求 | 说明 |
|---|---|
| Assetto Corsa（Steam 版） | 可从 Steam 启动；推荐用 Content Manager 启动（下载 <https://acstuff.ru>）。注意：Content Manager 中不要安装 Custom Shader Patch（CSP）——本软件与 CSP 不兼容。 |
| Windows 10 / 11 x64 | — |
| NVIDIA 显卡（支持 CUDA 13.x） | 显卡需支持 CUDA 13.x。 |
| 约 2 GB 空闲磁盘 | 另加你的 dump 录制所需空间。 |

游戏内设置一次（选项 → 视频）：

- **HDR：开**
- **运动模糊（Motion blur）：关**

`NATIVE_HDR` 管线的相机缺一不可用，缺了会直接禁用并报错。其余功能
不受影响。

> 安装器会自动完成所需的游戏内设置，并静默安装 VC++ 运行库——无需手工处理。

### 坐标系约定（全套统一）

全套软件统一使用机器人标准约定（REP-103）：**X 前、Y 左、Z 上**；世界系
x/y 水平、**Z 为高度**。相机/雷达的安装位姿、录制的 dump 数据、ROS 话题
与坐标变换、RViz 显示全部遵循同一约定——你在配置里怎么写，各处看到的
就是什么。

## 2. 安装

1. 运行 `ACSuiteSetup-<版本号>.exe`。Assetto Corsa 正在运行的话，
   安装器会要求先退出游戏。
2. 选择组件：
   - **Core**（必装）——游戏内插件、数据桥、启动器、图像/遥测查看器、
     配置、文档、dump 工具；
   - **3D viewer** ——实时点云可视化（可选）；
   - **Lidar renderer** ——雷达仿真（可选；需 NVIDIA 显卡）；
   - **Semantic annotator** ——赛道标注工具（可选；需 NVIDIA 显卡）。
3. 安装器经 Steam 自动定位你的 Assetto Corsa 文件夹。如果安装了其它画面增强类
   mod（画面滤镜/补丁类），可能与本软件冲突——见
   [troubleshooting.md](troubleshooting.md)。

桌面与开始菜单会出现 "AC Simulator Suite" 快捷方式。

## 3. 激活

启动启动器（桌面快捷方式）。已安装但还没有有效许可证时，全部控件
置灰、顶部显示 **[Activate]**。

输入你拿到的产品密钥 → 套件解锁并显示 `License OK (<到期日>)`。
详见 [activation.md](activation.md)。

## 4. 第一场

1. **启动 Assetto Corsa**，进入任意赛道 session。
2. 启动器点 **Start Bridge**。进入 session 后
   *Game plugin* 行显示 `scene OK (frame N)`，*Bridge* 行显示
   `running`。
3. 可选，边开边看：
   - **View Images** ——相机画面网格（彩色/深度/语义）；
   - **3D View** ——点云 + 真值框（需要先 **Start Lidar** 开雷达渲染器）；
   - **View Telemetry** ——MoTeC 风格仪表盘与通道绘图。
   每个查看器各有专属文档（见索引 [README.md](README.md)）。
4. 录数据：**先**勾选 **Record dump data** **再**点 Start Bridge
   （复选框实际编辑 `recorder.ini`，该文件在 bridge 启动时读取），
   然后跑一圈。
5. 点 **Stop Bridge**——退出前排空录制队列并写会话 manifest（这是
   结束录制的正规方式）。
6. 点 **Open Dump**，体检最新会话：

```bat
cd /d <AC 安装目录>
python ac_simulator_suite\tools\inspect_dump.py dump\<最新会话目录>
```

体检报告主要是各通道计数与提示，有告警时按 [troubleshooting.md](troubleshooting.md) 的建议处理即可。
想直接看数据：

```bat
python ac_simulator_suite\tools\view_image.py dump\<会话>\runs\epoch_1 --cam front --modality color
python ac_simulator_suite\tools\view_lidar.py dump\<会话>\runs\epoch_1
```

工具需要 Python 的 `numpy`、`open3d`、`opencv-python`
（`pip install numpy open3d opencv-python`）。

## 5. 接下来看什么

| 我想… | 读 |
|---|---|
| 搞懂启动器每个按钮和状态面板 | [launcher.md](launcher.md) |
| 用按钮 / zenoh / ROS2 控制比赛（开赛/暂停/重开） | [game-control.md](game-control.md) |
| 改相机 / 加相机 | [configs/custom_cameras.md](configs/custom_cameras.md) |
| 改雷达 | [configs/custom_lidars.md](configs/custom_lidars.md) |
| 调录制（格式、频率、录什么） | [configs/recorder.md](configs/recorder.md) |
| 调实时转发（转发哪些相机、多快） | [configs/streamer.md](configs/streamer.md) |
| 查看实时相机画面 | [viewer-images.md](viewer-images.md) |
| 查看 3D 点云与真值框 | [viewer-3d.md](viewer-3d.md) |
| 查看遥测仪表盘 | [viewer-telemetry.md](viewer-telemetry.md) |
| 把数据实时流到 Linux / ROS2 机器 | [ros2-relay.md](ros2-relay.md) |
| 语义标注与场景编辑 | [annotator.md](annotator.md) |
| 弄懂 dump 文件与坐标约定 | [configs/recorder.md](configs/recorder.md) |
| 有东西不工作 | [troubleshooting.md](troubleshooting.md) |

完整索引：[README.md](README.md)。English docs: `docs\en\`。
