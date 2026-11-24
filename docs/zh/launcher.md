# 启动器参考（Launcher）

启动器是套件的控制面板：启停各个数据程序、显示它们的状态、编辑主要
配置开关、转发 bridge 日志。桌面/开始菜单的 "AC Simulator Suite"
快捷方式打开的就是它。

子程序都不弹控制台窗口；bridge 的输出实时显示在启动器底部日志区。
关闭启动器会一并停掉它拉起的所有程序（录制先排空收尾）。启动器是
单实例——重复启动只会唤起已有窗口。

## 程序控制

| 控件 | 作用 |
|---|---|
| **Start / Stop Bridge** | 启动 / 停止数据桥（dump 录制、zenoh 转发、控制下行、遥测都在这里）。Stop 会先排空录制队列、写完会话 manifest 再退出——录完数据请一律这样收尾。 |
| **Start / Stop Lidar** | 启动 / 停止雷达渲染器（无界面）。只在需要雷达点云时开启。下面的选项只在启动那一刻生效。 |
| **View Images / 3D View / View Telemetry / Annotator** | 拉起四个查看器。未安装 viewer3d 组件时 3D View 置灰；Annotator 需要 annotator 组件。各自文档见索引。 |

## 启动选项（这些开关本身就是配置编辑器）

| 选项 | 编辑的配置 | 含义 |
|---|---|---|
| ☑ **Record dump data** | `recorder.ini [recorder] ENABLED` | dump 录制总开关。须在点 Start Bridge **之前**勾选——bridge 运行期间复选框为禁用状态。复选框始终反映文件当前状态。 |
| ☑ **Real lidar rate (--real-hz)** | `launcher.ini [optix] REAL_HZ` | 勾 = 雷达按配置的 `SIM_HZ` 扫描（真实传感器频率，含扫描间运动）；不勾 = 每渲染帧一扫（点云更平滑）。 |
| **GPU 下拉（雷达）** | `launcher.ini [optix] GPU` | 雷达渲染器用哪块 NVIDIA 显卡（建议选择不跑游戏的那张）。 |
| **3D viewer GPU 下拉** | `launcher.ini [viewer3d] ADAPTER` | 3D 查看器用哪块显卡（同样建议选择不跑游戏的那张）。查看器运行中不可更改——拉起时生效。 |

## Game Control 行

**Start Race / Pause(Resume) / Restart**——暂停/恢复是同一个
切换按钮；按钮旁的标签实时显示会话状态（如 `Session: Race | GO`）。
需要 bridge 运行中且 `control.ini MODE=shm`；置灰时标签会说明原因。
三种会话控制方式（按钮 / zenoh / ROS2）见
[game-control.md](game-control.md)。

## 状态区

每秒刷新。每行显示进程态（running / stopped / 退出码 / 外部启动的
pid）+ 数据心跳——用以区分"进程活着"和"数据在流"：

| 状态项 |显示 |
|---|---|
| Game plugin | 游戏在 session 中显示 `scene OK (frame N)`；否则 `waiting for game` |
| Bridge | scene/image/control 数据心跳 |
| Lidar renderer | 雷达数据心跳 + 点率 |
| Image viewer / 3D viewer / Telemetry viewer / Semantic annotator | 运行状态 |

## Open… / Edit… 按钮

- **Open Dump / Open Configs / Open Logs**——资源管理器直达
  `<AC 根>\dump`、`ac_simulator_suite\configs\`、`ac_simulator_suite\logs\`。
- **Edit recorder.ini / streamer.ini / control.ini / custom_cameras.ini /
  custom_lidars.ini / classes.json**——记事本打开对应文件。每个文件的
  作用见 [configs/](configs/) 各文档。改完按各文件注明的方式重启
  （bridge 或游戏）生效。

## 更新

- **Check Updates** 按钮（启动 3 秒后还会自动查一次），对照
  `launcher.ini [update] URL` 配置的更新服务器。
- 有新版本时顶栏显示 `Update available: <版本>` 并出现 **[Update]**
  按钮：点击后自动完成更新（组件选择保持不变），之后重开启动器即可。

## launcher.ini 键

`ac_simulator_suite\configs\launcher.ini`（随包出厂模板；GPU/频率等
选项由启动器自己写回）：

| 键 | 默认 | 含义 |
|---|---|---|
| `[update] URL` | `https://suite.dym-lab.com` | 更新服务器基址。供应商要求时才改。 |
| `[license] URL` | `https://suite.dym-lab.com/api` | 许可证服务器地址。见 [activation.md](activation.md)。 |
| `[optix] REAL_HZ` | `0` | 1 = 雷达渲染器按真实传感器频率运行。 |
| `[optix] GPU` | `1` | 雷达渲染器 / 标注工具使用的显卡编号。 |
| `[viewer3d] ADAPTER` | *(空 = 自动)* | 3D 查看器使用的显卡；留空 = 自动选不跑游戏的那张 NVIDIA 卡。 |
| `[annotator] GPU` | *(空)* | 标注工具显卡（编号同 optix）。 |
