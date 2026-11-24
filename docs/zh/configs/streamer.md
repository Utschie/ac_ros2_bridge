# streamer.ini — 图像转发配置

选择数据桥经 zenoh 实时转发哪些相机与模态（Linux/ROS2 端收到的就是
这些）。bridge 启动时读取一次——修改后重启 bridge。

格式：每个要转发的相机一个 `[CAMERA_<name>]` 段（名字 = custom_cameras.ini
里该相机的 NAME）；没写的相机不转发。

| 键 | 默认 | 说明 |
|---|---|---|
| `BRIDGE_COLOR` | `0` | 转发彩色图（1/0）。 |
| `BRIDGE_DEPTH` | `0` | 转发米制深度图。 |
| `BRIDGE_SEMANTIC` | `0` | 转发语义分割图。 |
| `HZ` | `10` | 转发频率（帧/秒），每相机独立。 |

无本文件时：全部相机、仅彩色、10 Hz。有文件但没启用任何相机 = 不转发。

带宽参考：默认五个相机全开约 18–23 MB/s——链路吃不满时见
[ros2-relay.md](../ros2-relay.md) 的网线直连方案。
