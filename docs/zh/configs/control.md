# control.ini — 控制下行配置

配置控制下行（驾驶指令与会话指令如何到达游戏）。bridge 启动时读取
一次——修改后重启 bridge。

| 键 | 出厂值 | 说明 |
|---|---|---|
| `MODE` | `shm` | `shm`（默认，模型/脚本控制用）、`vjoy`（虚拟设备方式）、`off`（关闭）。**会话指令（开赛/暂停等）仅在 `shm` 下可用**，见 [game-control.md](../game-control.md)。 |
| `RATE_HZ` | `100` | 控制指令写入频率（Hz）。 |
| `RX_TIMEOUT_MS` | `500` | 停止发送驾驶指令多久后自动交还人工（毫秒）。 |
| `NEUTRAL_ON_TIMEOUT` | `1` | 交还人工时是否写一次中立输入。 |
