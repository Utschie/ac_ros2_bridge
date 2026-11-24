# 游戏控制指南：控制比赛的三种方式

AC Simulator Suite 提供四个会话指令：**Start Race（开始比赛）/ Pause（暂停）/ Resume（恢复）/ Restart（重开）**。有三种等价的发送方式，按场景任选其一，效果完全相同。

**五个指令的作用**

| 指令 | 效果 |
|---|---|
| Start Race | 开始比赛 |
| Pause | 暂停比赛（声音一并停止） |
| Resume | 从暂停继续（声音恢复） |
| Restart | 重新开始 | |

> 注意：Start Race 控制的是**比赛 session**，不是游戏程序本身。

## 前提条件（三种方式共用）

1. **游戏正在运行**且已加载我们的插件（启动器状态区的 "Game plugin" 行心跳正常即已加载；游戏已进 session 却一直显示 waiting 时，重装套件即可修复）。
2. **数据桥正在运行**——用启动器的 **Start Bridge** 按钮启动。
3. 控制模式为 `shm`（默认）：`ac_simulator_suite\configs\control.ini` 中
   `[control] MODE=shm`。**MODE=vjoy 或 off 时会话指令不可用**
   （vJoy 后端只转发驾驶输入，不转发会话指令）。

---

## 方式一：启动器按钮（最简单）

1. 打开 AC Simulator Suite 启动器。
2. 点击 **Start Bridge**（只要控制游戏的话不需要 Start Lidar——雷达
   渲染器可以不开）。
3. 在游戏内进入任意 session 后，"Game Control" 按钮行变为可用：
   **Start Race / Pause(Resume) / Restart**——暂停/恢复是同一个
   切换按钮：运行中显示 Pause、暂停中显示 Resume（2026-09-29 合并）。
4. 按钮旁的标签实时显示当前 session 状态（如 `Session: Race | GO`）。

按键后约 10 毫秒指令即到达 bridge——完全不经过网络。

**按钮置灰的原因**（旁边的标签会说明）：

- `game ctrl: start bridge first`——bridge 没在运行，先点 Start Bridge；
- `game ctrl needs MODE=shm`——control.ini 的 MODE 不是 shm；
- 软件许可证未激活。

---

## 方式二：zenoh 直调（模型 / 第三方程序）

适合自动驾驶模型、脚本、以及从任意机器远程控制。无需 ROS——只要
Python 带 zenoh 库：

```
pip install eclipse-zenoh
```

### 指令格式

向 zenoh key **`ac/game_cmd`** 发布一个 **4 字节小端整数**（位掩码，
可组合）：

| 位 | 数值 | 指令 |
|---|---|---|
| bit0 | 0x01 | START（开始比赛） |
| bit1 | 0x02 | PAUSE（暂停） |
| bit2 | 0x04 | RESUME（恢复） |
| bit3 | 0x08 | RESTART（重开当前 session） |

### 单行示例（在 Windows 上、与 bridge 同网络）

```bat
python -c "import zenoh,sys; s=zenoh.open(zenoh.Config()); s.put('ac/game_cmd', int(sys.argv[1],0).to_bytes(4,'little'))" 0x01
```

把末尾参数换成 0x02 暂停、0x04 恢复、0x08 重开。

### 连接配置

bridge 以 peer 模式启动（配置在 `ac_simulator_suite\configs\bridge.json5`，
默认连接 `tcp://<中继 IP>:7447`）。远程机器上的 zenoh 客户端只要配置
相同的 endpoint 即可找到它：

```python
import zenoh
cfg = zenoh.Config()
cfg.insert_json5("connect/endpoints", '["tcp/192.168.0.79:7447"]')  # 你的中继地址
session = zenoh.open(cfg)
session.put("ac/game_cmd", (0x01).to_bytes(4, "little"))   # 开始比赛
```

### 配套状态反馈（可选）

- 订阅 `ac/game_state`（16 字节 `<QII>`：时间戳、sessionType、flags）——
  flags bit0 = GO（绿灯）、bit1 = 发车倒计时灯亮。
- 订阅 `ac/vehicle_state`（20 字节 `<QifI>`）——当前挡位/车速/GO。
- sessionType：1=练习 2=排位 3=正赛 4=Hotlap 5=TimeAttack 6=Drift 7=Drag。

### 现成工具

随包的驾驶测试脚本（在 `ac_simulator_suite\tools\`；自动开赛 → 等 GO →
把驾驶指令全部过一遍）：

```bat
python tools\vehicle_cmd_scenario.py            &:: 驾驶测试（开赛+等 GO）
python tools\vehicle_cmd_scenario.py --skip-go  &:: 练习赛：不等 GO
python tools\vehicle_cmd_test.py --sub-state    &:: 只看挡位/车速/GO 反馈
```

驾驶控制（油门/转向/挡位，`ac/vehicle_cmd`）不在本文范围——见
`ac_simulator_suite\tools\README.zh.md`。

---

## 方式三：ROS2（Linux 机器人集成）

需要在 Linux 侧部署 ROS 中继（随套件提供，部署步骤见 [ros2-relay.md](ros2-relay.md)）。

### 发送指令（话题 `/ac/game_cmd`，消息类型 `ac_msgs/GameControl`）

```bash
ros2 topic pub --once /ac/game_cmd ac_msgs/GameControl "{start_race: true}"
ros2 topic pub --once /ac/game_cmd ac_msgs/GameControl "{pause: true}"
ros2 topic pub --once /ac/game_cmd ac_msgs/GameControl "{resume: true}"
ros2 topic pub --once /ac/game_cmd ac_msgs/GameControl "{restart_session: true}"
```

（`GameControl.msg` 字段：`start_race / pause / resume / restart_session`，全 bool；每条消息是一次独立的边沿触发——再发一条即可
再次触发。）

### 查看状态

```bash
ros2 topic echo /ac/game_state    # sessionType + GO/倒计时灯（中继上行翻译）
ros2 topic echo /ac/vehicle_state # 挡位/车速/GO
```

启动中继（Linux 侧；`relay.json5` 指向 bridge 所在网络）：

```bash
ros2 run ac_ros2_bridge relay
```

---

## 指令语义与安全行为

- **边沿触发、可重复**：每条指令触发一次动作；要再来一次就再发一条。
  bridge 收到指令后将位保持 500 毫秒自动清零（重新武装），因此间隔
  0.5 秒以上的连续两次按键计为两次。
- **与驾驶接管相互独立**：会话指令与驾驶接管（`ac/vehicle_cmd`）相互
  独立。模型停止发送驾驶指令约 0.5 秒后自动交还人类控制
  （`RX_TIMEOUT_MS`）；会话指令不受该超时影响。
- **暂停期间数据流不中断**：暂停后游戏仍逐帧渲染，相机/雷达/遥测数据
  继续输出——适合"暂停 → 标注 → 继续"的数据生产流程。
- **三个通道可混用**：按钮、zenoh、ROS2 可同时使用；指令在 bridge 中
  排队合并进同一路径，互不冲突。

## 故障排查

| 症状 | 检查 |
|---|---|
| 启动器按钮置灰 | bridge 是否在运行（Start Bridge）；MODE 是否 shm；许可证是否激活 |
| 指令发了没反应 | 游戏是否在运行且已进入 session（等待画面也算）；bridge 日志有无 `game_cmd` 行（见下）。两者都正常则多半是插件未加载——重装套件即可 |
| 远程 zenoh 不通 | bridge 机器 7447 端口可达性；bridge.json5 与客户端 endpoint 是否指向同一个中继 |
| 在 bridge 日志中确认 | `ac_simulator_suite\logs\` 下最新的 bridge 日志：按钮应显示 `game_cmd (launcher) bits=0x..`；zenoh/ROS2 应显示 `game_cmd bits=0x..` |
| 按钮有效、bridge 有日志、游戏没反应 | control.ini 的 MODE（vjoy 不转发）；或版本过旧——用安装器升级 |

---
*本文档随安装包发布于 `ac_simulator_suite\docs\zh\game-control.md`
（英文版：`docs\en\game-control.md`）。*
