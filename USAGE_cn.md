# ac_ros2_bridge 部署与联测手册

从零把 Windows 采集端和 Linux ROS2 消费端连通的完整步骤说明书（含实测踩坑）。
数据流：**Windows（游戏 + ac_sim_bridge 桥）→ zenoh TCP → Linux relay → ROS2 话题 / RViz2**。
桥是主动连接方（connect 到 relay 的 7447 监听），所以只需保证"桥能连到 relay 的 IP:7447"。

2026-10-14 全链实测基线：五相机三模态全量 ~112 帧/s（约 17MB/s）；
WiFi 链路只能送达 ~8%，**网线直连千兆实测满额**，relay 仅占单核 ~6%。

---

## 1. 拓扑选型

| 形态 | 适用 | 说明 |
|---|---|---|
| A. 同一路由器下（WiFi/有线均可） | 低速率联测、功能验证 | 桥 endpoint 填 Linux 机的局域网 IP。**WiFi 只够 ~2Hz×5 相机量级**，全量流会丢 90%+（zenoh DROP 拥塞策略，表现为帧 gap 秒级） |
| B. 网线直连专段（推荐） | 全量吞吐联测 | Windows 网口 ↔ Linux USB/内置网口，专段 `192.168.50.0/24`，两边各自静态 IP、不设网关；Windows 上网走 WiFi 不受影响。见 §4 |

---

## 2. Linux 端设置

### 2.1 系统与 ROS2

Ubuntu 22.04（ROS2 Humble 原生目标）+ ROS2 Humble desktop：

```bash
# ROS2 Humble 官方安装（apt 源密钥用 ros-apt-source 包分发；老 ros.key URL 已失效）
sudo apt install -y software-properties-common curl
# 若 GitHub 直连不畅：在能通的机器下载 deb 后拷入安装
#   https://github.com/ros-infrastructure/ros-apt-source/releases → ros2-apt-source_<ver>.jammy_all.deb
sudo dpkg -i ros2-apt-source_*.deb
sudo apt update && sudo apt install -y ros-humble-desktop ros-dev-tools
```

### 2.2 包依赖

```bash
pip3 install eclipse-zenoh lz4          # 注意：zenoh 1.x 的 pip 包名是 eclipse-zenoh
# pypi 直连不畅时用镜像：pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple eclipse-zenoh lz4
sudo apt install -y ros-humble-vision-msgs                 # Detection3DArray
sudo apt install -y ros-humble-image-transport-plugins     # RViz2 看彩色图（JPEG 解码）
```

numpy 走系统包（`python3-numpy`），无需另装。

### 2.3 取代码与构建

```bash
mkdir -p ~/colcon_ws/src && cd ~/colcon_ws/src
git clone git@github.com:<你的账号>/ac_msgs.git
git clone git@github.com:<你的账号>/ac_ros2_bridge.git     # 私有仓需先配 GitHub ssh key
source /opt/ros/humble/setup.bash
cd ~/colcon_ws && colcon build --symlink-install
source install/setup.bash
```

**踩坑**：工作空间以前用非 symlink 模式构建过的话，`ament_cmake_python` 的 symlink 步骤会因旧目录报
"existing path cannot be removed: Is a directory"——`rm -rf build install log` 后全新构建即可。

### 2.4 启动 relay

```bash
# 关键：relay 只认 ZENOH_CONFIG 环境变量。不设它 relay 照常启动打 "ready"
# 但走 zenoh 默认值、不监听 7447，桥永远连不上（症状：relay 正常、话题全空）。
ZENOH_CONFIG=~/colcon_ws/src/ac_ros2_bridge/relay.json5 \
  setsid nohup ros2 run ac_ros2_bridge relay > ~/relay.log 2>&1 &
```

验证：`ss -tln | grep 7447` 应见 `0.0.0.0:7447` LISTEN；日志出现 `ac_relay ready`。
重启后需重新执行（想开机自启可配 systemd 用户服务）。

---

## 3. Windows 端设置

1. 装好 AC 套件（gamehook_ac 引擎 + ac_sim_bridge 桥），`streamer.ini` 配置转发哪些相机/模态/频率；
2. 编辑桥配置 `ac_simulator_suite\configs\bridge.json5`（**注意：桥读的是安装目录这份，不是仓库副本**）：
   ```
   { mode: "peer", connect: { endpoints: ["tcp/<Linux 机 IP>:7447"] } }
   ```
3. 桥只在启动时读该文件——**改完必须从 launcher 重启 bridge**；
4. 游戏进赛道后桥自动发布（bridge 日志 `img fwd: fwd=... cams=...` 可看发送率）。

---

## 4. 网线直连形态（全量吞吐推荐）

物理：Windows 网口 ↔ Linux 机网口（现代网卡免交叉线；Linux USB 千兆网卡如 RTL8153 免驱）。

配置（专段 192.168.50.0/24，**双方都不设网关**——上网各走各的，互不干扰）：

```bash
# Linux（假设网卡名 enx3c18a0d56850，按实际改）：
sudo ip addr add 192.168.50.2/24 dev enx3c18a0d56850 && sudo ip link set enx3c18a0d56850 up
```

```powershell
# Windows（管理员 PowerShell；<ifIndex> 用 Get-NetAdapter 查，如 14）：
Get-NetIPAddress -AddressFamily IPv4 -InterfaceIndex <ifIndex> -ErrorAction SilentlyContinue |
  Where-Object IPAddress -like '192.168.0.*' | Remove-NetIPAddress -Confirm:$false
New-NetIPAddress -InterfaceIndex <ifIndex> -IPAddress 192.168.50.1 -PrefixLength 24
```

然后 `bridge.json5` endpoint 改 `tcp/192.168.50.2:7447` 并重启桥。

**验证**：`ping 192.168.50.2`（Windows→Linux 应 0% 丢、<1ms）。反向 ping 不通是 Windows 防火墙拦
入站 ICMP，无害——桥是出站连接方。

**测完还原 Windows 有线上网**（静态 IP 会跟着网卡走，插回路由器也拿不到 DHCP——症状就是"有线无法上网"）：

```powershell
Get-NetIPAddress -AddressFamily IPv4 -InterfaceIndex <ifIndex> -ErrorAction SilentlyContinue |
  Where-Object IPAddress -like '192.168.50.*' | Remove-NetIPAddress -Confirm:$false
Set-NetIPInterface -InterfaceIndex <ifIndex> -Dhcp Enabled
Set-DnsClientServerAddress -InterfaceIndex <ifIndex> -ResetServerAddresses
```

（Linux 侧的 .50.2 留着无害；`sudo ip addr del 192.168.50.2/24 dev <网卡>` 可清。）

---

## 5. RViz2 使用

| 图 | 话题 | 类型 | 看法 |
|---|---|---|---|
| 深度 | `/ac/camera/<cam>/image_depth` | Image 32FC1 米制 | RViz2 Image 显示直接选，自动归一化 |
| 语义 | `/ac/camera/<cam>/image_classes` | Image mono8（像素=类别 ID） | 同上；类名查 `/ac/semantic/classes` |
| **彩色** | `/ac/camera/<cam>/image_raw/compressed` | **CompressedImage**（JPEG 透传） | **RViz2 的 Image 显示不认此类型**（话题选择器按类型过滤，`ros2 topic list` 能见但选不到）——需先 republish 解码： |

```bash
# 彩色图解码转发（Humble 实测：in:= 简写会把订阅落到字面量 /in/compressed 上，
# 必须用全名重映射 + 独立节点名）：
source /opt/ros/humble/setup.bash && source ~/colcon_ws/install/setup.bash
setsid nohup ros2 run image_transport republish compressed raw --ros-args \
  -r /in/compressed:=/ac/camera/front/image_raw/compressed \
  -r /out:=/ac/camera/front/image_raw -r __node:=republish_front \
  > ~/republish.log 2>&1 &
```

之后 RViz2 里选 `/ac/camera/front/image_raw`（sensor_msgs/Image）即可。每个要看的相机起一路。

内参：`/ac/camera/<cam>/camera_info`（sensor_msgs/CameraInfo，K 由 fov 算 Pinhole，TRANSIENT_LOCAL
latched——RViz 订晚也能立刻拿到）。

---

## 6. 验收清单

```bash
source /opt/ros/humble/setup.bash && source ~/colcon_ws/install/setup.bash
ros2 topic list | grep camera          # 每相机应有 image_raw/compressed(+depth+classes)+camera_info
ros2 topic hz /ac/camera/front/image_depth     # ≈ streamer.ini 该相机 HZ
ros2 topic echo /ac/camera/front/camera_info --field k --once   # 9 个内参
# 送达率诊断（区分链路丢 vs 消费端慢）：
python3 ~/colcon_ws/src/ac_ros2_bridge/tools/zenoh_rate_probe.py --seconds 12
```

健康基线（千兆直连实测）：探针逐 key 速率=各相机配额、gap 最大 ~100ms；relay CPU ~6%。

---

## 7. 排查表

| 症状 | 原因与处理 |
|---|---|
| relay "ready" 但话题全空 | relay 没带 `ZENOH_CONFIG` 启动 → 没监听 7447（§2.4）；或桥连错 IP/没重启 |
| 桥日志满发、Linux 探针速率远低于配额、gap 秒级 | 链路带宽不够（WiFi 典型）→ 换网线直连（§4） |
| RViz2 找不到彩色图话题 | CompressedImage 类型被 RViz2 选择器滤掉 → republish（§5） |
| republish 起了但没图 | 用了 `in:=` 简写（订阅悬空在 /in/compressed）→ 全名重映射（§5） |
| camera_info 话题缺失 | 桥只在环连接瞬间一次性发布，撞上图像洪水可能被丢 → 重启桥（relay 保持运行时大概率收到）；长期方案为周期重发（挂账） |
| RViz 报 incompatible QoS 无图 | 图像发布器为 RELIABLE；RViz2 默认 RELIABLE 订阅兼容，报错多为其他发布器混入 |
| Linux 构建报 "existing path cannot be removed: Is a directory" | 旧构建产物撞 symlink 模式 → `rm -rf build install log` 重建（§2.3） |
| Windows 有线插回路由器无法上网 | 直连静态 IP 残留 → 还原 DHCP（§4 末尾命令块） |

---

## 8. 附：WSL2 备用测试台

无实体 Linux 机时的本机形态：Windows 起 WSL2 Ubuntu-22.04 跑 relay，桥连
`tcp/localhost:7447`（WSL2 NAT 的 localhost 自动转发覆盖 host→WSL 方向，零网络配置）。
功能验证等价；但 **relay 与满载游戏同机抢 CPU**，解压模态吞吐测不准（实测 depth 只剩
1-5Hz，同负载下实体机 94-100%）——吞吐结论以实体机为准。
