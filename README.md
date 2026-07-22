# 空地协同智能消防车（车端）

本仓库是 2023 年全国大学生电子设计竞赛 G 题“空地协同智能消防系统”的消防车端 ROS 2 工作空间，目标板为 Orange Pi 5 Max（Ubuntu 22.04 / aarch64）。车端通过 UDP 与无人机交换启动命令、遥测、火源坐标和任务状态，使用激光雷达与 Cartographer 定位，规划绕开街区的消防路线，控制差速底盘到达照射位置并返回出发区。

当前代码已经补齐《G题_机车通信接口约定》要求的车端通信与显示功能，并在 ROS 2 Humble 环境完成构建和本机 UDP 闭环验证。场地标定、真实激光 GPIO、实车行驶和双机联调仍需在比赛硬件上完成。

> 安全提示：激光可能伤害眼睛或引发火灾。真实激光必须使用默认断电的驱动电路、物理总断电开关及 MOSFET/光耦等隔离方案，禁止用开发板 GPIO 直接驱动激光负载。

## 本次更新加入了什么

- 新增 `fire_link_bridge`：
  - 监听 UDP `8892`，接收无人机 32 字节遥测包。
  - 校验定长、`0xF14E` magic、消息类型和 16 位循环序号。
  - 丢弃重复包、乱序旧包、错误类型、异常浮点数以及过长/过短数据包。
  - 发布 `/drone_telemetry`，包含位置、里程、高度、阶段和序号。
  - 通过 `/drone_start`、`/drone_start_button` 或物理 GPIO 按键触发启动。
  - 向无人机 UDP `8893` 连发 5 个相同序号的 `CAR_START` 包。
- 新增 `fire_dashboard.py` 车载全屏显示：
  - 显示无人机实时坐标、高度、任务阶段和累计巡逻里程。
  - 按 48 dm × 40 dm 比例绘制场地、六个街区、起降区和消防车出发点。
  - 绘制无人机历史航迹、当前位置和火源标记。
  - 超过 3 秒未收到遥测时显示红色离线告警。
  - 使用任务栏、地图卡片、状态徽标和分组遥测卡片强化信息层级，并自适应窗口尺寸。
  - 内置默认场地参数，支持脱离 YAML 独立启动；异常 ROS 数据不会导致界面退出。
  - 支持空格或回车直接触发无人机启动。
- 加固已有 `fire_event_bridge`：严格接收 16 字节火源包，避免超长 UDP 包被截断后误判为合法包。
- 修复消防车到达照射位后未校验朝向的问题：车头对准火源后才允许开启激光，故障路径会立即下发 `OFF`。
- 任务状态值与机车约定保持一致；车辆忙时重发当前合法状态，不再发送未约定的 `busy`。
- 将通信桥和显示程序加入 `fire_mission.launch.py`，随消防任务统一启动。
- 将通信端口、无人机 IP、按键、显示和场地参数集中到 `fire_params.yaml`。
- 增加显示程序运行依赖和构建安装规则。

## 当前完成度

| 模块 | 状态 | 说明 |
|---|---|---|
| 8889 火源坐标接收 | 已实现并本机验证 | 16 字节定长、magic、seq 去重 |
| 8890 任务状态回传 | 已实现并本机验证 | 回传给最近一次上报火源的无人机 IP |
| 8892 无人机遥测接收 | 已实现并本机验证 | 32 字节定长、类型和 seq 校验 |
| 8893 车端按键启动 | 已实现并本机验证 | 同一启动包默认连发 5 次 |
| 车载遥测与航迹显示 | 已实现并模拟渲染验证 | 已验证 1280×720、800×480，仍需在真实屏幕确认全屏效果 |
| Cartographer 定位 | 已有代码 | 需要使用实际雷达和场地验证 |
| 消防路径规划与返航 | 已有代码 | 需要精确场地数据和实车验证 |
| 差速底盘控制 | 已有代码 | 需要确认串口、控制点偏移和速度参数 |
| 激光执行器 | 已接 GPIO1_B0 | 物理 22 脚，低电平开启、高电平关闭，带 3 秒超时保护 |
| ROS 任务开机自动运行 | 未配置 | 已能一条 launch 启动，但尚未安装 systemd/桌面自启动项 |

## 系统链路

```text
车端按键或界面
  -> /drone_start
  -> fire_link_bridge
  -> UDP 8893 CAR_START ×5
  -> 无人机起飞

无人机 UDP 8892 TELEMETRY
  -> fire_link_bridge
  -> /drone_telemetry
  -> fire_dashboard（坐标、航迹、里程、高度、阶段、链路状态）

无人机 UDP 8889 FIRE_EVENT
  -> fire_event_bridge
  -> /fire_event
  -> fire_mission_manager
  -> /target_position
  -> diff_drive_controller
  -> /cmd_vel
  -> orangepi_to_carv2
  -> SR5E1E3 底盘 $VW

fire_mission_manager
  -> /laser_command
  -> laser_gpio_driver

/fire_mission_status
  -> fire_event_bridge
  -> UDP 8890
  -> 无人机
```

## 机车 UDP 接口

两块板的 `ROS_DOMAIN_ID` 可以不同，跨机数据不依赖 DDS，统一走 UDP。坐标均为场地坐标系，原点位于巡防区左下角，x 向右、y 向上，单位为 dm。

消防车与无人机接入同一台固定路由器，均使用静态 IPv4 地址；本项目不使用车端热点或 DHCP 地址学习：

| 设备 | 地址 |
|---|---|
| 消防车 | `192.168.10.113/24` |
| 无人机 | `192.168.10.171/24` |

| 端口 | 方向 | 内容 | 格式 |
|---|---|---|---|
| `8889` | 机 → 车 | 火源坐标 | 16 字节，小端，magic `0xFC11` |
| `8890` | 车 → 机 | 任务状态字符串 | 原始 UTF-8/ASCII 字节 |
| `8892` | 机 → 车 | 位置、里程、高度、阶段 | 32 字节，小端，magic `0xF14E`，type `1` |
| `8893` | 车 → 机 | 按键启动 | 32 字节，小端，magic `0xF14E`，type `3` |

任务状态只使用：

```text
ready / enroute / extinguishing / returning / done / failed:<reason>
```

完整字节布局、阶段枚举、时序和联合验收方式见 [G题_机车通信接口约定.md](G题_机车通信接口约定.md)。

## ROS 2 接口

| 话题 | 类型 | 方向/用途 |
|---|---|---|
| `/drone_start` | `std_msgs/Empty` | 触发一次无人机启动包连发 |
| `/drone_start_button` | `std_msgs/Bool` | 外部按键节点输入，仅稳定按下沿触发 |
| `/drone_telemetry` | `std_msgs/Float32MultiArray` | `[x_dm, y_dm, distance_dm, height_dm, phase, seq]` |
| `/fire_event` | `std_msgs/Float32MultiArray` | `[x_dm, y_dm, seq]` |
| `/fire_mission_status` | `std_msgs/String` | 消防车任务状态，同时回传无人机 |
| `/target_position` | `std_msgs/Float32MultiArray` | `[x_cm, y_cm, mode, yaw_deg]` |
| `/cmd_vel` | `geometry_msgs/Twist` | 差速底盘线速度和角速度 |
| `/laser_command` | `std_msgs/String` | `ON` / `OFF` |
| `/laser_status` | `std_msgs/String` | `ready` / `on` / `off` / `timeout_off` |

## 包结构

| 包/目录 | 作用 |
|---|---|
| `src/follower_pkg` | 通信桥、任务状态机、路径规划、差速控制、显示器和激光抽象 |
| `src/car_carto_pkg` | URDF、雷达启动和 Cartographer 配置 |
| `src/orangepi_to_car` | 将 `/cmd_vel` 转换为 SR5E1E3 `$VW` 串口帧 |
| `src/car_launch` | 消防任务与底盘定位测试的联合启动入口 |
| `src/bluesea2` | 蓝海光电雷达 ROS 2 驱动 |
| `src/scripts/chassis_diag.py` | 不经过 ROS 的底盘串口诊断工具 |

## 环境与构建

已验证的软件环境为 ARM64、Ubuntu 22.04、ROS 2 Humble。请在目标板原生构建，不要复制其他机器生成的 `build/` 或 `install/`。

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep \
  ros-humble-cartographer-ros ros-humble-robot-state-publisher \
  python3-numpy python3-opencv python3-pil fonts-noto-cjk

sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 必须至少列出一种中文字体，否则仪表盘会 WARN 且中文可能显示为方块
fc-list :lang=zh | head
```

只重建本次主要修改的包：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select follower_pkg car_launch --symlink-install
```

## 运行前必须配置

主要参数文件是 [`src/follower_pkg/config/fire_params.yaml`](src/follower_pkg/config/fire_params.yaml)。上车前至少检查以下项目：

1. `fire_link_bridge.drone_ip`：固定为无人机地址 `192.168.10.171`，两端更换地址时必须同步修改。
2. `telemetry_udp_port/start_udp_port`：默认分别为 `8892/8893`，两端必须一致且不能被占用。
3. `button_gpio_value_path`：物理按键尚未指定时保持空字符串；指定后需确保该 GPIO 已配置为输入。
4. `arena_origin_map_x_m/y_m/yaw_deg`：标定场地左下角在 Cartographer `map` 中的位置和朝向。
5. `obstacles_dm`：当前六个街区边界来自赛题图估算，必须按真实场地重新测量。
6. `home_x_dm/home_y_dm`：确认消防车红色出发区中心。
7. `aim_tol_deg`：默认 `8°`；车辆只有在照射位内且车头朝向火源误差不超过此值时才开启激光。
8. `laser_gpio_driver`：已配置为物理 22 脚 `GPIO1_B0`（Linux GPIO 40、wPi 13），实测高电平关闭，因此使用 `active_low: true`。
9. 雷达串口、波特率和型号：修改 `src/bluesea2/src/bluesea-ros2/params/uart_lidar.yaml`。
10. 底盘串口：Orange Pi 5 Max 默认 `/dev/ttyS6`，对应 40-pin 的物理 11/13 脚。

## 固定路由、防火墙和联调

在路由器中为消防车和无人机设置静态地址或 DHCP 地址保留，避免与其他设备冲突。消防车启动后确认地址和路由：

```bash
ip -4 address
ip route
ping -c 3 192.168.10.171
```

车端必须看到 `192.168.10.113/24`，并能 ping 通无人机 `192.168.10.171`。项目不会创建热点，也不会自动修改 NetworkManager 配置。

检查 UDP 监听和防火墙：

```bash
ss -lunp | grep -E ':(8889|8892)\b'
sudo nft list ruleset
sudo ufw status verbose
sudo tcpdump -ni any 'host 192.168.10.171 and (udp port 8889 or udp port 8892)'
```

若 UFW 已启用，放行车端两个入站端口：

```bash
sudo ufw allow proto udp from 192.168.10.171 to 192.168.10.113 port 8889
sudo ufw allow proto udp from 192.168.10.171 to 192.168.10.113 port 8892
```

从同一路由器内的另一台机器发送合法数据包验证接收链路：

```bash
python3 -c "import socket,struct;s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.sendto(struct.pack('<HHfff',0xFC11,1,12.0,20.0,0.0),('192.168.10.113',8889))"
python3 -c "import socket,struct,time;s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.sendto(struct.pack('<HBBHHIffffI',0xF14E,1,2,1,0,int(time.monotonic()*1000)&0xffffffff,12.0,20.0,30.0,18.0,0),('192.168.10.113',8892))"
```

车端 `tcpdump` 应看到两包，`ros2 topic echo --once /fire_event` 和 `ros2 topic echo --once /drone_telemetry` 应分别收到数据。无人机向 `192.168.10.113:8889/8892` 发送，并监听 8890/8893；车端启动包始终发送到配置的 `192.168.10.171:8893`。

## Orange Pi 5 Max 物理启动按键

推荐使用当前未被 UART6 占用的物理 16 脚：`GPIO1_A3`、Linux GPIO 35、wPi 9。按键一端接物理 16 脚，另一端接 GND（例如物理 14 脚）；GPIO 只能承受 3.3 V，禁止接 5 V。物理 11/13 脚是底盘 UART6，不要拿来接按键。

关机接线后开机，先把引脚设为带上拉的输入，再导出兼容 sysfs 路径：

```bash
sudo gpio mode 9 in
sudo gpio mode 9 up
test -d /sys/class/gpio/gpio35 || echo 35 | sudo tee /sys/class/gpio/export
echo in | sudo tee /sys/class/gpio/gpio35/direction
sudo chmod a+r /sys/class/gpio/gpio35/value
cat /sys/class/gpio/gpio35/value
```

松开应读到 `1`，按下应读到 `0`。然后在 `fire_params.yaml` 设置：

```yaml
button_gpio_value_path: "/sys/class/gpio/gpio35/value"
button_active_low: true
button_debounce_ms: 50
```

若改成“按下接 3.3 V、外部下拉”的接法，按下读 `1`，此时设 `button_active_low: false`。sysfs 导出、方向、上拉和读权限在重启后要重新设置，应把上述初始化做成 root 的 systemd oneshot，并让 ROS 任务服务 `After=` 它；先手工确认电平正确，再配置自启。[Orange Pi 5 Max 官方手册](https://orangepi.net/wp-content/uploads/2024/09/OrangePi_5_Max_RK3588_User-Manual_vv1.2-1.pdf)的 40-pin 表和 wiringOP 章节是引脚编号的基准，不要套用树莓派或其他 Orange Pi 型号的编号。

## 启动与调试

正式任务：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch car_launch fire_mission.launch.py
```

只验证激光硬件时，不启动雷达、底盘、网络和任务节点。下面命令会将激光开启
2 秒后自动关闭，launch 继续运行以保持 PB0 为关闭电平；看到自动关闭后按
`Ctrl+C` 退出：

```bash
ros2 launch car_launch laser_test.launch.py

# 可修改点亮时间，例如 1 秒
ros2 launch car_launch laser_test.launch.py duration:=1.0
```

> 激光接口为低电平有效。测试前移开人员视线并确认光路安全；正常退出、收到
> `OFF` 命令或达到时限时，驱动都会把 PB0 拉回高电平。

覆盖底盘串口：

```bash
ros2 launch car_launch fire_mission.launch.py \
  chassis_port:=/dev/ttyUSB0 chassis_baud:=115200
```

没有物理按键时发送一次启动命令：

```bash
ros2 topic pub --once /drone_start std_msgs/msg/Empty '{}'
```

查看关键数据：

```bash
ros2 topic echo /drone_telemetry
ros2 topic echo /fire_event
ros2 topic echo /fire_mission_status
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map laser_link
```

仅验证雷达、定位、航点控制和底盘链路：

```bash
ros2 launch car_launch lidar_test.launch.py
```

## 不连接无人机的遥测自测

先启动正式任务或单独启动 `fire_link_bridge`，再模拟无人机向车端发送 1 Hz 遥测：

```bash
python3 -c "
import socket, struct, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for i in range(100):
    packet = struct.pack(
        '<HBBHHIffffI', 0xF14E, 1, 2, i, 0,
        int(time.monotonic() * 1000) & 0xffffffff,
        10.0 + i * 0.3, 4.0, i * 3.0, 18.0, 0)
    s.sendto(packet, ('127.0.0.1', 8892))
    time.sleep(1)
"
```

显示器应看到无人机沿 `y=4 dm` 向右移动、里程持续增加；停止发送 3 秒后链路状态应变为离线。

## `stamp_ms` 的 ROS 接口处理

UDP 32 字节包中的 `stamp_ms` 保留不动，确保无人机端协议完全兼容；车端不再把它放入 `Float32MultiArray`。原因是 uint32 毫秒值转换为 float32 后只能保留约 7 位有效数字，无法可靠测量链路延迟。链路超时仍只使用车端本地收包时间。

## 已完成的软件验证

- `follower_pkg` 与 `car_launch` 在 ROS 2 Humble 下构建通过。
- `CAR_START` 的固定目标地址、32 字节小端布局和同序号 5 次冗余发送验证通过。
- 遥测包的定长校验、类型校验、序号去重和 `65535 → 0` 回绕验证通过。
- 火源包的 16 字节定长校验、序号去重和状态字符串回传验证通过。
- 超长数据包以及包含 `NaN/Inf` 的遥测/火源包不会被误接收，非法包不会占用序号。
- 显示器已在 headless 模式验证默认参数、异常数据防护、坐标、航迹、火源和状态渲染。
- 任务状态机已用模拟 TF 验证：到达照射位但未对准时不会启光，对准后才发布 `ON`。
- Python 语法、flake8、PEP 257、CMake lint、XML lint 和全部 C++ `uncrustify` 检查通过。

## 还有什么需要做

### 比赛前必须完成

- [ ] 确认消防车为 `192.168.10.113/24`、无人机为 `192.168.10.171/24`，且同一路由器内能互相收发 UDP。
- [ ] 确认无人机端已经同步使用 `8889/8890/8892/8893` 和本文档的两种定长包格式。
- [ ] 两端使用同一个物理场地左下角联合标定，并用已知格点互相核对坐标。
- [ ] 实测并更新六个街区、起降区和消防车出发区参数。
- [ ] 确认底盘 UART、雷达设备名和系统权限，完成断流自动停车测试。
- [ ] 在实物上验证激光 PB0 的上电默认 OFF、超时 OFF、退出 OFF 和故障 OFF。
- [ ] 接入物理启动按键并验证有效电平和 50 ms 去抖。
- [ ] 在车载屏幕上检查全屏分辨率、中文字体、字号和无遮挡显示。
- [ ] 配置系统开机自启，同时确保桌面会话或显示服务已就绪。
- [ ] 放行 UDP `8889`、`8892` 入站以及 `8890`、`8893` 出站，并进行丢包联调。
- [ ] 实车低速验证路径、到点容差、安全余量和返航，确保车轮不压街区边界。

### 建议继续完善

- [ ] 给 `done` 和 `failed:*` 状态增加 3 次冗余回传。
- [ ] 将通信数组话题替换为带字段名的自定义 ROS 消息，方便后续维护。
- [ ] 增加可重复运行的自动化协议测试和 CI。
- [ ] 清理上游雷达包的许可证/TODO 元数据。
- [ ] 若火源附近四个候选照射位均不可达，扩展更多候选角度和距离。

## 文档

- [G题_机车通信接口约定.md](G题_机车通信接口约定.md)：机车唯一 UDP 通信契约。
- [fire_mission_design.md](fire_mission_design.md)：消防任务规划、坐标与状态机设计。
- [SR5E1E3_CHASSIS_DEBUG_GUIDE.md](SR5E1E3_CHASSIS_DEBUG_GUIDE.md)：底盘协议、串口接线和诊断说明。
- [G题_空地协同智能消防系统.pdf](G题_空地协同智能消防系统.pdf)：赛题原文。

## 许可证

本工作区自研 ROS 2 包当前声明为 Apache-2.0。`bluesea2` 等第三方或上游代码请以其各自文件和原厂许可为准；公开分发前应再次核对相关许可信息。
