# 空地协同智能消防车（车端）

本仓库是 2023 年全国大学生电子设计竞赛 G 题“空地协同智能消防系统”的消防车端 ROS 2 工作空间，目标板为 Orange Pi 5 Max（Ubuntu 22.04 / aarch64）。车端通过 UDP 与无人机交换启动命令、遥测、火源坐标和任务状态，使用激光雷达与 Cartographer 定位，规划绕开街区的消防路线，控制差速底盘到达照射位置并返回出发区。

当前代码已经补齐《G题_机车通信接口约定》要求的车端通信与显示功能，并在 ROS 2 Humble 环境完成构建和本机 UDP 闭环验证。场地标定、物理启动按键、触屏、实车行驶和双机联调仍需在比赛硬件上完成。

> 安全提示：激光可能伤害眼睛或引发火灾。真实激光必须使用默认断电的驱动电路、物理总断电开关及 MOSFET/光耦等隔离方案，禁止用开发板 GPIO 直接驱动激光负载。

## 本次更新加入了什么

- 新增车载相机火源微调：
  - 固定停车点粗定位完成后，通过 OpenCV 检测红色打印目标或红色 LED。
  - 火源偏上/下时低速前进/后退，偏左/右时通过 yaw 微调。
  - 连续 8 帧稳定在可标定瞄准点附近后才允许开启激光。
  - 目标丢失时在 `±15°` 内搜索；相机断流、位移超限和障碍风险都会停车失败。
  - 差速控制器统一仲裁导航与视觉命令，视觉命令断流后保持锁定零速。
- 新增 `fire_link_bridge`：
  - 监听 UDP `8892`，接收无人机 32 字节遥测包。
  - 校验定长、`0xF14E` magic、消息类型和 16 位循环序号。
  - 丢弃重复包、乱序旧包、错误类型、异常浮点数以及过长/过短数据包。
  - 连续 3 秒无有效包后复位序号过滤器，允许无人机重启后从 `seq=0` 恢复。
  - 发布 `/drone_telemetry`，包含位置、里程、高度、阶段和序号。
  - 通过 `/drone_start`、`/drone_start_button` 或物理 GPIO 按键触发启动。
  - 向无人机 UDP `8893` 连发 5 个相同序号的 `CAR_START` 包。
- 新增 `fire_dashboard.py` 车载全屏显示：
  - 显示无人机实时坐标、高度、任务阶段和累计巡逻里程。
  - 后台持续接收带瞄准点和目标标记的摄像头画面，可通过顶部页签切换查看，断流时显示状态提示。
  - 按 48 dm × 40 dm 比例绘制场地、六个街区、起降区和消防车出发点。
  - 绘制无人机历史航迹、当前位置和火源标记。
  - 超过 3 秒未收到遥测时显示红色离线告警。
  - 使用任务栏、地图卡片、状态徽标和分组遥测卡片强化信息层级，并自适应窗口尺寸。
  - 与任务节点共用 `fire_params.yaml` 中的场地参数；异常 ROS 数据不会导致界面退出。
  - 支持触屏点击、空格或回车触发无人机启动，带 2 秒防连点和发送反馈。
  - 支持点击界面右上角 `×` 或按 `ESC` 退出窗口。
  - 支持顶部“任务态势 / 摄像头”页签或 `Tab` 键切换主视图。
- 加固已有 `fire_event_bridge`：严格接收 16 字节火源包，避免超长 UDP 包被截断后误判为合法包。
- 火源流中断 3 秒后复位序号过滤器；`done` 和 `failed:*` 终态以 50 ms 间隔回传 3 次。
- 修复消防车到达照射位后未校验朝向的问题：车头对准火源后才允许开启激光，故障路径会立即下发 `OFF`。
- 定位查询短时异常时停车并等待恢复，连续异常超过可配置阈值才进入
  `SAFE_STOP`；TF 丢失与场地越界分别上报，边界外保留可配置裕量。
- 新增 `/fire_mission_reset` 和仪表盘“复位消防车”触屏按钮，无需重启
  launch 即可从 `SAFE_STOP` 返回待命并重新接收火源。
- 任务状态值与机车约定保持一致；车辆忙时重发当前合法状态，不再发送未约定的 `busy`。
- 将通信桥和显示程序加入 `fire_mission.launch.py`，随消防任务统一启动。
- 将通信端口、无人机 IP、按键、显示和场地参数集中到 `fire_params.yaml`。
- 增加显示程序运行依赖和构建安装规则。

## 当前完成度

| 模块 | 状态 | 说明 |
|---|---|---|
| 8889 火源坐标接收 | 已实现并本机验证 | 16 字节定长、magic、seq 去重 |
| 8890 任务状态回传 | 已实现 | 回传给最近一次上报火源的无人机 IP，终态连发 3 次 |
| 8892 无人机遥测接收 | 已实现 | 32 字节定长、类型、seq 校验和断流复位 |
| 8893 车端按键启动 | 已实现并本机验证 | 同一启动包默认连发 5 次 |
| 相机识别与视觉微调 | 已实现并完成合成图测试 | 实机可打开相机并发布有效 480×640 标注帧；阈值和瞄准点仍需现场标定 |
| 车载遥测、航迹与摄像头显示 | 已实现并实机验证图像链路 | 含页签、启动/复位按钮和右上角关闭按钮；仍需手指触屏验收 |
| Cartographer 定位 | 已有代码 | 需要使用实际雷达和场地验证 |
| 消防路径规划与返航 | 已有代码 | 需要精确场地数据和实车验证 |
| 差速底盘控制 | 已有代码 | 需要确认串口、控制点偏移和速度参数 |
| 激光执行器 | 已接 GPIO1_B0 | 物理 22 脚，低电平开启、高电平关闭，带 3 秒超时保护 |
| ROS 任务开机自动运行 | 已提供 shell 入口 | 需自行加入桌面自启动并完成拔电重启验收 |

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
  -> VISUAL_ALIGN
  -> fire_vision (/dev/v4l/by-id/...-video-index0)
  -> /fire_vision/detection
  -> /fire_vision/servo_command
  -> diff_drive_controller（视觉优先级）
  -> /laser_command
  -> laser_gpio_driver

fire_vision
  -> /fire_vision/debug_image（480×640 BGR 标注图）
  -> fire_dashboard“摄像头”页签

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
| 无人机 | `192.168.10.197/24` |

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
| `/fire_mission_reset` | `std_msgs/Empty` | 安全关闭激光并将任务状态复位到 `IDLE` |
| `/drone_telemetry` | `std_msgs/Float32MultiArray` | `[x_dm, y_dm, distance_dm, height_dm, phase, seq]` |
| `/fire_event` | `std_msgs/Float32MultiArray` | `[x_dm, y_dm, seq]` |
| `/fire_mission_status` | `std_msgs/String` | 消防车任务状态，同时回传无人机 |
| `/target_position` | `std_msgs/Float32MultiArray` | `[x_cm, y_cm, mode, yaw_deg]` |
| `/fire_vision/detection` | `follower_pkg/FireDetection` | 红色目标中心、归一化误差、面积和图像尺寸 |
| `/fire_vision/servo_command` | `follower_pkg/VisionServoCommand` | 带使能、时间戳的低速前后/yaw 微调命令 |
| `/fire_vision/debug_image` | `sensor_msgs/Image` | 仪表盘持续显示的视觉标注图 |
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
  ros-humble-cv-bridge python3-numpy python3-opencv python3-pil fonts-noto-cjk

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

1. `fire_link_bridge.drone_ip`：固定为无人机地址 `192.168.10.197`，两端更换地址时必须同步修改。
2. `telemetry_udp_port/start_udp_port`：默认分别为 `8892/8893`，两端必须一致且不能被占用。
3. `seq_reset_timeout_s`：火源和遥测均默认 `3.0` 秒；连续流内仍严格丢弃重复/乱序包。
4. `button_gpio_value_path`：已按推荐物理 16 脚配置为 `/sys/class/gpio/gpio35/value`，使用前必须从 root 开机钩子或手工执行 GPIO 初始化脚本。
5. `arena_origin_map_x_m/y_m/yaw_deg`：标定场地左下角在 Cartographer `map` 中的位置和朝向。
6. `obstacles_dm`：六个街区的统一图纸坐标，位于参数文件顶部共享区；必须按真实场地抽验。
7. `home_x_dm/home_y_dm`：确认消防车红色出发区中心。
8. `district_stop_points_dm`：按 `obstacles_dm` 的顺序给出六个街区各自唯一的下方停车点，必须现场逐点测量。
9. `aim_tol_deg`：默认 `8°`；这是进入相机微调前的地图粗瞄容差，不再直接触发激光。
10. `pose_failure_timeout_s`：连续定位异常的停车宽限期，默认 `1.0` 秒。
11. `tf_max_age_s`：超过该时间未更新的 TF 视为丢失，默认 `0.5` 秒。
12. `outside_arena_margin_dm`：场地边界外仍可容忍的标定/压线裕量，默认 `3.0` dm。
13. `fire_vision.device_path`：必须指向采集接口 `video-index0`；`video-index1` 是元数据接口，不能读取图像。
14. `rotate_cw_deg`：当前安装为 `90`；调试图的上方必须对应车辆前方。
15. `aim_x_ratio/aim_y_ratio`：默认画面中心，装车后应改为激光实际命中像素对应的比例。
16. HSV、红色显著性和面积阈值：必须分别用当前打印目标和最终红色 LED 在现场照明下抽验。
17. `publish_debug_image`：必须保持为 `true`，否则仪表盘摄像头页不会收到图像。
18. `camera_timeout_s`：仪表盘判断画面断流的时限，默认 `1.0` 秒。
19. `laser_gpio_driver`：已配置为物理 22 脚 `GPIO1_B0`（Linux GPIO 40、wPi 13），实测高电平关闭，因此使用 `active_low: true`。
20. 雷达串口、波特率和型号：修改 `src/bluesea2/src/bluesea-ros2/params/uart_lidar.yaml`。
21. 底盘串口：Orange Pi 5 Max 默认 `/dev/ttyS6`，对应 40-pin 的物理 11/13 脚。

## 固定路由、防火墙和联调

在路由器中为消防车和无人机设置静态地址或 DHCP 地址保留，避免与其他设备冲突。消防车启动后确认地址和路由：

```bash
ip -4 address
ip route
ping -c 3 192.168.10.197
```

车端必须看到 `192.168.10.113/24`，并能 ping 通无人机 `192.168.10.197`。项目不会创建热点，也不会自动修改 NetworkManager 配置。

检查 UDP 监听和防火墙：

```bash
ss -lunp | grep -E ':(8889|8892)\b'
sudo nft list ruleset
sudo ufw status verbose
sudo tcpdump -ni any 'host 192.168.10.197 and (udp port 8889 or udp port 8892)'
```

若 UFW 已启用，放行车端两个入站端口：

```bash
sudo ufw allow proto udp from 192.168.10.197 to 192.168.10.113 port 8889
sudo ufw allow proto udp from 192.168.10.197 to 192.168.10.113 port 8892
```

从同一路由器内的另一台机器发送合法数据包验证接收链路：

```bash
python3 -c "import socket,struct;s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.sendto(struct.pack('<HHfff',0xFC11,1,12.0,20.0,0.0),('192.168.10.113',8889))"
python3 -c "import socket,struct,time;s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.sendto(struct.pack('<HBBHHIffffI',0xF14E,1,2,1,0,int(time.monotonic()*1000)&0xffffffff,12.0,20.0,30.0,18.0,0),('192.168.10.113',8892))"
```

车端 `tcpdump` 应看到两包，`ros2 topic echo --once /fire_event` 和 `ros2 topic echo --once /drone_telemetry` 应分别收到数据。无人机向 `192.168.10.113:8889/8892` 发送，并监听 8890/8893；车端启动包始终发送到配置的 `192.168.10.197:8893`。

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

若改成“按下接 3.3 V、外部下拉”的接法，按下读 `1`，此时设 `button_active_low: false`。sysfs 导出、方向、上拉和读权限在重启后会丢失；本仓库的 `scripts/init_start_button_gpio.sh` 会恢复它们。先手工确认电平正确，再配置自启。[Orange Pi 5 Max 官方手册](https://orangepi.net/wp-content/uploads/2024/09/OrangePi_5_Max_RK3588_User-Manual_vv1.2-1.pdf)的 40-pin 表和 wiringOP 章节是引脚编号的基准，不要套用树莓派或其他 Orange Pi 型号的编号。

按下后验收：

```bash
tail -f ~/.local/state/diansai-fire/fire-mission.log
```

日志必须出现 `start requested by GPIO button`。必须在实际接线后检查松开为 `1`、按下为 `0`；代码或模拟电平不能替代此项真机验收。

## 开机自启（shell 入口，不使用 systemd）

先完成构建并配置桌面自动登录，然后把下面这个**普通用户命令**加入你使用的桌面自启动工具：

```bash
/本仓库绝对路径/scripts/start_fire_mission.sh
```

不要用 root 启动整套 ROS；否则桌面授权、日志归属和设备权限都容易出错。入口脚本会自动定位工作区，等待 X11，加载 ROS 2 Humble 与 `install/setup.bash`，防止重复启动，然后执行 `ros2 launch car_launch fire_mission.launch.py`。固定日志为 `~/.local/state/diansai-fire/fire-mission.log`，ROS 自身日志在同目录的 `ros/`；主日志超过 10 MiB 时保留一份 `.previous`。

入口脚本会显式设置车端 `ROS_DOMAIN_ID=6` 和 `ROS_LOCALHOST_ONLY=0`，不依赖桌面自启动是否读取 `~/.bashrc`。这样自动任务与当前车端调试终端保持在同一 DDS domain，且与无人机端的 ROS domain 隔离；两机仍只通过约定的 UDP `8889/8890/8892/8893` 通信。启动日志中必须出现：

```text
ROS_DOMAIN_ID=6 ROS_LOCALHOST_ONLY=0
```

如确实需要更换车端 domain，只给自启动命令设置专用变量 `CAR_ROS_DOMAIN_ID`，并同步修改车端调试终端；不要依赖外部偶然继承的 `ROS_DOMAIN_ID`：

```bash
CAR_ROS_DOMAIN_ID=7 /本仓库绝对路径/scripts/start_fire_mission.sh

# 调试这个自动任务的终端也必须使用相同值
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=0
ros2 node list
```

允许的 domain 值为 `0..232`；非法值会在启动底盘、激光和任务节点之前直接报错退出。无人机必须使用不同的 `ROS_DOMAIN_ID`。

GPIO 初始化必须以 root 身份每次开机执行。请在你自己的 root 开机脚本入口中，把它放在桌面自启动之前：

```bash
/本仓库绝对路径/scripts/start_fire_mission.sh --gpio-only
```

若你的桌面自启动环境已经有免交互 sudo 权限，普通入口会自动补做 GPIO 初始化；没有权限时会在日志中明确报警，但仍启动仪表盘，使触屏启动路径可用。不要让桌面自启动弹出 sudo 密码框。

现场调试时，用仓库提供的停止脚本发送 `SIGINT`，让 ROS 节点按正常退出路径收尾：

```bash
./scripts/stop_fire_mission.sh
./scripts/start_fire_mission.sh
tail -f ~/.local/state/diansai-fire/fire-mission.log
```

如果日志报告 `X11 display was not ready`，确认脚本确实由已登录的桌面会话启动、实际显示为 `:0`，并检查该用户的 `~/.Xauthority` 或 `/run/user/<uid>/gdm/Xauthority`。全屏 OpenCV 触摸依赖 X11 把触屏映射成鼠标；必须在车载屏幕上用手指验收。

## 场地坐标标定

车端参数的方向不能照抄无人机端：

- `arena_origin_map_x_m/y_m` 是**场地原点 `(0,0)` 在车端 Cartographer `map` 系里的坐标**，单位 m。
- `arena_origin_map_yaw_deg` 是场地 `+x` 轴在 `map` 系里的朝向；从 `map +x` 到场地 `+x` 逆时针为正，符合 ROS 平面 yaw 约定。
- 无人机侧填写的是**飞机起飞点在场地系里的坐标**。这是反方向的变换，不能把无人机起飞坐标直接填进车端 `arena_origin_map_*`。

具体例子：车在场地 `(13.5, 2.5) dm` 的固定点上电，车头、Cartographer `map +x` 与场地 `+x` 完全同向，且上电车位就是 `map (0,0) m`。此时场地原点在车后方 `1.35 m`、右侧/负 y 方向 `0.25 m`，所以应填：

```yaml
arena_origin_map_x_m: -1.35
arena_origin_map_y_m: -0.25
arena_origin_map_yaw_deg: 0.0
home_x_dm: 13.5
home_y_dm: 2.5
```

现场按以下步骤操作：

1. 在红色出发区标出唯一上电点和唯一车头方向。Cartographer 的 `map` 原点由上电位姿决定，所以每次重启前都必须把车摆回这个固定点并朝固定方向。
2. 测量该车位和六个街区的场地坐标，单位统一换算成 dm；在 `fire_params.yaml` 顶部共享区更新 `home_x_dm/home_y_dm` 和唯一一份 `obstacles_dm`。边界不要预先加安全余量，规划器另有 `safety_margin_dm`。
3. 启动 Cartographer，读取固定车位在 `map` 中的实际位置与朝向，据此求场地原点在 `map` 中的位置和场地 `+x` 在 `map` 中的 yaw，填入三个 `arena_origin_map_*`。不要反过来填“车在场地中的位置”。
4. 重启整套任务使参数生效，让车保持在固定上电位姿，检查地图定位和屏幕上的 home 标记。
5. 两边各自标完后，把无人机搬到一个远离边界、坐标已知的场地格点并发送遥测/火源坐标；车端屏幕必须显示同一组坐标。再换至少一个格点复验，以同时发现平移、旋转和轴方向错误。对不上就不能算标定完成。

## 启动与调试

正式任务：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch car_launch fire_mission.launch.py
```

### 仪表盘操作

仪表盘默认显示“任务态势”页，顶部页签可切换到摄像头大画面。摄像头节点在两个
页签间切换时不会重启，会一直采集并处理画面。

| 操作 | 触屏/鼠标 | 键盘 |
|---|---|---|
| 切换任务态势/摄像头 | 点击顶部页签 | `Tab` |
| 启动无人机 | 点击“启动无人机” | `Enter` 或空格 |
| 复位消防车任务 | 点击“复位消防车” | `R` |
| 退出仪表盘窗口 | 点击右上角 `×` | `ESC` |

右上角 `×` 只关闭仪表盘节点，不会停止同一 launch 中的雷达、底盘、激光和任务
节点。需要停止整套任务时应使用 `./scripts/stop_fire_mission.sh`。

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
ros2 topic echo /fire_vision/detection
```

仅验证雷达、定位、航点控制和底盘链路：

```bash
ros2 launch car_launch lidar_test.launch.py
```

## 相机识别与激光瞄准标定

先保持激光物理断电。仪表盘会持续显示标注图，也可以用
`rqt_image_view` 单独查看：

```bash
ros2 run rqt_image_view rqt_image_view /fire_vision/debug_image
ros2 topic hz /fire_vision/detection
```

当前相机原始 640×480 画面顺时针旋转 90°，处理后为 480×640。黄色十字是
`aim_x_ratio/aim_y_ratio` 指定的瞄准点，青色圆点是红色目标中心。确认红色打印
目标在远近、左右和不同亮度下都能形成稳定轮廓；若有误检，依次缩小 `roi`、
提高 `saturation_min`/红色通道差值或提高 `min_area_ratio`，每次改 YAML 后重启。

最后在安全挡光板上短时点亮激光，记录光斑在处理后图像中的像素
`(laser_x, laser_y)`，配置：

```text
aim_x_ratio = laser_x / 480
aim_y_ratio = laser_y / 640
```

先架空驱动轮并保持激光断电验证运动方向：目标移到画面上方只允许出现正线速度，
下方为负线速度，左侧为正角速度，右侧为负角速度。方向不符时优先修正
`rotate_cw_deg` 或镜像参数，不要通过交换控制符号掩盖安装方向错误。
`publish_debug_image` 应保持为 `true`，供仪表盘持续显示摄像头画面。

### 摄像头页一直显示“等待”的排查

自动启动默认使用 `ROS_DOMAIN_ID=6`。先在调试终端进入同一个 domain，再检查设备、
节点、参数和图像话题：

```bash
export ROS_DOMAIN_ID="${CAR_ROS_DOMAIN_ID:-6}"
export ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash
source install/setup.bash

ls -l /dev/v4l/by-id/*video-index0
ros2 node list | grep -E '/fire_vision|/fire_dashboard'
ros2 param get /fire_vision device_path
ros2 param get /fire_vision publish_debug_image
ros2 topic info -v /fire_vision/debug_image
ros2 topic hz /fire_vision/debug_image
tail -n 100 ~/.local/state/diansai-fire/fire-mission.log
```

正常状态应满足：

- `video-index0` 存在，且当前用户属于 `video` 组或对设备有读写权限；
- `/fire_vision` 与 `/fire_dashboard` 都在节点列表中；
- `publish_debug_image` 为 `True`；
- `/fire_vision/debug_image` 各有 1 个 publisher 和 subscriber，并能持续收到帧；
- 日志出现 `camera opened`，且没有 `camera device does not exist` 或
  `failed to open camera`。

修改 YAML 或重新构建不会改变已经运行的 ROS 参数。若参数仍是 `False`，确认
`src/follower_pkg/config/fire_params.yaml` 中设置为 `true`，然后用正常退出流程
重启整套任务：

```bash
./scripts/stop_fire_mission.sh
./scripts/start_fire_mission.sh
```

不要直接杀死视觉节点后只重开仪表盘；图像 publisher 是视觉节点启动时根据参数创建
的，必须让视觉节点重新读取配置。

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

同时启动两个通信桥后，可运行仓库内的黑盒验收脚本。它会依次验证火源和遥测的连续流 `seq=5,3,4` 只接受 `5`、停流超过 3 秒后从 `seq=0` 恢复，以及 `enroute` 单发、`done` 连发 3 次：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/verify_udp_bridges.py
```

成功时输出 `PASS: restart reset, ordering, and done/failed terminal x3`，两个桥的日志应各出现一条 `stream restarted`。测试占用本机 UDP 8890，因此不要和无人机联调同时执行。

## `stamp_ms` 的 ROS 接口处理

UDP 32 字节包中的 `stamp_ms` 保留不动，确保无人机端协议完全兼容；车端不再把它放入 `Float32MultiArray`。原因是 uint32 毫秒值转换为 float32 后只能保留约 7 位有效数字，无法可靠测量链路延迟。链路超时仍只使用车端本地收包时间。

## 已完成的软件验证

- `follower_pkg` 与 `car_launch` 在 ROS 2 Humble 下构建通过。
- OpenCV 红色目标检测包含合成图单元测试，覆盖打印红、发光红、噪声过滤、方向符号、旋转和标定瞄准点。
- 实机视觉节点已成功打开固定 `video-index0` 设备，并验证仪表盘图像话题可收到
  非空的 480×640 `bgr8` 标注帧。
- `CAR_START` 的固定目标地址、32 字节小端布局和同序号 5 次冗余发送验证通过。
- 遥测包的定长校验、类型校验、序号去重和 `65535 → 0` 回绕验证通过。
- 火源包的 16 字节定长校验、序号去重和状态字符串回传验证通过。
- 火源与遥测流中断后从 `seq=0` 恢复、连续流逆序丢弃以及终态 3 次冗余已通过黑盒测试。
- 超长数据包以及包含 `NaN/Inf` 的遥测/火源包不会被误接收，非法包不会占用序号。
- 显示器已在 headless 模式验证默认参数、异常数据防护、坐标、航迹、火源和状态渲染。
- 任务状态机已用模拟 TF 验证：到达照射位但未对准时不会启光，对准后才发布 `ON`。
- Python 语法、flake8、PEP 257、CMake lint、XML lint 和全部 C++ `uncrustify` 检查通过。

## 还有什么需要做

### 比赛前必须完成

- [ ] 确认消防车为 `192.168.10.113/24`、无人机为 `192.168.10.197/24`，且同一路由器内能互相收发 UDP。
- [ ] 确认无人机端已经同步使用 `8889/8890/8892/8893` 和本文档的两种定长包格式。
- [ ] 两端使用同一个物理场地左下角联合标定，并用已知格点互相核对坐标。
- [ ] 实测并更新六个街区、起降区和消防车出发区参数。
- [ ] 确认底盘 UART、雷达设备名和系统权限，完成断流自动停车测试。
- [ ] 标定相机旋转、ROI、红色阈值和激光命中像素，架空验证四个微调方向。
- [ ] 在实物上验证激光 PB0 的上电默认 OFF、超时 OFF、退出 OFF 和故障 OFF。
- [ ] 实际接入物理启动按键，把 `--gpio-only` 加入 root 开机脚本，并验证有效电平和 50 ms 去抖。
- [ ] 在车载屏幕上用手指验证全屏按钮、中文字体、字号和无遮挡显示。
- [ ] 把 shell 入口加入桌面自启动后拔电重启，验证仪表盘、GPIO 初始化和日志落盘。
- [ ] 放行 UDP `8889`、`8892` 入站以及 `8890`、`8893` 出站，并进行丢包联调。
- [ ] 实车低速验证路径、到点容差、安全余量和返航，确保车轮不压街区边界。

### 建议继续完善

- [ ] 将通信数组话题替换为带字段名的自定义 ROS 消息，方便后续维护。
- [ ] 增加可重复运行的自动化协议测试和 CI。
- [ ] 清理上游雷达包的许可证/TODO 元数据。
- [ ] 实测六个街区下方固定停车点，确认车身不越界且激光距离、角度满足要求。

## 文档

- [G题_机车通信接口约定.md](G题_机车通信接口约定.md)：机车唯一 UDP 通信契约。
- [G题_场地区域坐标.md](G题_场地区域坐标.md)：机端、车端共用的固定区域坐标源。
- [fire_mission_design.md](fire_mission_design.md)：消防任务规划、坐标与状态机设计。
- [SR5E1E3_CHASSIS_DEBUG_GUIDE.md](SR5E1E3_CHASSIS_DEBUG_GUIDE.md)：底盘协议、串口接线和诊断说明。
- [G题_空地协同智能消防系统.pdf](G题_空地协同智能消防系统.pdf)：赛题原文。

## 许可证

本工作区自研 ROS 2 包当前声明为 Apache-2.0。`bluesea2` 等第三方或上游代码请以其各自文件和原厂许可为准；公开分发前应再次核对相关许可信息。
