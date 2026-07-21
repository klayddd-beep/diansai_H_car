# 空地协同智能消防车

这是 2023 年全国大学生电子设计竞赛 G 题的地面消防车 ROS 2 Humble 工作区。系统使用激光雷达和 Cartographer 定位，接收无人机 UDP 火点坐标后规划避障路径，到达照射位、控制激光灭火并返航。底盘为 SR5E1E3 差速底盘，控制链路为 `/target_position → /cmd_vel → $VW`。

> 安全提示：激光可能伤害眼睛或引发火灾。硬件调试时应使用合规低功率模块、物理总断电和默认断电的驱动电路，不要用开发板 GPIO 直接驱动激光器。

## 功能与包结构

- `bluesea2` / `base_lidar`：蓝海光电雷达 ROS 2 驱动及服务接口。
- `car_carto_pkg`：Cartographer、URDF 和 TF 配置。
- `follower_pkg`：差速控制、UDP 火情桥、任务状态机与激光 GPIO 抽象。
- `orangepi_to_car`：把 ROS 2 `cmd_vel` 转为 SR5E1E3 串口协议。包名保留了原平台名称，但实现本身可在 RDK X5 的 Linux UART 上运行。
- `car_launch`：联合启动入口。

详细任务接口、场地参数和验证清单见 [fire_mission_design.md](fire_mission_design.md)。原 Orange Pi 5 Max 的串口接线、底盘协议和诊断见 [SR5E1E3_CHASSIS_DEBUG_GUIDE.md](SR5E1E3_CHASSIS_DEBUG_GUIDE.md)；其中的 Orange Pi 针脚定义不能套用到 RDK X5。

## 环境与构建

已知可构建环境是 ARM64 (`aarch64`)、Ubuntu 22.04 和 ROS 2 Humble。首次部署建议只克隆源码，在目标板重新构建；仓库不包含 `build/`、`install/` 和 `log/` 中的机器相关产物。

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep \
  ros-humble-cartographer-ros ros-humble-robot-state-publisher

sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

正式运行（Orange Pi 原默认串口是 `/dev/ttyS6`）：

```bash
ros2 launch car_launch fire_mission.launch.py
```

## 移植到 RDK X5

### 结论

**软件移植可行性高，整车落地可行性中等偏高。** RDK X5 是 ARM64 平台，官方机器人环境已适配 Ubuntu 22.04 和 ROS 2 Humble；本项目也已在相同 OS/ROS 组合的 ARM64 机器上构建。ROS 2 节点、UDP 网络、差速控制和雷达驱动没有明显的架构障碍。

不能直接插卡运行的部分是板级 I/O：Orange Pi 的 `/dev/ttyS6` 与物理针脚在 X5 上不存在对应关系；激光节点当前也故意保持 `mock_mode: true`，尚未驱动真实 GPIO。Cartographer 是 CPU 工作负载，不会自动使用 X5 BPU，但本项目的 2D 定位不要求 BPU 才能运行。

### 需要的操作

1. **准备系统。** 给 X5 刷入官方 Ubuntu 22.04 镜像，安装 ROS 2 Humble/官方机器人环境，并使用稳定的 5 V/5 A 供电。参考 [RDK X5 硬件介绍](https://developer.d-robotics.cc/rdk_doc/Quick_start/hardware_introduction/rdk_x5/) 和 [TogetheROS.Bot 环境准备](https://developer.d-robotics.cc/rdk_doc/Robot_development/quick_start/preparation/)。
2. **原生构建。** 按上文命令用 `rosdep` 安装依赖，再用 `colcon build --symlink-install` 在 X5 本机编译。不要复制 Orange Pi 的 `install/` 目录。
3. **迁移底盘 UART。** X5 默认在 40-pin 的物理 8/10 脚开启 UART1，IO 电平为 3.3 V，Linux 设备通常为 `/dev/ttyS1`。核对 X5 TX → 底盘 RX、X5 RX ← 底盘 TX 和共地；若底盘不是 3.3 V TTL，必须加电平转换。详见官方 [UART 使用说明](https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/01_40pin_user_sample/uart)。也可用 USB-TTL 转换器获得更稳定的 `/dev/ttyUSB*` 设备。
4. **设置串口权限并单独验证。** 将用户加入 `dialout` 组后重新登录，先停车架空车轮运行诊断，再启动 ROS 2。

   ```bash
   sudo usermod -aG dialout "$USER"
   python3 src/scripts/chassis_diag.py --port /dev/ttyS1 --baud 115200
   ros2 launch car_launch fire_mission.launch.py \
     chassis_port:=/dev/ttyS1 chassis_baud:=115200
   ```

5. **迁移雷达。** USB 雷达通常显示为 `/dev/ttyUSB0`；建议用仓库内 `LHLiDAR.rules` 的 VID/PID 思路创建稳定 udev 别名，并在 `src/bluesea2/src/bluesea-ros2/params/uart_lidar.yaml` 中配置实际设备、波特率和雷达型号。先运行 `ros2 launch car_launch lidar_test.launch.py`，确认 `/scan` 和 TF 正常。
6. **实现激光 GPIO。** 在 X5 上通过 `srpi-config` 将选定针脚切换为 GPIO，按 [40-pin 引脚复用说明](https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/01_40pin_user_sample/40pin_define/) 确认线号和电平。将 `laser_gpio_driver.cpp` 中的 mock 分支替换为 `libgpiod` 输出（或独立的 `Hobot.GPIO` ROS 2 节点），实现上电默认 OFF、超时 OFF、节点退出 OFF 和故障 OFF。通过 MOSFET/光耦驱动负载，完成万用表测试前不得把 `mock_mode` 改为 `false`。官方 [GPIO 示例](https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/01_40pin_user_sample/gpio) 可用于查针脚。
7. **配置网络与场地。** 确保无人机能访问 X5，放行 UDP 8889 入站和 8890 回包。实测场地后修改 `fire_params.yaml` 中的原点、朝向、障碍物、安全边界和激光时间；当前障碍物数据只是赛题图估计值。
8. **分阶段验收。** 依次检查 UART 停车/看门狗、雷达 `/scan`、`map -> base_link` TF、悬空车轮的 `cmd_vel`、GPIO 空载电平、UDP 丢包/重复包，最后才进行低速整车联调。

### X5 上的快速检查

```bash
# 硬件与节点
uname -m
ls -l /dev/ttyS1 /dev/ttyUSB* 2>/dev/null
ros2 pkg list | grep -E 'cartographer|bluesea2|follower_pkg|orangepi_to_car'

# 运行后检查关键话题/TF
ros2 topic list
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /fire_mission_status
```

## 当前限制

- `laser_gpio_driver` 仍是 mock；未指定 X5 GPIO 前，不具备真实激光控制能力。
- 场地坐标和障碍物尺寸必须现场标定，不能直接用于比赛。
- 蓝海雷达的具体型号、通信方式和 X5 内核中的 USB-UART 驱动需上板确认。
- `bluesea2` / `base_lidar` 上游包的 manifest 仍标注 `TODO` 许可证；在公开分发或商用前应确认原厂许可条款。
