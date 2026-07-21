# 消防车任务设计

## 运行链路

```text
无人机 UDP 火情包 → fire_event_bridge → /fire_event
                                      ↓
雷达 → Cartographer → TF map←laser_link → fire_mission_manager
                                      ↓
                           /target_position (cm/deg)
                                      ↓
                    diff_drive_controller → /cmd_vel → $VW 底盘

fire_mission_manager → /laser_command → laser_gpio_driver → 激光笔 GPIO
```

## 接口

- UDP 火情包为小端 16 字节：`[0xFC11:u16][seq:u16][x_dm:f32][y_dm:f32][reserved:f32]`，车监听 UDP `8889`。
- `/fire_event`：`Float32MultiArray[x_dm, y_dm, seq]`。
- `/fire_mission_status`：`ready`、`enroute`、`extinguishing`、`returning`、`done` 或 `failed:<reason>`；桥回传给最近的 UDP 发送方 `8890` 端口。
- `/laser_command`：`String`，取值 `ON` / `OFF`；`/laser_status` 返回 `ready`、`on`、`off`、`timeout_off`。
- `/target_position` 不变：`Float32MultiArray[x_cm, y_cm, mode, yaw_deg]`。

## 场地、规划与安全

`fire_params.yaml` 使用题目场地坐标：原点在左下角，单位 dm，大小为 48×40。`arena_origin_map_*` 是场地坐标和 Cartographer `map` 的标定变换。`obstacles_dm` 是街区矩形 `[xmin,ymin,xmax,ymax,...]`；规划时再膨胀 `safety_margin_dm`，防止车轮压街区边界。

任务管理器收到火点后，在火点四周寻找 4 dm 的可达照射位，用 1 dm 栅格 BFS 绕开膨胀后的街区；到位后激光照射 2.1 秒，再规划回红色出发区域。TF 丢失、目标不可达或返航不可达会停止任务，并向现有控制器发送当前位置保持目标。

## 已实现

- 删除飞车位姿桥、面包屑跟随、补货 UDP、推货任务与相关启动入口。
- 删除底盘桥的 `$SERVO`、上电舵机回零、`car_movement` 5/6 命令及舵机参数；保留差速 `$VW`、`/cmd_vel` 和基础人工调试命令。
- 实现火情 UDP 接收与状态回传、消防状态机、栅格避障路径、返航逻辑。
- 新增激光 GPIO 抽象节点，当前 `mock_mode=true`，具备 `ON/OFF`、最长照射时间保护和状态反馈。
- 新增 `fire_mission.launch.py` 正式启动入口与场地参数文件。

## 仍需实现或验证

- **必须现场量取**街区精确边界、红色出发区中心、`map↔arena` 变换，并替换当前 PDF 图形估算的 `obstacles_dm`。
- 在开发板执行 `colcon build`；本仓库按双设备约定未在本地编译。
- 用实车验证控制点偏移、速度/角速度、到点容差及 1.5 dm 障碍安全余量，确认任何车轮都不压线。
- 确认无人机端 UDP 包格式、IP、端口与状态回包解析；必要时增加消息校验和重传策略。
- 激光笔 GPIO 接线、有效电平和实际输出针脚尚未确定。确认后以 `libgpiod` 替换 `laser_gpio_driver` 中标注的 mock 分支，并验证连续照射约 2 秒可可靠控制模拟火源。
- 当前路径规划以街区外的照射位为目标；若火点在街区深处而四个候选点都不可达，需要按实测扩展候选角度和距离。
