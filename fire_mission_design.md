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
                                      ↑
摄像头 → fire_vision → /fire_vision/detection
                         ↓
              fire_mission_manager(VISUAL_ALIGN)
                         ↓
              /fire_vision/servo_command

fire_mission_manager → /laser_command → laser_gpio_driver → 激光笔 GPIO
```

## 接口

- UDP 火情包为小端 16 字节：`[0xFC11:u16][seq:u16][x_dm:f32][y_dm:f32][reserved:f32]`，车监听 UDP `8889`。
- `/fire_event`：`Float32MultiArray[x_dm, y_dm, seq]`。
- `/fire_mission_status`：`ready`、`enroute`、`extinguishing`、`returning`、`done` 或 `failed:<reason>`；桥回传给最近的 UDP 发送方 `8890` 端口。
- `/laser_command`：`String`，取值 `ON` / `OFF`；`/laser_status` 返回 `ready`、`on`、`off`、`timeout_off`。
- `/target_position` 不变：`Float32MultiArray[x_cm, y_cm, mode, yaw_deg]`。
- `/fire_vision/detection`：`FireDetection`，包含红色目标中心、归一化误差、
  面积和处理后图像尺寸。
- `/fire_vision/servo_command`：`VisionServoCommand`，带显式使能和时间戳的
  低速线速度/yaw 指令；差速控制器仍是唯一 `/cmd_vel` 发布者。

## 场地、规划与安全

`fire_params.yaml` 使用题目场地坐标：原点在左下角，单位 dm，大小为 48×40。文件顶部的共享参数区是车端唯一几何数据源，任务管理器和仪表盘读取同一份 `obstacles_dm`。`arena_origin_map_*` 是场地坐标和 Cartographer `map` 的标定变换；规划时再膨胀 `safety_margin_dm`，防止车轮压街区边界。

任务管理器收到火点后，先用六个街区矩形判断火点所属街区，再读取该街区唯一的固定下方停车点 `district_stop_points_dm`，用 1 dm 栅格 BFS 绕开膨胀后的街区。到位并按火源坐标完成地图粗瞄后进入 `VISUAL_ALIGN`：红色目标偏上前进、偏下后退、偏左正 yaw、偏右负 yaw，连续 8 帧进入死区后才照射 2.1 秒，再规划回红色出发区域。视觉阶段限制前后累计位移、yaw 搜索范围和总时长，并继续使用 TF 与膨胀障碍检查运动安全。火点不在任何街区、TF 丢失、目标不可达、视觉失败或返航不可达都会立即发送激光 `OFF` 并停车。

## 已实现

- 删除飞车位姿桥、面包屑跟随、补货 UDP、推货任务与相关启动入口。
- 删除底盘桥的 `$SERVO`、上电舵机回零、`car_movement` 5/6 命令及舵机参数；保留差速 `$VW`、`/cmd_vel` 和基础人工调试命令。
- 实现火情 UDP 接收与状态回传、消防状态机、栅格避障路径、返航逻辑。
- 新增红色目标检测、视觉微调状态、低速控制仲裁和视觉断流停车保护。
- 新增激光 GPIO 抽象节点，具备 `ON/OFF`、最长照射时间保护和状态反馈。
- 新增 `fire_mission.launch.py` 正式启动入口与场地参数文件。

## 仍需实现或验证

- **必须现场量取**街区精确边界、红色出发区中心、`map↔arena` 变换，并替换当前 PDF 图形估算的 `obstacles_dm`。
- 在实车上标定相机 ROI、红色阈值、处理方向和激光实际命中像素。
- 用实车验证控制点偏移、速度/角速度、到点容差及 1.5 dm 障碍安全余量，确认任何车轮都不压线。
- 确认无人机端 UDP 包格式、IP、端口与状态回包解析；必要时增加消息校验和重传策略。
- 验证激光 GPIO 有效电平、上电默认关闭和连续照射约 2 秒的可靠性。
- 六个 `district_stop_points_dm` 当前仍是按图估算的街区下方停车点，必须结合车身尺寸、激光有效距离和真实场地逐点测量。
