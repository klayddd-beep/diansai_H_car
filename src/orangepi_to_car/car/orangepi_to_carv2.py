#!/usr/bin/env python3
"""
SR5E1E3 小车底盘 ROS2 话题控制节点 v2。

订阅 std_msgs/msg/String 类型的 car_movement：
  data="0"    -> 停车，发送 $STOP
  data="1"    -> 航向保持直行，发送 $GET,IMU + $MODE,VW + $LINE,0.5
  data="1,v"  -> 航向保持直行，发送 $GET,IMU + $MODE,VW + $LINE,v
  data="2"    -> 左转 90 度，发送 $GET,IMU + $MODE,VW + $TURN,-90
  data="3"    -> 右转 90 度，发送 $GET,IMU + $MODE,VW + $TURN,90
  data="4"    -> 右转掉头 180 度，发送 $GET,IMU + $MODE,VW + $TURN,180

另订阅 geometry_msgs/msg/Twist 类型的 cmd_vel（跟随任务连续速度控制，M3 新增）：
  linear.x = v (m/s), angular.z = w (rad/s) -> 进入 VW 模式后持续发送 $VW,v,w
  - 流式发送不逐帧等应答（快速通道），发送间隔下限 VW_MIN_INTERVAL_S；
  - cmd_vel 断流超过 CMD_VEL_TIMEOUT_S 自动发 $STOP（桥侧看门狗）；
  - --chassis-timeout-ms > 0 时启动后下发 $SET,TIMEOUT,1,ms 启用底盘侧通信超时
    兜底。注意：底盘超时也会让离散命令（如 $LINE）在静默后被自动刹停，
    所以默认 0（关闭），只在跟随任务 launch 里开启。

本脚本只发送 SR5E1E3_CHASSIS_DEBUG_GUIDE.md 中已经写明的核心串口帧。
激光笔不经底盘串口控制，由独立的 /laser_command GPIO 节点处理。
"""

import argparse
import os
import re
import select
import sys
import termios
import time

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from geometry_msgs.msg import Twist
except ImportError:
    print(
        "ERROR: 需要在 ROS2 环境中运行，请先 source /opt/ros/<distro>/setup.bash",
        file=sys.stderr,
    )
    raise


DEFAULT_PORT = "/dev/ttyS6"
DEFAULT_BAUD = 115200
TOPIC_NAME = "car_movement"

# 普通命令读取时间可以短一些；IMU 查询稍长，避免偶发读不到完整 $ATT。
READ_TIMEOUT_S = 0.35
IMU_READ_TIMEOUT_S = 0.6

# $LINE 的 v 单位是 m/s，范围与固件 VW/航向控制调试值保持一致。
DEFAULT_LINE_V = 0.50
LINE_MIN_V = -2.0
LINE_MAX_V = 2.0


CMD_STOP = "$STOP\r\n"
CMD_VW_MODE = "$MODE,VW\r\n"
CMD_IMU = "$GET,IMU\r\n"

# cmd_vel 连续速度通道（M3 跟随任务）。
CMD_VEL_TOPIC = "cmd_vel"
VW_MIN_INTERVAL_S = 0.04     # $VW 流式发送的最小间隔（约 25Hz 上限）
VW_DRAIN_TIMEOUT_S = 0.02    # 快速通道只顺手清空接收缓冲，不等待应答
CMD_VEL_TIMEOUT_S = 0.5      # cmd_vel 断流判定，超时发 $STOP
CMD_VEL_WATCHDOG_PERIOD_S = 0.1
VW_MAX_W = 3.0               # w 限幅 rad/s（v 限幅复用 LINE_MIN_V/LINE_MAX_V）

# 解析 MD 中定义的 IMU 返回帧，例如：
# $ATT,yaw=0.00,gz=0.00,bias=-2.50,cal=1
ATT_RE = re.compile(
    r"\$ATT,yaw=([-+]?\d+(?:\.\d+)?),"
    r"gz=([-+]?\d+(?:\.\d+)?),"
    r"bias=([-+]?\d+(?:\.\d+)?),"
    r"cal=(\d+)"
)


class CommandResult:
    """一条串口命令的返回结果，供级联动作判断是否继续。"""

    def __init__(self, command, response):
        self.command = command
        self.response = response
        self.text = decode_response(response) if response else ""
        self.status = classify_response(response)

    @property
    def can_continue(self):
        return self.status not in ("NO_RESPONSE", "ERR")


def classify_response(response):
    if not response:
        return "NO_RESPONSE"

    text = decode_response(response)
    if "$ERR," in text:
        return "ERR"
    if "$OK," in text:
        return "OK"
    if "$ATT" in text:
        return "DATA"
    return "RX_ONLY"


def baud_to_termios(baud):
    """把数字波特率转换为 termios 常量，例如 115200 -> B115200。"""
    constant_name = f"B{baud}"
    if not hasattr(termios, constant_name):
        raise ValueError(f"unsupported baud rate: {baud}")
    return getattr(termios, constant_name)


def configure_uart(fd, baud):
    """配置串口为 MD 要求的原始 8N1 模式。"""
    attrs = termios.tcgetattr(fd)

    attrs[0] = 0  # 输入不做换行、控制字符等转换。
    attrs[1] = 0  # 输出不做换行转换。
    attrs[3] = 0  # 关闭本地回显和规范输入，直接收发字节。

    # CLOCAL/CREAD 打开本地串口接收；CS8、无校验、1 停止位对应 8N1。
    attrs[2] |= termios.CLOCAL | termios.CREAD
    attrs[2] &= ~termios.CSIZE
    attrs[2] |= termios.CS8
    attrs[2] &= ~termios.PARENB
    attrs[2] &= ~termios.CSTOPB
    if hasattr(termios, "CRTSCTS"):
        attrs[2] &= ~termios.CRTSCTS

    baud_flag = baud_to_termios(baud)
    attrs[4] = baud_flag
    attrs[5] = baud_flag
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0

    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)


def read_available(fd, timeout_s):
    """读取 timeout_s 内已经到达的所有串口字节。"""
    deadline = time.monotonic() + timeout_s
    chunks = []

    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break

        readable, _, _ = select.select([fd], [], [], remaining)
        if not readable:
            break

        try:
            data = os.read(fd, 1024)
        except BlockingIOError:
            continue

        if not data:
            break
        chunks.append(data)

    return b"".join(chunks)


def decode_response(response):
    return response.decode("ascii", errors="replace").rstrip()


def send_command(fd, command, read_timeout_s=READ_TIMEOUT_S, logger=None):
    """发送一帧已带 CRLF 包尾的 MD 命令，并读取驱动板返回。"""
    if logger is not None:
        logger.info(f"TX: {command.rstrip()!r}")

    os.write(fd, command.encode("ascii"))
    termios.tcdrain(fd)

    response = read_available(fd, read_timeout_s)
    result = CommandResult(command, response)

    if logger is not None:
        if response:
            logger.info(f"RX: {decode_response(response)!r} [{result.status}]")
        else:
            logger.info(f"RX: <no response> [{result.status}]")

    return result


def make_frame(*fields):
    """统一生成 '$字段,字段\\r\\n'，避免手写帧时漏掉 CRLF。"""
    return "$" + ",".join(str(field) for field in fields) + "\r\n"


def make_line_frame(v):
    return make_frame("LINE", format_float(v))


def make_turn_frame(angle_deg):
    return make_frame("TURN", format_float(angle_deg))


def make_vw_frame(v, w):
    return make_frame("VW", format_float(v), format_float(w))



def format_float(value):
    """把浮点数压成适合串口调试阅读的短格式，例如 0.300 -> 0.3。"""
    return f"{float(value):.3f}".rstrip("0").rstrip(".")


def parse_att_response(response):
    """从串口返回中提取最后一帧 $ATT；没有合法帧时返回 None。"""
    text = decode_response(response)
    matches = list(ATT_RE.finditer(text))
    if not matches:
        return None

    match = matches[-1]
    return {
        "yaw": float(match.group(1)),
        "gz": float(match.group(2)),
        "bias": float(match.group(3)),
        "cal": int(match.group(4)),
    }


class CarMovementV2Node(Node):
    def __init__(self, port, baud, chassis_timeout_ms=0):
        super().__init__("orangepi_to_carv2")
        self.fd = None
        # cmd_vel 流式通道状态
        self.vw_stream_active = False   # 是否已为流式控制进入 VW 模式
        self.last_cmd_vel_time = None   # 最近一次 cmd_vel 到达时刻（monotonic）
        self.last_vw_send_time = 0.0    # 最近一次 $VW 发送时刻（限频用）
        self.last_vw_nonzero = False    # 上一帧是否非零速（保证零速帧不被限频丢掉）

        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            configure_uart(self.fd, baud)
        except Exception:
            os.close(self.fd)
            self.fd = None
            raise

        if chassis_timeout_ms > 0:
            # 底盘侧通信超时兜底：静默 chassis_timeout_ms 后底盘自动刹停
            self.send_command(make_frame("SET", "TIMEOUT", 1, int(chassis_timeout_ms)))

        self.subscription = self.create_subscription(
            String,
            TOPIC_NAME,
            self.on_movement,
            10,
        )
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            CMD_VEL_TOPIC,
            self.on_cmd_vel,
            10,
        )
        self.cmd_vel_watchdog = self.create_timer(
            CMD_VEL_WATCHDOG_PERIOD_S,
            self.check_cmd_vel_timeout,
        )
        self.get_logger().info(f"Opened {port} at {baud} 8N1")
        self.get_logger().info(
            'Topic car_movement String (manual debug only): "0"=STOP, '
            '"1[,v]"=LINE, "2"=left90, "3"=right90, "4"=right180'
        )
        self.get_logger().info(
            "Topic cmd_vel Twist: linear.x=v m/s, angular.z=w rad/s -> $VW stream"
        )

    def close(self):
        if self.fd is not None:
            try:
                self.send_command(CMD_STOP)
            except OSError as exc:
                self.get_logger().error(f"Failed to send $STOP on close: {exc}")
            os.close(self.fd)
            self.fd = None

    def send_command(self, command, read_timeout_s=READ_TIMEOUT_S):
        return send_command(
            self.fd,
            command,
            read_timeout_s=read_timeout_s,
            logger=self.get_logger(),
        )

    def send_command_fast(self, command):
        """流式快速通道：写出后只清空接收缓冲，不等待逐帧应答。

        20Hz 的 $VW 流如果走 send_command（每帧等 0.35s 应答）串口会跟不上。
        """
        os.write(self.fd, command.encode("ascii"))
        termios.tcdrain(self.fd)
        read_available(self.fd, VW_DRAIN_TIMEOUT_S)

    def on_cmd_vel(self, msg):
        v = max(LINE_MIN_V, min(LINE_MAX_V, float(msg.linear.x)))
        w = max(-VW_MAX_W, min(VW_MAX_W, float(msg.angular.z)))
        is_zero = v == 0.0 and w == 0.0

        now = time.monotonic()
        self.last_cmd_vel_time = now

        # 限频：非零帧间隔不足时丢弃；零速帧在上一帧非零时必须放行（保证刹停到位）
        if now - self.last_vw_send_time < VW_MIN_INTERVAL_S:
            if not (is_zero and self.last_vw_nonzero):
                return

        if not self.vw_stream_active:
            result = self.send_command(CMD_VW_MODE)
            if not result.can_continue:
                self.get_logger().warning(
                    "cmd_vel: VW mode was not accepted, dropping this command"
                )
                return
            self.vw_stream_active = True

        self.send_command_fast(make_vw_frame(v, w))
        self.last_vw_send_time = now
        self.last_vw_nonzero = not is_zero

    def check_cmd_vel_timeout(self):
        """桥侧看门狗：cmd_vel 断流即刹停，不依赖上游是否发完零速。"""
        if not self.vw_stream_active or self.last_cmd_vel_time is None:
            return
        if time.monotonic() - self.last_cmd_vel_time > CMD_VEL_TIMEOUT_S:
            self.get_logger().warning("cmd_vel timeout -> $STOP")
            self.send_command(CMD_STOP)
            self.vw_stream_active = False
            self.last_cmd_vel_time = None
            self.last_vw_nonzero = False

    def read_imu(self):
        result = self.send_command(CMD_IMU, read_timeout_s=IMU_READ_TIMEOUT_S)
        if not result.can_continue:
            return None
        return parse_att_response(result.response)

    def ensure_imu_ready(self):
        imu = self.read_imu()
        if imu is None:
            self.get_logger().warning(
                "Abort: no valid IMU response, so no heading command was sent"
            )
            return False
        if imu["cal"] != 1:
            self.get_logger().warning(
                "Abort: IMU cal=0; keep the car still and retry after calibration"
            )
            return False
        return True

    def enter_vw_mode(self):
        result = self.send_command(CMD_VW_MODE)
        if not result.can_continue:
            self.get_logger().warning(
                "Abort: VW mode was not accepted, so no motion command was sent"
            )
            return False
        return True

    def stop_car(self):
        self.send_command(CMD_STOP)

    def line_forward(self, line_v):
        if not self.ensure_imu_ready():
            return
        if not self.enter_vw_mode():
            return
        self.send_command(make_line_frame(line_v))

    def turn_angle(self, angle_deg, direction_name):
        self.get_logger().info(
            f"Start {direction_name}: $TURN,{format_float(angle_deg)}"
        )
        if not self.ensure_imu_ready():
            return
        if not self.enter_vw_mode():
            return
        self.send_command(make_turn_frame(angle_deg))

    def parse_message(self, data):
        fields = [field.strip() for field in data.split(",")]
        if not fields or not fields[0]:
            raise ValueError("empty data")

        try:
            command = int(fields[0])
        except ValueError as exc:
            raise ValueError("command must be an integer") from exc

        if command == 0:
            if len(fields) != 1:
                raise ValueError('stop command format must be "0"')
            return command, None

        if command == 1:
            if len(fields) > 2:
                raise ValueError('line command format must be "1" or "1,v"')
            if len(fields) == 1:
                line_v = DEFAULT_LINE_V
            else:
                if not fields[1]:
                    raise ValueError("line speed must not be empty")
                line_v = float(fields[1])
            if line_v < LINE_MIN_V or line_v > LINE_MAX_V:
                raise ValueError(
                    f"line speed must be in [{LINE_MIN_V}, {LINE_MAX_V}] m/s"
                )
            return command, line_v

        if command in (2, 3, 4):
            if len(fields) != 1:
                raise ValueError(f'command {command} format must be "{command}"')
            return command, None

        raise ValueError(f"unknown command {command}")

    def on_movement(self, msg):
        try:
            command, line_v = self.parse_message(msg.data)
        except ValueError as exc:
            self.get_logger().warning(f"Invalid message: {exc}")
            return

        # 离散命令会自行切换底盘模式，cmd_vel 流式状态作废，下次 cmd_vel 重新进 VW 模式
        self.vw_stream_active = False
        self.last_cmd_vel_time = None

        if command == 0:
            self.stop_car()
        elif command == 1:
            self.line_forward(line_v)
        elif command == 2:
            self.turn_angle(-90.0, "left 90 deg")
        elif command == 3:
            self.turn_angle(90.0, "right 90 deg")
        elif command == 4:
            self.turn_angle(180.0, "right 180 deg")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="ROS2 car_movement UART bridge v2.")
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=f"UART device path, default: {DEFAULT_PORT}",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help=f"UART baud rate, default: {DEFAULT_BAUD}",
    )
    parser.add_argument(
        "--chassis-timeout-ms",
        type=int,
        default=0,
        help="enable chassis-side comm timeout ($SET,TIMEOUT,1,ms); "
        "0 disables (default). Enable for autonomous cmd_vel tasks.",
    )
    # parse_known_args 保留 ROS2 自己的 --ros-args / remap 参数。
    return parser.parse_known_args(argv)


def main(argv=None):
    parsed_args, ros_args = parse_args(argv)
    rclpy.init(args=ros_args if ros_args else None)
    node = None
    exit_code = 0

    try:
        node = CarMovementV2Node(
            parsed_args.port,
            parsed_args.baud,
            chassis_timeout_ms=parsed_args.chassis_timeout_ms,
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        exit_code = 130
    except OSError as exc:
        print(f"ERROR: cannot open/use {parsed_args.port}: {exc}", file=sys.stderr)
        exit_code = 1
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        exit_code = 1
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
