#!/usr/bin/env python3
"""SR5E1E3 底盘诊断脚本:绕开 ROS,直接串口按调试手册的阶梯定位"$VW 不转"。

按手册顺序打三层,每层都把底盘返回原样打印,重点看 bat / fault / pwm / rpm:
  1) $GET,STATUS      —— 读电池电压 bat、故障码 fault、当前 mode/state
  2) $MODE,PWM + $PWM —— 开环直接给 PWM,跳过编码器/PID。这一层不转 = 纯硬件
                         (电源/接线/驱动使能),跟软件无关
  3) $MODE,VW + REPORT —— 闭环 v/w,边发边看 tgt/rpm/pwm 有没有响应

⚠️ 跑之前把车轮悬空!(手册 1.1.2)电机可能突然全速。
⚠️ 先停掉占用 /dev/ttyS6 的 ROS 桥(orangepi_to_carv2),否则串口打架。

用法:  python3 chassis_diag.py [--port /dev/ttyS6] [--baud 115200]
"""

import argparse
import os
import select
import termios
import time


def configure_uart(fd, baud):
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[3] = 0
    attrs[2] |= termios.CLOCAL | termios.CREAD
    attrs[2] &= ~termios.CSIZE
    attrs[2] |= termios.CS8
    attrs[2] &= ~termios.PARENB
    attrs[2] &= ~termios.CSTOPB
    if hasattr(termios, "CRTSCTS"):
        attrs[2] &= ~termios.CRTSCTS
    flag = getattr(termios, f"B{baud}")
    attrs[4] = flag
    attrs[5] = flag
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)


def read_for(fd, seconds):
    """收集 seconds 秒内到达的所有字节并解码。"""
    deadline = time.monotonic() + seconds
    chunks = []
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        r, _, _ = select.select([fd], [], [], remaining)
        if not r:
            continue
        try:
            data = os.read(fd, 1024)
        except BlockingIOError:
            continue
        if data:
            chunks.append(data)
    return b"".join(chunks).decode("ascii", errors="replace")


def send(fd, frame, wait=0.4):
    """发一帧(自动补 \\r\\n)并打印底盘返回。"""
    line = frame + "\r\n"
    os.write(fd, line.encode("ascii"))
    termios.tcdrain(fd)
    reply = read_for(fd, wait).strip()
    print(f"  TX {frame:<20} RX {reply!r}")
    return reply


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyS6")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    configure_uart(fd, args.baud)
    print(f"opened {args.port} @ {args.baud}\n")

    try:
        print("[1] 读状态(看 bat 电压 / fault 故障码 / mode / state):")
        send(fd, "$GET,STATUS", wait=0.6)
        send(fd, "$FAULT,GET", wait=0.6)   # 故障文字详情(LOWBAT/ENCODER/BLOCK/COMM)
        send(fd, "$GET,IMU", wait=0.6)     # 你要求:看 IMU 是否在线/cal;VW 其实不依赖它

        print("\n[2] 开环 PWM 测试(跳过编码器/PID;这层不转=纯硬件问题):")
        send(fd, "$MODE,PWM")
        send(fd, "$PWM,2000,2000")   # 低占空比正向,轮子应缓慢转
        print("    ...给 2s PWM,看轮子转不转...")
        time.sleep(2.0)
        send(fd, "$GET,STATUS", wait=0.6)   # 看 pwm_l/pwm_r 有没有输出
        send(fd, "$PWM,0,0")
        send(fd, "$STOP")

        print("\n[3] 闭环 VW 测试(边发边开周期回传,看 tgt/rpm/pwm 响应):")
        send(fd, "$MODE,VW")
        send(fd, "$SET,REPORT,1,200")   # 200ms 周期回传
        os.write(fd, b"$VW,0.2,0.0\r\n")
        termios.tcdrain(fd)
        print("    ...发 $VW,0.2,0 并连续读 3s 周期状态...")
        print(read_for(fd, 3.0).strip())
        send(fd, "$SET,REPORT,0,200")
        send(fd, "$STOP")
    finally:
        os.write(fd, b"$STOP\r\n")
        termios.tcdrain(fd)
        os.close(fd)
        print("\n done, port closed.")


if __name__ == "__main__":
    main()
