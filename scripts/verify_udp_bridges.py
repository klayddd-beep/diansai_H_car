#!/usr/bin/env python3
"""
Black-box acceptance test for stream restart, ordering, and status redundancy.

Run fire_event_bridge and fire_link_bridge first, using the normal 3.0 s timeout.
This script does not change either UDP packet format.
"""

import socket
import struct
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


class BridgeVerifier(Node):
    def __init__(self):
        super().__init__("verify_udp_bridges")
        self.fire_messages = []
        self.telemetry_messages = []
        self.create_subscription(
            Float32MultiArray, "/fire_event",
            lambda msg: self.fire_messages.append(tuple(msg.data)), 10)
        self.create_subscription(
            Float32MultiArray, "/drone_telemetry",
            lambda msg: self.telemetry_messages.append(tuple(msg.data)), 10)
        self.status_publisher = self.create_publisher(
            String, "/fire_mission_status", 10)

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.05, deadline - time.monotonic()))

    def wait_for_count(self, messages, count, timeout=1.5):
        deadline = time.monotonic() + timeout
        while len(messages) < count and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if len(messages) != count:
            raise AssertionError(
                f"expected {count} messages, received {len(messages)}")


def receive_datagrams(sock, duration=0.5):
    packets = []
    deadline = time.monotonic() + duration
    sock.settimeout(0.05)
    while time.monotonic() < deadline:
        try:
            packets.append(sock.recvfrom(1024)[0])
        except socket.timeout:
            pass
    return packets


def main():
    rclpy.init()
    node = BridgeVerifier()
    fire_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    telemetry_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    status_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    status_rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    status_rx.bind(("127.0.0.1", 8890))
    try:
        node.spin_for(0.8)

        def send_fire(seq):
            fire_tx.sendto(
                struct.pack("<HHfff", 0xFC11, seq, 30.0, 25.0, 0.0),
                ("127.0.0.1", 8889))

        def send_telemetry(seq):
            telemetry_tx.sendto(
                struct.pack(
                    "<HBBHHIffffI", 0xF14E, 1, 2, seq, 0,
                    int(time.monotonic() * 1000) & 0xFFFFFFFF,
                    30.0, 25.0, 10.0, 18.0, 0),
                ("127.0.0.1", 8892))

        # First fire-report flight starts at zero, exactly like the aircraft.
        send_fire(0)
        node.wait_for_count(node.fire_messages, 1)

        # Continuous telemetry retains the original ordering rule: 3 and 4
        # are older than the accepted 5 and must both disappear.
        send_telemetry(5)
        node.wait_for_count(node.telemetry_messages, 1)
        send_telemetry(3)
        send_telemetry(4)
        node.spin_for(0.4)
        assert len(node.telemetry_messages) == 1, "telemetry ordering filter regressed"

        # A gap longer than the configured 3.0 s starts a new stream, so seq=0
        # must be accepted by both bridges.
        node.spin_for(3.2)
        send_fire(0)
        send_telemetry(0)
        node.wait_for_count(node.fire_messages, 2)
        node.wait_for_count(node.telemetry_messages, 2)
        assert int(node.fire_messages[-1][2]) == 0
        assert int(node.telemetry_messages[-1][5]) == 0

        # Check the same continuous-stream ordering rule on fire reports.
        send_fire(5)
        node.wait_for_count(node.fire_messages, 3)
        send_fire(3)
        send_fire(4)
        node.spin_for(0.4)
        assert len(node.fire_messages) == 3, "fire-event ordering filter regressed"

        # fire_event above teaches the bridge to reply to localhost:8890.
        node.status_publisher.publish(String(data="enroute"))
        node.spin_for(0.2)
        assert receive_datagrams(status_rx).count(b"enroute") == 1
        node.status_publisher.publish(String(data="done"))
        node.spin_for(0.3)
        assert receive_datagrams(status_rx).count(b"done") == 3
        node.status_publisher.publish(String(data="failed:test"))
        node.spin_for(0.3)
        assert receive_datagrams(status_rx).count(b"failed:test") == 3

        print("PASS: restart reset, ordering, and done/failed terminal x3")
    finally:
        fire_tx.close()
        telemetry_tx.close()
        status_rx.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
