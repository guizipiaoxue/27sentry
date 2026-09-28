"""Pseudo-terminal integration check for the ROS USB CDC bridge."""

import os
import pty
import select
import signal
import struct
import subprocess
import time
import unittest

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.node import Node
from receiver.msg import ChassisState, GimbalState, UplinkFrame
from std_msgs.msg import UInt8MultiArray


def uplink_crc(data):
    crc = 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if crc & 1 else crc >> 1
    return crc


def read_frames(fd, timeout):
    end = time.monotonic() + timeout
    buffer = bytearray()
    frames = []
    while time.monotonic() < end:
        ready, _, _ = select.select([fd], [], [], 0.02)
        if ready:
            buffer.extend(os.read(fd, 512))
        while len(buffer) >= 13:
            if buffer[0] != 0x21:
                del buffer[0]
                continue
            frames.append(bytes(buffer[:13]))
            del buffer[:13]
    return frames


def coord_crc(data):
    crc = 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def path_crc(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc


def read_mixed_frames(fd, timeout):
    lengths = {0: 13, 1: 6, 2: 64, 3: 36, 4: 17, 5: 26}
    end = time.monotonic() + timeout
    buffer = bytearray()
    frames = []
    while time.monotonic() < end:
        ready, _, _ = select.select([fd], [], [], 0.02)
        if ready:
            buffer.extend(os.read(fd, 512))
        while len(buffer) >= 2:
            if buffer[0] != 0x21 or buffer[1] not in lengths:
                del buffer[0]
                continue
            length = lengths[buffer[1]]
            if len(buffer) < length:
                break
            frames.append(bytes(buffer[:length]))
            del buffer[:length]
    return frames


class BridgeTest(unittest.TestCase):
    def test_bridge(self):
        master, slave = pty.openpty()
        path = os.ttyname(slave)
        os.close(slave)
        rclpy.init()
        node = Node('bridge_test')
        raw = []
        gimbal = []
        chassis = []
        node.create_subscription(UplinkFrame, '/base/uplink/raw', raw.append, 10)
        node.create_subscription(GimbalState, '/base/gimbal/state', gimbal.append, 10)
        node.create_subscription(ChassisState, '/base/chassis/state', chassis.append, 10)
        velocity_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        path_pub = node.create_publisher(UInt8MultiArray, '/base/map_path_payload', 10)
        coordinate_pub = node.create_publisher(PointStamped, '/base/sentry_coordinate', 10)
        common = ['--ros-args', '-p', f'device:={path}', '-p', 'reconnect_period_ms:=50']
        receiver = subprocess.Popen(['ros2', 'run', 'receiver', 'receiver_node', *common])
        sender = subprocess.Popen(['ros2', 'run', 'sender', 'sender_node', *common])
        try:
            time.sleep(1.0)
            payload = struct.pack('<ffHBB', 12.5, -4.0, 42, 2, 10)
            frame = b'\x21\x00' + payload
            state6 = b'\x21\x06' + struct.pack('<Hhhhhh', 90, -12, 23, 5, 80, -40)
            for wire_data, received in (
                    (b'noise\x21\x00' + bytes(12) + b'\x00' +
                     frame + bytes([uplink_crc(frame)]), gimbal),
                    (state6 + bytes([uplink_crc(state6)]), chassis)):
                deadline = time.monotonic() + 3.0
                next_send = 0.0
                while time.monotonic() < deadline and not received:
                    if time.monotonic() >= next_send:
                        os.write(master, wire_data)
                        next_send = time.monotonic() + 0.25
                    rclpy.spin_once(node, timeout_sec=0.05)
            self.assertTrue(raw and all(message.type_id in (0, 6) for message in raw))
            self.assertIn(0, [message.type_id for message in raw])
            self.assertIn(6, [message.type_id for message in raw])
            self.assertAlmostEqual(gimbal[-1].yaw_deg, 12.5)
            self.assertAlmostEqual(gimbal[-1].pitch_deg, -4.0)
            self.assertAlmostEqual(chassis[-1].velocity_x_mps, 0.8)
            self.assertAlmostEqual(chassis[-1].velocity_y_mps, -0.4)

            command = Twist()
            command.linear.x = 1.0
            command.linear.y = -0.5
            for _ in range(4):
                velocity_pub.publish(command)
                rclpy.spin_once(node, timeout_sec=0.05)
            frames = read_frames(master, 0.2)
            self.assertTrue(any(frame[2] == 50 and frame[3] == 231 for frame in frames))
            self.assertTrue(any(frame[2] == 0 and frame[3] == 0 for frame in
                                read_frames(master, 0.3)))

            # Verify both downlink CRC variants and the path fragment layout.
            payload = bytes([1] + list(range(1, 105)))
            path_msg = UInt8MultiArray(data=list(payload))
            coordinate = PointStamped()
            coordinate.header.stamp = node.get_clock().now().to_msg()
            coordinate.point.x = 1.23
            coordinate.point.y = -4.56
            for _ in range(3):
                path_pub.publish(path_msg)
                coordinate_pub.publish(coordinate)
                rclpy.spin_once(node, timeout_sec=0.05)
            packets = read_mixed_frames(master, 0.5)
            fragments = [frame for frame in packets if frame[1] == 2]
            coordinates = [frame for frame in packets if frame[1] == 4]
            self.assertGreaterEqual(len(fragments), 2)
            first, second = fragments[:2]
            self.assertEqual((first[2], first[3:6]), (second[2], b'\x00\x02\x38'))
            self.assertEqual(second[3:6], b'\x01\x02\x31')
            self.assertEqual(first[6:62] + second[6:55], payload)
            self.assertEqual(struct.unpack_from('<H', first, 62)[0], path_crc(first[:62]))
            self.assertEqual(struct.unpack_from('<H', second, 62)[0], path_crc(second[:62]))
            self.assertTrue(any(struct.unpack_from('<hh', frame, 2) == (123, -456)
                                and frame[16] == coord_crc(frame[:16])
                                for frame in coordinates))
        finally:
            for process in (receiver, sender):
                process.send_signal(signal.SIGINT)
            for process in (receiver, sender):
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
            node.destroy_node()
            rclpy.shutdown()
            os.close(master)


if __name__ == '__main__':
    unittest.main()
