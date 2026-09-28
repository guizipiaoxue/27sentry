"""ROS integration check for MPC path tracking and goal stopping."""

import math
import signal
import subprocess
import time
import unittest

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from receiver.msg import ChassisState
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


class MpcControlTest(unittest.TestCase):
    def test_tracking_and_stop(self):
        rclpy.init()
        node = Node('mpc_control_test')
        broadcaster = TransformBroadcaster(node)
        path_pub = node.create_publisher(Path, '/sPath', 10)
        chassis_pub = node.create_publisher(ChassisState, '/base/chassis/state', 10)
        commands = []
        reached = []
        node.create_subscription(Twist, '/cmd_vel', commands.append, 10)
        node.create_subscription(Bool, '/ly/navi/reached', reached.append, 10)
        process = subprocess.Popen(['ros2', 'run', 'test_control', 'mpc_control'])
        try:
            path = Path()
            path.header.frame_id = 'map'
            for x in (0.0, 2.0):
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = x
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)

            # Keep TF and velocity telemetry fresh throughout both phases.
            def tick(x):
                stamp = node.get_clock().now().to_msg()
                transform = TransformStamped()
                transform.header.stamp = stamp
                transform.header.frame_id = 'map'
                transform.child_frame_id = 'base_link'
                transform.transform.translation.x = x
                transform.transform.rotation.w = 1.0
                broadcaster.sendTransform(transform)
                chassis = ChassisState()
                chassis.header.stamp = stamp
                chassis.header.frame_id = 'base_link'
                chassis_pub.publish(chassis)
                rclpy.spin_once(node, timeout_sec=0.04)

            start = time.monotonic()
            while time.monotonic() - start < 2.0:
                path_pub.publish(path)
                tick(0.0)
            moving = [cmd for cmd in commands if cmd.linear.x > 0.01]
            self.assertTrue(moving, 'MPC did not command motion along the path')
            self.assertTrue(all(math.hypot(cmd.linear.x, cmd.linear.y) <= 1.001
                                for cmd in commands))

            commands.clear()
            reached.clear()
            start = time.monotonic()
            while time.monotonic() - start < 0.6:
                tick(2.0)
            self.assertTrue(any(message.data for message in reached))
            self.assertTrue(commands and all(abs(cmd.linear.x) < 1e-6 and
                                             abs(cmd.linear.y) < 1e-6
                                             for cmd in commands[-3:]))
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
