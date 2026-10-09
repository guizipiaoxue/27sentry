"""Exercise Point-LIO's keyframe output with stationary synthetic sensors."""

import os
import signal
import subprocess
import time
import unittest

import rclpy
from loop_closure.msg import Keyframe
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class KeyframeOutputTest(unittest.TestCase):
    def test_first_keyframe_uses_odom_frame_and_world_cloud(self):
        rclpy.init()
        node = Node('point_lio_keyframe_test')
        imu_pub = node.create_publisher(Imu, '/test_plio/imu', qos_profile_sensor_data)
        cloud_pub = node.create_publisher(
            point_cloud2.PointCloud2, '/test_plio/cloud', 20)
        keyframes = []
        keyframe_clouds = []
        node.create_subscription(
            Keyframe, '/test_plio/keyframe', keyframes.append, 20)
        node.create_subscription(
            point_cloud2.PointCloud2, '/test_plio/keyframe_cloud',
            keyframe_clouds.append, 20)
        params = [
            'terminal.enabled:=false',
            'imu.initialization_samples:=20',
            'mapping.initialization_points:=20',
            'mapping.max_tracking_points:=128',
            'mapping.point_time_bin_ms:=10.0',
            'preprocess.point_filter_num:=1',
            'preprocess.blind:=0.1',
            'filter_size_surf:=0.3',
        ]
        command = [
            'ros2', 'run', 'plio', 'point_lio', '--ros-args',
            '-r', 'imu:=/test_plio/imu',
            '-r', 'pointcloud:=/test_plio/cloud',
            '-r', 'keyframe:=/test_plio/keyframe',
            '-r', 'kf_cloud:=/test_plio/keyframe_cloud',
        ]
        for param in params:
            command.extend(['-p', param])
        process = subprocess.Popen(
            command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            env={**os.environ, 'ROS_LOG_DIR': '/tmp'})
        try:
            fields = [
                PointField(name=name, offset=4 * index,
                           datatype=PointField.FLOAT32, count=1)
                for index, name in enumerate(('x', 'y', 'z', 'intensity', 'time'))
            ]
            points = [
                (x * 0.3, y * 0.3, 0.0, 1.0, ((x + 5) * 11 + y + 5) / 120.0 * 0.09)
                for x in range(-5, 6) for y in range(-5, 6)
            ]
            start = time.monotonic()
            last_cloud = start
            while time.monotonic() - start < 7.0 and not (keyframes and keyframe_clouds):
                stamp = node.get_clock().now().to_msg()
                imu = Imu()
                imu.header.stamp = stamp
                imu.linear_acceleration.z = 9.80665
                imu_pub.publish(imu)
                if time.monotonic() - start > 0.5 and time.monotonic() - last_cloud > 0.2:
                    header = Header(stamp=stamp, frame_id='gimbal')
                    cloud_pub.publish(point_cloud2.create_cloud(header, fields, points))
                    last_cloud = time.monotonic()
                rclpy.spin_once(node, timeout_sec=0.01)

            self.assertIsNone(process.poll(), 'Point-LIO exited during the test')
            self.assertTrue(keyframes, 'Point-LIO did not publish its first keyframe')
            self.assertTrue(keyframe_clouds, 'Point-LIO did not publish its KD-tree cloud')
            self.assertEqual(keyframes[0].id, 0)
            self.assertEqual(keyframes[0].cloud.header.frame_id, 'odom')
            self.assertGreater(keyframes[0].cloud.width, 20)
            self.assertAlmostEqual(keyframes[0].pose.orientation.w, 1.0, delta=0.01)
            self.assertEqual(keyframe_clouds[0], keyframes[0].cloud)
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
