"""Run with: python3 -s -m unittest discover -s debug -p 'test_vision_odom.py'."""

from pathlib import Path
import sqlite3
import struct
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np

from vision_odom import Bag, cloud_points, rotation


class PointCloudTests(unittest.TestCase):
    def test_organized_cloud_padding_endianness_and_invalid_points(self):
        for big_endian in (False, True):
            endian = ">" if big_endian else "<"
            data = b"".join(struct.pack(endian + "fff", *point) for point in
                            ((1, 2, 3), (0, 0, 0))) + b"padding!"
            data += b"".join(struct.pack(endian + "fff", *point) for point in
                             ((4, 5, 6), (float("nan"), 7, 8))) + b"padding!"
            message = SimpleNamespace(
                height=2, width=2, row_step=32, point_step=12,
                fields=[SimpleNamespace(name=name, offset=i * 4, datatype=7)
                        for i, name in enumerate("xyz")], data=data, is_bigendian=big_endian)
            np.testing.assert_array_equal(cloud_points(message, 100), [[1, 2, 3], [4, 5, 6]])

    def test_livox_sampling_limit(self):
        message = SimpleNamespace(points=[SimpleNamespace(x=i, y=1, z=2) for i in range(101)])
        points = cloud_points(message, 10)
        self.assertLessEqual(len(points), 10)
        np.testing.assert_array_equal(points[0], [0, 1, 2])

    def test_quaternion_normalization(self):
        q = SimpleNamespace(x=0, y=0, z=2, w=0)
        np.testing.assert_allclose(rotation(q), np.diag([-1, -1, 1]))


class BagTests(unittest.TestCase):
    def test_split_bag_time_order_seek_and_read_only(self):
        from rclpy.serialization import serialize_message
        from std_msgs.msg import Bool

        with tempfile.TemporaryDirectory() as directory:
            for filename, tid, records in (
                    ("bag_0.db3", 4, [(300, True), (100, False)]),
                    ("bag_1.db3", 7, [(200, True)])):
                with sqlite3.connect(str(Path(directory) / filename)) as connection:
                    connection.execute("CREATE TABLE topics (id INTEGER, name TEXT, type TEXT, serialization_format TEXT)")
                    connection.execute("CREATE TABLE messages (id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB)")
                    connection.execute("INSERT INTO topics VALUES (?, '/state', 'std_msgs/msg/Bool', 'cdr')", (tid,))
                    for timestamp, value in records:
                        connection.execute("INSERT INTO messages(topic_id,timestamp,data) VALUES (?,?,?)",
                                           (tid, timestamp, serialize_message(Bool(data=value))))
            bag = Bag(directory)
            try:
                self.assertEqual(len(bag.topics), 1)
                topic = bag.topics[0]
                np.testing.assert_array_equal(topic.times, [100, 200, 300])
                self.assertIsNone(bag.latest(topic, 99))
                self.assertFalse(bag.latest(topic, 100).data)
                self.assertTrue(bag.latest(topic, 250).data)
                self.assertFalse(bag.latest(topic, 150).data)
                self.assertEqual(bag.count(topic, 250), 2)
                with self.assertRaises(sqlite3.OperationalError):
                    bag.connections[0].execute("DELETE FROM messages")
            finally:
                bag.close()

    def test_missing_bag(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                Bag(directory)


if __name__ == "__main__":
    unittest.main()
