"""Extract the recorded fused sensors for the C++ Point-LIO offline replay."""

import argparse
import sqlite3
import struct
from pathlib import Path

import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    db = sqlite3.connect(f"file:{args.bag}?mode=ro", uri=True)
    topics = dict(db.execute("SELECT name, id FROM topics"))
    with (args.output / "imu.bin").open("wb") as output:
        for (data,) in db.execute(
            "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (topics["/gimbal/imu_fused"],),
        ):
            message = deserialize_message(data, Imu)
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            gyro = message.angular_velocity
            accel = message.linear_acceleration
            output.write(struct.pack("<7d", stamp, gyro.x, gyro.y, gyro.z,
                                     accel.x, accel.y, accel.z))

    with (args.output / "clouds.bin").open("wb") as output:
        for (data,) in db.execute(
            "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (topics["/gimbal/cloud_fused"],),
        ):
            message = deserialize_message(data, PointCloud2)
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            fields = {field.name: field for field in message.fields}
            dtype = np.dtype({
                "names": ["x", "y", "z", "intensity", "time"],
                "formats": ["<f4"] * 5,
                "offsets": [fields[name].offset for name in
                            ("x", "y", "z", "intensity", "time")],
                "itemsize": message.point_step,
            })
            points = np.frombuffer(message.data, dtype=dtype).copy()
            packed = np.column_stack([points[name] for name in
                                      ("x", "y", "z", "intensity", "time")])
            packed = np.ascontiguousarray(packed, dtype="<f4")
            output.write(struct.pack("<dI", stamp, len(packed)))
            output.write(packed.tobytes())

    config_path = Path(__file__).resolve().parents[1] / "config/point_lio.yaml"
    config = yaml.safe_load(config_path.read_text())
    parameters = config["/**"]["ros__parameters"]
    rotation = parameters["imu.correction_rotation"]
    lever = parameters["imu.effective_lever_arm"]
    with (args.output / "imu_conditioning.txt").open("w") as output:
        for row in range(3):
            output.write(" ".join(map(str, rotation[3 * row:3 * row + 3])) + "\n")
        output.write(" ".join(map(str, lever)) + "\n")


if __name__ == "__main__":
    main()
