#!/usr/bin/env python3
"""Browse ROS 2 SQLite bags without replaying messages into the ROS graph.

source /opt/ros/humble/setup.bash
source livox/install/setup.bash
python3 -s debug/vision_odom.py /path/to/bag.db3
python3 -s debug/vision_odom.py /path/to/bag --time 30 --snapshot /tmp/bag.png
"""

import argparse
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
import os
import sqlite3
import sys
import textwrap
import time
from zoneinfo import ZoneInfo

import numpy as np


DEFAULT_BAG = (Path(__file__).resolve().parents[1] / "rosbag" /
               "20261001_133441_plio/20261001_133441_plio_0.db3")


@dataclass
class Topic:
    id: int
    name: str
    type: str
    serialization: str
    records: list = field(default_factory=list)
    times: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.int64))
    ids: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.int64))
    message_class: object = None
    error: str = ""
    index: int = -1
    message: object = None


class Bag:
    def __init__(self, path):
        path = Path(path).expanduser().resolve()
        self.files = sorted(path.glob("*.db3")) if path.is_dir() else [path]
        if not self.files or any(not p.is_file() or p.suffix != ".db3" for p in self.files):
            raise ValueError(f"No .db3 bag found: {path}")
        self.connections = []
        self.topics = []
        by_name = {}
        try:
            for file_index, filename in enumerate(self.files):
                connection = sqlite3.connect(filename.as_uri() + "?mode=ro", uri=True)
                self.connections.append(connection)
                local = {}
                for tid, name, kind, serialization in connection.execute(
                        "SELECT id, name, type, serialization_format FROM topics ORDER BY id"):
                    if name not in by_name:
                        by_name[name] = Topic(len(by_name), name, kind, serialization)
                        self.topics.append(by_name[name])
                    topic = by_name[name]
                    if (topic.type, topic.serialization) != (kind, serialization):
                        raise ValueError(f"Conflicting topic definitions: {name}")
                    local[tid] = topic
                # Only index timestamps and row IDs; payloads remain on disk.
                for mid, tid, stamp in connection.execute(
                        "SELECT id, topic_id, timestamp FROM messages ORDER BY timestamp, id"):
                    local[tid].records.append((stamp, file_index, mid))
            for topic in self.topics:
                topic.records.sort()
                records = np.asarray(topic.records, dtype=np.int64).reshape(-1, 3)
                topic.times = records[:, 0].copy()
                topic.ids = records[:, 1:].copy()
                topic.records.clear()
            populated = [t for t in self.topics if len(t.times)]
            if not populated:
                raise ValueError("Bag contains no messages")
            self.start = min(int(t.times[0]) for t in populated)
            self.end = max(int(t.times[-1]) for t in populated)
            self.duration = (self.end - self.start) / 1e9
        except Exception:
            self.close()
            raise

    def close(self):
        for connection in self.connections:
            connection.close()

    def timestamp(self, seconds):
        return min(self.end, max(self.start, self.start + round(seconds * 1e9)))

    def count(self, topic, stamp):
        return int(np.searchsorted(topic.times, stamp, side="right"))

    def read(self, topic, index):
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message

        if topic.serialization != "cdr":
            raise ValueError(f"Unsupported serialization: {topic.serialization}")
        if topic.message_class is None:
            topic.message_class = get_message(topic.type)
        file_index, mid = topic.ids[index]
        data = self.connections[int(file_index)].execute(
            "SELECT data FROM messages WHERE id=?", (int(mid),)).fetchone()[0]
        return deserialize_message(data, topic.message_class)

    def latest(self, topic, stamp):
        index = self.count(topic, stamp) - 1
        if index != topic.index:
            topic.index = index
            topic.message = None
            if index >= 0:
                try:
                    topic.message = self.read(topic, index)
                    topic.error = ""
                except Exception as exc:
                    topic.error = f"{type(exc).__name__}: {exc}"
        return topic.message

    def info(self):
        print(f"Duration: {self.duration:.3f} s; messages: {sum(len(t.times) for t in self.topics):,}")
        for topic in self.topics:
            print(f"{topic.name:28} {len(topic.times):8,}  {topic.type}")


def cloud_points(message, max_points):
    if hasattr(message, "points"):
        points = message.points
        stride = max(1, (len(points) + max_points - 1) // max_points)
        xyz = np.asarray([(p.x, p.y, p.z) for p in points[::stride]], dtype=float).reshape(-1, 3)
    else:
        fields = {f.name: f for f in message.fields}
        if not all(axis in fields for axis in "xyz"):
            raise ValueError("PointCloud2 has no x/y/z fields")
        formats = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}
        endian = ">" if message.is_bigendian else "<"
        dtype = np.dtype({
            "names": list("xyz"),
            "formats": [endian + formats[fields[a].datatype] for a in "xyz"],
            "offsets": [fields[a].offset for a in "xyz"],
            "itemsize": message.point_step,
        })
        # Respect row padding in organized PointCloud2 messages.
        points = np.ndarray((message.height, message.width), dtype=dtype,
                            buffer=memoryview(message.data),
                            strides=(message.row_step, message.point_step)).reshape(-1)
        stride = max(1, (len(points) + max_points - 1) // max_points)
        xyz = np.column_stack([points[a][::stride] for a in "xyz"])
    return xyz[np.isfinite(xyz).all(axis=1) & np.any(xyz != 0, axis=1)]


def rotation(quaternion):
    q = np.asarray([quaternion.x, quaternion.y, quaternion.z, quaternion.w], dtype=float)
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.eye(3)
    x, y, z, w = q / norm
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def preview(message, max_lines=18):
    lines = []

    def walk(value, name="", depth=0):
        if len(lines) >= max_lines:
            return
        indent = "  " * depth
        if hasattr(value, "get_fields_and_field_types"):
            keys = list(value.get_fields_and_field_types())
            if set(keys) == {"sec", "nanosec"}:
                lines.append(f"{indent}{name}: {value.sec}.{value.nanosec:09d}")
                return
            if set(keys) in ({"x", "y", "z"}, {"x", "y", "z", "w"}):
                components = ", ".join(f"{key}={getattr(value, key):.6g}" for key in keys)
                lines.append(f"{indent}{name}: {components}")
                return
            if name:
                lines.append(indent + name + ":")
            for key in keys:
                walk(getattr(value, key), key, depth + bool(name))
        elif isinstance(value, (list, tuple, np.ndarray)) or hasattr(value, "typecode"):
            lines.append(f"{indent}{name}: [{len(value)} values]")
            if name != "data":
                for i, item in enumerate(value[:2]):
                    walk(item, str(i), depth + 1)
        else:
            val = f"{value:.6g}" if isinstance(value, float) else str(value)
            lines.extend(textwrap.wrap(f"{indent}{name}: {val}", width=52,
                                       subsequent_indent=indent + "  ") or [""])

    walk(message)
    if len(lines) >= max_lines:
        lines = lines[:max_lines - 1] + ["..."]
    return "\n".join(lines)


class Viewer:
    def __init__(self, bag, args):
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button, RadioButtons, Slider

        self.plt, self.bag, self.args = plt, bag, args
        self.seconds = min(args.time, bag.duration)
        self.playing = not args.paused
        self.rate = args.rate
        self.last_tick = time.monotonic()
        self.imu_samples = deque()
        self.imu_index = -1
        self.cloud_index = None
        self.cloud_error = ""
        self.cloud = np.empty((0, 3))
        self.first_cloud = True
        self.trajectory = np.empty((0, 3))
        self.trajectory_frame = ""
        self.odom_samples = deque(maxlen=10000)
        self.path_index = None
        self.tf_artists = []
        self.selected = next((t for t in bag.topics if t.name == "/cloud_registered"), bag.topics[0])
        self.cloud_topic = next((t for t in bag.topics if self.is_cloud(t)), None)
        if self.is_cloud(self.selected):
            self.cloud_topic = self.selected
        imus = [t for t in bag.topics if t.type == "sensor_msgs/msg/Imu"]
        self.imu_topic = next((t for t in imus if "fused" in t.name), imus[0] if imus else None)

        plt.rcParams.update({"font.size": 9, "axes.spines.top": False,
                             "axes.spines.right": False, "figure.facecolor": "#fafbfc"})
        self.figure = plt.figure(figsize=(16, 10))
        self.figure.canvas.manager.set_window_title("ROS bag data viewer")
        self.clock = self.figure.text(.025, .974, "", ha="left", va="top", fontsize=13)
        self.figure.text(.975, .971, bag.files[0].name, ha="right", fontsize=10, color="#59636e")
        self.spatial = self.figure.add_axes([.035, .49, .54, .41], projection="3d")
        self.scatter = self.spatial.scatter([], [], [], s=1, c="#158b87", alpha=.65)
        self.world_path, = self.spatial.plot([], [], [], color="#d35c46", linewidth=1.5)
        self.spatial.set(xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)")
        self.spatial.set_box_aspect((1, 1, .65))
        self.accel = self.figure.add_axes([.66, .72, .31, .17])
        self.gyro = self.figure.add_axes([.66, .49, .31, .17])
        self.imu_lines = []
        for axis, label in ((self.accel, "Acceleration (m/s^2)"), (self.gyro, "Angular velocity (rad/s)")):
            self.imu_lines.append([axis.plot([], [], color=c, label=n, lw=1)[0]
                                   for n, c in zip("xyz", ("#d35c46", "#158b87", "#626ac5"))])
            axis.set_ylabel(label)
            axis.grid(alpha=.2)
            axis.legend(loc="upper right", ncol=3, fontsize=8)
        self.gyro.set_xlabel("Bag elapsed (s)")
        topic_axis = self.figure.add_axes([.025, .14, .275, .29], facecolor="#fafbfc")
        topic_axis.set_title("Topics / received / total", loc="left", fontsize=10)
        self.radio = RadioButtons(topic_axis, [t.name for t in bag.topics],
                                  active=bag.topics.index(self.selected))
        for label in self.radio.labels:
            label.set_fontsize(8)
        self.radio.on_clicked(self.select)
        self.xy = self.figure.add_axes([.365, .14, .215, .29])
        self.xy.set(xlabel="X (m)", ylabel="Y (m)", title="Odometry / path")
        self.xy.set_aspect("equal", adjustable="datalim")
        self.xy.grid(alpha=.2)
        self.xy_path, = self.xy.plot([], [], color="#d35c46", lw=1.5)
        self.xy_current, = self.xy.plot([], [], "o", color="#158b87", markersize=5)
        detail_axis = self.figure.add_axes([.66, .14, .315, .29])
        detail_axis.axis("off")
        self.detail = detail_axis.text(0, 1, "", va="top", fontfamily="monospace", fontsize=8)
        self.slider = Slider(self.figure.add_axes([.09, .075, .81, .024]),
                             "Time (s)", 0, max(bag.duration, .001), valinit=self.seconds)
        self.slider.on_changed(self.seek)
        self.buttons = []
        for rect, label, callback in (
                ([.025, .015, .075, .033], "Pause" if self.playing else "Play", self.toggle),
                ([.11, .015, .055, .033], "< 5s", lambda _: self.seek(self.seconds - 5)),
                ([.175, .015, .055, .033], "5s >", lambda _: self.seek(self.seconds + 5)),
                ([.25, .015, .04, .033], "-", lambda _: self.speed(.5)),
                ([.355, .015, .04, .033], "+", lambda _: self.speed(2)),
                ([.415, .015, .08, .033], "Fit view", lambda _: self.fit()),
                ([.515, .015, .08, .033], "Save PNG", self.save)):
            button = Button(self.figure.add_axes(rect), label)
            button.on_clicked(callback)
            self.buttons.append(button)
        self.speed_text = self.figure.text(.321, .031, f"{self.rate:g}x", ha="center", va="center")
        self.status = self.figure.text(.62, .03, "", fontsize=8, color="#59636e")
        self.figure.canvas.mpl_connect("key_press_event", self.key)
        self.figure.canvas.mpl_connect("close_event", lambda _: self.timer.stop())
        self.timer = self.figure.canvas.new_timer(interval=round(1000 / args.fps))
        self.timer.add_callback(self.tick)
        self.update(reset=True)
        self.fit()

    @staticmethod
    def is_cloud(topic):
        return topic.type == "sensor_msgs/msg/PointCloud2" or topic.type.endswith("/CustomMsg")

    def select(self, label):
        self.selected = next(t for t in self.bag.topics if label == t.name or label.startswith(t.name + "  "))
        if self.is_cloud(self.selected):
            self.cloud_topic = self.selected
            self.cloud_index = None
        if self.selected.type == "sensor_msgs/msg/Imu":
            self.imu_topic = self.selected
            self.imu_index = -1
            self.imu_samples.clear()
        self.update()
        self.fit()

    def seek(self, value):
        self.seconds = min(self.bag.duration, max(0, float(value)))
        self.last_tick = time.monotonic()
        self.update(reset=True)
        self.set_slider()
        self.fit()

    def set_slider(self):
        self.slider.eventson = False
        self.slider.set_val(self.seconds)
        self.slider.eventson = True

    def toggle(self, _=None):
        if self.seconds >= self.bag.duration and not self.playing:
            self.seek(0)
        self.playing = not self.playing
        self.buttons[0].label.set_text("Pause" if self.playing else "Play")
        self.last_tick = time.monotonic()
        self.figure.canvas.draw_idle()

    def speed(self, factor):
        self.rate = min(32, max(.125, self.rate * factor))
        self.speed_text.set_text(f"{self.rate:g}x")
        self.last_tick = time.monotonic()
        self.figure.canvas.draw_idle()

    def key(self, event):
        if event.key == " ":
            self.toggle()
        elif event.key in ("left", "right"):
            self.seek(self.seconds + (5 if event.key == "right" else -5))

    def tick(self):
        now = time.monotonic()
        if self.playing:
            self.seconds = min(self.bag.duration, self.seconds + (now - self.last_tick) * self.rate)
            self.update()
            self.set_slider()
            if self.seconds >= self.bag.duration:
                self.toggle()
        self.last_tick = now

    def update_imu(self, stamp, reset):
        topic = self.imu_topic
        if topic is None:
            return
        stop = self.bag.count(topic, stamp)
        start = self.bag.count(topic, stamp - round(self.args.history * 1e9))
        if reset:
            self.imu_samples.clear()
            self.imu_index = -1
        start = max(start, self.imu_index + 1)
        # Bound rendering cost when seeking or playing at high speed.
        stride = max(1, (stop - start + 1499) // 1500)
        for i in range(start, stop, stride):
            try:
                m = self.bag.read(topic, i)
                a, g = m.linear_acceleration, m.angular_velocity
                self.imu_samples.append(((int(topic.times[i]) - self.bag.start) / 1e9,
                                         a.x, a.y, a.z, g.x, g.y, g.z))
            except Exception as exc:
                topic.error = f"{type(exc).__name__}: {exc}"
                break
        self.imu_index = stop - 1
        while self.imu_samples and self.imu_samples[0][0] < self.seconds - self.args.history:
            self.imu_samples.popleft()
        values = np.asarray(self.imu_samples, dtype=float).reshape(-1, 7)
        for group, axis in enumerate((self.accel, self.gyro)):
            for component, line in enumerate(self.imu_lines[group]):
                line.set_data(values[:, 0], values[:, 1 + group * 3 + component])
            axis.set_xlim(max(0, self.seconds - self.args.history), max(self.seconds, .1))
            axis.relim()
            axis.autoscale_view(scalex=False)
        self.accel.set_title(topic.name, fontsize=10, loc="left")

    def update(self, reset=False):
        stamp = self.bag.timestamp(self.seconds)
        if reset:
            self.odom_samples.clear()
            self.xy_current.set_data([], [])
            self.path_index = None
            self.trajectory = np.empty((0, 3))
            self.trajectory_frame = ""
        for topic, label in zip(self.bag.topics, self.radio.labels):
            message = self.bag.latest(topic, stamp)
            label.set_text(f"{topic.name}  {self.bag.count(topic, stamp):,}/{len(topic.times):,}")
            label.set_color("#b23b35" if topic.error else "#252d34")
            if message is None:
                continue
            if topic.type == "nav_msgs/msg/Path" and self.path_index != (topic.id, topic.index):
                self.trajectory = np.asarray([(p.pose.position.x, p.pose.position.y, p.pose.position.z)
                                              for p in message.poses], dtype=float).reshape(-1, 3)
                self.trajectory_frame = message.header.frame_id
                self.path_index = (topic.id, topic.index)
            elif topic.type == "nav_msgs/msg/Odometry":
                p = message.pose.pose.position
                if not self.odom_samples or self.odom_samples[-1][0] != (topic.id, topic.index):
                    self.odom_samples.append(((topic.id, topic.index), (p.x, p.y, p.z)))
                self.xy_current.set_data([p.x], [p.y])
                if self.path_index is None:
                    self.trajectory_frame = message.header.frame_id
        if self.path_index is None:
            self.trajectory = np.asarray([p for _, p in self.odom_samples], dtype=float).reshape(-1, 3)
        self.xy_path.set_data(self.trajectory[:, 0], self.trajectory[:, 1])
        self.xy.relim()
        self.xy.autoscale_view()
        self.xy.set_title(f"Odometry / path [{self.trajectory_frame or '-'}]", fontsize=10)

        cloud_frame = ""
        if self.cloud_topic:
            topic = self.cloud_topic
            message = topic.message
            cloud_frame = message.header.frame_id if message is not None else ""
            if self.cloud_index != (topic.id, topic.index):
                self.cloud = np.empty((0, 3))
                self.cloud_error = ""
                if message is not None:
                    try:
                        self.cloud = cloud_points(message, self.args.max_points)
                    except Exception as exc:
                        self.cloud_error = str(exc)
                self.cloud_index = (topic.id, topic.index)
                self.scatter._offsets3d = tuple(self.cloud[:, i] for i in range(3))
                if len(self.cloud) and self.first_cloud:
                    self.first_cloud = False
                    self.fit()
            self.spatial.set_title(f"{topic.name} [{cloud_frame or '-'}] | {len(self.cloud):,} displayed points", fontsize=10)
        show_world = not cloud_frame or cloud_frame == self.trajectory_frame
        points = self.trajectory if show_world else np.empty((0, 3))
        self.world_path.set_data_3d(*points.T)
        for artist in self.tf_artists:
            artist.remove()
        self.tf_artists.clear()
        for topic in self.bag.topics:
            if topic.type != "tf2_msgs/msg/TFMessage" or topic.message is None:
                continue
            for transform in topic.message.transforms:
                if transform.header.frame_id != (cloud_frame or self.trajectory_frame):
                    continue
                p = transform.transform.translation
                basis = rotation(transform.transform.rotation)
                for i, color in enumerate(("#d35c46", "#158b87", "#626ac5")):
                    self.tf_artists.append(self.spatial.quiver(p.x, p.y, p.z, *basis[:, i],
                                                              length=.8, color=color))
        self.update_imu(stamp, reset)
        absolute = datetime.fromtimestamp(stamp // 1_000_000_000, ZoneInfo(self.args.timezone))
        self.clock.set_text(f"{absolute:%Y-%m-%d %H:%M:%S}.{stamp % 1_000_000_000 // 1_000_000:03d} "
                            f"({self.args.timezone})    {self.seconds:.2f} / {self.bag.duration:.2f} s")
        topic = self.selected
        last_time = ((int(topic.times[topic.index]) - self.bag.start) / 1e9
                     if topic.index >= 0 else None)
        heading = f"{topic.name}\n{topic.type}\n"
        heading += f"Bag time: {last_time:.3f} s\n\n" if last_time is not None else "\n"
        content = topic.error or (preview(topic.message) if topic.message is not None else "No message yet")
        self.detail.set_text(heading + content)
        errors = sum(bool(t.error) for t in self.bag.topics)
        self.status.set_text(self.cloud_error or (f"{errors} topic decode error(s)" if errors else ""))
        self.figure.canvas.draw_idle()

    def fit(self):
        arrays = [self.cloud]
        if self.trajectory_frame and (self.cloud_topic is None or self.cloud_topic.message is None or
                                     self.cloud_topic.message.header.frame_id == self.trajectory_frame):
            arrays.append(self.trajectory)
        points = np.concatenate(arrays)
        points = points[np.isfinite(points).all(axis=1)]
        if len(points):
            low, high = points.min(axis=0), points.max(axis=0)
            center = (low + high) / 2
            radius = max(float(np.max(high - low)) / 2, 1) * 1.05
            for setter, c in zip((self.spatial.set_xlim, self.spatial.set_ylim, self.spatial.set_zlim), center):
                setter(c - radius, c + radius)
        self.figure.canvas.draw_idle()

    def save(self, _=None, filename=None):
        filename = Path(filename) if filename else Path.cwd() / f"bag_{self.seconds:.3f}s.png"
        filename.parent.mkdir(parents=True, exist_ok=True)
        self.figure.savefig(filename, dpi=140)
        print(f"Saved: {filename.resolve()}")

    def show(self):
        self.last_tick = time.monotonic()
        self.timer.start()
        self.plt.show()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("path", nargs="?", type=Path, default=DEFAULT_BAG, help=".db3 file or bag directory")
    parser.add_argument("--time", type=float, default=0, help="Start at elapsed bag seconds")
    parser.add_argument("--rate", type=float, default=1, help="Playback speed, e.g. 0.5 or 2")
    parser.add_argument("--fps", type=float, default=10, help="Maximum GUI refresh rate")
    parser.add_argument("--history", type=float, default=20, help="IMU history window in seconds")
    parser.add_argument("--max-points", type=int, default=12000, help="Display limit per cloud frame")
    parser.add_argument("--timezone", default="Asia/Shanghai")
    parser.add_argument("--paused", action="store_true")
    parser.add_argument("--info", action="store_true", help="Print topic inventory without opening GUI")
    parser.add_argument("--snapshot", type=Path, help="Save a PNG at --time without a display server")
    args = parser.parse_args()
    if args.time < 0 or args.rate <= 0 or args.fps <= 0 or args.history <= 0 or args.max_points <= 0:
        parser.error("time must be nonnegative; rate, fps, history and max-points must be positive")
    bag = None
    try:
        ZoneInfo(args.timezone)
        bag = Bag(args.path)
        bag.info()
        if args.info:
            return 0
        import matplotlib
        if args.snapshot:
            matplotlib.use("Agg")
        elif not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            raise RuntimeError("No desktop display. Use --snapshot /tmp/bag.png --time 30")
        viewer = Viewer(bag, args)
        if args.snapshot:
            viewer.save(filename=args.snapshot)
            errors = [t for t in bag.topics if t.error]
            for topic in errors:
                print(f"{topic.name}: {topic.error}", file=sys.stderr)
            return 1 if errors or viewer.cloud_error else 0
        viewer.show()
        return 0
    except (ImportError, RuntimeError, ValueError, sqlite3.Error, KeyError) as exc:
        print(f"Error: {exc}\nLoad ROS/Livox: source /opt/ros/humble/setup.bash; "
              "source livox/install/setup.bash\n"
              "For system Matplotlib with incompatible user NumPy, run: python3 -s debug/vision_odom.py",
              file=sys.stderr)
        return 1
    finally:
        if bag is not None:
            bag.close()


if __name__ == "__main__":
    sys.exit(main())
