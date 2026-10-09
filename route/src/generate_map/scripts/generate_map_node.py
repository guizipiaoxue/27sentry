#!/usr/bin/env python3
"""Project a PCD point cloud into a Nav2-compatible PGM/YAML map."""

from __future__ import annotations

import math
import os
import pathlib
import struct
from typing import Iterator

import rclpy
from rclpy.node import Node


def _header_and_data(path: pathlib.Path) -> tuple[dict[str, list[str]], bytes]:
    with path.open("rb") as stream:
        header: dict[str, list[str]] = {}
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PCD header has no DATA line")
            text = line.decode("ascii").strip()
            if not text or text.startswith("#"):
                continue
            parts = text.split()
            key = parts[0].upper()
            header[key] = parts[1:]
            if key == "DATA":
                return header, stream.read()


def _points(path: pathlib.Path) -> Iterator[tuple[float, float, float]]:
    header, payload = _header_and_data(path)
    fields = header.get("FIELDS") or header.get("FIELD")
    sizes = [int(value) for value in header.get("SIZE", [])]
    types = header.get("TYPE", [])
    counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields or []))]
    if (
        not fields
        or len(fields) != len(sizes)
        or len(fields) != len(types)
        or len(fields) != len(counts)
    ):
        raise ValueError("PCD must define matching FIELDS, SIZE, TYPE and COUNT")
    try:
        x_index, y_index, z_index = (fields.index(name) for name in ("x", "y", "z"))
    except ValueError as exc:
        raise ValueError("PCD must contain x, y and z fields") from exc
    data_kind = (header.get("DATA") or [""])[0].lower()
    if data_kind == "ascii":
        for line in payload.decode("ascii").splitlines():
            values = line.split()
            if len(values) >= len(fields):
                yield float(values[x_index]), float(values[y_index]), float(values[z_index])
        return
    if data_kind != "binary":
        raise ValueError(f"unsupported PCD DATA type: {data_kind}")

    format_parts: list[str] = []
    offsets: list[int] = []
    offset = 0
    for size, kind, count in zip(sizes, types, counts):
        offsets.append(offset)
        if kind == "F":
            code = {4: "f", 8: "d"}.get(size)
        elif kind == "I":
            code = {1: "b", 2: "h", 4: "i", 8: "q"}.get(size)
        elif kind == "U":
            code = {1: "B", 2: "H", 4: "I", 8: "Q"}.get(size)
        else:
            code = None
        if code is None:
            raise ValueError(f"unsupported PCD field type {kind}{size}")
        format_parts.append(code * count)
        offset += size * count
    point_size = offset
    point_count = int((header.get("POINTS") or header.get("WIDTH", ["0"]))[0])
    if len(payload) < point_size * point_count:
        raise ValueError("PCD binary payload is shorter than POINTS declares")
    unpack = struct.Struct("<" + "".join(format_parts)).unpack_from
    scalar_offsets = [sum(counts[:index]) for index in (x_index, y_index, z_index)]
    for index in range(point_count):
        values = unpack(payload, index * point_size)
        yield (
            float(values[scalar_offsets[0]]),
            float(values[scalar_offsets[1]]),
            float(values[scalar_offsets[2]]),
        )


def _write_map(pixels: list[int], width: int, height: int, output_pgm: pathlib.Path,
               output_yaml: pathlib.Path, resolution: float, origin_x: float, origin_y: float,
               occupied_thresh: float, free_thresh: float) -> None:
    output_pgm.parent.mkdir(parents=True, exist_ok=True)
    output_yaml.parent.mkdir(parents=True, exist_ok=True)
    with output_pgm.open("wb") as stream:
        stream.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        for row in range(height - 1, -1, -1):
            stream.write(bytes(pixels[row * width:(row + 1) * width]))
    image_ref = pathlib.Path(os.path.relpath(output_pgm, output_yaml.parent)).as_posix()
    output_yaml.write_text(
        f"image: {image_ref}\nmode: trinary\nresolution: {resolution}\n"
        f"origin: [{origin_x}, {origin_y}, 0.0]\nnegate: 0\n"
        f"occupied_thresh: {occupied_thresh}\nfree_thresh: {free_thresh}\n",
        encoding="utf-8",
    )


class GenerateMapNode(Node):
    def __init__(self) -> None:
        super().__init__("generate_map")
        self.input_pcd = self.declare_parameter("input_pcd", "maps/optimized_map.pcd").value
        self.output_pgm = self.declare_parameter("output_pgm", "maps/navigation_map.pgm").value
        self.output_yaml = self.declare_parameter("output_yaml", "maps/navigation_map.yaml").value
        self.resolution = float(self.declare_parameter("resolution", 0.05).value)
        self.z_min = float(self.declare_parameter("z_min", 0.1).value)
        self.z_max = float(self.declare_parameter("z_max", 2.0).value)
        self.min_points = int(self.declare_parameter("min_points_per_cell", 1).value)
        self.unknown_policy = str(self.declare_parameter("unknown_policy", "unknown").value)
        self.origin_x = float(self.declare_parameter("origin_x", math.nan).value)
        self.origin_y = float(self.declare_parameter("origin_y", math.nan).value)
        self.occupied_thresh = float(self.declare_parameter("yaml_occupied_thresh", 0.65).value)
        self.free_thresh = float(self.declare_parameter("yaml_free_thresh", 0.196).value)

    def run(self) -> bool:
        if self.resolution <= 0 or self.z_min >= self.z_max or self.min_points < 1:
            self.get_logger().error("resolution, z range, or min_points_per_cell is invalid")
            return False
        if self.unknown_policy not in {"unknown", "free", "occupied"}:
            self.get_logger().error("unknown_policy must be unknown, free, or occupied")
            return False
        points: list[tuple[float, float, float]] = []
        min_x = min_y = math.inf
        max_x = max_y = -math.inf
        try:
            for x, y, z in _points(pathlib.Path(str(self.input_pcd))):
                if (
                    not all(math.isfinite(value) for value in (x, y, z))
                    or not self.z_min <= z <= self.z_max
                ):
                    continue
                points.append((x, y, z))
                min_x, min_y = min(min_x, x), min(min_y, y)
                max_x, max_y = max(max_x, x), max(max_y, y)
        except (OSError, ValueError, struct.error) as exc:
            self.get_logger().error(f"failed to read PCD: {exc}")
            return False
        if not points:
            self.get_logger().error("no points remain after filtering")
            return False
        if not math.isfinite(self.origin_x):
            self.origin_x = min_x
        if not math.isfinite(self.origin_y):
            self.origin_y = min_y
        width = math.ceil((max_x - self.origin_x) / self.resolution) + 1
        height = math.ceil((max_y - self.origin_y) / self.resolution) + 1
        if width <= 0 or height <= 0 or width * height > 100_000_000:
            self.get_logger().error(f"unreasonable map size: {width} x {height}")
            return False
        counts = [0] * (width * height)
        for x, y, _ in points:
            ix = math.floor((x - self.origin_x) / self.resolution)
            iy = math.floor((y - self.origin_y) / self.resolution)
            if 0 <= ix < width and 0 <= iy < height:
                counts[iy * width + ix] += 1
        pixels: list[int] = []
        occupied = free = unknown = 0
        for count in counts:
            if count >= self.min_points or (self.unknown_policy == "occupied" and count == 0):
                pixels.append(0)
                occupied += 1
            elif self.unknown_policy == "free":
                pixels.append(254)
                free += 1
            else:
                pixels.append(205)
                unknown += 1
        try:
            _write_map(
                pixels,
                width,
                height,
                pathlib.Path(str(self.output_pgm)),
                pathlib.Path(str(self.output_yaml)),
                self.resolution,
                self.origin_x,
                self.origin_y,
                self.occupied_thresh,
                self.free_thresh,
            )
        except OSError as exc:
            self.get_logger().error(f"failed to write map: {exc}")
            return False
        self.get_logger().info(
            f"generated {self.output_pgm} and {self.output_yaml}: points={len(points)} "
            f"size={width}x{height} resolution={self.resolution:.3f} "
            f"z=[{self.z_min:.3f},{self.z_max:.3f}] "
            f"min_points={self.min_points} unknown_policy={self.unknown_policy} "
            f"occupied={occupied} free={free} unknown={unknown}")
        return True


def main() -> int:
    rclpy.init()
    node = GenerateMapNode()
    result = node.run()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result else 1


if __name__ == "__main__":
    raise SystemExit(main())
