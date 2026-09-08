#!/usr/bin/env python3
"""Visualize lidar-3/lidar-5 to gimbal calibration transforms.

The YAML files contain T_gimbal_lidar, i.e. p_gimbal = T * p_lidar.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import matplotlib
import numpy as np
import yaml


def load_transform(path: Path) -> np.ndarray:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    rotation = np.asarray(data["lidar_to_gimbal_rotation"], dtype=float)
    translation = np.asarray(data["lidar_to_gimbal_translation"], dtype=float)
    if rotation.shape != (3, 3) or translation.shape != (3,):
        raise ValueError(f"invalid transform in {path}")
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def draw_frame(ax, transform: np.ndarray, name: str, length: float = 0.12) -> None:
    origin = transform[:3, 3]
    rotation = transform[:3, :3]
    colors = ("r", "g", "b")
    labels = ("x", "y", "z")
    for index, (color, label) in enumerate(zip(colors, labels)):
        endpoint = origin + rotation[:, index] * length
        ax.plot(
            [origin[0], endpoint[0]],
            [origin[1], endpoint[1]],
            [origin[2], endpoint[2]],
            color=color,
            linewidth=2.2,
        )
        ax.text(*endpoint, f"{name} {label}", color=color, fontsize=9)
    ax.scatter(*origin, s=45, depthshade=False)
    ax.text(*origin, f"  {name}", fontsize=10, weight="bold")


def set_equal_axes(ax, points: np.ndarray, margin: float) -> None:
    minimum = points.min(axis=0) - margin
    maximum = points.max(axis=0) + margin
    center = (minimum + maximum) / 2.0
    radius = max((maximum - minimum).max() / 2.0, margin)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)


def main() -> None:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--lidar3",
        type=Path,
        default=root / "slam/config/gimbal_lidar_3.yaml",
        help="lidar 3 calibration YAML",
    )
    parser.add_argument(
        "--lidar5",
        type=Path,
        default=root / "slam/config/gimbal_lidar_5.yaml",
        help="lidar 5 calibration YAML",
    )
    parser.add_argument("--axis-length", type=float, default=0.12)
    parser.add_argument("--save", type=Path, help="save PNG instead of only showing it")
    args = parser.parse_args()

    if args.save:
        matplotlib.use("Agg", force=True)
    else:
        if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
            raise RuntimeError(
                "No graphical display is available. Run from a desktop terminal "
                "or use --save output.png."
            )
        try:
            import tkinter  # noqa: F401
        except ImportError as error:
            raise RuntimeError(
                "Tk GUI support is unavailable. Install it with: "
                "sudo apt install python3-tk"
            ) from error
        matplotlib.use("TkAgg", force=True)

    try:
        import matplotlib.pyplot as plt
    except ImportError as error:
        raise RuntimeError(
            "Matplotlib cannot open a GUI window in this session. Run this "
            "script as the logged-in desktop user (not root), with DISPLAY and "
            "XAUTHORITY set, or use --save output.png."
        ) from error

    lidar3 = load_transform(args.lidar3)
    lidar5 = load_transform(args.lidar5)
    gimbal = np.eye(4, dtype=float)

    print("T_gimbal_lidar3 =")
    print(lidar3)
    print("T_gimbal_lidar5 =")
    print(lidar5)
    print(f"lidar 3 origin in gimbal: {lidar3[:3, 3]}")
    print(f"lidar 5 origin in gimbal: {lidar5[:3, 3]}")

    figure = plt.figure(figsize=(9, 7))
    ax = figure.add_subplot(111, projection="3d")
    draw_frame(ax, gimbal, "gimbal", args.axis_length)
    draw_frame(ax, lidar3, "lidar3", args.axis_length)
    draw_frame(ax, lidar5, "lidar5", args.axis_length)

    origins = np.vstack((gimbal[:3, 3], lidar3[:3, 3], lidar5[:3, 3]))
    set_equal_axes(ax, origins, max(args.axis_length * 1.5, 0.05))
    ax.set_xlabel("gimbal X [m]")
    ax.set_ylabel("gimbal Y [m]")
    ax.set_zlabel("gimbal Z [m]")
    ax.set_title("Lidar 3 / Lidar 5 to Gimbal Calibration")
    ax.grid(True)
    figure.tight_layout()

    if args.save:
        figure.savefig(args.save, dpi=180, bbox_inches="tight")
        print(f"saved visualization to {args.save}")
    else:
        print(f"opening GUI with Matplotlib backend {plt.get_backend()}")
        plt.show()


if __name__ == "__main__":
    main()
