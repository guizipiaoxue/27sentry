#!/usr/bin/env python3
"""Mouse editor for a Nav2 PGM/YAML map."""

from __future__ import annotations

import argparse
import os
import pathlib
import tkinter as tk

from PIL import Image, ImageTk
import yaml


def _write_pgm(image: Image.Image, path: pathlib.Path) -> None:
    gray = image.convert("L")
    width, height = gray.size
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as stream:
        stream.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        stream.write(gray.tobytes())


class MapEditor:
    def __init__(self, yaml_path: pathlib.Path, output_pgm: pathlib.Path | None,
                 output_yaml: pathlib.Path | None, brush_radius: int) -> None:
        config = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        image_path = pathlib.Path(str(config["image"]))
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path
        self.output_pgm = output_pgm or image_path
        self.output_yaml = output_yaml or yaml_path
        self.config = config
        self.image = Image.open(image_path).convert("L")
        self.brush_radius = max(1, brush_radius)
        self.mode = "move"
        self.last_pixel: tuple[int, int] | None = None

        self.root = tk.Tk()
        self.root.title("Navigation map editor")
        self.status = tk.StringVar()
        tk.Label(self.root, textvariable=self.status, anchor="w").pack(fill="x")
        self.canvas = tk.Canvas(self.root, background="#303030", cursor="fleur")
        self.canvas.pack(fill="both", expand=True)
        self.photo = ImageTk.PhotoImage(self.image)
        self.image_item = self.canvas.create_image(0, 0, image=self.photo, anchor="nw")
        self.canvas.configure(scrollregion=(0, 0, self.image.width, self.image.height))
        self._bind_events()
        self._set_mode("move")

    def _bind_events(self) -> None:
        self.root.bind_all("<KeyPress-a>", lambda _event: self._toggle_edit("erase"))
        self.root.bind_all("<KeyPress-p>", lambda _event: self._toggle_edit("paint"))
        self.root.bind_all("<KeyPress-q>", self._quit_or_move)
        self.root.bind_all("<Escape>", self._escape)
        self.root.bind_all("<KeyPress-bracketleft>", lambda _event: self._resize_brush(-1))
        self.root.bind_all("<KeyPress-bracketright>", lambda _event: self._resize_brush(1))
        self.canvas.bind("<ButtonPress-1>", self._press_left)
        self.canvas.bind("<B1-Motion>", self._drag_left)
        self.canvas.bind("<ButtonRelease-1>", self._release_left)
        for button in (2, 3):
            self.canvas.bind(f"<ButtonPress-{button}>", self._start_pan)
            self.canvas.bind(f"<B{button}-Motion>", self._pan)

    def _set_mode(self, mode: str) -> None:
        self.mode = mode
        self.status.set(f"模式: {mode} | a=擦除 p=占用 q=返回/保存 Esc=退出 | brush={self.brush_radius}")
        self.canvas.configure(cursor="crosshair" if mode != "move" else "fleur")

    def _toggle_edit(self, mode: str) -> None:
        self._set_mode("move" if self.mode == mode else mode)

    def _quit_or_move(self, _event=None) -> None:
        if self.mode != "move":
            self._set_mode("move")
        else:
            self.save()
            self.root.destroy()

    def _escape(self, _event=None) -> None:
        if self.mode != "move":
            self._set_mode("move")
        else:
            self.save()
            self.root.destroy()

    def _resize_brush(self, delta: int) -> None:
        self.brush_radius = max(1, self.brush_radius + delta)
        self._set_mode(self.mode)

    def _canvas_pixel(self, event) -> tuple[int, int]:
        return int(self.canvas.canvasx(event.x)), int(self.canvas.canvasy(event.y))

    def _press_left(self, event) -> None:
        if self.mode != "move":
            self._paint(event)
        else:
            self.canvas.scan_mark(event.x, event.y)

    def _drag_left(self, event) -> None:
        if self.mode == "move":
            self.canvas.scan_dragto(event.x, event.y, gain=1)
        else:
            self._paint(event)

    def _release_left(self, _event) -> None:
        self.last_pixel = None

    def _start_pan(self, event) -> None:
        self.canvas.scan_mark(event.x, event.y)

    def _pan(self, event) -> None:
        self.canvas.scan_dragto(event.x, event.y, gain=1)

    def _paint(self, event) -> None:
        x, y = self._canvas_pixel(event)
        if not (0 <= x < self.image.width and 0 <= y < self.image.height):
            return
        if self.last_pixel == (x, y):
            return
        pixels = self.image.load()
        value = 254 if self.mode == "erase" else 0
        radius = self.brush_radius
        for py in range(max(0, y - radius), min(self.image.height, y + radius + 1)):
            for px in range(max(0, x - radius), min(self.image.width, x + radius + 1)):
                if (px - x) ** 2 + (py - y) ** 2 <= radius ** 2:
                    pixels[px, py] = value
        self.last_pixel = (x, y)
        self.photo = ImageTk.PhotoImage(self.image)
        self.canvas.itemconfigure(self.image_item, image=self.photo)

    def save(self) -> None:
        _write_pgm(self.image, self.output_pgm)
        config = dict(self.config)
        relative_image = os.path.relpath(self.output_pgm, self.output_yaml.parent)
        config["image"] = pathlib.Path(relative_image).as_posix()
        self.output_yaml.parent.mkdir(parents=True, exist_ok=True)
        self.output_yaml.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
        print(f"已生成: {self.output_pgm} 和 {self.output_yaml}")

    def run(self) -> None:
        self.root.mainloop()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("yaml", nargs="?", default="maps/navigation_map.yaml", type=pathlib.Path)
    parser.add_argument("--output-pgm", type=pathlib.Path)
    parser.add_argument("--output-yaml", type=pathlib.Path)
    parser.add_argument("--brush-radius", type=int, default=3)
    args = parser.parse_args()
    try:
        MapEditor(args.yaml, args.output_pgm, args.output_yaml, args.brush_radius).run()
    except (OSError, KeyError, ValueError, tk.TclError, yaml.YAMLError) as exc:
        print(f"无法打开或保存地图：{exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
