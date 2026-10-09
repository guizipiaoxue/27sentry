# generate_map

The package now uses Python. `generate_map_node` converts the PCD point cloud
to a Nav2 `PGM + YAML` map. `map_editor` opens that map in a desktop window and
saves edits back to PGM/YAML.

Build from the `route` workspace:

```bash
source /opt/ros/humble/setup.bash
cd route
colcon build --packages-select generate_map
source install/setup.bash
cd ..
```

Generate a map with parameters:

```bash
ros2 run generate_map generate_map_node --ros-args \
  -p input_pcd:=/home/hustlyrm/sentry_test_py/maps/optimized_map.pcd \
  -p output_pgm:=/home/hustlyrm/sentry_test_py/maps/navigation_map.pgm \
  -p output_yaml:=/home/hustlyrm/sentry_test_py/maps/navigation_map.yaml \
  -p z_min:=0.3 -p z_max:=1.0 \
  -p resolution:=0.03 -p min_points_per_cell:=5
```

There must be no space around `:=`. For example, use `-p z_min:=0.3`, not
`-p z_min:= 0.3`; the latter is parsed as an empty value and leaves the
default parameter in effect.

The generator parameters are `resolution` (meters per cell), `z_min` and
`z_max` (height filter in the PCD frame), `min_points_per_cell`,
`unknown_policy` (`unknown`, `free`, or `occupied`), and optional `origin_x`
and `origin_y`. Default paths are relative to the current working directory.

Use a smaller `resolution` for more detail at the cost of a larger map. Raise
`min_points_per_cell` to remove isolated points; lower it when the cloud is
sparse. Set `z_min`/`z_max` to the height band containing obstacles. With
`unknown_policy:=unknown`, cells without enough points stay gray and are
treated as unknown by Nav2; use `free` only when unobserved space is known to
be traversable.

Open the generated map:

```bash
ros2 run generate_map map_editor maps/navigation_map.yaml
```

The editor starts in move mode. Left-drag pans the map. Middle- or right-drag
also pans the map in every mode. Press `a` to enter erase mode; left-drag turns
occupied pixels into free pixels (PGM value 254). Press `a` again or `q` to
return to move mode. Press `p` to enter occupied mode; left-drag writes black
occupied pixels (value 0). Press `p` again or `q` to return to move mode. In
move mode, `q` or `Esc` saves the edited map and exits. `[` and `]` decrease
or increase the brush radius. The default brush radius is 3 pixels and can be
changed with `--brush-radius`.

To preserve the original map, write to a separate output pair:

```bash
ros2 run generate_map map_editor maps/navigation_map.yaml \
  --output-pgm maps/navigation_map_edited.pgm \
  --output-yaml maps/navigation_map_edited.yaml \
  --brush-radius 4
```

The editor requires a graphical desktop (`tkinter` and Pillow). The map editor
changes image pixels only; it does not perform obstacle inflation or publish a
runtime `nav_msgs/OccupancyGrid`. Inflation remains a costmap configuration.
