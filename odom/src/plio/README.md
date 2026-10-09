# plio

ROS 2 Point-LIO integration for this repository's dual Livox setup.

The existing `fusion_ws` node synchronizes lidar 5/lidar 3, applies their
calibrated transforms, preserves each point's Livox time offset, and publishes
`/gimbal/cloud_fused`. It also calibrates, rotates, synchronizes and averages
both IMUs into `/gimbal/imu_fused`. This package consumes those two fused
streams, just like `odom_ws`, and runs the Point-LIO point-by-point IEKF and
iVox map locally without a Point-LIO subrepository.

Build `fusion_ws` in `slam/` and `plio` in `odom/`, sourcing the Livox driver
overlay first. After sourcing both resulting overlays, run:

```bash
ros2 launch plio point_lio.launch.py
```

The launch file starts both `fusion_ws/fusion_pcl` and Point-LIO. The Livox
driver must already be publishing its per-device custom-message and IMU topics
(the repository's existing driver command uses `xfer_format:=1`).

The executable publishes `/point_lio/odom`, `/path`, `/cloud_registered`,
`/point_lio/keyframe`, and the `odom -> gimbal` transform. Keep both lidars
stationary while `fusion_ws` performs its startup IMU calibration. The default
launch also starts the shared Scan Context/GICP loop detector and GTSAM iSAM2
backend, which publish `/loop_closure/constraint`, `/mapping/optimized_path`,
and `map -> odom`. Use `ENABLE_GTSAM=0 bash startup/start_odom.sh -a plio` to run only
the continuous odometry pipeline.

Keyframes use the same `loop_closure/Keyframe` message as DLIO: increasing ID,
scan-end pose in `odom`, and registered cloud in `odom`. A new keyframe is
published after 1 m of translation or 45 degrees of rotation by default;
`publish.keyframes`, `keyframe.distance_m`, and `keyframe.rotation_deg` are in
`config/point_lio.yaml`. Point-LIO's registered cloud is already sampled for
tracking (at most `mapping.max_tracking_points` points), so the optimized PCD
contains those sampled points rather than the dense DLIO keyframe scans.

The iVox map keeps at most `mapping.ivox_max_points_per_voxel` recent returns
per 30 cm cell (64 by default). Without that limit, repeated scans of a static
scene make every nearest-neighbor query progressively slower. At startup,
`mapping.initialization_scans` (5 by default) builds a denser initial map
before the first point-to-plane correction; this reduces random centimeter
offsets from matching a new scan against only one sparse Livox scan.

## Timing and CPU impact

Point-LIO's terminal dashboard reports the estimator wall time for one cloud,
its rolling average and maximum, IMU/LiDAR rates, CPU load, and input queue
drops. This is the value to compare with and without GTSAM. The loop detector
prints its keyframe callback average/maximum every 20 keyframes. The graph
backend prints separate keyframe-update and accepted-loop-update timings.

For a controlled comparison, run the same rosbag or route twice:

```bash
ENABLE_GTSAM=0 bash startup/start_odom.sh -a plio --no-record-bag
ENABLE_GTSAM=1 bash startup/start_odom.sh -a plio --no-record-bag
pidstat -urd -p "$(pgrep -f 'point_lio|loop_detector|pose_graph_backend' | paste -sd, -)" 1
ros2 topic hz /point_lio/keyframe
ros2 topic hz /loop_closure/constraint
ros2 topic bw /point_lio/keyframe
```

The loop nodes run in separate processes, so GICP and GTSAM do not block the
Point-LIO executor directly. They still consume CPU, memory, and DDS bandwidth;
contention shows up as increased Point-LIO computation time, lower LiDAR rate,
or queue drops. Keyframe clouds and the graph are retained in memory, so the
long-run memory trend should also be monitored with `pidstat -r` or `top`.

From the repository root, `bash startup/build_plio.sh --start` rebuilds the fusion and
PLIO packages in Release mode, runs the focused regression tests, and starts
`startup/start_odom.sh -a plio` with its SDK discovery and rosbag recording. Without
`--start`, it only builds/tests and requires no connected lidar.

The MID360 input contract is float32 `time` in **seconds** relative to the
cloud header, and specific force in **m/s²**, angular velocity in **rad/s**.
PLIO internally stores point offsets in milliseconds in `curvature`. Untimed
or malformed input scans are rejected. A voxel retains one actual return and
its acquisition time; at most 1200 points enter matching, in 1 ms groups.
Registered body clouds are deskewed to the scan-end body pose. Their curvature
retains the original scan-start offset for diagnostics, not a new scan-end offset.

Odometry and TF have a 100 Hz timer and integrate the latest IMU measurements
between scan corrections. IMU receipt, matching, and publishing have separate
callback groups. All output stamps stay in the sensor clock domain; neither
the host clock nor a hardcoded 37-second shift enters integration. Path/cloud
output follows scan corrections (about 10 Hz) and uses scan-end stamps.
The timer skips duplicate sensor stamps and stops publishing on IMU gaps over
50 ms or scan-correction age over 0.5 s. Odometry linear twist is in the child
body frame. Actual timer scheduling still depends on the deployment computer.

The supplied calibration is specific to this two-lidar mounting. Both lidar
Z offsets are constrained to zero; yaw-only data cannot determine height or
absolute heading. `imu.correction_rotation` and `imu.effective_lever_arm` correct
the fused virtual IMU inside PLIO, including centripetal/angular-acceleration
terms. For another mounting, recalibrate or use identity/zero respectively.
Static initialization retains gravity and estimates its direction and biases;
do not subtract gravity from the incoming IMU a second time.

The offline calibration, validation split, limitations, and before/after
results are in `analysis/20260919_205858_plio/OPTIMIZATION.md` at the repository
root. Its `replay_final.sh` reruns the final pipeline without ROS/DDS transport.

The vendored IKFoM, iVox, and Point-LIO-derived estimator code remains under
the upstream BSD-3-Clause terms in `POINT_LIO_LICENSE`.
