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

The executable publishes `/point_lio/odom`, `/path`, `/cloud_registered`, and
the `odom -> gimbal` transform. Keep both lidars stationary while `fusion_ws`
performs its startup IMU calibration.

From the repository root, `bash build_plio.sh --start` rebuilds the fusion and
PLIO packages in Release mode, runs the focused regression tests, and starts
`start_odom.sh -a plio` with its SDK discovery and rosbag recording. Without
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
