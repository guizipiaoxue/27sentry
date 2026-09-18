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

The vendored IKFoM, iVox, and Point-LIO-derived estimator code remains under
the upstream BSD-3-Clause terms in `POINT_LIO_LICENSE`.
