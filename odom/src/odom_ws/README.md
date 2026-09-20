# Dual MID360 DLIO

Use `bash build_dlio.sh --start` from the repository root to build the updated
DLIO/fusion nodes, run the motion regression test, and launch the existing
SDK-aware pipeline with diagnostic rosbag recording. After building,
`bash start_odom.sh -a dlio` starts it directly.

The fused input cloud has float32 `time` in seconds relative to its header.
DLIO now deskews it before the 0.25 m voxel filter, caps tracking at 4000 points,
and uses 4 matching threads by default. Dense deskewed keyframes are retained
for the mapping/loop-closure interface. Untimed or malformed scans are rejected
when deskew is enabled; old bags whose fused `time` was zeroed must be rebuilt
from raw Livox points.

`odom/config/odom.yaml` enables an additional 0.5 s static gravity alignment
after upstream dual-IMU calibration. Keep the platform still until the DLIO
initialization message. Specific force is m/s², angular velocity rad/s. The
fused IMU direction/lever correction is specific to this mounting; the nominal
fused IMU and cloud frame transforms are identity. Gravity alignment rotates
the initial odom frame, unlike PLIO's initially coincident gimbal frame.

Odometry, pose and `odom -> gimbal` TF share a 100 Hz timer, an atomic state
snapshot and the timestamp of the latest integrated IMU. A delayed scan
corrects the historical state at its median point time, followed by replay of
newer IMU samples. Duplicate timestamps are not republished; outputs stop on
IMU staleness over 50 ms or correction age over 0.5 s. Path/cloud/keyframe
timestamps use the scan's median point time and publish at scan/keyframe rate.
Linear twist uses the child body frame.

Parameters added: `publish/state_rate_hz`, `publish/path_capacity`,
`odom/preprocessing/maxPoints`, `odom/threads`, `terminal/enabled`,
`imu/fused_correction_R`, `imu/fused_lever_arm`, `imu/lever_smoothing_seconds`.
Edit the YAML and restart to change them. `publish/pose_odom: false` retains
scan-rate TF while disabling the high-rate odometry/pose timer.

The source also removes detached worker lifetimes, snapshots IMU buffers
before integration, bounds waits, bounds dashboard/path history, and keeps
keyframe publication on the owned submap worker. The timer does not wait for
scan matching or terminal output.

See `analysis/20260919_205858_dlio/REPORT.md` for kernel benchmarks and their
limits. These are not full-node ROS/DDS replay or hardware timing results.
