# 项目开发与时间戳约定

## 项目概览

这是狼牙哨兵的 ROS 2 Humble 实机工程，以 C++17 为主，配有 Bash 启动脚本和
Python 诊断工具。根目录不是单一 colcon 工作区；以下目录分别构建和 source：

| 目录 | 职责与主要入口 |
| --- | --- |
| `livox/` | Livox ROS Driver 2；MID360 点云及内置 IMU 的采集和发布 |
| `slam/` | 双雷达/IMU 融合 `fusion_ws`、外参标定 `cali_ws`、KD-tree 地图 `map_ws` |
| `odom/` | DLIO、Point-LIO、单雷达入口和回环检测/位姿图后端 |
| `base_driver/` | 串口 `receiver`/`sender`，底盘、云台及比赛状态消息 |
| `route/` | costmap/planner/smoother 包；当前为构建骨架，不能视作完整规划实现 |
| `navigation/` | MPC 与键盘控制，接收里程计和串口状态 |
| `debug/` | 视觉里程计检查与相应 Python 测试 |
| `gtsam/` | 固定在仓库中的第三方 GTSAM，供回环后端使用 |
| `scripts/` | 项目级运行前检查和时间同步工具 |
| `tools/`、`analysis/` | 本机标定、回放和分析资料，多数被 Git 忽略 |

先读入口、配置、生产者/消费者，再修改数据链路。第三方实现包括 GTSAM、
RapidJSON、Eigen、IKFoM 和 iVox；不要把第三方内部重构混入项目功能修改。
`build/`、`install/`、`log/`、`maps/`、`runlog/` 和 rosbag 是生成或实机数据。

## 活跃数据链路

```text
MID360 192.168.1.5 / 192.168.1.3
  -> livox_ros_driver2
  -> /sentry/raw/lidar5, /sentry/raw/lidar3 (CustomMsg)
  -> /sentry/raw/imu5, /sentry/raw/imu3 (sensor_msgs/Imu)
  -> fusion_ws/fusion_pcl (安装外参、静止 IMU 标定、双 IMU 插值)
  -> /gimbal/cloud_fused (PointCloud2: x/y/z/intensity/time)
  -> /gimbal/imu_fused, /gimbal/imu_calibrated
  -> odom_ws/odom (DLIO) 或 plio/point_lio
  -> 里程计、TF、关键帧
  -> loop_closure/loop_detector -> pose_graph_backend (GTSAM iSAM2)
  -> map_ws/kdtree_map / 保存后的优化地图
```

`odom_ws/src/odom.cpp` 和 `single_test/src/single_lidar_odom.cpp` 是 DLIO 的入口，
算法实现位于 `direct_lidar_inertial_odometry/`。Point-LIO 的 ROS 边界在
`plio/src/point_lio.cpp`，估计器在 `point_lio_estimator.cpp`。
融合输出坐标系为 `gimbal`，实机外参在 `slam/config/gimbal_lidar_{5,3}.yaml`。
不要通过改变坐标系名或时间戳来补偿外参错误。

## 启动与配置

- `start_odom.sh --algorithm dlio|plio`：双雷达采集、融合、标定、里程计，默认录包。
- `start_mapping.sh`：调用里程计入口并启动 KD-tree 地图；退出时保存地图。
- `rotate_cali.sh`：双雷达 yaw 圆周标定入口；调用单雷达 DLIO 和标定节点。
- `odom/start_single_lidar.sh`：单雷达 DLIO 检查。
- `start_pcl.sh`：旧点云展示入口，依赖外部运行的驱动和显式提供的标定文件。

双雷达实机网络配置在 `livox/src/livox_ros_driver2/config/MID360_config_2.json`，
当前主机地址 `192.168.1.50`。不要根据文件名猜设备地址，以实际 JSON 和路由为准。
其他 Livox 示例配置有不同网段和设备类型，不要无依据地全部改成实机地址。
DLIO 主配置为 `odom/config/odom.yaml`，PLIO 为 `odom/src/plio/config/point_lio.yaml`，
回环为 `odom/config/loop.yaml`，地图为 `slam/config/map.yaml`。

主机 PTP 服务用 `sudo bash scripts/install_ptp_services.sh --interface enp86s0 --utc-offset 37`
部署。该脚本先备份、再移除旧四个专用服务，安装 `sentry-phc-bootstrap@`、
`sentry-ptp4l@` 和 `sentry-phc2sys@`；配置位于 `/etc/linuxptp/sentry/`。
bootstrap 只初始化 PHC，已对齐时跳过 step；偏差超限且存在采集或其他 PHC writer
时拒绝初始化。持续 servo 方向为系统 UTC 到 PHC，禁止 step 并限制频率调整。
相同部署复用运行实例；改变已运行服务的文件须在维护窗口先停用，不能隐式重启。
安装和运维细节见 `docs/PTP.md`。

## 时间戳契约

目标是双 MID360 的点云和各自内置 IMU 共用 PTP 时间基准。PTP 对齐时钟，
不会让独立 200 Hz IMU 的采样相位相同；融合应在同一时间线上插值。

1. 实机 ROS 时间戳使用 Unix/UTC 纳秒。严格模式必须确认 Livox 包 `time_type=1`
   （PTP/gPTP），拒绝未同步包、GPS 包和不合法/倒退时间；不能用接收时间冒充采样时间。
2. 同步状态必须按设备和数据流维护；一路的同步标志不能代表另一台雷达或 IMU。
3. PTP 的 PHC 可能使用 TAI；应明确 PHC 与系统 UTC 的偏差，以及设备包实际采用的
   时间尺度。只能按明确配置转换 UTC 偏移，禁止通过“首包与 now 差值”自动拟合。
4. `CustomMsg.header.stamp == timebase`，两者单位均为纳秒；
   `CustomPoint.offset_time` 是相对 timebase 的无符号纳秒。
5. 融合 `PointCloud2.time` 为相对融合 header 的有符号 float32 秒。
   绝对采样时间是 `header + time`；采用较晚 header 时，另一帧前部的 time 可以为负。
   变换、拼接、过滤不得丢掉 time 字段或把相对秒误认为绝对纳秒。
6. 融合 IMU 保留 lidar5 采样时间，在 lidar3 样本包围该时间且间隔有界时插值。
   无覆盖时丢弃或等待，禁止无限外推或强行把两条 header 改成相同值。
7. 失锁、时间跳变和数据流超时必须阻止不可信融合，清理旧缓存并报告原因。
   DLIO 的 `odom/computeTimeOffset` 在 PTP 实机链路中保持 false。
8. 实机 `use_sim_time=false`；回放使用录制的采样时间。需要关闭实时新鲜度检查时，
   应明确使用回放配置，不能削弱实机默认检查。
9. 串口 receiver 的 header 是主机接收时间；`sample_tick_ms` 是 MCU tick，当前协议
   缺少 MCU 与 PTP 的映射。不能把串口云台数据称作硬件 PTP IMU。

实机驱动默认 `require_ptp_sync=true`、`ptp_utc_offset_seconds=37`、
`ptp_max_host_skew_seconds=2.0`、`ptp_stream_timeout_seconds=0.5`。
驱动和融合的 PTP 参数仅在启动时设置，运行时为只读。
`/livox/ptp_locked` 是 reliable/transient-local 的 500 ms 心跳；全部配置设备的
点云和 IMU 都有效才为 true。失锁会作废该设备的排队数据，恢复前至少保持
1 秒失锁状态；融合在状态缺失、false 或 2 秒未刷新时停止输出并重做 IMU 标定。
主机运行前检查及硬件/软件 PTP 偏移设置见 `docs/PTP.md`，验收证据与限制见
`docs/PTP_VALIDATION.md`。

旧 `cali_ws/rotate_cali.cpp` 和 `pcl_publish.cpp` 具有显示/旧标定用途，未保留完整
逐点时间链路；不要把这些节点当作融合里程计输入。优先使用 `fusion_pcl`。

## 构建与验证

先 source `/opt/ros/humble/setup.bash`。构建顺序为 `livox -> slam -> odom`，
串口独立构建后供 navigation 使用。Livox 使用 ROS2 模式和本机安装的 SDK2：

```bash
cd livox
colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
cd ../slam
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
cd ../odom
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

`build_dlio.sh`、`build_plio.sh` 提供增量构建；回环默认编译仓库 GTSAM，
`-DLOOP_CLOSURE_USE_SYSTEM_GTSAM=ON` 仅适用于兼容的本机安装。
SDK 运行库可通过 `SDK_DIR` 指定；常见目录为 `/usr/local/lib`。
测试与配置变更应按边界执行：驱动时间规则、融合时间/插值、PLIO/DLIO 回放及启动检查。
现有测试位于 `plio/test/`、`odom_ws/test/`、`base_driver/tests/`、
`navigation/src/test_control/tests/` 和 `debug/test_vision_odom.py`。
检查 Bash 用 `bash -n`，Python 用 `py_compile`/unittest，ROS 包用 colcon/CTest。

有实机采集进程运行时，使用独立 `--build-base`、`--install-base`、`--log-base` 验证，
避免覆盖正在加载的共享库。只读检查 ROS 话题和 PTP 状态可以进行；不要停止用户采集、
调整系统时钟、改网卡地址或启动第二个 PHC servo。先检查既有 ptp4l/phc2sys 服务。

## 修改规则

- 保留用户未提交的修改和删除；不提交生成文件、地图、录包、日志或本机凭据。
- 公共消息/话题或时间契约改动必须搜索全部生产者、消费者、launch 和测试。
- 保持启动脚本的进程组、错误退出、录包刷新和地图保存语义。
- 校验失败应给出设备/数据流和原因；不允许静默回退至接收时间。
- 记录验证的实际结果；离线测试、主机 PTP 状态和传感器硬件实测是不同证据。
- 新增约定和运行方式及时更新本文件及 README；源码和实机配置是最终依据。
