# 狼牙27赛季哨兵测试代码

作者：彭友

项目结构、构建顺序和数据时间戳约定见 [AGENTS.md](AGENTS.md)。

## PTP 时间对齐

双 MID360 的点云和内置 IMU 统一使用 UTC 采样时间。驱动逐设备、逐数据流
检查 PTP 包，并发布 `/livox/ptp_locked`；融合在失锁或状态超时后清空缓存、
停止输出并重新标定。未同步数据不会改成主机接收时间。

本机硬件 PTP 的设备包使用 TAI，实测比主机 UTC 超前约 37 秒；驱动默认通过
`ptp_utc_offset_seconds=37` 转换。使用 UTC 软件 PTP 时设置 `PTP_UTC_OFFSET=0`。
主机检查会验证对应时间尺度，禁止根据首包接收延迟自动拟合偏移。

项目提供 `sentry-ptp4l@enp86s0`、`sentry-phc2sys@enp86s0` 及仅初始化 PHC
的 bootstrap 服务，配置和 UDS 按网卡隔离。安装会备份、停用并移除旧四个
PTP/clock 服务，保留 linuxptp 工具和现有时钟管理 mask；系统 UTC 不会被校跳。

```bash
sudo bash scripts/install_ptp_services.sh --interface enp86s0 --utc-offset 37
```

PHC 已与 UTC + 37 秒匹配时 bootstrap 不 step；有实机采集进程且 PHC 超出
容差时拒绝初始化。安装、复用、卸载流程见 [PTP 部署与验收](docs/PTP.md)。

修改后先按 `livox → slam → odom` 重新构建，完整命令见
[PTP 部署与验收](docs/PTP.md)。已有采集进程运行时，使用独立构建目录验证；
新版驱动和融合节点需在下次启动时加载。
本次构建、功能测试及现机只读检查记录见 [PTP 验证记录](docs/PTP_VALIDATION.md)。

```bash
# PTP 服务已运行时，启动入口会先检查主机及四路传感器锁定。
./startup/start_odom.sh  # 默认 PLIO；可用 --algorithm dlio 切换。

# 在另一终端 source ROS 2 与 livox/install/setup.bash 后检查采样时间。
python3 scripts/check_sensor_time.py --duration 10 --include-fused
```

`startup/start_mapping.sh`、实机 `startup/rotate_cali.sh` 和 `startup/start_single_lidar.sh` 使用同一
PTP 检查。回放旧包时可显式给融合节点设置 `require_ptp_sync=false`，保留录制的
采样时间；该选项仅用于回放。PTP 对齐时钟，双 IMU 采样相位通过有界插值处理。

传感器锁定等待失败时，入口会直接查询 DDS：缺少 `/livox/ptp_locked` 发布者时，
先确认新版驱动已成功构建到 `livox/install`、加载的工作区和 `ROS_DOMAIN_ID`；
已有发布者却为 false 时，检查驱动的逐设备时间诊断。仅有四条原始话题不足以证明
锁定。若从普通复制安装改用 `--symlink-install` 后出现“failed to create symbolic
link”及“existing path cannot be removed: Is a directory”，需先停止采集、备份并
清理受影响的包 build/install 后再构建；
编译失败后单纯重启仍会加载旧版。

## 复测日志与 UDP 接收缓冲

启用 SDK 错误日志需要先构建隔离 SDK 和驱动（本机已完成）：

```bash
bash livox/build_isolated_sdk.sh # 可传入 /path/to/Livox-SDK2
source /opt/ros/humble/setup.bash
cd livox
colcon build --packages-select livox_ros_driver2 --cmake-args \
  -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release
cd ..
```

SDK 内置 spdlog 1.3，ROS Humble 使用系统 spdlog 1.9；直接打开普通 SDK 的控制台
日志会混用两版 C++ 符号并段错误，原驱动关闭日志也仍可能在退出时崩溃。
`build_isolated_sdk.sh` 保持上游源文件和系统 SDK 不变，仅导出公共 C API，
生成独立库名 `liblivox_lidar_sdk_sentry.so`，驱动安装时复制到 ROS 包的 lib 目录。
入口优先选择项目隔离 SDK；独立 `ros2 run` 在 source 驱动工作区后也能找到它。
新增 `sdk_logging_test` 同时加载 ROS 和 SDK 日志，防止该冲突回归。

测试前先扩容 UDP 接收上限，再按原命令启动：

```bash
sudo bash scripts/expand_udp_buffers.sh
bash startup/start_odom.sh -a plio
```

扩容脚本将 `net.core.rmem_max` 持久化为 Linux `SO_RCVBUF` 请求的最大可用值
`1073741823` 字节，备份旧值到 `/var/backups/sentry-udp-*`，并用新 UDP socket
验证 SDK 的 200 MiB 请求可得到 400 MiB 的 Linux 双倍记账值。
不增加所有 socket 的默认缓冲，不预分配 1 GiB；已打开的 socket 须在下次启动采集时
重新创建。脚本需要 root，验证失败会恢复旧配置。它不重启 PTP 或调整时钟。

`startup/start_odom.sh` 每次建立 `runlog/YYYYMMDD_HHMMSS_算法_PID/`，启动时显示完整路径。
`SENTRY_RUNLOG_DIR` 是组件共用的精确目录，PLIO 在此入口中自动启用日志；独立启动
节点仍遵循原有 runlog 配置。目录内保存：

| 文件 | 用途 |
| --- | --- |
| `console.log`、`ros/` | 全部节点 stdout/stderr、融合标定过程、异常、先退出的 PID 和状态码 |
| `driver.log` | 四路收包统计/间隔、包序号变化、PTP 拒绝原因/原始时间/主机时间、失锁前设备与流、generation、锁等待/发布/SDK 回调耗时 |
| `lidar_health.log` | MID360 主动状态推送、HMS 出现/清除、诊断变化、配置命令失败、驱动异常镜像及每台设备每秒状态摘要 |
| `host.jsonl` | 每秒 UDP 全局计数和增量、各 socket 缓冲/排队/丢包、网卡统计、进程/线程 CPU 与调度、内存/IO 压力、磁盘空间 |
| `ptp.log` | 既有 ptp4l/phc2sys 的 journal，包括启动前两分钟；权限或命令不可用时保留错误 |
| `odomplio.log` / `odomdlio.log` | IMU、点云、队列和状态；PLIO 额外记录同步/标定状态、IMU 中断、等待 IMU、位姿及原点距离 |
| `backend.log` | 位姿图更新前的关键帧/回环、位姿增量、图大小、耗时及异常上下文；GTSAM 异常仍终止进程，先刷新日志 |
| `context.txt`、`config/` | Git 版本/工作差异、SDK/节点校验值、ROS/PTP 环境、录包路径及实际配置快照 |

驱动日志中 `stream=0` 为点云、`stream=1` 为 IMU。
包序号变化仅记录原始观察，不能直接等同于 UDP 丢包；全局 UDP 累计值也不能
归因于单次测试，应结合本次增量和传感器 socket 的丢包计数。

MID360 通过 `SetLivoxLidarInfoCallback` 接收设备主动状态推送，收到后在专用线程
写入 `lidar_health.log` 并输出控制台；另每 5 秒只读查询内部信息，首次延迟 5 秒，
不改采集或 PTP 配置。`LIDAR_PUSH` 保存 SDK 提供的完整 JSON，未知字段也保留；
非法推送单独记录 `LIDAR_PUSH_INVALID`。已有进程需在下次正常启动时加载新构建。
`startup/start_odom.sh` 自动使用共用会话目录；独立运行驱动且未设 `SENTRY_RUNLOG_DIR` 时，
健康日志保存到当前目录下 `runlog/YYYYMMDD_HHMMSS/lidar_health.log`，启动会输出
`LIDAR_HEALTH_LOG file=...`。SDK 内部错误没有统一的用户错误回调，因此保留 SDK
控制台日志，由 `startup/start_odom.sh` 的 `console.log` 捕获 socket、发送、解包和初始化错误；
独立运行时可用 `tee` 保存 stdout/stderr。
`core_temp_raw` 是 SDK 的原始 `int32_t`，官方协议单位为 0.01 ℃；
`core_temp_c=core_temp_raw/100.0` 是内部核心温度，不能当作环境温度。
`diag_system/scan/ranging/communication` 分别表示系统、扫描、测距、通信模块的级别：
0 正常、1 警告、2 错误、3 安全错误。`hms` 保存 8 个完整十六进制故障码，
高 16 位为异常 ID；温度相关 ID 包括 `0x0102/0x0103`（高温警告）、
`0x0111/0x0112`（内部器件温度异常）、`0x0114`（环境温度高）、
`0x0115`（环境温度超限、设备已停止工作）。

每台配置设备每秒输出 `LIDAR_STATUS`，包括 `status`、`device_status`、温度、工作
状态名称、同步类型、四模块诊断级别、完整 HMS、每项诊断的年龄、查询/命令失败计数，
以及 `cloud_status/imu_status`、最近收包年龄、包数和拒绝数。`status` 是设备与主机
链路的综合观察；`device_status` 仅由设备工作状态、诊断和 HMS 推导，不能用来判断
主机是否丢包。`discovered=1` 表示曾被发现，不代表设备当前在线。

| 状态字段/事件 | 含义 |
| --- | --- |
| `status=normal/warning/error/fatal` | 正常，设备警告，设备/链路/配置错误，或设备致命异常 |
| `status=not_discovered/not_sampling/unknown` | 配置设备尚未发现，设备未处于采样状态，或诊断不完整/过期/查询失败 |
| `cloud_status/imu_status=not_received/receiving/timeout/rejected` | 从未收到、收到并通过时间校验、接收超时、或收到但数据/时间校验未通过 |
| `health_fresh=0` | 至少一项必需诊断缺失或超过 15 秒；历史故障仍保留，不能当作当前正常 |
| `LIDAR_HMS_ACTIVE/CLEARED` | 完整故障码新增/清除，附异常 ID、严重等级及原因；重复码不重复产生事件 |
| `LIDAR_DIAG_CHANGED/STATUS_CHANGED` | 模块诊断/综合状态发生变化 |
| `LIDAR_COMMAND_FAILED/SUBMIT_FAILED/OK` | SDK 状态、设备返回码、错误参数键；成功必须同时满足 SDK success、非空响应和设备 ret_code=0 |
| `LIDAR_HEALTH_QUERY_FAILED` | 查询失败、非法响应或温度缺失；累计失败次数不因恢复而清零 |
| `push_invalid=1` | 最近有非法推送，等待新的诊断与 HMS 刷新；综合状态不能判为正常 |
| `LIDAR_HEALTH_DROPPED` | 健康报告队列溢出；相关缓存诊断失效，等待新数据刷新 |

数据流超时沿用 `ptp_stream_timeout_seconds`（默认 0.5 秒），由每秒摘要检测；
部分推送只刷新其中的字段，温度推送不会刷新旧 HMS 的年龄。命令失败列表由该
命令的成功响应清除；工作模式重试在专用线程执行，不在 SDK 回调中等待。
`PACKET_GAP`、`PTP_REJECT/EXPIRE/MALFORMED`、`PUBLISH_STALE/SLOW`、`RX_LOCK_WAIT`、
`SDK_CALLBACK_SLOW` 和 `RAW_QUEUE_BACKLOG` 等驱动异常同时镜像到健康日志；
高频收包汇总仍保存到 `driver.log`。

HMS 解释覆盖官方列出的高温/热保护、窗口污染、升级失败、IMU 停机、供电异常、
设备参数和内部器件、扫描模块、测距 TIA、通信链路及 PTP/GPS/PPS 同步异常。
未知 ID/等级完整保留为 `unknown_hms_id/unknown`，不会忽略。只能捕获设备/SDK
实际报告以及主机链路可观测的异常；设备突然断电或网络完全中断时，可能只有
接收超时和查询失败，无法从 SDK 获知未送达的内部故障原因。
当前协议提供温度及供电异常码，不提供可读的实时电压、电流值；日志不会填造
这些数值。完整推送还保留启动计数、最后同步时刻和时间偏差，供复位/同步故障排查。

实时查看（将目录替换成控制台输出的本次会话路径）：

```bash
tail -F runlog/实际会话目录/lidar_health.log
# 只看状态和故障：
tail -F runlog/实际会话目录/lidar_health.log | \
  rg --line-buffered 'LIDAR_STATUS |LIDAR_HMS_|FAILED|PTP_|MALFORMED|BACKLOG|DROPPED'
```

复测时按 IP 和 `host_utc_ns`/日志时间，将温度曲线、HMS 与 `driver.log` 的
`PACKET_GAP`、`PACKET_SUMMARY`、`PTP_REJECT`、`RAW_QUEUE_BACKLOG` 和 `host.jsonl`
的 socket/网卡丢包增量对照。`LIDAR_HEALTH_QUERY_FAILED` 表示诊断查询失败或响应
缺少合法温度，不能解释为正常温度。温度与丢包同时升高只能说明相关性；
若温度 HMS 报警同步发生、改善散热后在相同负载下恢复，才更支持热故障归因。

协议依据：[MID360 通信协议](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html)
及 [MID360 HMS 故障码](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/hms_code_mid360.html)。
高频驱动数据按秒汇总，长间隔/阻塞单独记录；PLIO 重复等待日志按秒限流。
异步日志队列有上限，溢出会记录 `LOGGER_DROPPED`，写失败会输出
`RUNLOG_WRITE_FAILED`。普通退出会刷新日志和 rosbag；断电或 SIGKILL 无法保证最后
一批日志落盘。这些日志用于查明原因，并未改变失锁或里程计恢复策略。

离线检查（无需启动传感器）：

```bash
python3 scripts/tests/runlog_supervisor_test.py
g++ -std=c++17 -I livox/src/livox_ros_driver2/src \
  scripts/tests/ptp_expiry_diagnostics_test.cpp -o /tmp/sentry_ptp_expiry_test
/tmp/sentry_ptp_expiry_test
g++ -std=c++17 -pthread -I livox/src/livox_ros_driver2/src \
  -I livox/src/livox_ros_driver2/3rdparty -I odom/src/common \
  -I livox/src/livox_ros_driver2/.livox_sdk/include \
  livox/src/livox_ros_driver2/test/lidar_health_monitor_test.cpp \
  livox/src/livox_ros_driver2/src/comm/lidar_health_monitor.cpp \
  -L livox/src/livox_ros_driver2/.livox_sdk/lib \
  -Wl,-rpath,"$PWD/livox/src/livox_ros_driver2/.livox_sdk/lib" -llivox_lidar_sdk_sentry \
  -o /tmp/sentry_lidar_health_test
/tmp/sentry_lidar_health_test
# 构建驱动后执行日志兼容性和健康监控测试：
ctest --test-dir livox/build/livox_ros_driver2 \
  -R '^(sdk_logging_test|lidar_health_monitor_test)$' --output-on-failure
```

## 高密度建图

建图链路为双 MID360 融合点云与 IMU、PLIO（Point-LIO）里程计、Scan Context + GICP
回环检测、GTSAM Pose3/iSAM2 位姿图优化。GTSAM 源码已经归档在仓库根目录
`gtsam/`，默认静态链接，迁移仓库后不需要单独安装 GTSAM。

在 ROS 2 Humble 环境中构建里程计工作区：

```bash
cd odom
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

若开发机已经安装兼容版本的 GTSAM，可缩短本地增量构建时间：

```bash
cd odom
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DLOOP_CLOSURE_USE_SYSTEM_GTSAM=ON
```

启动完整建图：

```bash
./startup/start_mapping.sh
```

`startup/start_mapping.sh` 和 `startup/start_odom.sh` 默认使用 PLIO，可通过 `--algorithm dlio`
或 `ODOM_ALGORITHM=dlio` 显式切换。PLIO 在 `/point_lio/keyframe_cloud` 发布
`PointCloud2` 关键帧点云供 KD-tree 建图；`/point_lio/keyframe` 发布包含位姿和
相同点云的 `Keyframe` 消息供回环及 GTSAM 使用。新增输出需重新构建 `plio` 包。

选择 DLIO 时，它使用体素降采样后的稀疏点云计算里程计，但每个关键帧会额外保留并发布一份
未做体素降采样的融合点云。KD-tree 只做最近邻重复点检查，不对输入、内存地图或
保存文件做体素降采样。

建图点云只保存在节点内存中，不发布 ROS 2 地图话题，也不依赖 RViz。按 Ctrl+C
时脚本会自动保存；也可以运行期间手动保存：

```bash
ros2 service call /mapping/save_map std_srvs/srv/Trigger '{}'
```

`slam/src/map_ws/src/map.cpp` 提供与 PLIO / DLIO 关键帧点云配合的 KD-tree 增量地图：

- 保存服务：`/dlio/save_kdtree_map`
- 清空服务：`/dlio/clear_kdtree_map`
- 参数文件：`slam/config/map.yaml`

```bash
ros2 service call /dlio/save_kdtree_map std_srvs/srv/Trigger '{}'
```

KD-tree 文件使用所选算法的原始里程计位姿；GTSAM 文件使用回环优化后的关键帧位姿
重新拼接高密度关键帧。两者都以二进制 PCD 格式保存，不做体素降采样。

使用 `startup/start_mapping.sh` 时按 Ctrl+C 会先保存已有地图，再停止所有节点。默认输出为
`maps/dlio_kdtree_map.pcd`（沿用原保存路径和 `/dlio/*` 地图服务名）；启用 GTSAM 时还会输出
`maps/optimized_map.pcd`。可用 `AUTO_SAVE_MAPS=0` 关闭自动保存。

只需要 PLIO 和 KD-tree 原始地图时，可以关闭 Scan Context++ 和 GTSAM 后端：

```bash
ENABLE_GTSAM=0 ./startup/start_mapping.sh
```

回环检测、iSAM2 噪声和保存路径配置在 `odom/config/loop.yaml`。默认地图保存为
启动目录下的
`maps/optimized_map.pcd`。

第三方源码的来源、固定提交和许可证信息见 `THIRD_PARTY.md`。
