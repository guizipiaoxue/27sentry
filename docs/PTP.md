# MID360 与 IMU 的 PTP 时间对齐

双 MID360 的点云及各自内置 IMU 必须使用包内采样时间，并逐设备、逐数据流确认
`time_type=1`。PTP 对齐两个设备的时钟；200 Hz IMU 的采样相位仍可不同，融合通过
有界插值对齐。串口底盘/云台的 MCU tick 不在这条硬件 PTP 链路内。

## 主机和传感器的时间尺度

当前两个设备 `192.168.1.5` 和 `192.168.1.3` 经 `enp86s0` 到主机
`192.168.1.50`，共享该网卡的 `/dev/ptp0`。硬件 `ptp4l` 使用 PHC/PTP 时间尺度，
`phc2sys -s CLOCK_REALTIME -c enp86s0 -w` 将系统 UTC 对齐到 PHC，维持
`PHC = CLOCK_REALTIME + currentUtcOffset`。示例 offset 为 37 秒；应核对本机 PTP
配置和 PMC 宣告值，不能根据首包与主机接收时间的差值自动拟合。

ROS header 必须是 Unix/UTC 纳秒。本机现有设备实测包时间比主机 UTC 提前约
37 秒，当前硬件 PTP 链路使用 `PTP_UTC_OFFSET=37`，传递给驱动
`ptp_utc_offset_seconds=37` 做统一减法。其他设备固件包的时间尺度仍须通过
协议/实测确认，与驱动的 UTC 转换配置一致。
主机 `CLOCK_TAI` 的内核偏移可能尚未设置（本机只读检查为 0），不能用它推断
有效的 UTC offset。

## 安装项目 PTP 服务与复用

项目服务由三个模板组成：`sentry-phc-bootstrap@enp86s0.service` 在网卡 carrier
和 PHC 可用后确认 `PHC = UTC + 37 秒`；随后 `sentry-ptp4l@enp86s0.service`
提供固定 MASTER，`sentry-phc2sys@enp86s0.service` 用 `-S 0.0 -F 0.0` 禁止
持续 servo 的运行期及首次更新跳变。配置安装到
`/etc/linuxptp/sentry/master-enp86s0.conf`，UDS 为 `/run/ptp4l-enp86s0`。

bootstrap 完全不调整系统 `CLOCK_REALTIME`。PHC 已匹配 1 ms 容差时直接跳过
step，包括采集仍运行的服务切换；偏差超限且存在已知 Livox/LIO/录包进程或其他
PHC 写入进程时拒绝 step。未来开机在开始采集前仅允许一次 PHC 初始化。`network-online.target`
不能替代实际 carrier/PHC 检查，bootstrap 会等待接口就绪。外部采集程序也应在
上述服务和主机预检通过后启动，避免与开机 PHC 初始化并发。

```bash
sudo bash scripts/install_ptp_services.sh --interface enp86s0 --utc-offset 37
systemctl status sentry-phc-bootstrap@enp86s0 sentry-ptp4l@enp86s0 sentry-phc2sys@enp86s0
journalctl -u sentry-phc-bootstrap@enp86s0 -u sentry-phc2sys@enp86s0 --no-pager -n 30
```

安装脚本会备份并停用本机旧的 `ptp4l-master-enp86s0`、
`phc2sys-master-enp86s0`、`odom-phc-bootstrap`、`odom-clock-bootstrap` 四个服务，
清理其专用 unit、enabled 链接、旧 master 配置和 `/usr/local/libexec/odom-clock`
helper，保留 `linuxptp` 工具、通用配置及现有 NTP/chrony/timesyncd mask。
备份路径在安装输出中报告，可用 `--backup-dir` 显式指定。它不停止 ROS/驱动采集，
也不会调整系统时钟；发现外部同 PHC servo 或服务依赖冲突则退出。

相同内容重复安装复用新服务。已启动的新服务若配置/脚本发生变化，安装脚本拒绝
隐式重启；应在维护窗口先停止新 PTP 服务，再部署改变。采集入口仅做读取和验证，
退出时不停止这些系统服务，避免多个 servo 同时调整同一个 PHC。
两个持续服务均配置 `Restart=on-failure`，异常退出后自动恢复；bootstrap 作为
静态依赖随 MASTER 服务启动，不需要单独 enable。

本机已于 2026-10-03 16:16 CST 完成上述替换，两个持续服务均为 active/enabled，
bootstrap 为 active/exited，实际动作是 `already_aligned_no_step`。
旧服务及专用文件的备份位于
`/var/backups/sentry-ptp/20261003T081649.531783600Z`。
5 秒稳定窗口的严格 PMC/PHC 检查已通过；详细证据见 `PTP_VALIDATION.md`。

```bash
python3 scripts/ptp_check.py \
  --config livox/src/livox_ros_driver2/config/MID360_config_2.json \
  --utc-offset 37 --settle-seconds 3 --timeout 30
```

检查每台设备路由、配置中的点云/IMU 目标主机地址、网卡 carrier、硬件能力、
唯一 `ptp4l`、domain 0、MASTER 角色、UTC offset、唯一 PHC servo 的方向、禁止
step 的参数，以及新鲜的同服务 invocation `s2`/`s3` PHC 锁定日志。采集开始前需
连续满足稳定窗口；主机时间在窗口内发生大于 10 ms 的跳变则拒绝启动。

有访问权限时额外直接读取 PMC 的端口状态/time properties 和 PHC。普通用户若
无法访问 root 所有的 PTP socket 或 PHC，默认明确使用当前进程配置和同 invocation
journal 证据；不会声称已经直接测得 PHC，也不会把此检查当作传感器已失锁/锁定的
证明。严格检查可由有相应权限的运维账户执行：

```bash
sudo python3 scripts/ptp_check.py \
  --config livox/src/livox_ros_driver2/config/MID360_config_2.json \
  --utc-offset 37 --strict-pmc --json
```

该命令只读，不启动 daemon、不改服务、不改网卡、不校系统时间。PMC 使用临时
客户端 Unix socket 并在结束时移除。若失败，按照具体错误检查现有服务和权限，
不要通过再次运行 `phc2sys` 来掩盖问题。`journalctl` 必须对执行账户可读，且现有
PHC 服务需要启用周期锁定日志。当前默认 PHC 误差上限为 1 ms；可通过
`--max-phc-offset-ms` 调整，离线测试不等于已经完成传感器硬件验收。

## 多网卡和新主机配置示例

`config/ptp/master-hardware.conf` 与 `master-software.conf` 是人工审阅后应用的
模板；安装脚本将硬件模板按指定网卡和 offset 写入项目专用 `/etc/linuxptp/sentry`
目录。硬件网卡必须经 `ethtool -T IFACE` 或等效
`ETHTOOL_GET_TS_INFO` 确认支持硬件收发时间戳和 PHC。`/sys/class/ptp` 出现某设备
并不保证某个网口支持可用的 Ethernet PTP timestamping。

硬件模板的持续同步命令对应自己的 UDS：

```bash
ptp4l -f config/ptp/master-hardware.conf -m
phc2sys -s CLOCK_REALTIME -c enp86s0 -w -z /run/ptp4l-enp86s0 \
  -m -q -x --max_frequency=100000 -S 0.0 -F 0.0
```

仅在不存在既有实例时配置上述服务。若 PHC 与 UTC 还相差很大，禁用 step 的
servo 不会迅速收敛；初始校正在所有传感器采集前由运维完成，不能在正在运行的
采集链路中临时 step。

无 PHC 的 USB 网卡只运行独立软件 `ptp4l`，其时间戳直接采用系统 UTC，无需
`phc2sys`。软件方案的误差由网络、调度和驱动决定，必须实测：

```bash
ptp4l -f config/ptp/master-software.conf -m
```

软件链路的检查和采集均需显式设置 UTC 偏移为 0：

```bash
python3 scripts/ptp_check.py --config /path/to/software-lidar.json --utc-offset 0
PTP_UTC_OFFSET=0 ./start_odom.sh
```

单个驱动实例只有一个包时间 UTC 偏移。若硬件 PHC 的设备包采用 TAI、软件设备包
采用 UTC，必须分别使用对应 offset 的驱动实例并明确 remap 各自话题。默认启动
脚本不支持把这两种 profile 混进同一 driver；检查会拒绝不一致的配置。

硬件与软件网口使用各自的 `ptp4l` 实例和不同 UDS，不能在同一实例中混用。
多个不同 PHC 的硬件网口可各起一个固定 MASTER 实例及一个系统 UTC 到该 PHC
的 servo。单个多端口硬件实例仅在共享 PHC 时自然成立；不同 PHC 的
`boundary_clock_jbod` 还需要外部同步，并可能涉及自动源切换，不作为本项目默认。
修改网络拓扑后先修正 Livox JSON 的主机 IP，再验证每台设备 route。

## 卸载或暂停

停用本项目的单个网卡实例，保留其他网卡实例和 `linuxptp` 可执行工具：

```bash
sudo systemctl disable --now sentry-phc2sys@enp86s0.service sentry-ptp4l@enp86s0.service
sudo systemctl stop sentry-phc-bootstrap@enp86s0.service
```

需要彻底卸载该实例时，再移除 `/etc/linuxptp/sentry/master-enp86s0.conf` 并运行
`sudo systemctl daemon-reload`。只有不存在其他项目 PTP 实例时，才能清理三个
`/etc/systemd/system/sentry-*@.service` 模板和 `/usr/local/libexec/sentry-ptp`
helper。备份保留以便审阅恢复，恢复旧配置也应先确认没有第二个 PHC servo。
停用 PTP 会造成设备失锁，严格驱动/融合应停止可信输出；暂停服务不是停止采集。

## 传感器验收

驱动严格模式默认 `require_ptp_sync=true`、`ptp_utc_offset_seconds=37`，点云和
IMU 使用统一偏移转换；`/livox/ptp_locked` 仅在配置的每个设备两条数据流全部
满足 PTP 同步、新鲜度和时间递增检查时为 true。默认实机入口还检查原始流
新鲜度，要求 `use_sim_time=false`。驱动和融合的 PTP 参数为只读启动参数，
修改策略需要重启节点，运行时 `ros2 param set` 会被拒绝。
回放已经录制的历史数据不需要重新对设备
锁定，可给融合节点显式设置 `require_ptp_sync=false`，保留原始采样时间；若直接
回放驱动输出的录包，不要对 UTC header 再减 37 秒。

完整构建顺序如下。本机已于 2026-10-03 16:42 CST 使用普通 install 路径的新版本
完成实机启动和 10 秒时间戳检查，测试进程随后全部停止。今后更新普通 install
路径仍须先停止实机采集；采集期间验证使用独立 build/install/log 路径。

```bash
source /opt/ros/humble/setup.bash
cd livox
colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
cd ../slam
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
cd ../odom
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

重载新版采集入口后，在另一个终端进行实际硬件验收：

```bash
source /opt/ros/humble/setup.bash
source livox/install/setup.bash
python3 scripts/check_sensor_time.py --duration 10 --include-fused
```

monitor 检查 PTP 状态的新鲜度、四条原始流及融合流的 UTC header/时间递增、
`CustomMsg.header == timebase` 和双雷达帧的可配对覆盖，返回 JSON 与成败退出码。
`--max-age` 默认 2 秒、`--max-pair-delta-ms` 默认 30 ms，可按设备发布策略调整。
帧 header 差值衡量扫描相位与覆盖，不等于硬件 PTP 精度测量。旧驱动的 header
实测提前约 37 秒，monitor 会正确拒绝；新版实机已通过上述检查，设备精度及失锁
恢复测试仍需单独验证。

主机检查通过后，驱动仍需等到两台设备的点云和 IMU 四条数据流分别提供有效
PTP 时间戳，过滤未同步/倒退/跳变包，并持续检查失锁。检查点云逐点绝对时间
`header + time` 和 IMU 采样时间处于同一 UTC 时间线上；近邻 IMU/点云包时间
无需完全相等，插值覆盖必须有界。录包保留四条原始数据流及同步诊断，断开 PTP
再恢复时应确认没有不可信的融合数据继续送给里程计。
