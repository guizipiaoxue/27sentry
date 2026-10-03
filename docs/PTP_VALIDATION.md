# PTP 修改验证记录

验证日期：2026-10-03。本次先梳理独立工作区、实机入口及所有传感器生产者和
消费者，并将项目地图写入根目录 `AGENT.md`；`AGENTS.md` 引用该文件。

## 替换服务前的只读基线

- 两台 MID360（`192.168.1.5`、`192.168.1.3`）均经 `enp86s0` 到主机
  `192.168.1.50`，该网卡具有硬件收发时间戳和 `/dev/ptp0`。
- 原有 `ptp4l-master-enp86s0.service` 使用硬件时间戳、domain 0、MASTER；
  `phc2sys-master-enp86s0.service` 以系统 UTC 为源同步 PHC，禁止运行期及首次
  servo step。主机预检通过，同一服务 invocation 的采样日志曾显示 `s2`
  且 offset 为 `-1 ns`。该值是主机 servo 日志，不代表雷达同步精度。
- 最初以普通账户检查时无法直接读取 PHC，PMC 查询也未返回数据。预检明确采用活跃进程
  配置及同 invocation journal，并标注 `sensor_lock_verified=false`。
  有权限的运维账户可用 `--strict-pmc` 要求直接 PMC/PHC 证据。
- 在现有旧驱动上订阅四条原始流，IMU 的 header 比主机 UTC 超前约
  `36.97–37.00 s`，点云约 `36.63–36.84 s`，符合未经转换的 TAI 时间。
  `check_sensor_time.py` 对全部样本报告 `outside_host_utc_window`；旧驱动
  没有 `/livox/ptp_locked`，因此验收结果为 false。
- 37 对点云 header 的最大差值为 `0.243654 ms`。这个数字只说明两条扫描流
  在该采样窗口内可以配对，不能作为硬件 PTP 精度证明。

## 离线构建及功能测试

全部编译使用 `/tmp/sentry_ptp_verify/` 内独立的 build/install/log 目录，
未覆盖正在运行的驱动库、停止采集、改动系统时钟或重启 PTP 服务。

| 边界 | 验证内容 | 结果 |
| --- | --- | --- |
| Livox 驱动 | ROS 2 构建；整数纳秒 TAI→UTC、四流隔离、未同步/倒退/错误时间尺度拒绝、超时及恢复、分帧跨界、真实点云/IMU 队列的 generation 和时间戳保真 | 通过 |
| 融合节点 | ROS 2 构建；时间守卫；实际节点的未锁定门控、逐点有符号时间、IMU 插值、失锁/心跳超时/错误包清缓存与重新标定、显式仿真回放 | 通过 |
| DLIO/PLIO | `odom_ws`、`plio`、`single_test` 构建；运动积分、静止 IMU、预测及协方差四项 CTest | 通过 |
| PLIO 实际节点 | 含 `-20 ms` 点时间的点云仍产生有效关键帧；保留点时间字段 | 通过 |
| 主机预检、监视和 PHC bootstrap | 36 项 Python unittest，覆盖路由/网卡/权限证据、servo、时间尺度、37 秒错误、丢失流、一次性配对、仅 PHC 初始化、采集及竞争 writer 拒绝 | 通过 |
| 启动入口及系统服务 | 修改过的 Bash 脚本语法、Python 编译、systemd 单元验证、`git diff --check` | 通过 |

融合包的完整 lint 包含历史 `fusion_pcl.cpp` 全文件格式错误；从 HEAD 提取的
未修改版本同样不能通过 uncrustify。新增时间守卫和测试文件单独通过格式检查。
没有为本次时间同步修改重排整个旧文件。功能测试通过不等于完整 lint 全绿。

可重复执行不接触实机的项目级检查：

```bash
bash -n start_odom.sh rotate_cali.sh odom/start_single_lidar.sh start_pcl.sh scripts/ptp_runtime.sh scripts/install_ptp_services.sh
git diff --check
```

## 项目 PTP 服务实际部署

用户授权替换旧服务后，于 2026-10-03 16:16 CST 执行项目安装脚本。
此前的只读和隔离构建记录仍为替换前证据；本次实际停止了旧 PTP 服务并启动新服务。
未调整系统 UTC、修改网卡地址或停止当前 ROS 驱动/录包。

- 备份目录：`/var/backups/sentry-ptp/20261003T081649.531783600Z`。
  四个旧 unit、两个 enabled 链接、旧 master 配置和 `odom-clock` helper 均已核对备份存在；
  原路径已清理，四个旧服务的 `LoadState=not-found`。
- `sentry-ptp4l@enp86s0.service` 和 `sentry-phc2sys@enp86s0.service`
  均为 active/running、enabled，分别为 PID 32284/32285，自动重启次数为 0。
  全机仅有这一个 `ptp4l` 和一个 `phc2sys`，没有额外 `ts2phc`/`phc_ctl`。
- `sentry-phc-bootstrap@enp86s0.service` 为 active/exited、static，通过依赖启动。
  实际报告 `already_aligned_no_step`、`system_clock_changed=false`，
  初始化检查的 PHC 偏差为 `-548 ns`、读取不确定度 `1352 ns`，未打开 PHC 写入路径。
- 以 root 运行 `ptp_check.py --utc-offset 37 --strict-pmc --timeout 30 --settle-seconds 5 --json`
  返回 `ok=true`。直接 PMC GET 确认 hardware/domain 0/MASTER；
  同次服务 invocation 的 PHC servo 为 `s2`，offset `-34 ns`，
  frequency `7107 ppb`，低于 `100000 ppb` 限制。
  直接读取 PHC−UTC 为 `37000038920 ns`，读取不确定度 `44022 ns`，
  满足 UTC＋37 秒和 1 ms 门限。`sensor_lock_verified=false` 保持明确标记。
- 普通账户的 3 秒稳定窗口预检也返回 `ok=true`，采用新服务的活跃进程配置和
  同 invocation journal；无 PMC/PHC 读取权限的限制被明确输出，采集入口可继续使用。
- 已安装的三个 unit 和 bootstrap helper 与项目源码逐字节一致。
  `linuxptp` 工具仍保留；chrony、chronyd、timesyncd 和 hwclock 的 mask 未改变。
  采集 PID 17658/17660 和录包 PID 17686 的启动时间保持不变。

以上是主机服务部署验收；servo offset 和直接 PHC 读取不代表两台雷达的硬件同步精度。

## 新版实际启动与时间戳检查

随后用户两次启动遇到传感器锁定超时。核查实际加载的普通 `livox/install`
驱动库缺少 `/livox/ptp_locked`，四路 header 仍比 UTC 超前约 37 秒。
16:30 的构建日志和再次执行构建均复现同一错误：从复制安装切换到
`--symlink-install` 时，已有 Python 生成目录无法替换为符号链接；旧库因此未更新。

确认无采集进程运行后，旧驱动 build/install 包目录备份到
`/tmp/sentry_ptp_install_backup_20261003T083542Z`，清理对应生成路径并重新构建。
普通安装目录中的 Livox、`fusion_ws`、`odom_ws`、`plio` 和 `single_test` 均已成功
构建。驱动库包含新 PTP 策略和状态发布者；驱动时间契约 1/1、融合时间守卫和
实际节点集成测试 2/2、DLIO/PLIO 四项算法回归均通过。

第一次新版实机启动已通过 PTP 和融合标定，随后 DLIO 拒绝整数类型的
`odom/imu/calibration/time`。默认 YAML 从 `3` 修正为 `3.0`，保持相同标定时长。
真实节点的隔离构造器回归先复现异常，再通过参数服务校验浮点及数组类型。
失败门禁也新增直接 DDS 诊断，区分缺少状态发布者和状态 false，继续拒绝启动。
新增回归后的项目工具测试共 43 项，全部通过，包含真实 DDS 图及 DLIO 参数服务。

2026-10-03 16:42 CST 通过 `bash start_odom.sh --no-record-bag` 验证实际完整链路，
保持默认 GTSAM 后端：新版策略日志出现、全部设备 PTP locked、双 IMU 标定完成、
DLIO 及回环/建图后端启动，DLIO health 为 GOOD 并发布里程计。
同步执行 10 秒检查：

```bash
source /opt/ros/humble/setup.bash
source livox/install/setup.bash
python3 scripts/check_sensor_time.py --duration 10 --include-fused
```

实测返回 `ok=true`、`ptp_locked=true`；四条原始流及两条融合流均有样本，
UTC、新鲜度、递增及 `CustomMsg.header == timebase` 检查没有错误。
两路雷达配对 96 帧，无未配对样本，最大 header 差 `0.404634 ms`。
验证日志位于 `/tmp/sentry_ptp_startup_verify_20261003T084151Z/`，包含
`startup.log` 和 `sensor_time.json`。验证结束向自己的 supervisor 发送 SIGINT，
退出码 130，确认没有残留测试子进程；PTP 系统服务继续运行。

失锁停发/清缓存及恢复重标定已有离线集成验证，但仍需可中断的硬件失锁测试。
设备最终同步误差需要设备诊断或独立测量，不能由主机 servo offset 或两条
header 差值推算。

按用户要求，本次新增测试源码、测试构建注册、隔离验证目录及临时验证日志已清理；
上述测试结果作为历史记录保留，临时路径不再可用。原有项目测试和备份仍保留。
