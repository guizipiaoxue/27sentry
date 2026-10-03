# 狼牙27赛季哨兵测试代码

作者：彭友

项目结构、构建顺序和数据时间戳约定见 [AGENT.md](AGENT.md)。

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
./start_odom.sh --algorithm plio

# 在另一终端 source ROS 2 与 livox/install/setup.bash 后检查采样时间。
python3 scripts/check_sensor_time.py --duration 10 --include-fused
```

`start_mapping.sh`、实机 `rotate_cali.sh` 和 `odom/start_single_lidar.sh` 使用同一
PTP 检查。回放旧包时可显式给融合节点设置 `require_ptp_sync=false`，保留录制的
采样时间；该选项仅用于回放。PTP 对齐时钟，双 IMU 采样相位通过有界插值处理。

传感器锁定等待失败时，入口会直接查询 DDS：缺少 `/livox/ptp_locked` 发布者时，
先确认新版驱动已成功构建到 `livox/install`、加载的工作区和 `ROS_DOMAIN_ID`；
已有发布者却为 false 时，检查驱动的逐设备时间诊断。仅有四条原始话题不足以证明
锁定。若从普通复制安装改用 `--symlink-install` 后出现“failed to create symbolic
link”及“existing path cannot be removed: Is a directory”，需先停止采集、备份并
清理受影响的包 build/install 后再构建；
编译失败后单纯重启仍会加载旧版。

## 高密度建图

建图链路为双 MID360 融合点云与 IMU、DLIO 里程计、Scan Context + GICP
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
./start_mapping.sh
```

DLIO 使用体素降采样后的稀疏点云计算里程计，但每个关键帧会额外保留并发布一份
未做体素降采样的融合点云。KD-tree 只做最近邻重复点检查，不对输入、内存地图或
保存文件做体素降采样。

建图点云只保存在节点内存中，不发布 ROS 2 地图话题，也不依赖 RViz。按 Ctrl+C
时脚本会自动保存；也可以运行期间手动保存：

```bash
ros2 service call /mapping/save_map std_srvs/srv/Trigger '{}'
```

`slam/src/map_ws/src/map.cpp` 提供与 DLIO 高密度关键帧配合的 KD-tree 增量地图：

- 保存服务：`/dlio/save_kdtree_map`
- 清空服务：`/dlio/clear_kdtree_map`
- 参数文件：`slam/config/map.yaml`

```bash
ros2 service call /dlio/save_kdtree_map std_srvs/srv/Trigger '{}'
```

KD-tree 文件使用 DLIO 原始里程计位姿；GTSAM 文件使用回环优化后的关键帧位姿
重新拼接高密度关键帧。两者都以二进制 PCD 格式保存，不做体素降采样。

使用 `start_mapping.sh` 时按 Ctrl+C 会先保存已有地图，再停止所有节点。默认输出为
`maps/dlio_kdtree_map.pcd`；启用 GTSAM 时还会输出
`maps/optimized_map.pcd`。可用 `AUTO_SAVE_MAPS=0` 关闭自动保存。

只需要 DLIO 和 KD-tree 原始地图时，可以关闭 Scan Context++ 和 GTSAM 后端：

```bash
ENABLE_GTSAM=0 ./start_mapping.sh
```

回环检测、iSAM2 噪声和保存路径配置在 `odom/config/loop.yaml`。默认地图保存为
启动目录下的
`maps/optimized_map.pcd`。

第三方源码的来源、固定提交和许可证信息见 `THIRD_PARTY.md`。
