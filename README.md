# 狼牙27赛季哨兵测试代码

作者：彭友

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
