# 狼牙27赛季哨兵测试代码

作者：彭友

## GTSAM 在线建图

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

启动完整在线建图：

```bash
./start_mapping.sh
```

RViz 的 Fixed Frame 使用 `map`。优化地图发布在 `/mapping/map`，优化轨迹发布在
`/mapping/optimized_path`，`map -> odom` 由后端发布。保存地图使用：

```bash
ros2 service call /mapping/save_map std_srvs/srv/Trigger '{}'
```

`slam/src/map_ws/src/map.cpp` 同时提供与 DLIO 关键帧直接配合的 KD-tree 增量地图：

- 原始里程计地图：`/dlio/kdtree_map`，Fixed Frame 使用 `odom`
- 保存服务：`/dlio/save_kdtree_map`
- 清空服务：`/dlio/clear_kdtree_map`
- 参数文件：`slam/config/map.yaml`

```bash
ros2 service call /dlio/save_kdtree_map std_srvs/srv/Trigger '{}'
```

KD-tree 地图用于低延迟增量显示和检查 DLIO 前端；`/mapping/map` 是经过回环优化
并重新拼接关键帧的最终地图。

只需要 DLIO 和 KD-tree 原始地图时，可以关闭 Scan Context++ 和 GTSAM 后端：

```bash
ENABLE_GTSAM=0 ./start_mapping.sh
```

回环检测、iSAM2 噪声、优化频率、地图降采样和保存路径统一配置在
`odom/config/loop.yaml`。默认地图保存为启动目录下的
`maps/optimized_map.pcd`。

第三方源码的来源、固定提交和许可证信息见 `THIRD_PARTY.md`。
