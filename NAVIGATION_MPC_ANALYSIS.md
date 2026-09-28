# 旧版导航模式的控制链与 MPC 构造说明

> 本文按代码中的控制算法理解提问里的“mcp”：通常该算法缩写为 **MPC**（模型预测控制）。核对 `old/navigation` 与 `old/2026_Embedded/Sentry_Steer` 后，**当前导航模式没有实现 MPC 求解器**。云台协议里的 `0x05 MPC 轨迹帧`用于云台 Yaw/Pitch 自瞄，不承载底盘速度，不能当作导航 MPC。下文先说明实际运行的公式与接口，最后给出基于现有接口构造导航 MPC 时可采用的数学模型；后者是设计草案，不是现有代码。

## 1. 现有导航控制链

| 环节 | 输入 → 输出 | 源码 |
| --- | --- | --- |
| 全局规划 | `/map`、`/goal_pose`、障碍物 → A* 路径，经平滑后发布 `/sPath` (`nav_msgs/Path`，`map` 坐标系) | [`plan_manager.cpp`](old/navigation/src/path_searching/src/plan_manager.cpp) |
| 速度管理 | 区域、速度档位 → `/setFollowSpeed` (`std_msgs/Float64`) | [`speed_manager.cpp`](old/navigation/src/speed_manager/src/speed_manager.cpp)、[`speed.yaml`](old/navigation/src/speed_manager/cfg/speed.yaml) |
| 路径跟踪 | `/sPath`、`map → base_link` TF、目标速度 → `/cmd_vel` (`geometry_msgs/Twist`) | [`PathFollower.cpp`](old/navigation/src/path_following/src/PathFollower.cpp)、[`follow_param.yaml`](old/navigation/src/path_following/cfg/follow_param.yaml) |
| 速度转发 | `/cmd_vel.linear.{x,y}` → `gimbal_driver/msg/Vel`，发布到 `/ly/navi/vel` | [`forwarder.cpp`](old/navigation/src/vel_forwarder/src/forwarder.cpp) |
| PC→云台 | 控制帧 `0x00` 中 `vel_x/vel_y` → 云台 `nav_speed_x/y` | [`pc_serial.h`](old/2026_Embedded/Sentry_Steer/Gimbal/application/inc/pc_serial.h)、[`pc_serial.c`](old/2026_Embedded/Sentry_Steer/Gimbal/application/src/pc_serial.c) |
| 云台→底盘 | 云台选择 PC 导航模式，将 `nav_speed_x/y` 放入 `chassis_solver`，经 CAN 速度字段下发 | [`ChassisSolver.c`](old/2026_Embedded/Sentry_Steer/Gimbal/application/src/ChassisSolver.c)、[`ChassisSend.c`](old/2026_Embedded/Sentry_Steer/Gimbal/application/src/ChassisSend.c) |
| 底盘执行 | CAN 解码、速度设定、舵轮逆运动学、轮/舵电机 PID | [`GimbalReceive.c`](old/2026_Embedded/Sentry_Steer/Chassis/User/src/app/GimbalReceive.c)、[`ChasisControlTask.c`](old/2026_Embedded/Sentry_Steer/Chassis/Task/src/ChasisControlTask.c)、[`steer.c`](old/2026_Embedded/Sentry_Steer/Chassis/User/src/app/steer.c) |

这里的 ROS 转发节点依赖外部 `gimbal_driver` 包；`old` 中没有找到该包实现。因此 `/ly/navi/vel` 到串口 `0x00` 的具体发布/打包逻辑，不能仅凭本仓库源码确认。协议文档说明 `gimbal_driver` 的 `/ly/control/vel` 更新会触发 `0x00` 控制帧；转发节点的 launch 文件又配置了 `/ly/navi/vel` 到 `/ly/control/vel` 的重映射。联调时应检查实际话题名称与发送方的量化规则：[`vel_forwarder.launch.py`](old/navigation/src/vel_forwarder/launch/vel_forwarder.launch.py)、[`云台上位机通信协议总览.md`](old/2026_Embedded/Sentry_Steer/docs/protocol/云台上位机通信协议总览.md)。

## 2. 现有路径跟踪公式

设全局路径点为 \(p_i^m\)，机器人位姿为 \((p_r^m,\theta)\)。`Follower` 用 `map → base_link` 的 TF 把全部路径点变换到车体系：

\[
p_i^b = R(\theta)^\mathsf{T}(p_i^m-p_r^m),\qquad
R(\theta)=\begin{bmatrix}\cos\theta&-\sin\theta\\\sin\theta&\cos\theta\end{bmatrix}.
\]

车体原点为 \((0,0)\)。代码先选欧氏距离最近的路径点 \(i_0\)，再向后找到首个满足 \(\|p_i^b\|>L\) 的点作为前视目标；找不到时仍用 \(i_0\)。前视距离为

\[
L=\max\bigl(L_{\min},\ k_L\|v_{\mathrm{current}}^b\|\bigr).
\]

令 \(q=p_{i_*}^b\)、\(d=\|q\|\)、目标速度为 \(v_s\)、终点减速系数为 \(\rho\)。实际输出是

\[
v_{\mathrm{cmd}}^b=\begin{cases}
(0,0),&d>3\ \mathrm m,\\
\dfrac{q}{d}v_s\rho,&L<d\leq3\ \mathrm m,\\
\dfrac{q}{L}v_s\rho,&0\leq d\leq L.
\end{cases}
\]

距路径终点 \(D<1\ \mathrm m\) 时，代码令 \(\rho=D\)；其他时候 \(\rho=1\)。`/cmd_vel.angular.z` 没有赋值，因此路径跟踪器只发平移速度，不发目标角速度。默认参数为 `target_speed=1.5`、`k_lookahead=0.65`、`min_lookahead=0.2`；速度管理器可通过 `/setFollowSpeed` 覆盖速度。

源码虽计算了增量 PID：\(\Delta u=K_p(e_k-e_{k-1})+K_i e_k+K_d(e_k-2e_{k-1}+e_{k-2})\)，但最终用 `control_velocity_ = target_velocity_` 覆盖了 PID 输出，所以该 PID **没有作用于发布的 `/cmd_vel`**。`getCurrentVelocity()` 的计算主体也被注释，当前速度变量没有有效反馈更新；前视距离公式不能视为可靠的实时车速自适应。到达判定中先置 `reached_flag_=true` 又立即置 `false`，因此不能依赖 `/ly/navi/reached` 的现有实现触发停车。这些行为均可在 [`PathFollower.cpp`](old/navigation/src/path_following/src/PathFollower.cpp) 中核查。

## 3. 导航模式与下位机接口

### 3.1 速度量化与帧格式

`forwarder.cpp` 对每个速度分量先限幅到 \([-2.4,2.4]\ \mathrm{m/s}\)，小于 `0.05 m/s` 的绝对值归零，再转换为 `int8`：

\[
q_x=\operatorname{int8}(50v_x),\qquad q_y=\operatorname{int8}(50v_y).
\]

云台接收的 `GimbalControlFrame_t` 是 13 字节：`head=0x21`、`type_id=0x00`、`int8 vel_x`、`int8 vel_y`、两个 `float32` 云台目标角以及 `uint8 fire_code`。云台解析速度时使用

\[
v_x^{g}=q_x/50,\qquad v_y^{g}=q_y/50,\qquad v_\omega=0.
\]

这与转发节点的 `50` 倍量化在数值上互逆；中间仍会丢失小于 `0.02 m/s` 的精度。代码未提供从 ROS 速度消息到串口帧的 `gimbal_driver` 实现，故这一步是否原样传值需要实机/驱动核对。

### 3.2 模式选择与安全状态

云台侧 `DJIRemoteUpdate()` 中，左拨杆处于中位且记录的右拨杆上一状态为下位时进入测试导航分支：要求 PC 控制帧在线，设置 `NOT_FOLLOW_GIMBAL`、云台保持当前角度、`chassis_speed_x/y = pc_control.nav_speed_x/y`、`chassis_speed_w=0`。比赛 PC 模式 `PCStateControl()` 同样读取速度，但可按 `follow_mode`/`rotate_state` 选择 `SPEED_FOLLOW` 或 `CV_ROTATE`，此时底盘 yaw 行为不同，不能与测试导航分支混为一谈。云台 `PCControlGetSnapshot()` 将 `0x00` 控制帧超过 **200 ms** 未更新视为离线，导航分支调用安全停车：[`ChassisSolver.c`](old/2026_Embedded/Sentry_Steer/Gimbal/application/src/ChassisSolver.c)、[`pc_serial.c`](old/2026_Embedded/Sentry_Steer/Gimbal/application/src/pc_serial.c)。

云台 CAN 打包为 `robot_speed_x/y = int8(30 * chassis_speed_x/y)`，`robot_speed_w = int8(7 * chassis_speed_w)`；底盘按 `/30`、`/7` 还原到 `infantry.receive_x_v/y_v/yaw_v`。随后 `wheels_accel()` 将平移目标乘以恢复斜坡系数 `chassis_recovery_scale`；当前 TD 平滑调用被注释。控制任务以约 **1 ms** 周期执行感测、舵轮正运动学、速度设定、逆运动学与电机控制。参见 [`ChassisSend.c`](old/2026_Embedded/Sentry_Steer/Gimbal/application/src/ChassisSend.c)、[`GimbalReceive.c`](old/2026_Embedded/Sentry_Steer/Chassis/User/src/app/GimbalReceive.c)、[`ChasisController.c`](old/2026_Embedded/Sentry_Steer/Chassis/User/src/app/ChasisController.c)。

### 3.3 舵轮公式接口

底盘 `NOT_FOLLOW_GIMBAL` 模式下调用 `steer_chassis_control()`。设底盘中心到舵轮中心的距离 \(R_s=0.2178\ \mathrm m\)，轮半径 \(r_w=0.057\ \mathrm m\)。逆运动学在每个轮组上将平移速度矢量与切向旋转速度矢量相加：

\[
\mathbf v_i=\mathbf v_{\mathrm{translation}}+\omega\,\mathbf t_i R_s,\qquad
\phi_i=\operatorname{atan2}(v_{iy},v_{ix}),\qquad
\dot\alpha_i=\frac{180}{\pi r_w}\|\mathbf v_i\|.
\]

其中 \(\mathbf t_i\) 的方向由第 1～4 个轮组的固定切线角 `-135°、135°、-45°、45°` 和旋转方向确定。实际代码还处理舵角最短转向/轮速反向、舵角未对齐时降低轮速、轮速 PID、舵角与舵速 PID。底盘正运动学由舵角 \(\phi_i\) 和轮速 \(s_i\) 估计：

\[
v_x^c=\frac14\sum_i s_i\cos\phi_i,\quad
v_y^c=\frac14\sum_i s_i\sin\phi_i,\quad
\omega^c=\frac1{4R_s}\sum_i\left[s_i-(v_x^c\cos\phi_i+v_y^c\sin\phi_i)\right]\cos(\phi_i-\beta_i).
\]

\(\beta_i\) 为上述切线角。随后依据云台与底盘夹角旋转平移速度；代码反馈的 `y_v` 额外带负号，这是下游协议约定。源码接口与参数见 [`steer.c`](old/2026_Embedded/Sentry_Steer/Chassis/User/src/app/steer.c)、[`steer.h`](old/2026_Embedded/Sentry_Steer/Chassis/User/inc/app/steer.h)。`STEER_TORQUE_FEEDFORWARD` 在头文件中被注释，相关整车力/转矩前馈公式是可选代码，**当前默认未编译启用**。

## 4. 若要构造导航 MPC：模型与对接点（设计草案）

当前底盘对上位机开放的是速度指令，故第一版导航 MPC 可只在 ROS 上位机预测二维位置与速度，保持下位机现有速度内环。令状态 \(z_k=[x_k,y_k,v_{x,k},v_{y,k}]^\mathsf T\) 位于 `map` 系，优化变量为加速度 \(a_k=[a_{x,k},a_{y,k}]^\mathsf T\)，采样周期 \(T\)：

\[
z_{k+1}=\underbrace{\begin{bmatrix}I_2&T I_2\\0&I_2\end{bmatrix}}_{A}z_k+
\underbrace{\begin{bmatrix}\tfrac12T^2I_2\\TI_2\end{bmatrix}}_{B}a_k.
\]

以 `/sPath` 为参考路径 \(p_k^{\rm ref}\)，沿路径生成参考速度 \(v_k^{\rm ref}\)。长度为 \(N\) 的一次优化可写为

\[
\min_{a_0,\ldots,a_{N-1}}\sum_{k=0}^{N-1}
\left(\|p_k-p_k^{\rm ref}\|_{Q_p}^2+
\|v_k-v_k^{\rm ref}\|_{Q_v}^2+
\|a_k\|_{R}^2+
\|a_k-a_{k-1}\|_{S}^2\right)
+\|p_N-p_N^{\rm ref}\|_{Q_f}^2.
\]

约束至少包括输出到 `base_link` 后的逐轴速度 \(|v_{x,k}^b|,|v_{y,k}^b|\le2.4\ \mathrm{m/s}\)（现有转发器的限幅）、\(\|a_k\|\le a_{\max}\) 和障碍安全距离 \(d_{\rm ESDF}(p_k)\ge r_{\rm robot}+d_{\rm margin}\)。实际速度上限还应服从速度管理器档位及底盘功率限制。这里的 \(N,T,Q_p,Q_v,R,S,a_{\max},r_{\rm robot},d_{\rm margin}\) 均是**待标定设计参数**；旧代码未提供它们的 MPC 数值，也没有控制器求解器。现有 ESDF/A* 用于全局规划，可为障碍约束或代价提供地图信息，但将非线性 ESDF 约束接入求解器仍需单独实现。

每轮只执行优化结果的首个速度 \(v_1^{m,*}\)，再按当前 `map → base_link` 姿态转换：

\[
v_{\rm cmd}^{b}=R(\theta)^\mathsf T v_1^{m,*}.
\]

输出沿用 `/cmd_vel.linear.x/y`，由既有转发节点与下位机速度内环执行；若求解失败、TF/路径/控制帧失效，应发布零速度并停止更新目标。该接口**没有让导航 MPC 直接控制车体 yaw**：测试导航模式固定 `w=0`，若要联合优化 \(\theta,\omega\)，还须扩展上位机到云台的速度协议和模式选择，不能仅在 MPC 状态里增加 yaw 维度。

建议的模块边界：`/sPath` + `map→base_link` + 可信速度反馈 + ESDF/局部障碍 → MPC → `/cmd_vel`。旧 `Follower::getCurrentVelocity()` 当前不可用；实施前应接入里程计/底盘速度反馈并核对坐标系。尤其 `Follower` 的 `base_link` 和底盘协议所称“云台方向速度”是否同轴，需要用 TF 与实测验证。若不同轴，必须在输出端增加显式坐标变换。

## 5. 结论

现有导航控制是 **A* + 路径平滑 + 前视速度跟踪 + 下位机舵轮/PID**。它没有 MPC 的预测时域、状态转移矩阵、目标函数、约束和滚动求解。现有“公式接口”是 `/sPath` → `/cmd_vel` → 量化速度 → 云台控制帧 `0x00` → CAN → 舵轮目标。第 4 节给出了在该接口上新增导航 MPC 的一种构造方法，不能视为已部署行为。
