# 视觉与导航 ROS2 桥接协议

**适用工程**: `rm_vision_2025` + `fwh` 导航工程  
**当前视觉入口**: `src/QYG_sentry.cpp`, `src/QYG_sentry_debug.cpp`  
**当前桥接代码**: `io/ros2/nav2aim.*`, `io/ros2/aim2nav.*`  
**核心原则**: MiniPC 到 RoboMaster C 型开发板的唯一串口由视觉端 `io/gimbal/gimbal.cpp` 负责，导航端只通过 ROS2 与视觉端通信。

## 1. 当前只保留的三条链路

```text
导航/Nav2
  |
  | ROS2 /cmd_vel
  | geometry_msgs/msg/Twist
  v
io::Nav2Aim
  |
  | latest Twist
  v
Gimbal::send(..., linear.x, linear.y, angular.z)
  |
  | UART QY 下行帧
  v
RoboMaster C 型开发板

RoboMaster C 型开发板
  |
  | UART GD 上行帧，含云台 yaw/pitch 和底盘实测速度
  v
Gimbal::read_thread()
  |
  | GimbalState.vyaw/vpitch，视觉统一符号，单位 degree
  v
io::Aim2Nav
  |
  | ROS2 /serial/gimbal_joint_state
  | sensor_msgs/msg/JointState，position 单位为 rad
  v
机器人描述 / robot_state_publisher

Gimbal::read_thread()
  |
  | GimbalState.actual_vx/actual_vy/actual_wz
  v
io::Aim2Nav
  |
  | ROS2 /cmd_vel_real
  | geometry_msgs/msg/Twist
  v
rqt MatPlot / 调试工具
```

已废弃并移除：`/ec2nav_chassis_state`、`nav_ec_communication`、`std_msgs/msg/UInt8MultiArray` 模式消息。

## 2. 导航到视觉: `/cmd_vel`

| 项目 | 内容 |
|------|------|
| 方向 | 导航 -> 视觉 |
| Topic | `/cmd_vel` |
| 类型 | `geometry_msgs/msg/Twist` |
| 视觉接收类 | `io::Nav2Aim` |
| 文件 | `io/ros2/nav2aim.hpp`, `io/ros2/nav2aim.cpp` |

字段约定：

| 字段 | 含义 | 视觉端用途 |
|------|------|------------|
| `linear.x` | 底盘 X 方向速度指令 | 原样传入 `Gimbal::send(..., linear_x, ...)`，QY 下行封包时取反 |
| `linear.y` | 底盘 Y 方向速度指令 | 原样传入 `Gimbal::send(..., linear_y, ...)`，QY 下行封包时取反 |
| `angular.z` | 底盘 Z 轴角速度指令 | 原样传入 `Gimbal::send(..., angular_z)`，QY 下行封包时取反 |
| `linear.z` | 暂不使用 | 保持 0 |
| `angular.x` | 暂不使用 | 保持 0 |
| `angular.y` | 暂不使用 | 保持 0 |

`Nav2Aim` 内部开独立 spin 线程，持续接收 `/cmd_vel`，视觉发送串口帧前读取最近一帧：

```cpp
io::Nav2Aim nav2aim;
nav2aim.start();

auto cmd_vel = nav2aim.get_latest_state();
gimbal.send(
  control,
  fire,
  yaw,
  pitch,
  cmd_vel.linear.x,
  cmd_vel.linear.y,
  cmd_vel.angular.z);
```

如果还没有收到 `/cmd_vel`，`latest_state_` 默认全 0，车辆不会动。

注意：`Nav2Aim` 缓存和 `Gimbal::send()` 调用参数保留导航原始值，便于日志和 `/cmd_vel_real` 对比；真正发给电控的 QY 下行帧在 `io::gimbal_protocol::make_send_frame()` 中统一编码为 `-linear.x`、`-linear.y`、`-angular.z`。

## 3. 视觉到机器人描述: `/serial/gimbal_joint_state`

| 项目 | 内容 |
|------|------|
| 方向 | 视觉 -> 机器人描述 |
| Topic | `/serial/gimbal_joint_state` |
| 类型 | `sensor_msgs/msg/JointState` |
| 视觉发布类 | `io::Aim2Nav` |
| 文件 | `io/ros2/aim2nav.hpp`, `io/ros2/aim2nav.cpp` |

电控上行 `GD` 帧里的 `vyaw`、`vpitch` 是角度制 degree，其中线路原始 Pitch 与视觉 Pitch 符号相反。串口协议解析时统一执行 `GimbalState.vpitch = -ReceiveFrame.vpitch`；ROS 只消费转换后的 `GimbalState`，不得再次取反。`JointState.position` 必须是弧度 rad，因此发布前只需乘 `pi / 180`。

| 字段 | 值 |
|------|----|
| `name[0]` | `gimbal_yaw_joint` |
| `position[0]` | `state.vyaw * pi / 180` |
| `name[1]` | `gimbal_pitch_joint` |
| `position[1]` | `state.vpitch * pi / 180` |

当前不发布 `velocity` 和 `effort`。

## 4. 视觉到调试工具: `/cmd_vel_real`

| 项目 | 内容 |
|------|------|
| 方向 | 视觉 -> ROS2 调试工具 |
| Topic | `/cmd_vel_real` |
| 类型 | `geometry_msgs/msg/Twist` |
| 视觉发布类 | `io::Aim2Nav` |
| 文件 | `io/ros2/aim2nav.hpp`, `io/ros2/aim2nav.cpp` |

`/cmd_vel` 是导航期望底盘速度，`/cmd_vel_real` 是电控通过 `GD` 帧反馈的底盘实测速度。二者使用同一种 `Twist` 字段布局，方便在 rqt MatPlot 中直接同字段对比。

字段映射：

| `GimbalState` 字段 | `/cmd_vel_real` 字段 | 含义 |
|-------------------|----------------------|------|
| `actual_vx` | `linear.x` | 底盘 X 方向实测速度 |
| `actual_vy` | `linear.y` | 底盘 Y 方向实测速度 |
| `actual_wz` | `angular.z` | 底盘 Z 轴实测角速度 |
| 固定 0 | `linear.z` | 暂不使用 |
| 固定 0 | `angular.x` | 暂不使用 |
| 固定 0 | `angular.y` | 暂不使用 |

rqt MatPlot 推荐对比：

| 期望速度 | 实测速度 |
|----------|----------|
| `/cmd_vel/linear/x` | `/cmd_vel_real/linear/x` |
| `/cmd_vel/linear/y` | `/cmd_vel_real/linear/y` |
| `/cmd_vel/angular/z` | `/cmd_vel_real/angular/z` |

注意：`/cmd_vel_real` 不做滤波、限幅、坐标变换或单位换算，直接发布电控反馈值。如果曲线方向或比例不一致，应先检查电控反馈协议和 `/cmd_vel` 的单位、坐标系定义是否一致。

## 5. 串口字段对应

电控上行 `GD` 帧：

| 字段 | 用途 |
|------|------|
| `actual_vx` | 底盘 X 方向实测速度，发布到 `/cmd_vel_real.linear.x` |
| `actual_vy` | 底盘 Y 方向实测速度，发布到 `/cmd_vel_real.linear.y` |
| `actual_wz` | 底盘 Z 轴实测角速度，发布到 `/cmd_vel_real.angular.z` |
| `vyaw` | 云台 yaw，degree，发布到 JointState 前转 rad |
| `vpitch` | 线路原始云台 pitch，degree；解析为 `GimbalState` 时取反，发布前只转 rad |
| `vroll` | 云台 roll，当前不发布到 JointState |

电控下行 `QY` 帧：

| `Gimbal::send()` 参数 | 来源 |
|-----------------------|------|
| `pitch` | 视觉目标 Pitch，rad；QY 封包时取反为电控线路符号 |
| `linear_x` | `/cmd_vel.linear.x`，QY 下行帧以取反后的 `float32` 发送 |
| `linear_y` | `/cmd_vel.linear.y`，QY 下行帧以取反后的 `float32` 发送 |
| `angular_z` | `/cmd_vel.angular.z`，QY 下行帧以取反后的 `float32` 发送 |

注意：`Gimbal::send()` 保留导航原始值，`make_send_frame()` 在协议边界取反并限制到 `[-1, 1]`，随后以小端 IEEE-754 `float32` 发送。如果导航输出是 m/s、rad/s，需要确保电控端和视觉端对限幅范围的理解一致。

## 6. 验证方式

启动视觉程序后检查 ROS graph：

```bash
ros2 topic list --no-daemon -t
ros2 node info /nav2aim --no-daemon
ros2 node info /aim2nav --no-daemon
```

应看到：

```text
/cmd_vel [geometry_msgs/msg/Twist]                  # nav2aim 订阅
/serial/gimbal_joint_state [sensor_msgs/msg/JointState]  # aim2nav 发布
/cmd_vel_real [geometry_msgs/msg/Twist]             # aim2nav 发布
```

验证 `/cmd_vel`：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.3, y: -0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}" -r 10
```

视觉日志中的 `tx_vx/tx_vy/tx_wz` 仍是 `Nav2Aim` 读到的原始值，应接近 `0.3/-0.2/0.1`；QY 下行帧实际发给电控的三轴编码值为 `-0.3/0.2/-0.1`。

验证 `/cmd_vel_real`：

```bash
ros2 topic echo /cmd_vel_real
ros2 topic info /cmd_vel_real --verbose --no-daemon
```

通过标准：

- `linear.x` 对应电控反馈 `actual_vx`。
- `linear.y` 对应电控反馈 `actual_vy`。
- `angular.z` 对应电控反馈 `actual_wz`。
- `ros2 topic info /cmd_vel_real --verbose --no-daemon` 能看到 `aim2nav` publisher。

验证云台位姿：

```bash
ros2 topic echo /serial/gimbal_joint_state
```

通过标准：

- `name` 包含 `gimbal_yaw_joint` 和 `gimbal_pitch_joint`。
- `position` 是弧度。云台 yaw 约 90 度时，`position[0]` 应接近 `1.57`，不是 `90`。
- `ros2 topic info /serial/gimbal_joint_state --verbose --no-daemon` 能看到 `aim2nav` publisher。

## 7. 常见错误

- 同时运行导航侧串口节点和视觉侧 `gimbal.cpp`，导致两个进程抢同一条电控串口。
- 忘记启动 `Nav2Aim::start()`，导致 `/cmd_vel` 订阅回调不执行。
- 用普通 `ros2 topic list` 被 daemon 缓存误导；排查时优先用 `--no-daemon`。
- 把电控上行的 degree 直接塞进 `JointState.position`。
- 把 `/cmd_vel_real` 当成控制输入；它只用于显示和调试，底盘控制仍来自 `/cmd_vel`。
- 忽略电控反馈单位或符号定义，导致 `/cmd_vel` 和 `/cmd_vel_real` 曲线看起来比例或方向不一致。
- 继续依赖 `/ec2nav_chassis_state`；这条链路已经废弃。
