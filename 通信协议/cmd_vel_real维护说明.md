# `/cmd_vel_real` 实测底盘速度维护说明

## 1. 修改目的

本次改动在视觉端新增 ROS2 话题 `/cmd_vel_real`，用于实时发布电控通过 `GD` 上行帧反馈的底盘实测速度。

调试时可以在 rqt MatPlot 中把导航期望速度 `/cmd_vel` 和电控反馈速度 `/cmd_vel_real` 放在同一张图里比较：

| 期望速度 | 实测速度 |
|----------|----------|
| `/cmd_vel/linear/x` | `/cmd_vel_real/linear/x` |
| `/cmd_vel/linear/y` | `/cmd_vel_real/linear/y` |
| `/cmd_vel/angular/z` | `/cmd_vel_real/angular/z` |

## 2. 修改位置

### `io/ros2/aim2nav.hpp`

新增 `geometry_msgs/msg/twist.hpp` 头文件，并在 `io::Aim2Nav` 中增加 publisher 成员：

```cpp
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_real_pub_;
```

### `io/ros2/aim2nav.cpp`

在 `Aim2Nav::Aim2Nav()` 中创建 `/cmd_vel_real` publisher：

```cpp
cmd_vel_real_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
  "/cmd_vel_real", 10);
```

在 `Aim2Nav::publish(const GimbalState & state)` 中发布电控反馈速度：

```cpp
geometry_msgs::msg::Twist cmd_vel_real_msg;
cmd_vel_real_msg.linear.x = state.actual_vx;
cmd_vel_real_msg.linear.y = state.actual_vy;
cmd_vel_real_msg.linear.z = 0.0;
cmd_vel_real_msg.angular.x = 0.0;
cmd_vel_real_msg.angular.y = 0.0;
cmd_vel_real_msg.angular.z = state.actual_wz;
cmd_vel_real_pub_->publish(cmd_vel_real_msg);
```

### `通信协议/Vision_Nav_ROS2_Bridge_Protocol.md`

补充 `/cmd_vel_real` 的链路说明、字段映射、验证命令和 rqt MatPlot 对比方式。

## 3. 数据来源和链路

电控反馈链路如下：

```text
RoboMaster C 型开发板
  |
  | UART GD 上行帧
  v
Gimbal::read_thread()
  |
  | ReceiveFrame.actual_vx/actual_vy/actual_wz
  v
GimbalState.actual_vx/actual_vy/actual_wz
  |
  | aim2nav_->publish(latest_state)
  v
io::Aim2Nav::publish()
  |
  | ROS2 /cmd_vel_real
  v
rqt MatPlot / ros2 topic echo
```

字段对应关系：

| 电控反馈字段 | `GimbalState` 字段 | `/cmd_vel_real` 字段 |
|--------------|--------------------|----------------------|
| `actual_vx` | `state.actual_vx` | `linear.x` |
| `actual_vy` | `state.actual_vy` | `linear.y` |
| `actual_wz` | `state.actual_wz` | `angular.z` |

`/cmd_vel_real` 不参与控制闭环，只用于调试显示。底盘下发速度仍来自 `/cmd_vel`，由 `Nav2Aim` 缓存后传给 `Gimbal::send()`。

## 4. 对运行环境的影响

- 需要 ROS2 Humble 环境中的 `rclcpp` 和 `geometry_msgs`。
- 工程原本已经依赖 `geometry_msgs`，因为 `Nav2Aim` 订阅 `/cmd_vel` 使用的也是 `geometry_msgs/msg/Twist`。
- `io/CMakeLists.txt` 已经在 ROS2 依赖存在时对 `io` 目标执行 `ament_target_dependencies(io rclcpp geometry_msgs sensor_msgs)`，因此本次改动不需要新增 CMake 依赖。
- 如果没有 ROS2 依赖，现有 CMake 逻辑会跳过 `io/ros2/aim2nav.cpp` 和 `io/ros2/nav2aim.cpp`，本次新增代码也会随之不参与编译。

## 5. 对其它代码执行的影响

- 不改变 `Gimbal::read_thread()` 的解析流程、CRC 校验、缓冲区处理和串口重连逻辑。
- 不改变 `Gimbal::send()` 的串口下发内容，底盘控制仍由 `/cmd_vel` 决定。
- 不改变 `Nav2Aim` 对 `/cmd_vel` 的订阅、缓存和读取接口。
- 不改变 `/serial/gimbal_joint_state` 的发布内容和频率。
- 每个合法 `GD` 帧会额外发布一条 `geometry_msgs/msg/Twist`，消息很小，主要影响是 ROS2 graph 中多一个 publisher 和一个调试话题。

## 6. 潜在风险

- 如果电控反馈的 `actual_vx/actual_vy/actual_wz` 单位和 `/cmd_vel` 不一致，rqt 曲线会出现比例差异。本代码不做单位换算。
- 如果电控反馈的坐标系或正方向和 `/cmd_vel` 不一致，rqt 曲线会出现方向差异。本代码不做坐标变换。
- 如果电控没有发送合法 `GD` 帧，`/cmd_vel_real` 不会更新；应先检查串口、帧头和 CRC。
- 如果同时运行多个视觉程序，可能出现多个 `/cmd_vel_real` publisher；调试时应确认只启动一个实际控制进程。

## 7. 验证方法

构建：

```bash
colcon build --packages-select rm_vision
```

运行视觉程序后检查话题：

```bash
ros2 topic list --no-daemon -t | grep cmd_vel_real
ros2 topic info /cmd_vel_real --verbose --no-daemon
ros2 topic echo /cmd_vel_real
```

回归检查：

```bash
ros2 topic info /serial/gimbal_joint_state --verbose --no-daemon
ros2 topic info /cmd_vel --verbose --no-daemon
```

期望结果：

- `/cmd_vel_real` 类型为 `geometry_msgs/msg/Twist`。
- `/cmd_vel_real` 的 publisher 节点为 `aim2nav`。
- `/serial/gimbal_joint_state` 仍由 `aim2nav` 发布。
- `/cmd_vel` 仍由 `nav2aim` 订阅。
