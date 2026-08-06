# `/cmd_vel` 三轴取反回溯说明

## 本次改动目的

导航通过 ROS2 发布 `/cmd_vel`，视觉端由 `Nav2Aim` 接收并传给 `Gimbal::send()`。为匹配当前电控底盘方向约定，本次改动在 QY 下行帧封包处统一取反三轴底盘速度：

- `/cmd_vel.linear.x` -> QY `linear_x` 编码为负值
- `/cmd_vel.linear.y` -> QY `linear_y` 编码为负值
- `/cmd_vel.angular.z` -> QY `angular_z` 编码为负值

`Nav2Aim` 的缓存值不取反，视觉日志中直接打印的 `cmd_vel` 或 `tx_vx/tx_vy/tx_wz` 仍表示导航原始输出。

## 修改文件和行为变化

| 文件 | 本次行为 |
|------|----------|
| `io/gimbal/gimbal_protocol.cpp` | `make_send_frame()` 编码 QY 底盘三轴前加负号 |
| `tests/protocol_ros_loop_test.cpp` | 验证 QY 帧解码后三轴等于 `/cmd_vel` 三轴的负值 |
| `通信协议/Vision_Nav_ROS2_Bridge_Protocol.md` | 记录 `/cmd_vel` 原样接收、QY 下行封包取反 |
| `通信协议/cmd_vel取反回溯说明.md` | 记录本次改动和恢复原设置步骤 |

## 原设置

恢复前的原始设置是：QY 下行帧直接编码 `Gimbal::send()` 收到的底盘速度参数，不做符号取反。

```cpp
frame.linear_x = std::clamp(linear_x, -1.0f, 1.0f);
frame.linear_y = std::clamp(linear_y, -1.0f, 1.0f);
frame.angular_z = std::clamp(angular_z, -1.0f, 1.0f);
```

对应测试期望为：

```cpp
tx.linear_x == cmd_vel.linear.x
tx.linear_y == cmd_vel.linear.y
tx.angular_z == cmd_vel.angular.z
```

## 恢复原设置步骤

1. 修改 `io/gimbal/gimbal_protocol.cpp`，去掉 `make_send_frame()` 中三轴编码前的负号，恢复为直接编码 `linear_x`、`linear_y`、`angular_z`。
2. 修改 `tests/protocol_ros_loop_test.cpp`，把 QY frame pack 中三轴断言恢复为等于 `cmd_vel.linear.x`、`cmd_vel.linear.y`、`cmd_vel.angular.z`。
3. 修改 `通信协议/Vision_Nav_ROS2_Bridge_Protocol.md`，删除或改回“QY 下行封包取反”的说明。
4. 重新构建并运行 `protocol_ros_loop_test`，确认协议闭环测试通过。

## 验证命令和预期结果

```bash
cmake --build /tmp/rm_vision_2025_verify --target protocol_ros_loop_test -j2
ROS_LOG_DIR=/tmp/ros_log /tmp/rm_vision_2025_verify/protocol_ros_loop_test
```

当前取反设置下，`protocol_ros_loop_test` 的 QY frame pack 期望：

- 输入 `/cmd_vel.linear.x = 0.42`，QY `linear_x` 解码为 `-0.42`
- 输入 `/cmd_vel.linear.y = -0.18`，QY `linear_y` 解码为 `0.18`
- 输入 `/cmd_vel.angular.z = 0.09`，QY `angular_z` 解码为 `-0.09`

恢复原设置后，上述三项应分别回到 `0.42`、`-0.18`、`0.09`。
