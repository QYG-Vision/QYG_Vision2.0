# 云台串口数据流

1. `Gimbal::read_thread()` 从串口缓冲区寻找 51 字节 GD 帧，经 CRC 校验后解析为 `GimbalState`。
2. GD offset 2 的 `current_mode` 是导航任务状态，当前只保存在 `GimbalState` 和 `Aim2Nav` 缓冲区中，暂不新增发给 fwh 的 ROS2 topic。
3. GD offset 35~36 是小端 `uint16_t sentry_state`：高 2 bit 为视觉模式，低 14 bit 为机器人状态。
4. GD offset 37~48 是云台 Yaw/Pitch/Roll；Pitch 在协议解析边界转换成视觉统一符号。
5. `Aim2Nav` 发布 `/serial/gimbal_joint_state` 和 `/cmd_vel_real`；旧 `/ec2nav_chassis_state` 链路已废弃。
6. `Nav2Aim` 接收 `/cmd_vel`，视觉解算开火、Yaw、Pitch 后由 `Gimbal::send()` 生成 25 字节 QY 帧。
7. QY 的五个数值字段使用小端 IEEE-754 `float32`；三轴速度和 Pitch 在协议边界按当前电控坐标约定取反。
8. GD 和 QY 的 CRC 计算结束后直接返回，不执行最终异或。

完整字节布局见 `通信协议/MiniPC_Protocol_Doc.md`。
