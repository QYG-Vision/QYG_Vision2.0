# Tests 测试程序文档

> **目标读者**: 刚入门的学弟学妹  
> **更新日期**: 2026-05-15

## 📋 目录

1. [概述](#概述)
2. [分类速查表](#分类速查表)
3. [测试程序详解](#测试程序详解)
4. [编译与运行](#编译与运行)
5. [使用建议](#使用建议)
6. [常见问题](#常见问题)

---

## 概述

`tests/` 目录包含了 `rm_vision` 项目中所有的测试和调试程序。每个程序都是独立的可执行文件（有自己的 `main()` 函数），用于验证特定模块的功能是否正常。

**注意**：这些程序不是常规的单元测试（如 gtest）。它们是实实在在的程序，会打开相机、连接串口、读取文件等。可以理解成"用来调试某个模块的小工具"。

### 分类说明

测试程序可根据运行方式分为两类：

- **离线测试**：读取录制好的视频文件（`assets/demo/demo.avi`）和云台数据（`assets/demo/demo.txt`），不需要连接任何硬件。适合在没有机器人的情况下调试算法。
- **在线测试**：需要连接相机、串口、控制板等硬件，在真实机器人上运行。

---

## 分类速查表

| 类别 | 程序 | 状态 | 在线/离线 | 需要硬件 |
|------|------|------|----------|---------|
| 自瞄系统 | `auto_aim_test` | ✅ 已启用 | 离线 | ❌ 无 |
| 自瞄系统 | `minimum_vision_system` | ✅ 已启用 | 在线 | ✅ 相机 |
| 自瞄系统 | `camera_detect_test` | ⬜ 已注释 | 在线 | ✅ 相机 |
| 自瞄系统 | `camera_thread_test` | ✅ 已启用 | 在线 | ✅ 相机 |
| 自瞄系统 | `detector_video_test` | ⬜ 已注释 | 离线 | ❌ 无 |
| 自瞄系统 | `usbcamera_detect_test` | ⬜ 已注释 | 在线 | ✅ USB相机 |
| 能量机关 | `auto_buff_test` | ⬜ 已注释 | 离线 | ❌ 无 |
| 云台通信 | `gimbal_test` | ✅ 已启用 | 在线 | ✅ 串口+EC |
| 云台通信 | `gimbal_response_test` | ⬜ 已注释 | 在线 | ✅ 云台 |
| 云台通信 | `serial_test` | ✅ 已启用 | 在线 | ✅ 串口 |
| 云台通信 | `serial_debug` | ⬜ 已注释 | 在线 | ✅ 串口 |
| 云台通信 | `fire_test` | ⬜ 已注释 | 在线 | ✅ 云台 |
| 轨迹规划 | `planner_test` | ⬜ 已注释 | 在线 | ✅ 云台 |
| 轨迹规划 | `planner_test_offline` | ⬜ 已注释 | 离线 | ❌ 无 |
| 相机 | `camera_test` | ⬜ 已注释 | 在线 | ✅ 相机 |
| 相机 | `multi_usbcamera_test` | ⬜ 已注释 | 在线 | ✅ USB相机 |
| 传感器 | `cboard_test` | ⬜ 已注释 | 在线 | ✅ C板 |
| 传感器 | `dm_test` | ⬜ 已注释 | 在线 | ✅ DM-IMU |
| 标定 | `handeye_test` | ⬜ 已注释 | 在线 | ✅ 相机+云台 |
| 通信闭环 | `protocol_ros_loop_test` | ✅ 已启用 | 离线 | ❌ 无 |
| ROS2 | `nav2aim_test` | ✅ 已启用 | 在线 | ✅ ROS2 |
| ROS2 | `publish_test` | ⬜ 已注释 | 在线 | ✅ ROS2 |
| ROS2 | `subscribe_test` | ⬜ 已注释 | 在线 | ✅ ROS2 |
| ROS2 | `topic_loop_test` | ⬜ 已注释 | 在线 | ✅ ROS2 |
| USB相机 | `usbcamera_test` | ⬜ 已注释 | 在线 | ✅ USB相机 |

---

## 测试程序详解

### 一、自瞄系统测试

#### auto_aim_test（离线自瞄回放）

这个是**最重要的调试工具**。它读取录好的视频和云台数据，离线跑一整套自瞄流程。

**功能**：
1. 打开 `.avi` 视频文件和 `.txt` 云台姿态文件
2. 对每帧图像运行 YOLO 检测 → EKF 跟踪 → Aimer 瞄准
3. 在屏幕上显示检测结果（绿框=EKF预测、红框=Aimer瞄准点）
4. 通过 Plotter 发送 EKF 状态数据（位置、速度、角速度、卡方检验等）

**用途**：
- **调 EKF 参数**：改 `configs/QYG_sentry.yaml` 里的 `ekf_*` 参数，重跑同一段视频，对比效果
- **调检测参数**：改 YOLO 置信度、曝光时间等，看检测效果变化
- **验证算法修改**：改了跟踪或瞄准代码后，先用离线视频验证再上实车

**特别说明**：由于是读文件，无论跑多少次结果都一样（可复现），这是发现"改参数后到底变好还是变差"的唯一可靠方法。

```bash
cmake --build build --target auto_aim_test -j$(nproc)
./build/auto_aim_test --config-path=configs/demo.yaml assets/demo/demo
```

---

#### minimum_vision_system（最小视觉系统）

最精简的在线自瞄系统，不依赖云台串口也能运行（用 DM-IMU 代替云台 IMU）。

**功能**：读相机 → YOLO 检测 → EKF 跟踪 → Aimer 瞄准 → 显示结果

**用途**：验证"从相机到瞄准"的完整算法链路是否通畅，排查问题时用于隔离串口相关故障。

```bash
cmake --build build --target minimum_vision_system -j$(nproc)
./build/minimum_vision_system --config-path=configs/QYG_sentry.yaml
```

---

#### camera_detect_test（在线检测对比）

实时显示相机画面，同时用传统 CV 方法和 YOLO 检测装甲板，方便对比两种方法的差异。适合调试检测阈值和模型参数时使用。

**状态**：已注释，需在 `CMakeLists.txt` 中取消注释后编译。

---

#### camera_thread_test（多线程检测）

使用线程池把 YOLO 检测放到独立线程，主线程只负责读取相机和显示画面，帧率更高。

**已启用**，可直接编译运行。

---

#### detector_video_test（离线检测对比）

和 `camera_detect_test` 类似，但读的是视频文件而不是实时相机。适合在无硬件时调试检测参数。

**状态**：已注释。

---

#### usbcamera_detect_test（USB 相机检测）

使用 USB 相机（而非工业相机）运行 YOLO 检测。如果你用的是 USB 相机，可以用这个测试。

**状态**：已注释。

---

### 二、能量机关测试

#### auto_buff_test（离线能量机关回放）

和 `auto_aim_test` 类似，但针对的是能量机关（大符/小符）。读取录好的能量机关视频，离线运行检测→跟踪→瞄准流程。

**状态**：已注释。

---

### 三、云台和串口测试

#### gimbal_test（云台调试工具）

**这是调试云台硬件最重要的工具**。它不运行任何视觉算法，只做两件事：

1. **接收**：从 EC 接收 GD 帧，显示云台当前的真实角度
2. **发送**：向 EC 发送固定的 QY 控制指令，让云台转到指定角度

**用途**：
- 验证云台是否能正常响应指令
- 判断云台抖动是硬件问题还是算法问题
- 测试串口通信是否稳定

```bash
cmake --build build --target gimbal_test -j$(nproc)
./build/gimbal_test -f configs/QYG_sentry.yaml
```

程序会打开摄像头，画面上显示两行信息：
- 绿色 `TX: yaw=0.0deg pitch=10.0deg` — 你发给 EC 的目标角度
- 青色 `RX: yaw=9.8deg pitch=10.1deg roll=0.5deg` — EC 回传的真实角度

如果 RX 的 yaw 接近你发的 TX yaw，说明通信和控制链路正常。如果到位后云台还在抖，那是电机 PID 或机械问题。

---

#### serial_test（串口通信测试）

只读不写，单纯查看云台状态（模式、角度、弹速），不发送任何控制指令。适合先确认串口能正常收到数据。

**已启用**。

---

#### serial_debug（串口调试）

持续读取云台状态，并向 EC 发送固定指令。和 `gimbal_test` 类似但不带摄像头。

**状态**：已注释。

---

#### fire_test（开火测试）

模拟目标轨迹，测试云台跟随和开火逻辑，并录制结果供回放分析。

**状态**：已注释。

---

#### gimbal_response_test（云台响应测试）

给云台发送三角波或正弦波信号，测量云台的实际跟踪精度和延迟。

**状态**：已注释。

---

### 四、轨迹规划测试

#### planner_test（在线规划器测试）

在实车上测试 MPC 轨迹规划器。配置模拟目标的距离和角速度，观察云台是否按规划轨迹平滑移动。

**状态**：已注释。

---

#### planner_test_offline（离线规划器测试）

不接云台，纯仿真测试 MPC 轨迹规划器。输出 JSON 格式数据用于分析轨迹平滑度和加速度限制是否合理。

**状态**：已注释。

---

### 五、相机测试

#### camera_test（相机基础测试）

最简单的相机读图测试，显示画面并打印帧率。用来确认相机是否正常、帧率是否达标。

**状态**：已注释。

#### multi_usbcamera_test（多相机测试）

同时测试 2 个 USB 相机和 1 个工业相机，测量各自的帧率。

**状态**：已注释。

#### usbcamera_test（USB 相机测试）

单独测试 USB 相机的图像采集。如果你换成 USB 相机，先跑这个确认能正常工作。

**状态**：已注释。

---

### 六、传感器测试

#### cboard_test（C 板 IMU 测试）

读取 C 板内置 BMI088 IMU 的欧拉角和弹速数据。用来确认 CAN 通信和 IMU 是否正常。

**状态**：已注释。

#### dm_test（达妙 IMU 测试）

读取达妙（DM）系列 IMU 模块的欧拉角。如果你用外置 IMU 代替 C 板 IMU，先跑这个测试。

**状态**：已注释。

---

### 七、标定测试

#### handeye_test（手眼标定验证）

把标定板的角点从世界坐标转换到相机坐标和像素坐标，验证相机-云台标定结果的精度。用重投影误差衡量标定好坏。

**状态**：已注释。

---

### 八、ROS2 测试

以下测试只有在编译时检测到 ROS2 环境（`rclcpp` + `geometry_msgs` + `sensor_msgs`）才会被编译。

#### nav2aim_test（导航桥测试）

监听 ROS2 的 `/cmd_vel` 话题，验证导航模块发来的底盘速度指令能否正确转发到串口。

**已启用**（条件编译）。

#### protocol_ros_loop_test（无外设通信闭环测试）

不打开串口、不连接电控、不连接相机。测试程序会模拟 EC 发出的 GD 上行帧，验证视觉端协议解析、`/cmd_vel_real` 和 `/serial/gimbal_joint_state` 发布、`/cmd_vel` 接收，以及 QY 下行帧打包和 CRC。

**用途**：
- 改 `ReceiveFrame`、`SendFrame` 或字段名后，确认串口协议布局没有被破坏。
- 改 `/cmd_vel`、`/cmd_vel_real`、`/serial/gimbal_joint_state` 链路后，确认 ROS2 通信仍能闭环。
- 上车前先在开发机做一次无硬件协议自检。

```bash
cmake --build build --target protocol_ros_loop_test -j$(nproc)
./build/protocol_ros_loop_test
```

**已启用**（条件编译）。

#### publish_test（发布测试）

向 ROS2 话题发布装甲板检测数据，1Hz 频率。用来验证 ROS2 发布通路是否正常。

**状态**：已注释。

#### subscribe_test（订阅测试）

监听敌方状态话题，记录无敌的机器人 ID。用于验证 ROS2 订阅通路。

**状态**：已注释。

#### topic_loop_test（话题回环测试）

同时发布和订阅同一个话题，验证 ROS2 通信是否能在同一个节点内正常工作。

**状态**：已注释。

---

## 编译与运行

### 查看哪些程序已启用

在 `CMakeLists.txt` 中搜索没有被 `#` 注释掉的 `add_executable`：

```bash
grep -n "add_executable" CMakeLists.txt | grep -v "^.*#"
```

### 常用编译命令

```bash
# 编译所有已启用的程序
cmake -S . -B build && cmake --build build -j$(nproc)

# 只编译某个程序（推荐，更快）
cmake --build build --target gimbal_test -j$(nproc)
cmake --build build --target auto_aim_test -j$(nproc)
cmake --build build --target minimum_vision_system -j$(nproc)
```

### 启用一个被注释的程序

编辑 `CMakeLists.txt`，找到对应行，去掉开头的 `#`：

```cmake
# 注释状态（不会编译）
# add_executable(fire_test tests/fire_test.cpp)
# target_link_libraries(fire_test ${OpenCV_LIBS} ...)

# 去掉 # 后（会编译）
add_executable(fire_test tests/fire_test.cpp)
target_link_libraries(fire_test ${OpenCV_LIBS} ...)
```

改完后重新运行 cmake：

```bash
cmake -S . -B build && cmake --build build --target fire_test -j$(nproc)
```

---

## 使用建议

### 新人上手顺序

如果你是第一次接触这个项目，建议按以下顺序熟悉各个测试程序：

1. **先读 `auto_aim_test` 的代码**：了解一帧图像从 YOLO → Tracker → Aimer → Plotter 的完整流程。这是最核心的代码。
2. **跑一次 `auto_aim_test`**：用 demo 数据离线跑一遍，看终端日志和画面输出。
3. **跑 `gimbal_test`**：接上实车后，先验证云台通信是否正常。
4. **跑 `minimum_vision_system`**：验证完整链路，确认相机、检测、跟踪、瞄准都没问题。
5. **调参用 `auto_aim_test` + Plotter**：离线调好 EKF 参数再上实车。

### 故障排查

| 现象 | 应该先跑哪个程序 | 预期结果 |
|------|----------------|---------|
| 云台不响应 | `gimbal_test` | 确认串口有数据、EC 收到指令 |
| 检测不到装甲板 | `camera_detect_test` | 确认相机画面正常 |
| 自瞄打不准 | `auto_aim_test` | 离线回放看 EKF 是否收敛 |
| 帧率低 | `camera_test` | 确认相机帧率达标 |

---

## 常见问题

### Q: 为什么有些测试编译了但 build 目录里没有？

检查 `CMakeLists.txt` 中对应的 `add_executable` 行是否被 `#` 注释掉了。只有没被注释的行才会被编译。

### Q: 如何判断一个测试需要哪些硬件？

看上面的速查表。"需要硬件"列：
- ❌ 无 = 只需文件，可在任何机器上运行
- ✅ 相机 = 需要连接相机
- ✅ 串口 = 需要连接 EC 串口
- ✅ USB相机 = 需要 USB 接口相机

### Q: 程序运行后卡住不动？

最常见的原因是串口没数据。在新终端运行：

```bash
sudo cat /dev/ttyUSB0 | xxd | head -20
```

看到 `47 44`（GD 帧头）或 `51 59`（QY 帧头）说明通信正常。如果没输出，检查 EC 是否上电、串口线是否接好。
