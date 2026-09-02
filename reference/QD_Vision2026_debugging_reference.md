# QYG_Vision 调试体系建设参考：基于 QD_Vision2026

> 文档定位：供 QYG_Vision 后续建设调试、录制、回放与可视化体系时参考。
>
> 调研对象：[NOMANE-0/QD_Vision2026](https://github.com/NOMANE-0/QD_Vision2026)，基准提交：[`82137bb8231cc0cccf3dcf9f4762eef140b066a4`](https://github.com/NOMANE-0/QD_Vision2026/tree/82137bb8231cc0cccf3dcf9f4762eef140b066a4)，核对日期：2026-09-03。
>
> 说明：本文把“QD 仓库中已经实现的行为”和“建议 QYG 采用的方案”分开描述。QD 的 README、配置和代码存在少量不一致，本文以该提交的实际代码为准。

## 1. 结论摘要

QD_Vision2026 的调试核心不是单独某个界面，而是一套闭环：

```text
NUC/工控机运行视觉程序
  ├─ 实时控制链：相机 → 检测 → 解算/跟踪 → 云台指令 → 串口反馈
  ├─ 在线观测链：ROS 2 topics → foxglove_bridge → 笔记本 Foxglove/PlotJuggler
  ├─ 事件录包链：10 s 预缓存 → 发现目标触发 → MCAP → 赛后复盘
  └─ 确定性回放链：原图 AVI + 图像时间戳 + 串口/IMU CSV → 重跑算法
```

最值得 QYG_Vision 参考的是以下五点：

1. **在线观察和实时控制解耦。** 控制链优先使用进程内数据，ROS 2/Foxglove 主要负责观测、录包和回放，避免可视化阻塞控制。
2. **按数据层观察，而不是只看最终画面。** 同时观察输入、检测、测量、滤波状态、控制输出和下位机反馈，才能定位问题属于哪一层。
3. **保留触发前的数据。** QD 的 MCAP 录制器持续维护 10 秒循环缓冲区，目标出现时把触发前数据一并写入。
4. **“复盘录包”和“算法重跑数据”分开。** MCAP 适合看当时发生了什么；原图、时间戳和串口数据适合改参数后重新运行完整算法。
5. **调参依据是对照曲线。** 重点比较“反馈值—期望值—实际下发值”，以及残差、速度、距离、弹速和开火建议，而不是凭单帧画面猜参数。

对 QYG_Vision 的总体建议是：**短期复用现有 Recorder、Plotter、Rerun 和调试程序；中期补齐统一 DebugFrame、事件录制和确定性回放；需要跨机器统一查看时，再把同一份调试数据映射为 ROS 2 topics，通过 Foxglove 展示。**

## 2. 调研范围与证据边界

本次核对包括 QD 仓库内全部 15 份 README，以及启动文件、参数文件、消息定义、检测/解算代码、录制/回放代码、看门狗脚本、Docker 配置和 `plot.xml`。

主要证据入口：

- [根 README](https://github.com/NOMANE-0/QD_Vision2026/blob/82137bb8231cc0cccf3dcf9f4762eef140b066a4/README.md)
- [单进程启动文件](https://github.com/NOMANE-0/QD_Vision2026/blob/82137bb8231cc0cccf3dcf9f4762eef140b066a4/src/rm_bringup/launch/bringup_SingleProcess.launch.py)
- [总启动参数](https://github.com/NOMANE-0/QD_Vision2026/blob/82137bb8231cc0cccf3dcf9f4762eef140b066a4/src/rm_bringup/config/launch_params.yaml)
- [Foxglove/PlotJuggler 布局](https://github.com/NOMANE-0/QD_Vision2026/blob/82137bb8231cc0cccf3dcf9f4762eef140b066a4/plot.xml)
- [自动 MCAP 录制参数](https://github.com/NOMANE-0/QD_Vision2026/blob/82137bb8231cc0cccf3dcf9f4762eef140b066a4/src/rm_auto_record/config/node_params.yaml)
- [自动 MCAP 录制实现](https://github.com/NOMANE-0/QD_Vision2026/blob/82137bb8231cc0cccf3dcf9f4762eef140b066a4/src/rm_auto_record/src/record_node.cpp)
- [确定性回放实现](https://github.com/NOMANE-0/QD_Vision2026/blob/82137bb8231cc0cccf3dcf9f4762eef140b066a4/src/rm_auto_replay/src/auto_replay_node.cpp)
- [看门狗脚本](https://github.com/NOMANE-0/QD_Vision2026/blob/82137bb8231cc0cccf3dcf9f4762eef140b066a4/src/rm_upstart/rm_watch_dog.sh)
- [自定义消息](https://github.com/NOMANE-0/QD_Vision2026/tree/82137bb8231cc0cccf3dcf9f4762eef140b066a4/src/rm_interfaces/msg)

仓库中没有一篇 README 把整套调试流程完整写出。根 README 只明确给出了 Foxglove Bridge 启动命令，具体调试方式需要从代码和配置反推。

## 3. QD 的调试环境

### 3.1 服务端环境

QD 给出的本地基线为：

- Ubuntu 22.04；
- ROS 2 Humble；
- OpenVINO；
- 海康相机驱动；
- 串口设备；
- `fmt`、Ceres 和其余 rosdep 依赖；
- 可选 Docker/Pixi 环境。

典型编译和运行命令：

```bash
colcon build --symlink-install --parallel-workers 4
source install/setup.bash
ros2 launch rm_bringup bringup_SingleProcess.launch.py
```

根 README 给出的并行度建议是 8 GB 内存使用 2 线程、16 GB 使用 4 线程，避免编译阶段内存不足。

### 3.2 Docker 与网络

QD 的 `docker-compose.yaml` 使用：

- `network_mode: host`：容器与 NUC 共用网络栈，Foxglove Bridge 可直接监听主机端口；
- `privileged: true` 并挂载 `/dev`：容器访问相机和串口；
- 工作目录 `/ros_ws`；
- 挂载 `.vscode-server`：方便笔记本通过 VS Code Remote SSH 进入 NUC/容器开发。

这里的 SSH 和 Foxglove 分工不同：

- SSH/VS Code Remote：编辑、编译、启动、查看文本日志；
- Foxglove WebSocket：把 ROS 2 话题传到笔记本做曲线、图像、TF 和 Marker 可视化。

在同一局域网内，客户端通常直接连接：

```text
ws://<NUC_IP>:8765
```

无需让图像 GUI 在 NUC 上本地渲染，也不必默认走 SSH 隧道。只有网络隔离或端口无法直连时，才考虑端口转发。

### 3.3 Foxglove Bridge

QD 没有自研名为 foxbridge 的程序；聊天中所说的 “foxbridge” 实际对应官方 `foxglove_bridge`：

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

官方默认 WebSocket 监听地址为 `0.0.0.0:8765`。QD 的 `plot.xml` 把客户端地址硬编码为 `192.168.1.113:8765`，换车或换网段时必须改成实际 NUC IP。

### 3.4 进程组织与自恢复

QD 使用 ROS 2 composable nodes，把机器人模型、相机/视频输入、串口、检测器、解算器、录制器和回放器装入 `component_container_mt`。主要意义是：

- 统一启动；
- 支持多线程回调；
- 允许进程内通信，减少大图像跨进程序列化；
- 通过 launch 参数切换物理设备、视频、虚拟串口、录制和回放。

看门狗会启动主 launch 和 Foxglove Bridge，并检查 `armor_detector`、`armor_solver`、`serial_driver` 等节点的 heartbeat。默认每 10 秒检查一次，心跳话题不存在或收不到数据时重启整套 ROS 进程。

## 4. QD 的在线调试路线

QD 队员所说的“服务端调试、客户端开 Foxglove”，可还原为以下流程。

### 4.1 启动前检查

1. 确认相机、串口设备和 udev 映射存在。
2. 确认相机标定、云台到相机外参、敌方颜色和弹道参数对应当前车辆。
3. 在 `launch_params.yaml` 选择输入和运行模式。
4. 确认 NUC 与笔记本同网段、TCP 8765 可达。
5. 确认输出目录有足够空间，避免录包写满系统盘。

QD 默认关键开关为：

| 参数 | 默认值 | 实际含义 |
|---|---:|---|
| `use_state_machine_camera` | `true` | 检测器内部状态机线程取图，不经普通图像订阅 |
| `rune` | `false` | 不启动能量机关模块 |
| `record` | `false` | 不录制原图 AVI+CSV |
| `auto_record` | `true` | 启用触发式 MCAP 录制 |
| `video_player` | `false` | 使用物理相机而非视频 |
| `virtual_serial` | `false` | 使用物理串口 |
| `replay` | `false` | 不启用确定性回放 |

`record` 和 `auto_record` 同时为真时，launch 会主动关闭 `record`。`replay=true` 时会关闭物理相机、物理串口、视频播放器和虚拟串口，避免多输入源冲突。

### 4.2 服务端启动

1. SSH 到 NUC，进入容器或本地工作区。
2. 编译并加载工作空间。
3. 启动 `bringup_SingleProcess.launch.py`。
4. 单独启动 Foxglove Bridge；使用看门狗时由脚本同时启动。
5. 检查节点、心跳和关键话题是否存在。

可采用的基本检查命令：

```bash
ros2 node list
ros2 topic list
ros2 topic hz /armor_solver/heartbeat
ros2 topic hz /armor_solver/cmd_gimbal
ros2 topic echo /serial/receive --once
```

### 4.3 客户端连接

在笔记本 Foxglove 中添加 Foxglove WebSocket 连接：

```text
ws://<NUC_IP>:8765
```

QD 的 `plot.xml` 实际选择六个主要话题：

- `/armor_solver/armors`
- `/armor_solver/cmd_gimbal`
- `/armor_solver/heartbeat`
- `/armor_solver/measurement`
- `/armor_solver/target`
- `/serial/receive`

然后按数据层检查：

1. **输入层**：相机帧率、图像曝光、串口反馈、时间戳；
2. **检测层**：装甲板数量、类别、置信度、角点和结果图；
3. **几何测量层**：PnP 位置、yaw、pitch、distance；
4. **状态估计层**：目标位置/速度、旋转速度、半径、高低差、NIS；
5. **控制层**：期望角、实际下发角、角差、超调速度、开火建议；
6. **执行反馈层**：串口收到的云台 yaw/pitch 和原始弹速。

### 4.4 一次参数调试循环

```text
固定工况和数据段
  → 找到异常所在层
  → 只改一组参数
  → 重新运行或热更新
  → 对比相同曲线和相同数据段
  → 记录参数、提交号、录包编号和结论
```

不要一边启动车辆、一边无基准地连续改多个参数。推荐先用录制数据离线筛选，再上车验证闭环响应。

## 5. QD 的具体数据接口

### 5.1 主控制链

```text
Camera / Replay image
  → ArmorDetector
      → ArmorFrame（当前实现为进程内 latest-only 数据）
  → ArmorSolver
      → GimbalCmd
  → SerialDriver
      → 下位机

SerialReceiveData
  → 姿态、弹速、模式
  → Detector / Solver / TF / Recorder
```

QD 当前检测器到解算器的关键控制数据不是依赖 ROS topic 排队传递，而是进程内 `ArmorFrame` 最新帧通道。ROS 话题是相同数据的观测镜像。这种做法值得 QYG 参考：调试系统失效时，不应拖死控制系统。

### 5.2 核心话题与字段

| 话题 | 类型 | 关键字段 | 调试用途 |
|---|---|---|---|
| `/serial/receive` | `SerialReceiveData` | `mode`、`bullet_speed`、`roll/pitch/yaw`、`mcu_timestamp` | 下位机反馈、坐标变换和弹道输入 |
| `/armor_solver/armors` | `Armors` | header、图像、装甲板数组 | 检测/解算输入镜像 |
| `/armor_solver/measurement` | `Measurement` | `x/y/z`、`yaw`、armor pitch/roll、center yaw、pitch、distance | EKF 原始测量和 PnP 检查 |
| `/armor_solver/target` | `Target` | tracking、id、position、velocity、yaw、v_yaw、top_level、radius/dz、nis | 跟踪与状态估计检查 |
| `/armor_solver/cmd_gimbal` | `GimbalCmd` | yaw/pitch、exp_yaw/exp_pitch、diff、distance、fire_advice、bullet_speed、yaw_vel/pitch_vel、target_v_yaw | 控制输出和开火判断 |
| `/armor_detector/marker` | `MarkerArray` | 检测可视化对象 | 3D/几何观察 |
| `/armor_solver/marker` | `MarkerArray` | 目标、装甲板、预测等 | TF 空间中的状态观察 |
| `/armor_solver/result_img/compressed` | `CompressedImage` | JPEG 图像 | 低带宽结果画面 |
| `/armor_detector/camera_info` | `CameraInfo` | 内参、畸变、分辨率 | PnP 和回放一致性 |
| `/tf`、`/tf_static` | `TFMessage` | 坐标变换 | 坐标系与时间同步检查 |
| `/parameter_events` | `ParameterEvent` | 参数变更 | 复盘时还原调参过程 |

### 5.3 `GimbalCmd` 为什么是调试中心

QD 把多层控制信息集中进同一条消息：

- `exp_yaw/exp_pitch`：算法真正期望到达的角度；
- `yaw/pitch`：加入超调或动态补偿后实际发送的角度；
- `yaw_diff/pitch_diff`：当前反馈与期望之间的误差；
- `yaw_vel/pitch_vel`：动态补偿依据；
- `distance`：目标有效性和录制触发条件；
- `fire_advice`：最终开火建议；
- `bullet_speed`：滤波后参与弹道计算的弹速。

这种“同一时间戳下汇总控制上下文”的接口很重要。若这些字段散落在日志文本中，Foxglove 很难直接叠加对比，也难以在回放中保持同步。

### 5.4 QD 预设的曲线

QD 的 `plot.xml` 预设了以下对照关系：

| 面板 | 曲线 | 要回答的问题 |
|---|---|---|
| Yaw 跟踪 | serial yaw、cmd yaw、exp_yaw | 云台反馈是否跟上期望，下发补偿是否合理？ |
| Yaw 误差/开火 | yaw_diff、fire_advice | 开火窗口是否与误差阈值一致？ |
| Yaw 速度 | yaw_vel | 高速转动时补偿是否稳定？ |
| Pitch 跟踪 | serial pitch、cmd pitch、exp_pitch | 弹道补偿和执行反馈是否一致？ |
| Pitch 误差/开火 | pitch_diff、fire_advice | pitch 是否成为禁止开火的主因？ |
| Pitch 速度 | pitch_vel | pitch 动态补偿是否抖动？ |
| 弹速 | raw bullet_speed、filtered bullet_speed | 弹速滤波是否延迟或跳变？ |
| 距离 | cmd distance | 目标丢失、切换和录制触发何时发生？ |
| 目标模型 | radius_list、dz_list | 旋转目标模型是否收敛到合理几何尺寸？ |

客户端只传数值话题时带宽很低。主要带宽来自图像、TF 和 Marker。QD 的结果图 JPEG 质量默认 50，体现了“曲线常开、图像压缩、原图按需录制”的取舍。

## 6. QD 的两套录制/回放数据

### 6.1 A 类：触发式 MCAP，用于复盘

`auto_record=true` 时，`rm_auto_record` 启动。实际代码行为是：

1. 平时订阅指定话题并维护最近 10 秒的内存循环缓冲；
2. 当 `/armor_solver/cmd_gimbal.distance > 0` 时开始写 MCAP；
3. 把触发前 10 秒缓存刷入文件；
4. 持续记录；
5. 当 distance 不再为正达到 10 秒后停止；
6. 返回缓冲状态，等待下次目标出现。

输出目录默认：

```text
/ros_ws/rosbag/record_<日期时间>/
```

底层存储为 MCAP，单个 bag 文件最长 30 秒，过长时自动分片。

默认记录 14 个话题：

```text
/armor_solver/result_img/compressed
/armor_detector/camera_info
/armor_solver/armors
/armor_detector/marker
/serial/receive
/armor_solver/cmd_gimbal
/armor_solver/target
/armor_solver/marker
/armor_solver/measurement
/tf
/tf_static
/joint_states
/robot_description
/parameter_events
```

适合回答：

- 当时检测到了什么；
- EKF 和控制量如何变化；
- 丢目标前后发生了什么；
- 开火建议为什么出现或消失；
- 坐标系、串口反馈和参数事件是否异常。

限制：默认没有录原始 `/image_raw`。结果压缩图和已经生成的中间消息可以复盘，但不足以在改动检测器后完整重跑原始算法。因此不能把这类 MCAP 当成唯一的数据集。

### 6.2 B 类：AVI + CSV，用于确定性重跑

`record=true`、`auto_record=false` 时，检测器内部的 `HighPerfDataRecorder` 录制：

```text
/ros_ws/record/record_<timestamp>/
  ├─ data.avi
  ├─ data_timestamps.csv
  └─ data_serial.csv
```

其中：

- `data.avi`：原始图像序列；
- `data_timestamps.csv`：帧序号和采集时间戳；
- `data_serial.csv`：串口模式、弹速、roll、yaw、pitch 及时间戳。

`replay=true` 时，`AutoReplayNode` 读取这三类数据，按照记录的相对时间重新发布：

- `image_raw`；
- `camera_info`；
- `serial/receive`；
- 云台相关 TF；
- heartbeat；
- 模式变化时调用 detector/solver 的 `set_mode` 服务。

回放器还提供：

- `video_hz`：回放图像频率；
- `serial_ahead_ms`：串口数据相对图像提前量，默认 10 ms；
- 图像前发布最近串口 TF，并额外留 2 ms 传播时间。

这套数据用于：

- 修改检测阈值、模型或 PnP 后重跑；
- 修改 EKF、预测、弹道或开火参数后对比；
- 在没有车、相机和下位机时复现问题；
- 将同一数据段作为不同提交之间的回归基准。

### 6.3 两类数据不能互相替代

| 目标 | MCAP 复盘包 | AVI + CSV 重跑包 |
|---|---:|---:|
| 快速看曲线/TF/Marker | 最适合 | 需要重新运行算法 |
| 保留触发前现场 | 支持 10 秒预缓存 | QD 当前为连续手工录制 |
| 修改算法后重新检测 | 默认不支持 | 支持 |
| 还原当时参数变化 | 有 `/parameter_events` | 需要额外保存配置快照 |
| 文件体积 | 取决于压缩图和话题 | 原图视频通常更大 |
| 比赛常开 | 合适 | 建议按需或事件触发 |

最佳实践是两者并存：常开轻量 MCAP 事件录制，关键工况再录原图重跑数据。

## 7. 参数调试策略

QD 参数更新能力并不完全一致，应先判断参数生命周期。

### 7.1 可在线调整的参数

检测器安装了参数回调，适合在线调整的典型参数包括：

- debug 开关；
- YOLO/分类阈值；
- 相机 gain、exposure；
- 部分检测阈值。

解算器在求解过程中会重复读取部分参数，例如：

- 延迟参数；
- 弹速与滤波 alpha；
- `use_armor_top`；
- gimbal frame 等。

这类参数可用 `ros2 param set` 或 Foxglove 参数面板快速扫值，但每次调整仍应记录 `/parameter_events`。

### 7.2 修改后需要重启或回放的参数

以下典型参数在对象构造时读取，不能假定设置后立即生效：

- EKF Q/R；
- tracker 阈值与丢失计数；
- 重力/空气阻力等弹道模型参数；
- 旋转目标 top 阈值；
- 固定角度偏置和部分几何配置。

调试这类参数的正确路线是：编辑 YAML → 重启或重新回放 → 对比同一数据段，而不是只执行一次在线 `param set`。

### 7.3 推荐的调参顺序

```text
时间同步与坐标系
  → 相机曝光/成像
  → 检测角点和类别
  → PnP 测量
  → EKF 状态与匹配
  → 弹道/延迟补偿
  → 云台闭环响应
  → 开火条件
```

前一层不稳定时，不应依靠后一层参数掩盖问题。例如 PnP 跳变时先查角点、内参、外参和时间同步，不要先放大 EKF 测量噪声。

## 8. QD 文档与代码中的注意事项

采用其方案时需要避开以下误区：

1. `rm_auto_record/README.md` 曾描述通过 `record_controller` 触发，但该提交实际通过 `GimbalCmd.distance > 0` 触发。
2. `armor_solver/README.md` 仍描述订阅 detector 的 armors topic；当前主控制链已经使用进程内数据，topic 更偏向调试镜像。
3. 根 README 说默认日志和内录视频位于 `qd2026-log/`；实际两个录制模块分别默认写 `/ros_ws/rosbag` 和 `/ros_ws/record`。
4. `plot.xml` 的 IP 是固定值，复制到其他车辆必须修改。
5. 能量机关模块仍依赖话题输入，而默认 `use_state_machine_camera=true` 的装甲板链路采用内部取图；组合使用前要验证 remap 和输入源。
6. 看门狗使用 `pkill -f ros` 重启全部 ROS 进程，简单有效但范围较大；QYG 若引入，应避免误杀同机无关 ROS 任务。

## 9. QYG_Vision 当前基础

QYG_Vision 不是纯 ROS 2 工程，因此不应直接照搬 QD 的 composable node 结构。当前仓库已经具备以下可复用能力：

### 9.1 已有录制能力

`tools::Recorder` 已经能够异步保存：

```text
records/<timestamp>.avi
records/<timestamp>.txt
```

TXT 每行保存相对时间和 IMU 四元数 `w x y z`。记录队列容量为 1，倾向保留最新帧，避免慢磁盘无限积压。

不足之处：

- 未保存弹速、模式、云台控制命令、检测/跟踪状态；
- 视频和元数据没有显式 schema/version；
- 没有事件触发、预缓存和磁盘配额；
- 没有统一回放入口保证图像与下位机数据按原时间关系送入。

### 9.2 已有在线曲线能力

`tools::Plotter` 会把任意 JSON 通过 UDP 发往 `127.0.0.1:9870`；`VofaPlotter` 会把固定字段转换为 VOFA 文本帧，默认端口为 1347。

`mt_auto_aim_debug.cpp` 已经组织了大量有价值的字段：

- 处理耗时、装甲板数量；
- 装甲板 x/y/yaw/yaw_raw；
- EKF x/vx/y/vy/z/vz/a/w/r/l/h；
- last_id；
- residual yaw/pitch/distance/angle；
- NIS/NEES 及失败计数；
- gimbal yaw/pitch 和 bullet speed。

这些字段可以作为 QYG 统一 DebugFrame 的第一版来源。

### 9.3 已有图像与离线可视化

多个 debug/test 程序已经支持：

- OpenCV 叠加检测角点、重投影装甲板和瞄准点；
- 显示 FPS、云台角等文字；
- `cv::imshow` 本地观察；
- `ekf_visual_experience_rerun` 输出 JSONL + 图片；
- Rerun 同时显示重投影图、观测点、EKF 目标、瞄准点和标量曲线；
- 可选 ROS 2 版本发布 `/ekf_visual/*` 给 RViz2/Foxglove。

现有 Rerun 路线已经解决离线、无 ROS 环境下的算法观察问题，应继续保留。

### 9.4 当前主要问题

- 能力分散在不同 executable，正式程序、debug 程序和测试程序字段不统一；
- 大量调试 target 在 `CMakeLists.txt` 中被注释，切换车辆时容易出现“代码存在但未编译”；
- Plotter 默认为 localhost，笔记本远程观察需要改地址或转发；
- 本地图像 GUI 会占用 NUC 计算/显示环境，不适合比赛常开；
- 录制数据不足以完整还原控制链；
- 缺少像 QD 那样统一的事件包、布局文件、参数快照和问题编号。

## 10. 建议 QYG 采用的目标架构

### 10.1 原则

1. **控制优先**：相机、检测、跟踪、规划、下位机通信继续走当前 C++ 进程内对象和 latest-only queue。
2. **单次采样、多路输出**：算法每帧生成一份统一 `DebugFrame`，各后端按需读取，避免每种工具重新取内部状态。
3. **观测后端可关闭**：关闭 Foxglove/Rerun/UDP/录制时，不改变算法行为。
4. **慢消费者不反压控制**：图像压缩、磁盘写入和网络发送使用有界队列；队列满时丢调试帧并统计 dropped count。
5. **时间戳贯穿全链路**：相机采集、IMU、检测、跟踪、规划、发送和反馈使用明确的同一时间基准或保存可转换关系。

### 10.2 推荐结构

```text
Camera + CBoard/Gimbal
        │
        ▼
QYG realtime pipeline
  detect → solve → track → aim/plan → send
        │
        └─ DebugFrame（只读快照，latest-only）
             ├─ OpenCV overlay（上车临时）
             ├─ UDP/VOFA（轻量实时曲线）
             ├─ Rerun writer（离线/开发机）
             ├─ ROS2 debug adapter → Foxglove Bridge
             ├─ Event MCAP recorder
             └─ Deterministic raw recorder
```

ROS 2 adapter 应是外围调试模块，而不是 QYG 控制链的强依赖。这样既参考 QD 的调试体验，也符合 QYG 当前非 ROS 核心架构。

## 11. QYG 推荐数据接口

### 11.1 统一 `DebugFrame`

建议每帧至少包含：

| 分组 | 推荐字段 |
|---|---|
| 身份 | schema_version、git_commit、session_id、frame_id |
| 时间 | capture_time、imu_time、detect_done、track_done、command_time、feedback_time |
| 运行状态 | mode、enemy_color、target_found、tracker_state、control、fire |
| 性能 | camera_fps、pipeline_ms、detect_ms、solve_ms、track_ms、plan_ms、queue_depth、dropped_debug_frames |
| 相机 | width/height、exposure、gain、camera_matrix_id、extrinsic_id |
| 检测 | armor_count、id/type/color/confidence、四角点、center、bbox |
| 测量 | armor_xyz_world、armor_ypr_world、distance、reprojection_error |
| 跟踪 | state x/vx/y/vy/z/vz/a/w/r/l/h、last_id、lost_count |
| 滤波诊断 | residual_yaw/pitch/distance/angle、NIS/NEES、gating_result |
| 瞄准 | aim_point xyz、target_yaw/pitch、prediction_time、flight_time |
| 命令 | tx_control、tx_fire、tx_yaw/pitch、expected_yaw/pitch |
| 反馈 | rx_yaw/pitch/roll、raw_bullet_speed、filtered_bullet_speed、mode |

### 11.2 推荐 Foxglove 话题

若引入 ROS 2 调试适配层，建议保持 QYG 命名空间，避免与导航和其他车辆冲突：

```text
/qyg/vision/heartbeat
/qyg/vision/result_image/compressed
/qyg/vision/detections
/qyg/vision/measurement
/qyg/vision/target
/qyg/vision/aim
/qyg/vision/gimbal_command
/qyg/vision/gimbal_feedback
/qyg/vision/performance
/qyg/vision/markers
/qyg/vision/recording_state
/tf
/tf_static
/parameter_events
```

早期可以先使用标准消息快速验证；长期应定义语义明确的消息。不要把多个无关值塞入匿名 `Vector3` 后靠记忆解释 x/y/z。

### 11.3 推荐首版曲线布局

1. `rx_yaw`、`expected_yaw`、`tx_yaw`；
2. `rx_pitch`、`expected_pitch`、`tx_pitch`；
3. yaw/pitch error 与 fire；
4. raw/filtered bullet speed；
5. target_found、control、tracker_state、last_id；
6. armor yaw raw、measurement yaw、EKF yaw；
7. residual、NIS、gating result；
8. target w/r/dz；
9. pipeline/detect/track/plan latency 与 dropped frames；
10. command_time - capture_time、feedback_time - command_time。

这组曲线也应同步配置到 VOFA/Rerun，保证不同工具看到的字段名称和单位一致。

## 12. QYG 推荐录制数据集

### 12.1 最小可重跑数据集

每个 session 建议采用自描述目录：

```text
records/<session_id>/
  ├─ manifest.yaml
  ├─ config_snapshot.yaml
  ├─ camera.yaml
  ├─ frames.mkv                 # 或无损/低损耗 AVI
  ├─ frame_timestamps.csv
  ├─ feedback.csv               # IMU、云台反馈、弹速、模式
  ├─ commands.csv               # 实际下发命令
  ├─ debug.jsonl                # 检测、测量、状态、性能
  └─ events.csv                 # 目标出现/丢失、模式切换、开火、异常
```

`manifest.yaml` 至少保存：

- schema version；
- git commit 和 dirty 状态；
- executable 与完整启动参数；
- 配置文件 SHA-256 或完整快照；
- 相机序列号、分辨率、像素格式、FPS；
- 内参/外参版本；
- 下位机协议版本；
- 主机时钟和 MCU 时钟的对应关系；
- 开始/结束时间、结束原因和丢帧统计。

相比 QD 的 AVI+CSV，QYG 应额外记录实际下发命令、配置快照和版本信息，避免“视频能播但无法还原当时参数”。

### 12.2 事件录制策略

建议借鉴 QD 的预缓存状态机：

```text
BUFFERING
  ├─ 持续保留最近 5~10 s 的轻量数据
  └─ 触发 → flush pre-buffer → RECORDING

RECORDING
  ├─ 持续写盘
  └─ 触发消失超过 5~10 s → close → BUFFERING
```

触发条件不应只依赖 `distance > 0`，建议支持组合：

- target_found 上升沿；
- fire 或 fire_advice；
- tracker reset/NIS gate 连续失败；
- 模式切换；
- 控制误差超过阈值；
- 处理时延或丢帧超过阈值；
- 人工触发。

应增加最大单段时长、总磁盘配额、低剩余空间保护和异常退出后的文件恢复。

### 12.3 回放契约

回放器必须使用与在线程序相同的输入接口，并保证：

1. 图像按记录时间戳输入，而非只按视频标称 FPS；
2. 为每帧选择正确时刻的 IMU/云台状态，插值规则固定且有记录；
3. 模式和敌方颜色按事件恢复；
4. 回放禁止向真实下位机发送有效控制命令，默认使用 mock sink；
5. 支持实时、倍速、逐帧和无等待全速运行；
6. 输出新的 DebugFrame 和结果文件，便于不同提交自动对比。

## 13. QYG 分阶段落地路线

### P0：统一调试字段，不改控制架构

- 从 `mt_auto_aim_debug.cpp` 提取统一 DebugFrame/JSON schema；
- 把正式程序和 debug 程序都接到同一数据生产接口；
- 统一字段名、单位和坐标系；
- 为现有 Plotter、VOFA 和 Rerun 适配同一数据。

验收：同一帧在日志、VOFA 和 Rerun 中的 yaw、pitch、时间戳一致。

### P1：补全 QYG 确定性录制与回放

- 扩展 Recorder，保存图像、反馈、命令、模式、时间戳和配置快照；
- 使用有界异步队列并记录丢帧；
- 建立统一 replay executable；
- 默认 mock 下位机输出。

验收：同一提交重复回放，检测数量、目标 ID、主要状态和控制输出在允许误差内一致。

### P2：建立标准曲线和问题数据集

- 固化 yaw/pitch、弹速、残差、NIS、延迟等面板；
- 每个典型问题保存最短可复现 session；
- 用脚本输出回放指标和提交间差异。

验收：新成员可以只凭 session、配置和文档复现问题，无需原车。

### P3：增加事件预缓存与自动录制

- 实现 BUFFERING/RECORDING 状态机；
- 支持目标、开火、滤波异常和人工触发；
- 增加磁盘管理、分片和 recording_state。

验收：目标首次出现前至少 5 秒的数据完整存在，长期运行不会写满磁盘或阻塞控制。

### P4：增加 ROS 2/Foxglove 外部观测适配层

- 把 DebugFrame 映射成 QYG topics；
- 压缩图像、限制 Marker 和 TF 频率；
- 启动官方 Foxglove Bridge；
- 保存一份可版本管理的 Foxglove layout。

验收：笔记本断开、客户端卡顿或网络丢包时，NUC 控制频率和结果不受影响。

### P5：部署与自恢复

- 统一启动脚本或 systemd 服务；
- heartbeat 同时包含 FPS、last frame age、queue depth 和 recorder 状态；
- 看门狗按组件分级恢复，保留崩溃前后日志；
- 建立每辆车的配置、IP、设备和标定清单。

验收：拔插相机/串口、网络断连、可视化客户端退出和录制目录异常均有明确告警及可控恢复路径。

## 14. 推荐的日常调试 SOP

### 14.1 上车前离线调试

1. 选择固定 session 和基准提交。
2. 回放并确认输入时间同步。
3. 检查检测角点和 PnP 重投影。
4. 检查 measurement、residual、NIS 和目标切换。
5. 检查预测点、命令和开火窗口。
6. 修改单一参数组，保存新配置和指标。
7. 通过后再部署到 NUC。

### 14.2 上车静态调试

1. 禁止发弹，先验证坐标系和控制方向。
2. 固定目标，检查 yaw/pitch 反馈与命令符号。
3. 改变距离，检查 PnP、弹道和 pitch 趋势。
4. 缓慢转动云台，检查时间同步和动态误差。
5. 确认无目标时 control 语义和保持策略。

### 14.3 动态闭环调试

1. 开启事件录制和标准曲线布局。
2. 先低速，再高速；先不开火，再开火。
3. 同时观察 feedback、expected、tx 三条曲线。
4. 记录每次参数、目标类型、距离、速度、弹速和结果。
5. 出现问题后停止盲调，导出最短数据段离线复现。

### 14.4 每次数据交接必须附带

- 车辆和云台编号；
- git commit、分支、dirty 状态；
- 配置快照；
- 标定版本；
- 数据 session 路径；
- 问题发生时间段；
- 预期现象与实际现象；
- 是否发弹、是否可能产生真实控制；
- 初步判断属于哪一数据层。

## 15. 验收清单

### 在线观测

- [ ] 客户端能通过 NUC IP:8765 连接，地址不硬编码到个人电脑配置中。
- [ ] 数值话题常开时不明显降低主循环 FPS。
- [ ] 图像关闭后带宽显著下降，说明图像流可独立控制。
- [ ] 曲线具有统一时间戳、单位和坐标系说明。
- [ ] feedback、expected、tx 能直接叠加。
- [ ] 客户端断开不影响控制链。

### 录制

- [ ] 触发前缓存数据完整。
- [ ] 文件中包含图像、反馈、命令、模式、配置和版本。
- [ ] 队列满时丢调试数据而不是阻塞控制，并可看到 dropped count。
- [ ] 文件分片、磁盘配额和低空间保护有效。
- [ ] 异常退出后已有数据可恢复或至少可诊断。

### 回放

- [ ] 回放默认不连接真实执行器。
- [ ] 图像与 IMU/串口按原始时间关系对齐。
- [ ] 支持实时、倍速、逐帧和全速。
- [ ] 同提交重复运行结果稳定。
- [ ] 不同提交能输出结构化差异指标。

### 参数调试

- [ ] 每个参数注明单位、范围、默认值和生效方式。
- [ ] 区分热更新参数与构造期参数。
- [ ] 参数变更可追溯到 session。
- [ ] 调参按数据层进行，不用滤波参数掩盖测量层错误。

## 16. 最终建议

QYG_Vision 应参考 QD 的是**调试闭环和数据分层思想**，不是把当前工程整体改造成 ROS 2：

- 近期以现有 `Recorder + Plotter/VOFA + Rerun + OpenCV overlay` 为基础，先统一 DebugFrame 和可重跑数据；
- 中期补齐事件预缓存、配置快照、命令/反馈同步以及标准曲线；
- 跨机器调试时增加薄 ROS 2 adapter 和 Foxglove Bridge，让笔记本承担可视化；
- 控制链始终保持进程内、latest-only、有界和不受调试后端反压；
- 每一个线上问题最终都沉淀成可离线复现的 session 和明确的验收指标。

如果只能优先完成一项，应先做**图像 + 精确时间戳 + IMU/云台反馈 + 实际命令 + 配置快照的确定性录制/回放**。它是后续曲线调参、回归测试、Foxglove 展示和新成员学习共同依赖的底座。
