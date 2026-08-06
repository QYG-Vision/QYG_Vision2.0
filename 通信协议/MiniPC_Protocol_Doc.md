# 电控 ↔ 上位机串口通信协议

**适用工程**：`rm_vision_2025`

**协议版本**：2026-08-06 联调确认版

**物理层**：UART，921600 baud

**字节序**：小端（Little-endian）

**浮点格式**：IEEE-754 `float32`

本文只描述线上实际使用的字节协议，不依赖旧 DianKong 工程中的结构体或函数名。

## 1. CRC16

GD、QY 两种帧使用同一套反射 CRC16 算法：

- 初值：`0xFFFF`
- 多项式反射形式：`0x8408`
- 对除末尾 CRC 字段外的整帧字节依次计算
- 计算结束后直接返回当前 CRC，**不再执行最终 `^ 0xFFFF`**
- CRC 在线路中以小端顺序发送：低字节在前，高字节在后

参考校验：字节串 `123456789` 的结果应为 `0x6F91`。

## 2. EC → MiniPC：GD 帧

GD 帧固定为 51 字节。

| 偏移 | 长度 | 类型 | 视觉端字段 | 含义 |
|------|------|------|------------|------|
| 0~1 | 2 | `uint8[2]` | `header` | `0x47 0x44`，即 `G D` |
| 2 | 1 | `uint8` | `current_mode` | 导航状态/任务模式，当前只接收保存，后续再通过 ROS2 给 fwh |
| 3~6 | 4 | `float` | `actual_vx` | 底盘实际 X 速度 |
| 7~10 | 4 | `float` | `actual_vy` | 底盘实际 Y 速度 |
| 11~14 | 4 | `float` | `actual_wz` | 底盘实际角速度 |
| 15~18 | 4 | `float` | `imu_yaw` | 底盘 Yaw |
| 19~22 | 4 | `float` | `imu_pitch` | 底盘 Pitch |
| 23~26 | 4 | `float` | `yaw_angular` | Yaw 角速度/协议保留量 |
| 27~30 | 4 | `float` | `pitch_angular` | Pitch 角速度/协议保留量 |
| 31~34 | 4 | `float` | `odom_x` | 里程计累计值 |
| 35~36 | 2 | `uint16` | `sentry_state` | 高 2 bit 为视觉模式，低 14 bit 为机器人状态 |
| 37~40 | 4 | `float` | `vyaw` | 云台 Yaw，degree |
| 41~44 | 4 | `float` | `vpitch` | 电控线路原始云台 Pitch，degree |
| 45~48 | 4 | `float` | `vroll` | 云台 Roll，degree |
| 49~50 | 2 | `uint16` | `crc16` | 前 49 字节的 CRC16 |

`G` 和 `D` 各占一个字节，所以 `current_mode` 是整帧第 3 个字节，即从 0 开始计数的 offset 2。电控从 `Tx_buffer[3]` 拷贝后续 46 字节，是特意为 offset 2 的 `current_mode` 留位；它必须由电控在发帧前另行赋值。

### 2.1 `sentry_state` 位布局

```text
bit 15  bit 14 | bit 13 ................ bit 0
  visual_mode  |          status
```

打包与解析规则：

```cpp
sentry_state = (status & 0x3FFF) | ((visual_mode & 0x03) << 14);
status        = sentry_state & 0x3FFF;
visual_mode   = (sentry_state >> 14) & 0x03;
```

视觉模式：

| 高 2 bit | 视觉端枚举 | 含义 |
|----------|------------|------|
| `00` | `IDLE` | 空闲 |
| `01` | `AUTO_AIM` | 自瞄 |
| `10` | `SMALL_BUFF` | 小符 |
| `11` | `BIG_BUFF` | 大符 |

示例：线路字节 `02 80` 按小端组成 `0x8002`，二进制为 `10|00000000000010`，因此视觉模式为小符，机器人状态为 `0x0002`。低 14 bit 的每一位具体代表什么，由电控的机器人状态定义决定，视觉端只按位完整保留。

### 2.2 Pitch 边界约定

GD 线路上的 Pitch 与视觉内部统一符号相反：

```cpp
GimbalState.vpitch = -ReceiveFrame.vpitch;
```

该转换只在 CRC 校验通过后进行，不改变线路字节和 CRC。

## 3. MiniPC → EC：QY 帧

QY 帧固定为 25 字节。新版协议直接发送 `float32`，不再使用 `uint32` 量化编码。

| 偏移 | 长度 | 类型 | 视觉端字段 | 含义 |
|------|------|------|------------|------|
| 0~1 | 2 | `uint8[2]` | `header` | `0x51 0x59`，即 `Q Y` |
| 2 | 1 | `uint8` | `mode` | `0` 禁用、`1` 控制不发射、`2` 控制并允许发射 |
| 3~6 | 4 | `float` | `yaw` | 目标 Yaw，rad |
| 7~10 | 4 | `float` | `pitch` | 目标 Pitch，rad，线路符号 |
| 11~14 | 4 | `float` | `linear_x` | 底盘 X 指令 |
| 15~18 | 4 | `float` | `linear_y` | 底盘 Y 指令 |
| 19~22 | 4 | `float` | `angular_z` | 底盘角速度指令 |
| 23~24 | 2 | `uint16` | `crc16` | 前 23 字节的 CRC16 |

正式视觉端在封包边界执行以下处理：

- Yaw 限制到 `[-pi, pi]`，符号不变。
- Pitch 限制到 `[-pi, pi]`，随后取反为电控线路符号。
- X、Y、角速度限制到 `[-1, 1]`，随后三轴分别取反以匹配当前底盘约定。
- 任一输入为 NaN 或正负无穷时，发送 `mode=0`、五个浮点字段全零的安全停机帧，并重新计算有效 CRC。

不接导航时，三项底盘指令传入 `0.0f`；直接浮点发送后对应的四字节就是 `00 00 00 00`（负零在线路上也按数值零处理）。

## 4. 解析与发送要求

- 只有帧头、完整长度和 CRC 都正确时，GD 帧才能更新 `GimbalState`。
- 串口缓冲区可能一次收到半帧、多帧或帧前噪声，解析器必须先找 `GD`，再确认其后至少有 51 字节。
- CRC 失败的帧直接丢弃，等待下一帧；不要用失败帧更新模式或姿态。
- `current_mode` 与 `sentry_state` 含义不同：前者是导航任务状态，后者包含机器人状态和视觉模式。
- 当前版本暂不新增把 `current_mode` 发布给 fwh 的 ROS2 topic，仅保留字段和缓存。

## 5. 调试验证

编译并监听 GD：

```bash
cmake --build build --target gimbal_raw_debug -j2
./build/gimbal_raw_debug configs/QYG_sentry.yaml
```

同时发送固定 QY：

```bash
./build/gimbal_raw_debug --send-fixed --yaw=5 --pitch=-10 configs/QYG_sentry.yaml
```

GD 日志会显式打印：

```text
sentry_state=0xC003 bits(mode[15:14]|status[13:0])=11|00000000000011 status=0x0003 mode=BIG_BUFF
```

通过标准：

- GD 日志持续为 `crc=OK`，原始帧固定 51 字节。
- QY 原始帧固定 25 字节，电控能按 float32 正确还原 mode、Yaw、Pitch 和三轴速度。
- `sentry_state` 的高 2 bit 模式与电控设置一致，低 14 bit 状态不丢位。
- 程序退出发送的停机帧仍有正确 CRC。
