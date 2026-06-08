# 顶板 → PC 串口上传帧格式 (UART1, 921600bps)

## 帧类型: IMU 四元数 + Yaw (0x69 0x6D)

| 偏移 | 长度 | 字段 | 类型 | 说明 |
|------|------|------|------|------|
| 0 | 1 | header_c1 | uint8 | `0x69` (`'i'`) |
| 1 | 1 | header_c2 | uint8 | `0x6D` (`'m'`) |
| 2 | 4 | quat_w | float32 LE | 四元数 W |
| 6 | 4 | quat_x | float32 LE | 四元数 X |
| 10 | 4 | quat_y | float32 LE | 四元数 Y |
| 14 | 4 | quat_z | float32 LE | 四元数 Z |
| 18 | 4 | yaw | float32 LE | 下板 yaw 角度 (CAN ID 0x104 转发) |
| 22 | 2 | crc16 | uint16 LE | Modbus CRC16 (byte 0-21) |

**总长: 24 字节**  
**频率: 500Hz**  
**发送方式: UART1 DMA TX**

### 数据来源

| 字段 | 来源 |
|------|------|
| quat_w/x/y/z | 顶板 FDCAN3 → DM_IMU_L1 陀螺仪 |
| yaw | 下板 FDCAN2 → 顶板 FDCAN1 (CAN ID 0x104, 4字节 float) |

### PC 端解析伪代码

```python
import struct

def parse_frame(data: bytes):
    if len(data) != 24:
        return None
    if data[0] != 0x69 or data[1] != 0x6D:
        return None

    # CRC16 校验 (byte 0-21)
    crc = crc16_modbus(data[:22])
    crc_rcvd = data[22] | (data[23] << 8)
    if crc != crc_rcvd:
        return None

    quat_w = struct.unpack('<f', data[2:6])[0]
    quat_x = struct.unpack('<f', data[6:10])[0]
    quat_y = struct.unpack('<f', data[10:14])[0]
    quat_z = struct.unpack('<f', data[14:18])[0]
    yaw    = struct.unpack('<f', data[18:22])[0]

    return quat_w, quat_x, quat_y, quat_z, yaw
```

### 时序示意

```
UART1 TX 波形 (500Hz = 2ms 周期):

  |<── 24 bytes DMA ──>|               |<── 24 bytes DMA ──>|
  |  im quat[16] yaw[4] crc  |  idle   |  im quat[16] yaw[4] crc  |  ...
  ───────────────────────────          ───────────────────────────
  |<──────────── 2000us ─────────────>|
```
