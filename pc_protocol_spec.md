# PC 端通信协议说明

## 串口配置

| 参数 | 值 |
|------|-----|
| 波特率 | **921600** |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验位 | 无 |
| 流控 | 无 |

## 数据包格式 (定长 15 字节，小端序)

```
Byte |  0    1    2    3    4    5    6    7    8    9   10   11   12   13   14
-----+----------------------------------------------------------------------
字段 | 'p'  'c' mode yaw(rad)            pitch(rad)          tid  tvld crc16
     |                 [3:LSB ... 6:MSB]       [7:LSB ... 10:MSB]       [13:LSB 14:MSB]
```

### 字段说明

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|------|------|------|------|------|
| 0 | 2 | char[2] | header | 帧头，固定为 `'p'` `'c'` (0x70 0x63) |
| 2 | 1 | uint8 | mode | 模式：0=不控制, 1=控制, 2=控制且开火 |
| 3 | 4 | float32 | yaw | 偏航角，单位 rad，小端序 |
| 7 | 4 | float32 | pitch | 俯仰角，单位 rad，小端序 |
| 11 | 1 | uint8 | target_id | 目标 ID：0=无目标, 1-8=机器人/建筑 |
| 12 | 1 | uint8 | target_valid | 目标是否有效：0=无效, 非0=有效 |
| 13 | 2 | uint16 | crc16 | Modbus CRC16，小端序，对 byte 0-12 计算 |

## CRC16 计算

- **算法**: Modbus CRC16
- **多项式**: 0x8005
- **初始值**: 0xFFFF
- **输入**: byte 0 ~ byte 12 (共 13 字节)
- **输出**: 小端序填入 byte 13 (低字节)、byte 14 (高字节)

### C 参考实现

```c
uint16_t Modbus_CRC16(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}
```

### Python 参考实现

```python
def modbus_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc
```

### Python 组包示例

```python
import struct

def build_packet(mode: int, yaw: float, pitch: float,
                 target_id: int, target_valid: int) -> bytes:
    # 前 13 字节
    header = b'pc'
    payload = struct.pack('<BffBB', mode, yaw, pitch, target_id, target_valid)
    data = header + payload  # 13 bytes

    # CRC16 计算
    crc = modbus_crc16(data)
    return data + struct.pack('<H', crc)  # 15 bytes

# 示例: 控制模式, yaw=0.5rad, pitch=-0.3rad, 目标1, 有效
pkt = build_packet(1, 0.5, -0.3, 1, 1)
# pkt = b'pc\x01\x00\x00\x00?\x9a\x99\x99\xbe\x01\x01\x??\x??'
```

## CAN 转发帧格式 (供参考)

上层板收到 UART 数据并校验通过后，拆为两帧 CAN 报文通过 FDCAN1 发送：

**帧1 (CAN ID 0x102, DLC=8):**
```
[0]='p' [1]='c' [2]=mode [3-6]=yaw float32 LE [7]=0x00
```

**帧2 (CAN ID 0x103, DLC=8):**
```
[0-3]=pitch float32 LE [4]=target_id [5]=target_valid [6-7]=crc16 LE
```
