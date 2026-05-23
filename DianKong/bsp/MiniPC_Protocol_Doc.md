# 电控 ↔ 上位机通信协议说明文档

**版本**：Version 2.0  
**作者**：Guangzhi Tao  
**适用模块**：`bsp/MiniPC.c` / `bsp/MiniPC.h`  
**日期**：2026-01-18  

---

## 1. 概述

本协议用于哨兵电控板（STM32F427，以下简称**EC**）与上位机（MiniPC/NUC，以下简称**MiniPC**）之间的双向通信。  
Version 2.0 将视觉通道与导航通道合并为单一通信链路，所有数据通过同一 UART 串口传输。

**物理层**：UART + DMA（`HAL_UART_Transmit_DMA`）  
**字节序**：小端（Little-endian）  
**校验方式**：CRC16（默认 CRC16/X25，可切换为 CRC16-CCITT-F）  

---

## 2. 帧头定义

| 方向 | 帧头字节1 | 帧头字节2 | 说明 |
|------|-----------|-----------|------|
| EC → MiniPC | `0x47` ('G') | `0x44` ('D') | 宏 `__Msg2MINIPC_Frame_Header_1/2` |
| MiniPC → EC | `0x51` ('Q') | `0x59` ('Y') | 宏 `__Msg2EC_Frame_Header_1/2` |

---

## 3. EC → MiniPC（上行数据帧）

### 3.1 帧结构（共 51 字节）

| 偏移 | 字节数 | 字段名 | 类型 | 说明 |
|------|--------|--------|------|------|
| 0 | 1 | 帧头1 | uint8 | `0x47` |
| 1 | 1 | 帧头2 | uint8 | `0x44` |
| 2 | 1 | frame_id | uint8 | 帧类型 ID（当前模式标识） |
| 3~6 | 4 | real_Vel_x | float | 底盘实际 X 方向速度（m/s） |
| 7~10 | 4 | real_Vel_y | float | 底盘实际 Y 方向速度（m/s） |
| 11~14 | 4 | real_Omega_z | float | 底盘实际 Z 轴角速度（rad/s） |
| 15~18 | 4 | Chassis_Eular_Angle[0] | float | 底盘 Yaw 角（rad） |
| 19~22 | 4 | Chassis_Eular_Angle[1] | float | 底盘 Pitch 角（rad） |
| 23~26 | 4 | Omrga_Chassis[0] | float | 云台 Yaw 轴角速度（rad/s） |
| 27~30 | 4 | Omrga_Chassis[1] | float | 云台 Pitch 轴角速度（rad/s） |
| 31~34 | 4 | ODO | float | 里程计累计值（m） |
| 35 | 1 | sentry_status | uint8 | 哨兵当前运行状态（见 3.2 节） |
| 36 | 1 | gimbal_req_mode | uint8 | 云台向视觉请求的工作模式（见 3.3 节） |
| 37~40 | 4 | Gimbal_Eular_Angle[0] | float | 云台 Yaw 角（rad） |
| 41~44 | 4 | Gimbal_Eular_Angle[1] | float | 云台 Pitch 角（rad） |
| 45~48 | 4 | Gimbal_Eular_Angle[2] | float | 云台 Roll 角（rad） |
| 49~50 | 2 | CRC16 | uint16 | 对前 49 字节计算的 CRC16 校验值 |

> **注**：所有 float 字段均以 IEEE 754 单精度格式、小端字节序传输，直接对原始 bit 进行拷贝（无量化压缩）。

---

### 3.2 sentry_status 定义

| 值 | 宏定义 | 含义 |
|----|--------|------|
| `0x00` | `_None_Mode_Navi_FDB` | 无模式 |
| `0x01` | `_Navigation_Mode_FDB` | 导航模式 |
| `0x02` | `_Visaul_First_Mode_FDB` | 视觉优先模式 |
| `0x03` | `_OASS_Mode_FDB` | 避障模式（OASS） |

---

### 3.3 gimbal_req_mode 定义（EC 向视觉请求的模式）

| 值 | 宏定义 | 含义 |
|----|--------|------|
| `0x10` | `_None_Mode` | 无请求 |
| `0x11` | `_Auto_Aim_Mode` | 自动瞄准（机器人/前哨站/基地） |
| `0x12` | `_L_E_D` | 小能量机关自动瞄准 |
| `0x13` | `_H_E_D` | 大能量机关自动瞄准 |

---

### 3.4 相关函数

```c
// 填充待发送数据
_MINIPC_FCN SET_DATA2MINIPC_2_0_(
    _NUC_UART_INFO_2_0_t *nuc,
    float   fdb_vel_x,          // 底盘X速度反馈
    float   fdb_vel_y,          // 底盘Y速度反馈
    float   fdb_omega,          // 底盘角速度反馈
    float   chassis_yaw,        // 底盘Yaw角
    float   chassis_pitch,      // 底盘Pitch角
    float   chassis_omega_yaw,  // 云台Yaw轴角速度
    float   chassis_omega_pitch,// 云台Pitch轴角速度
    float   odo,                // 里程计
    uint8_t sentry_status,      // 哨兵状态
    uint8_t gimbal_req_mode,    // 请求视觉模式
    float  *gimbal_eular_angle  // 云台欧拉角 [yaw, pitch, roll]
);

// 封帧并通过DMA发送
_MINIPC_FCN NUC_SendMsg2MINIPC_2_0_(
    _NUC_UART_INFO_2_0_t *nuc,
    CRC16_INFO_t         *crc_info,
    UART_HandleTypeDef   *huart,
    uint8_t               mode    // 填入 frame_id
);
```

---

## 4. MiniPC → EC（下行控制帧）

### 4.1 帧结构（有效载荷 25 字节）

| 偏移 | 字节数 | 字段名 | 类型 | 说明 |
|------|--------|--------|------|------|
| 0 | 1 | 帧头1 | uint8 | `0x51` |
| 1 | 1 | 帧头2 | uint8 | `0x59` |
| 2 | 1 | gimbal_mode | uint8 | 控制模式指令（见 4.2 节） |
| 3~6 | 4 | Gimbal_Eular_Angle_enc[0] | uint32 | 云台目标 Yaw 角（量化编码） |
| 7~10 | 4 | Gimbal_Eular_Angle_enc[1] | uint32 | 云台目标 Pitch 角（量化编码） |
| 11~14 | 4 | Chassis_Vel_enc[0] | uint32 | 底盘目标 X 速度（量化编码） |
| 15~18 | 4 | Chassis_Vel_enc[1] | uint32 | 底盘目标 Y 速度（量化编码） |
| 19~22 | 4 | Chassis_Omega_enc | uint32 | 底盘目标角速度（量化编码） |
| 23~24 | 2 | CRC16 | uint16 | 对前 23 字节计算的 CRC16 校验值 |

---

### 4.2 gimbal_mode 定义

| 值 | 宏定义 | 含义 |
|----|--------|------|
| `0x00` | `_DISABLE_CTRL_MINIPC` | 禁用控制，EC 忽略所有指令 |
| `0x01` | `_NO_FIRE_MODE_MINIPC` | 追踪但不自动开火 |
| `0x02` | `_FIRE_MODE_MINIPC` | 追踪并自动开火 |

---

### 4.3 量化解码规则

MiniPC 下发的角度与速度字段均为 **32 位无符号整数量化编码**，EC 按如下公式还原为浮点数：

$$
x = \frac{x\_int}{2^{32} - 1} \times (x\_max - x\_min) + x\_min
$$

| 字段 | $x\_min$ | $x\_max$ | 单位 |
|------|----------|----------|------|
| 云台 Yaw / Pitch 角 | $-\pi$ | $+\pi$ | rad |
| 底盘 X / Y 速度 | $-1$ | $+1$ | 归一化 |
| 底盘角速度 | $-1$ | $+1$ | 归一化 |

解码后结果存入：
- `RX_INFO_2_0.Gimbal_Eular_Angle[0/1]`（rad）
- `RX_INFO_2_0.Gimbal_Eular_Angle_Degree[0/1]`（自动转换为度）
- `RX_INFO_2_0.Chassis_Vel[0/1]`
- `RX_INFO_2_0.Chassis_Omega`

---

### 4.4 相关函数

```c
// 接收完成后校验并解包
_MINIPC_FCN NUC_UnpackMsgfromMiniPC(
    _NUC_UART_INFO_2_0_t *nuc,
    CRC16_INFO_t         *crc_info,
    uint8_t               len       // 有效数据长度
);

// 从 FIFO 中提取 CRC16 校验字节
_MINIPC_FCN GET_CRC16_from_NUC_Fifo_2_0_(
    _NUC_UART_INFO_2_0_t *nuc,
    uint8_t               data_Sec2L,
    uint8_t               data_Last,
    CRC16_INFO_t         *crc_info
);
```

---

## 5. 数据流总览

```
┌─────────────────────────────────────────────────────────────┐
│                        EC（STM32F427）                       │
│                                                             │
│  传感器/状态数据：                                            │
│  底盘速度(vx,vy,ω) + 底盘姿态(yaw,pitch)                    │
│  云台角速度(ω_yaw,ω_pitch) + 里程计(ODO)                    │
│  云台欧拉角(yaw,pitch,roll) + 哨兵状态 + 视觉模式请求         │
│                            │                                │
│                     UART TX (DMA)                           │
│                     51 字节 / 帧                             │
│                     帧头: 0x47 0x44                          │
│                            │                                │
│                            ▼                                │
│                    ┌──────────────┐                         │
│                    │   MiniPC     │                         │
│                    │  (NUC/PC)    │                         │
│                    │  视觉算法     │                         │
│                    │  导航算法     │                         │
│                    └──────────────┘                         │
│                            │                                │
│                     UART RX (DMA)                           │
│                     25 字节 / 帧                             │
│                     帧头: 0x51 0x59                          │
│                            │                                │
│  控制指令：                                                   │
│  云台目标角度(yaw,pitch) + 底盘速度指令(vx,vy,ω)              │
│  + 射击控制模式(DISABLE/NO_FIRE/FIRE)                        │
└─────────────────────────────────────────────────────────────┘
```

---

## 6. 接收解析流程

```
UART 空闲中断（IDLE）触发
        │
        ▼
检测帧头 [0x51, 0x59]
        │
        ▼
提取有效字节至 useful_info[]
        │
        ▼
GET_CRC16_from_NUC_Fifo_2_0_() 提取末尾 CRC16
        │
        ▼
NUC_UnpackMsgfromMiniPC()
   ├─ Verify_CRC16() 校验失败 → 清空缓冲区，丢弃本帧
   └─ 校验通过 → 解析 gimbal_mode / 角度 / 速度字段
                  → uint_to_float() 反量化
                  → 存入 RX_INFO_2_0 结构体
```

---

## 7. 关键数据结构

### 7.1 发送结构（EC → MiniPC）

```c
typedef struct {
    uint8_t TxFifo[51];              // 发送缓冲区
    uint8_t frame_Header[2];
    uint8_t frame_id;
    uint8_t frame_CRC16[2];

    float   real_Vel_x__;            // 底盘X速度
    float   real_Vel_y__;            // 底盘Y速度
    float   real_Omega_z;            // 底盘角速度
    float   Chassis_Eular_Angle[2];  // 底盘姿态 [yaw, pitch]
    float   Omrga_Chassis[2];        // 云台角速度 [yaw, pitch]
    float   ODO;                     // 里程计
    uint8_t sentry_status;           // 哨兵状态
    uint8_t gimbal_req_mode;         // 请求视觉模式
    float   Gimbal_Eular_Angle[3];   // 云台欧拉角 [yaw, pitch, roll]
} _NUC_TX_INFO_2_0_t;
```

### 7.2 接收结构（MiniPC → EC）

```c
typedef struct {
    uint8_t  RxFifo[128];
    uint8_t  useful_info[25];

    uint8_t  gimbal_mode;                  // 控制模式
    uint32_t Gimbal_Eular_Angle_enc[2];    // 云台目标角度（编码）
    uint32_t Chassis_Vel_enc[2];           // 底盘速度（编码）
    uint32_t Chassis_Omega_enc;            // 底盘角速度（编码）

    float    Gimbal_Eular_Angle[2];        // 云台目标角度（rad）
    float    Gimbal_Eular_Angle_Degree[2]; // 云台目标角度（度）
    float    Chassis_Vel[2];               // 底盘速度（归一化）
    float    Chassis_Omega;               // 底盘角速度（归一化）

    CRC16_t  crc16_check;
} _NUC_RX_INFO_2_0_t;
```

---

## 8. 注意事项

1. **字节序**：所有 float/uint32 字段均为**小端**传输，上位机解析时须注意。
2. **CRC 类型**：默认使用 CRC16/X25（非 CCITT-F 模式），CRC 字节排列为低字节在前（little-endian）。若切换为 CRC16-CCITT-F，字节顺序相反，代码已做分支处理。
3. **速度归一化**：MiniPC 下发的底盘速度为 $[-1, 1]$ 归一化值，EC 需根据实际最大速度参数自行换算为物理量。
4. **接收缓冲区清零**：解包成功或失败后均应调用 `memset(rxbuffer, 0, ...)` 清空缓冲区，防止脏数据残留。
5. **帧丢失处理**：CRC 校验失败时直接丢弃本帧，不做重传请求，依赖高频率周期发送保证实时性。
