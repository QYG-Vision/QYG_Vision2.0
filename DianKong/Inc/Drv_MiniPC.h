//
// Created by ASUS on 2026/8/5.
//

#ifndef JIUXIAN_SP_GIMBAL_XPP_DRV_MINIPC_H
#define JIUXIAN_SP_GIMBAL_XPP_DRV_MINIPC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <usart.h>

#define __TX_MINIPC_HEADER_1        0x47 // "G" 广
#define __TX_MINIPC_HEADER_2        0x44 // "D" 大

#define __RX_MINIPC_HEADER_1        0x51 // "Q" 庆
#define __RX_MINIPC_HEADER_2        0x59 // “Y" 园

#define MINIPC_TX_MSG_LEN             51
#define MINIPC_RX_MSG_LEN             25

    typedef enum {
        DRV_MINIPC_OK      = 0x00U,
        DRV_MINIPC_ERROR   = 0x01U,
        DRV_MINIPC_BUSY    = 0x02U,
        DRV_MINIPC_TIMEOUT = 0x03U
    }DRV_MiniPC_StatusTypedef;

    typedef struct __attribute__((packed))
    {
        /* data frame */
        float real_Vel_x__;
        float real_Vel_y__;
        float real_Omega_z;

        float Reserve1;
        float Reserve2;
        float Reserve3;
        float Reserve4;
        float Reserve5;

        /**
         * @brief 哨兵状态位
         * 0 is false, 1 is true.
         */
        uint16_t sentry_state : 14;

        /**
         * @brief 自瞄状态位
         * 00 失能自瞄
         * 01 使能自瞄（装甲板）
         * 10 使能自瞄（小能量机关）
         * 11 使能自瞄（大能量机关）
         */
        uint16_t Auto_Aim_mode : 2;

        float Gimbal_Eular_Yaw;
        float Gimbal_Eular_Pitch;
        float Gimbal_Eular_Roll;

    }__MiniPC_Tx_Info_t;

    typedef struct __attribute__((packed))
    {
        uint8_t        State_cmd;

        float          Auto_Aim_Yaw;
        float          Auto_Aim_Pitch;

        float          Navigation_Vel_x;
        float          Navigation_Vel_y;

        float          Chassis_Omega;

    }__MiniPC_Rx_Info_t;

    typedef struct __attribute__((packed))
    {
        uint8_t            Tx_buffer[MINIPC_TX_MSG_LEN];
        uint8_t            Rx_buffer[MINIPC_RX_MSG_LEN];

        __MiniPC_Tx_Info_t Tx_Info;
        __MiniPC_Rx_Info_t Rx_Info;
    }MiniPC_Msg_Info_t;

    DRV_MiniPC_StatusTypedef MiniPC_Data_init(MiniPC_Msg_Info_t *minipc);
    DRV_MiniPC_StatusTypedef Set_Data_MiniPC(MiniPC_Msg_Info_t *minipc, UART_HandleTypeDef *huart, float V_x_fdb, float V_y_fdb, float Omega_z_fdb, bool isSentryUnderHP_, bool isSentryLackOfBullets_, bool auto_aim_bit0, bool auto_aim_bit1, float Gimbal_Eular_Yaw, float Gimbal_Eular_Pitch, float Gimbal_Eular_Roll);

    void Rx_Info_Data_Process(MiniPC_Msg_Info_t *minipc, uint8_t *Data, uint16_t Length);

#ifdef __cplusplus
}
#endif

#endif //JIUXIAN_SP_GIMBAL_XPP_DRV_MINIPC_H