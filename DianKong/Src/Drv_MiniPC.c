//
// Created by ASUS on 2026/8/5.
//
#include "../Inc/Drv_MiniPC.h"
#include "../Inc/MiniPC_CRC.h"

DRV_MiniPC_StatusTypedef MiniPC_Data_init(MiniPC_Msg_Info_t *minipc) {
    minipc->Tx_Info.real_Vel_x__ = 0.0f;
    minipc->Tx_Info.real_Vel_y__ = 0.0f;
    minipc->Tx_Info.real_Omega_z = 0.0f;

    minipc->Tx_Info.Reserve1     = 0.0f;
    minipc->Tx_Info.Reserve2     = 0.0f;
    minipc->Tx_Info.Reserve3     = 0.0f;
    minipc->Tx_Info.Reserve4     = 0.0f;
    minipc->Tx_Info.Reserve5     = 0.0f;

    minipc->Tx_Info.sentry_state = 0;

    minipc->Tx_Info.Auto_Aim_mode = 0;

    minipc->Tx_Info.Gimbal_Eular_Yaw          = 0.0f;
    minipc->Tx_Info.Gimbal_Eular_Pitch        = 0.0f;
    minipc->Tx_Info.Gimbal_Eular_Roll         = 0.0f;

    minipc->Rx_Info.State_cmd        = 0;

    minipc->Rx_Info.Auto_Aim_Yaw     = 0.0f;
    minipc->Rx_Info.Auto_Aim_Pitch   = 0.0f;

    minipc->Rx_Info.Navigation_Vel_x = 0.0f;
    minipc->Rx_Info.Navigation_Vel_y = 0.0f;

    minipc->Rx_Info.Chassis_Omega    = 0.0f;

    return DRV_MINIPC_OK;
}

DRV_MiniPC_StatusTypedef Set_Data_MiniPC(MiniPC_Msg_Info_t *minipc, UART_HandleTypeDef *huart, float V_x_fdb,
                                                                                               float V_y_fdb,
                                                                                               float Omega_z_fdb,
                                                                                               bool  isSentryUnderHP_,
                                                                                               bool  isSentryLackOfBullets_,
                                                                                               bool  auto_aim_bit0,
                                                                                               bool  auto_aim_bit1,
                                                                                               float Gimbal_Eular_Yaw,
                                                                                               float Gimbal_Eular_Pitch,
                                                                                               float Gimbal_Eular_Roll)
{
    minipc->Tx_Info.real_Vel_x__              = V_x_fdb;
    minipc->Tx_Info.real_Vel_y__              = V_y_fdb;
    minipc->Tx_Info.real_Omega_z              = Omega_z_fdb;

    minipc->Tx_Info.sentry_state              = (isSentryUnderHP_       <<  0) |
                                                (isSentryLackOfBullets_ <<  1);

    minipc->Tx_Info.Auto_Aim_mode             = (auto_aim_bit0 << 0) |
                                                (auto_aim_bit1 << 1);

    minipc->Tx_Info.Gimbal_Eular_Yaw          = Gimbal_Eular_Yaw;
    minipc->Tx_Info.Gimbal_Eular_Pitch        = Gimbal_Eular_Pitch;
    minipc->Tx_Info.Gimbal_Eular_Roll         = Gimbal_Eular_Roll;

    minipc->Tx_buffer[0]                      = __TX_MINIPC_HEADER_1;
    minipc->Tx_buffer[1]                      = __TX_MINIPC_HEADER_2;

    memcpy(&minipc->Tx_buffer[3], &minipc->Tx_Info, 46);

    minipc_append_crc16_check_sum(minipc->Tx_buffer, MINIPC_TX_MSG_LEN);

    if (minipc_verify_crc16_check_sum(minipc->Tx_buffer, MINIPC_TX_MSG_LEN) != true) {
        return DRV_MINIPC_ERROR;
    }

    if (HAL_UART_Transmit_DMA(huart, minipc->Tx_buffer, MINIPC_TX_MSG_LEN) != HAL_OK) {
        return DRV_MINIPC_ERROR;
    }else {
        return DRV_MINIPC_OK;
    }
}

void Rx_Info_Data_Process(MiniPC_Msg_Info_t *minipc, uint8_t *Data, uint16_t Length) {
    for (uint16_t i = 0; i < Length; i++){
        if (Data[i] != __RX_MINIPC_HEADER_1) {
            continue;
        }

        if (Data[i + 1] != __RX_MINIPC_HEADER_2) {
            continue;
        }

        if (i + MINIPC_RX_MSG_LEN - 2 > Length) {
            break;
        }

        if (minipc_verify_crc16_check_sum(Data + i, MINIPC_RX_MSG_LEN) != true) {
            continue;
        }

        memcpy(&minipc->Rx_Info, Data + i + 2, MINIPC_RX_MSG_LEN - 4);
    }
}
