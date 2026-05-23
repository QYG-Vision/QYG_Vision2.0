#include "DJI_drv.h"

DJI_StatusType PITCH_ctrl(int16_t pitch_current)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t             tx_data[2];

    tx_header.StdId = 0x1fe;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = 2;

    tx_data[0] = (pitch_current >> 8) & 0xff;
    tx_data[1] =  pitch_current       & 0xff;

    if(HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
    {
        return DJI_ERROR;
    }else{
        return DJI_OK;
    }
}

DJI_StatusType YAW_ctrl(int16_t yaw_current)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t             tx_data[2];

    tx_header.StdId = 0x2fe;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = 2;

    tx_data[0] = (yaw_current >> 8) & 0xff;
    tx_data[1] =  yaw_current       & 0xff;

    if(HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
    {
        return DJI_ERROR;
    }else{
        return DJI_OK;
    }
}

DJI_StatusType FIRE_ctrl(int16_t current1, int16_t current2)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t             tx_data[4];

    tx_header.StdId = 0x200;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = 4;

    tx_data[0] = (current1 >> 8) & 0xff;
    tx_data[1] =  current1       & 0xff;
    tx_data[2] = (current2 >> 8) & 0xff;
    tx_data[3] =  current2       & 0xff;

    if(HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
    {
        return DJI_ERROR;
    }else{
        return DJI_OK;
    }
}

DJI_StatusType GET_FIRE_CMD(int16_t pluck_current)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t             tx_data[2];

    tx_header.StdId = 0x200;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = 2;

    tx_data[0] = (pluck_current >> 8) & 0xff;
    tx_data[1] =  pluck_current       & 0xff;

    if(HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
    {
        return DJI_ERROR;
    }else{
        return DJI_OK;
    }
}

