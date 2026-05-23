/**
 * @file MiniPC.c
 * @author Guangzhi Tao
 * @brief 
 * @version 2.0
 * @date 2026-01-18
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "MiniPC.h"

static float uint_to_float(uint32_t x_int, float x_min, float x_max, uint8_t bits) {
    float span = x_max - x_min;
    
    // (1ULL << bits) - 1 计算映射全量上限最大值 (保证中间变量为64位防止溢出)
    uint64_t max_val = (1ULL << bits) - 1;
    
    // 按比例转换回来，加上最小值 offset 作为偏移
    float offset = (float)(x_int) * span / (float)(max_val);
    return offset + x_min;
}

////////////////////////////////////////////////////////////Version 1.0 函数 ////////////////////////////////////////////////////////////////////////

_MINIPC_FCN SET_DATA2TX2Visual(_NUC_UART_INFO_t *nuc, float *quaternion, float yaw, float pitch, float shoot_vel)
{
    nuc ->TX_Visual_INFO.quaternion[0] = quaternion[0];
    nuc ->TX_Visual_INFO.quaternion[1] = quaternion[1];
    nuc ->TX_Visual_INFO.quaternion[2] = quaternion[2];
    nuc ->TX_Visual_INFO.quaternion[3] = quaternion[3];

    nuc ->TX_Visual_INFO.yaw   = yaw  ;
    nuc ->TX_Visual_INFO.pitch = pitch;

    nuc ->TX_Visual_INFO.bullet_speed = shoot_vel;

    return _MINIPC_OK;
}

_MINIPC_FCN SET_DATA2NAVI(_NUC_UART_INFO_t *nuc, 
                                                 float   real_vel_x       , 
                                                 float   real_vel_y       , 
                                                 float   real_omega       , 
                                                 float   imu_yaw          , 
                                                 float   imu_pitch        , 
                                                 float   omega_yaw        , 
                                                 float   omega_pitch      , 
                                                 float   odo              , 
                                                 uint8_t sentry_status
                         )
{
    nuc ->TX_Navigation_INFO.real_Vel_x__  = real_vel_x;
    nuc ->TX_Navigation_INFO.real_Vel_y__  = real_vel_y;
    nuc ->TX_Navigation_INFO.real_Omega_z  = real_omega;

    nuc ->TX_Navigation_INFO.IMU_yaw       = imu_yaw;
    nuc ->TX_Navigation_INFO.IMU_pitch     = imu_pitch;

    nuc ->TX_Navigation_INFO.Omrga_yaw     = omega_yaw;
    nuc ->TX_Navigation_INFO.Omrga_pitch   = omega_pitch;

    nuc ->TX_Navigation_INFO.ODO           = odo;

    nuc ->TX_Navigation_INFO.sentry_status = sentry_status;

    return _MINIPC_OK;
}

_MINIPC_FCN GET_CRC16_from_NUC_Fifo(_NUC_UART_INFO_t *nuc, uint8_t data_Sec2L, uint8_t data_Last, uint8_t mode, CRC16_INFO_t *crc_info)
{
    if(crc_info ->CRC16_type != CRC16_CCITT_F)
    {
        switch (mode)
        {
            case _IDLE_MODE_RX_       : 
            {
                break;
            }
            case _VISUAL_MODE_RX_     :
            {
                nuc ->RX_Visual_INFO.crc16_value     = (data_Last  << 8) | data_Sec2L;
                break;
            }
            case _NAVIGATION_MODE_RX_ :
            {
                nuc ->RX_Navigation_INFO.crc16_value = (data_Last  << 8) | data_Sec2L;
                break;
            }
            default                   :
            {
                return _MINIPC_ERROR;
            }
        }
    }else if(crc_info ->CRC16_type == CRC16_CCITT_F)
    {
        switch (mode)
        {
            case _IDLE_MODE_RX_       :
            {
                break;
            }
            case _VISUAL_MODE_RX_     :
            {
                nuc ->RX_Visual_INFO.crc16_value     = (data_Sec2L << 8) | data_Last ;
                break;
            }
            case _NAVIGATION_MODE_RX_ :
            {
                nuc ->RX_Navigation_INFO.crc16_value = (data_Sec2L << 8) | data_Last ; 
                break;
            }
            default                   :
            {
                return _MINIPC_ERROR;
            }
        }
    }else{
        return _MINIPC_ERROR;
    }
    return _MINIPC_OK;
}

/* Unpack the data frame sent by the NUC */
/*
    ** @attention : Listen! The data sent to us by the NUC is also in little-endian format, so be sure to pay attention when unpacking the data frames!
*/
_MINIPC_FCN NUC_UnpackMsgfromVisual(_NUC_UART_INFO_t *nuc, CRC16_INFO_t *crc_info, uint8_t len)
{
    if(Verify_CRC16(nuc ->RX_Visual_INFO.useful_info, len, crc_info) != __CRC16_True)    
    {
        return _MINIPC_ERROR;
    }else{
        nuc ->RX_Visual_INFO.gimbal_mode        =              nuc ->RX_Visual_INFO.useful_info[2]        ;

        nuc ->RX_Visual_INFO.dyaw_enc           =   (uint32_t)(nuc ->RX_Visual_INFO.useful_info[3]       ) | 
                                                    (uint32_t)(nuc ->RX_Visual_INFO.useful_info[4]  <<  8) | 
                                                    (uint32_t)(nuc ->RX_Visual_INFO.useful_info[5]  << 16) | 
                                                    (uint32_t)(nuc ->RX_Visual_INFO.useful_info[6]  << 24);

        nuc ->RX_Visual_INFO.pitch_enc          =   (uint32_t)(nuc ->RX_Visual_INFO.useful_info[7]       ) |
                                                    (uint32_t)(nuc ->RX_Visual_INFO.useful_info[8]  <<  8) |
                                                    (uint32_t)(nuc ->RX_Visual_INFO.useful_info[9]  << 16) |
                                                    (uint32_t)(nuc ->RX_Visual_INFO.useful_info[10] << 24);
        
        memset(nuc ->rxbuffer, 0, _RxFifofromNUC_SIZE_t);

        nuc ->RX_Visual_INFO.dyaw               = unpack_ieee754_32(nuc ->RX_Visual_INFO.dyaw_enc );
        nuc ->RX_Visual_INFO.pitch              = unpack_ieee754_32(nuc ->RX_Visual_INFO.pitch_enc);

        return _MINIPC_OK;
    }
}

_MINIPC_FCN NUC_UnpackMsgfromVisual_DM(_NUC_UART_INFO_t *nuc, CRC16_INFO_t *crc_info, uint8_t len)
{
    if(Verify_CRC16(nuc ->RX_Visual_INFO.useful_info, len, crc_info) != __CRC16_True)    
    {
        return _MINIPC_ERROR;
    }else{
        nuc ->RX_Visual_INFO.gimbal_mode        =              nuc ->RX_Visual_INFO.useful_info[2]        ;

        nuc ->RX_Visual_INFO.dyaw_enc           =   (uint32_t)((nuc ->RX_Visual_INFO.useful_info[3]       ) | 
                                                               (nuc ->RX_Visual_INFO.useful_info[4]  <<  8) | 
                                                               (nuc ->RX_Visual_INFO.useful_info[5]  << 16) | 
                                                               (nuc ->RX_Visual_INFO.useful_info[6]  << 24));

        nuc ->RX_Visual_INFO.pitch_enc          =   (uint32_t)((nuc ->RX_Visual_INFO.useful_info[7]       ) |
                                                               (nuc ->RX_Visual_INFO.useful_info[8]  <<  8) |
                                                               (nuc ->RX_Visual_INFO.useful_info[9]  << 16) |
                                                               (nuc ->RX_Visual_INFO.useful_info[10] << 24));
        
        memset(nuc ->rxbuffer, 0, _RxFifofromNUC_SIZE_t);

        nuc ->RX_Visual_INFO.yaw_f              = uint_to_float(nuc ->RX_Visual_INFO.dyaw_enc , -PI, PI, 32);
        nuc ->RX_Visual_INFO.pitch_f            = uint_to_float(nuc ->RX_Visual_INFO.pitch_enc, -PI, PI, 32);

        return _MINIPC_OK;
    }
}

_MINIPC_FCN NUC_UnpackMsgfromNavigation(_NUC_UART_INFO_t *nuc, CRC16_INFO_t *crc_info, uint8_t len)
{
    if(Verify_CRC16(nuc ->RX_Navigation_INFO.useful_info, len, crc_info) != __CRC16_True)    
    {
        return _MINIPC_ERROR;
    }else{
        nuc ->RX_Navigation_INFO.running_mode   =              nuc ->RX_Navigation_INFO.useful_info[2]  ;

        nuc ->RX_Navigation_INFO.ctrl_vel_x_enc =   (uint32_t)(nuc ->RX_Navigation_INFO.useful_info[3]        ) | 
                                                    (uint32_t)(nuc ->RX_Navigation_INFO.useful_info[4]   <<  8) | 
                                                    (uint32_t)(nuc ->RX_Navigation_INFO.useful_info[5]   << 16) |
                                                    (uint32_t)(nuc ->RX_Navigation_INFO.useful_info[6]   << 24);

        nuc ->RX_Navigation_INFO.ctrl_vel_y_enc =   (uint32_t)(nuc ->RX_Navigation_INFO.useful_info[7]        ) |
                                                    (uint32_t)(nuc ->RX_Navigation_INFO.useful_info[8]   <<  8) |
                                                    (uint32_t)(nuc ->RX_Navigation_INFO.useful_info[9]   << 16) |
                                                    (uint32_t)(nuc ->RX_Navigation_INFO.useful_info[10]  << 24);

        nuc ->RX_Navigation_INFO.ctrl_omega_enc =   (uint32_t)(nuc ->RX_Navigation_INFO.useful_info[11]       ) |
                                                    (uint32_t)(nuc ->RX_Navigation_INFO.useful_info[12]  <<  8) |
                                                    (uint32_t)(nuc ->RX_Navigation_INFO.useful_info[13]  << 16) |
                                                    (uint32_t)(nuc ->RX_Navigation_INFO.useful_info[14]  << 24);

        nuc ->RX_Navigation_INFO.max_vel_enc    =   (uint32_t)(nuc ->RX_Navigation_INFO.useful_info[15]       ) |
                                                    (uint32_t)(nuc ->RX_Navigation_INFO.useful_info[16]  <<  8) |
                                                    (uint32_t)(nuc ->RX_Navigation_INFO.useful_info[17]  << 16) |
                                                    (uint32_t)(nuc ->RX_Navigation_INFO.useful_info[18]  << 24);

        nuc ->RX_Navigation_INFO.max_omega_enc  =   (uint32_t)(nuc ->RX_Navigation_INFO.useful_info[19]       ) |
                                                    (uint32_t)(nuc ->RX_Navigation_INFO.useful_info[20]  <<  8) |
                                                    (uint32_t)(nuc ->RX_Navigation_INFO.useful_info[21]  << 16) |
                                                    (uint32_t)(nuc ->RX_Navigation_INFO.useful_info[22]  << 24);

        memset(nuc ->rxbuffer, 0, _RxFifofromNUC_SIZE_t);

        nuc ->RX_Navigation_INFO.ctrl_vel_x     = unpack_ieee754_32(nuc ->RX_Navigation_INFO.ctrl_vel_x_enc);
        nuc ->RX_Navigation_INFO.ctrl_vel_y     = unpack_ieee754_32(nuc ->RX_Navigation_INFO.ctrl_vel_y_enc);
        nuc ->RX_Navigation_INFO.ctrl_omega     = unpack_ieee754_32(nuc ->RX_Navigation_INFO.ctrl_omega_enc);
        nuc ->RX_Navigation_INFO.max_vel        = unpack_ieee754_32(nuc ->RX_Navigation_INFO.max_vel_enc   );
        nuc ->RX_Navigation_INFO.max_omega      = unpack_ieee754_32(nuc ->RX_Navigation_INFO.max_omega_enc );

        return _MINIPC_OK;
    }
}



/* send data to NUC */

_MINIPC_FCN NUC_SendMsg2Visual(_NUC_UART_INFO_t *nuc, UART_HandleTypeDef *huart, uint8_t mode, CRC16_INFO_t *crc_info)
{
    uint16_t _CRC_check_ = 0;
    uint16_t _CRC_Calc_  = 0;

    nuc ->TX_Visual_INFO.frame_Header[0] = _Visual_Frame_Header_1;
    nuc ->TX_Visual_INFO.frame_Header[1] = _Visual_Frame_Header_2;

    nuc ->TX_Visual_INFO.frame_id        = mode;

    /* Frame Header */
    nuc ->TX_Visual_INFO.TxFifo[0] = nuc ->TX_Visual_INFO.frame_Header[0];
    nuc ->TX_Visual_INFO.TxFifo[1] = nuc ->TX_Visual_INFO.frame_Header[1];

    /* Frame ID */
    nuc ->TX_Visual_INFO.TxFifo[2] = nuc ->TX_Visual_INFO.frame_id;

    /* Data Frame */
    /*
        ** @attention : Little-endian transmission!!! 
    */
    //q[0]
    nuc ->TX_Visual_INFO.TxFifo[3]  = (uint8_t)( *(uint32_t*)&nuc ->TX_Visual_INFO.quaternion[0]       );
    nuc ->TX_Visual_INFO.TxFifo[4]  = (uint8_t)((*(uint32_t*)&nuc ->TX_Visual_INFO.quaternion[0]) >> 8 );
    nuc ->TX_Visual_INFO.TxFifo[5]  = (uint8_t)((*(uint32_t*)&nuc ->TX_Visual_INFO.quaternion[0]) >> 16);
    nuc ->TX_Visual_INFO.TxFifo[6]  = (uint8_t)((*(uint32_t*)&nuc ->TX_Visual_INFO.quaternion[0]) >> 24);
    //q[1]
    nuc ->TX_Visual_INFO.TxFifo[7]  = (uint8_t)( *(uint32_t*)&nuc ->TX_Visual_INFO.quaternion[1]       );
    nuc ->TX_Visual_INFO.TxFifo[8]  = (uint8_t)((*(uint32_t*)&nuc ->TX_Visual_INFO.quaternion[1]) >> 8 );
    nuc ->TX_Visual_INFO.TxFifo[9]  = (uint8_t)((*(uint32_t*)&nuc ->TX_Visual_INFO.quaternion[1]) >> 16);
    nuc ->TX_Visual_INFO.TxFifo[10] = (uint8_t)((*(uint32_t*)&nuc ->TX_Visual_INFO.quaternion[1]) >> 24);
    //q[2]
    nuc ->TX_Visual_INFO.TxFifo[11] = (uint8_t)( *(uint32_t*)&nuc ->TX_Visual_INFO.quaternion[2]       );
    nuc ->TX_Visual_INFO.TxFifo[12] = (uint8_t)((*(uint32_t*)&nuc ->TX_Visual_INFO.quaternion[2]) >> 8 );
    nuc ->TX_Visual_INFO.TxFifo[13] = (uint8_t)((*(uint32_t*)&nuc ->TX_Visual_INFO.quaternion[2]) >> 16);
    nuc ->TX_Visual_INFO.TxFifo[14] = (uint8_t)((*(uint32_t*)&nuc ->TX_Visual_INFO.quaternion[2]) >> 24);
    //q[3]
    nuc ->TX_Visual_INFO.TxFifo[15] = (uint8_t)( *(uint32_t*)&nuc ->TX_Visual_INFO.quaternion[3]       );
    nuc ->TX_Visual_INFO.TxFifo[16] = (uint8_t)((*(uint32_t*)&nuc ->TX_Visual_INFO.quaternion[3]) >> 8 );
    nuc ->TX_Visual_INFO.TxFifo[17] = (uint8_t)((*(uint32_t*)&nuc ->TX_Visual_INFO.quaternion[3]) >> 16);
    nuc ->TX_Visual_INFO.TxFifo[18] = (uint8_t)((*(uint32_t*)&nuc ->TX_Visual_INFO.quaternion[3]) >> 24);
    //yaw
    nuc ->TX_Visual_INFO.TxFifo[19] = (uint8_t)( *(uint32_t*)&nuc ->TX_Visual_INFO.yaw                 );
    nuc ->TX_Visual_INFO.TxFifo[20] = (uint8_t)( *(uint32_t*)&nuc ->TX_Visual_INFO.yaw            >> 8 );
    nuc ->TX_Visual_INFO.TxFifo[21] = (uint8_t)( *(uint32_t*)&nuc ->TX_Visual_INFO.yaw            >> 16);
    nuc ->TX_Visual_INFO.TxFifo[22] = (uint8_t)( *(uint32_t*)&nuc ->TX_Visual_INFO.yaw            >> 24);
    //pitch
    nuc ->TX_Visual_INFO.TxFifo[23] = (uint8_t)( *(uint32_t*)&nuc ->TX_Visual_INFO.pitch               );
    nuc ->TX_Visual_INFO.TxFifo[24] = (uint8_t)( *(uint32_t*)&nuc ->TX_Visual_INFO.pitch          >> 8 );
    nuc ->TX_Visual_INFO.TxFifo[25] = (uint8_t)( *(uint32_t*)&nuc ->TX_Visual_INFO.pitch          >> 16);
    nuc ->TX_Visual_INFO.TxFifo[26] = (uint8_t)( *(uint32_t*)&nuc ->TX_Visual_INFO.pitch          >> 24);
    //bullet_speed
    nuc ->TX_Visual_INFO.TxFifo[27] = (uint8_t)( *(uint32_t*)&nuc ->TX_Visual_INFO.bullet_speed        );
    nuc ->TX_Visual_INFO.TxFifo[28] = (uint8_t)( *(uint32_t*)&nuc ->TX_Visual_INFO.bullet_speed   >> 8 );
    nuc ->TX_Visual_INFO.TxFifo[29] = (uint8_t)( *(uint32_t*)&nuc ->TX_Visual_INFO.bullet_speed   >> 16);
    nuc ->TX_Visual_INFO.TxFifo[30] = (uint8_t)( *(uint32_t*)&nuc ->TX_Visual_INFO.bullet_speed   >> 24);

    /* Calc CRC16/X25 */
    /* CRC Frame */
    uint32_t CRC_state = Append_CRC16(nuc ->TX_Visual_INFO.TxFifo, _TXFIFO2VISUAL_SIZE_t, crc_info);
    if(CRC_state == __CRC16_False){
        return _MINIPC_ERROR;
    }

    _CRC_Calc_ = Get_CRC16(nuc ->TX_Visual_INFO.TxFifo, _TXFIFO2VISUAL_SIZE_t - 2, crc_info);
    if(crc_info ->CRC16_type != CRC16_CCITT_F)
    {
        _CRC_check_ = nuc ->TX_Visual_INFO.TxFifo[_TXFIFO2VISUAL_SIZE_t - 1] << 8 | nuc ->TX_Visual_INFO.TxFifo[_TXFIFO2VISUAL_SIZE_t - 2];
    }
    else{
        _CRC_check_ = nuc ->TX_Visual_INFO.TxFifo[_TXFIFO2VISUAL_SIZE_t - 2] << 8 | nuc ->TX_Visual_INFO.TxFifo[_TXFIFO2VISUAL_SIZE_t - 1];
    }

    if(_CRC_Calc_ != _CRC_check_)
    {
        return _MINIPC_ERROR;
    }

    if(HAL_UART_Transmit_DMA(huart, nuc ->TX_Visual_INFO.TxFifo, 33) != HAL_OK)
    {
        return _MINIPC_ERROR;
    }else{
        return _MINIPC_OK;
    }
}

_MINIPC_FCN NUC_SendMsg2Navigation(_NUC_UART_INFO_t *nuc, UART_HandleTypeDef *huart, uint8_t mode, CRC16_INFO_t *crc_info)
{
    uint16_t _CRC_check_ = 0;
    uint16_t _CRC_Calc_  = 0;

    nuc ->TX_Navigation_INFO.frame_Header[0] = _Navigation_Frame_Header_1;
    nuc ->TX_Navigation_INFO.frame_Header[1] = _Navigation_Frame_Header_2;

    nuc ->TX_Navigation_INFO.frame_id        = mode;

    /* Frame Header */
    nuc ->TX_Navigation_INFO.TxFifo[0] = nuc ->TX_Navigation_INFO.frame_Header[0];
    nuc ->TX_Navigation_INFO.TxFifo[1] = nuc ->TX_Navigation_INFO.frame_Header[1];

    /* Frame ID */
    nuc ->TX_Navigation_INFO.TxFifo[2] = nuc ->TX_Navigation_INFO.frame_id;

    /* Data Frame */
    /*
        ** @attention : Little-endian transmission!!! 
    */
    //x_vel
    nuc ->TX_Navigation_INFO.TxFifo[3]  = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.real_Vel_x__         );
    nuc ->TX_Navigation_INFO.TxFifo[4]  = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.real_Vel_x__    >> 8 );
    nuc ->TX_Navigation_INFO.TxFifo[5]  = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.real_Vel_x__    >> 16);
    nuc ->TX_Navigation_INFO.TxFifo[6]  = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.real_Vel_x__    >> 24);

    //y_vel
    nuc ->TX_Navigation_INFO.TxFifo[7]  = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.real_Vel_y__         );
    nuc ->TX_Navigation_INFO.TxFifo[8]  = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.real_Vel_y__    >> 8 );
    nuc ->TX_Navigation_INFO.TxFifo[9]  = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.real_Vel_y__    >> 16);
    nuc ->TX_Navigation_INFO.TxFifo[10] = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.real_Vel_y__    >> 24);

    //z_omega
    nuc ->TX_Navigation_INFO.TxFifo[11] = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.real_Omega_z         );
    nuc ->TX_Navigation_INFO.TxFifo[12] = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.real_Omega_z    >> 8 );
    nuc ->TX_Navigation_INFO.TxFifo[13] = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.real_Omega_z    >> 16);
    nuc ->TX_Navigation_INFO.TxFifo[14] = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.real_Omega_z    >> 24);

    //imu_yaw
    nuc ->TX_Navigation_INFO.TxFifo[15] = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.IMU_yaw              );
    nuc ->TX_Navigation_INFO.TxFifo[16] = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.IMU_yaw         >> 8 );
    nuc ->TX_Navigation_INFO.TxFifo[17] = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.IMU_yaw         >> 16);
    nuc ->TX_Navigation_INFO.TxFifo[18] = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.IMU_yaw         >> 24);

    //imu_pitch
    nuc ->TX_Navigation_INFO.TxFifo[19] = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.IMU_pitch            );
    nuc ->TX_Navigation_INFO.TxFifo[20] = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.IMU_pitch       >> 8 );
    nuc ->TX_Navigation_INFO.TxFifo[21] = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.IMU_pitch       >> 16);
    nuc ->TX_Navigation_INFO.TxFifo[22] = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.IMU_pitch       >> 24);

    //omega_yaw
    nuc ->TX_Navigation_INFO.TxFifo[23] = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.Omrga_yaw            );
    nuc ->TX_Navigation_INFO.TxFifo[24] = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.Omrga_yaw       >> 8 );
    nuc ->TX_Navigation_INFO.TxFifo[25] = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.Omrga_yaw       >> 16);
    nuc ->TX_Navigation_INFO.TxFifo[26] = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.Omrga_yaw       >> 24);

    //omega_pitch
    nuc ->TX_Navigation_INFO.TxFifo[27] = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.Omrga_pitch          );
    nuc ->TX_Navigation_INFO.TxFifo[28] = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.Omrga_pitch     >> 8 );
    nuc ->TX_Navigation_INFO.TxFifo[29] = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.Omrga_pitch     >> 16);
    nuc ->TX_Navigation_INFO.TxFifo[30] = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.Omrga_pitch     >> 24);

    //ODO
    nuc ->TX_Navigation_INFO.TxFifo[31] = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.ODO                  );
    nuc ->TX_Navigation_INFO.TxFifo[32] = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.ODO             >> 8 );
    nuc ->TX_Navigation_INFO.TxFifo[33] = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.ODO             >> 16);
    nuc ->TX_Navigation_INFO.TxFifo[34] = (uint8_t)( *(uint32_t*)&nuc ->TX_Navigation_INFO.ODO             >> 24);

    //sentry_status
    nuc ->TX_Navigation_INFO.TxFifo[35] =                         nuc ->TX_Navigation_INFO.sentry_status         ;

    /* Calc CRC16/X25 */
    /* CRC Frame */
    uint32_t CRC_state = Append_CRC16(nuc ->TX_Navigation_INFO.TxFifo, _TXFIFO2NAVIGATION_SIZE_t, crc_info);
    if(CRC_state == __CRC16_False){
        return _MINIPC_ERROR;
    }

    _CRC_Calc_ = Get_CRC16(nuc ->TX_Navigation_INFO.TxFifo, _TXFIFO2NAVIGATION_SIZE_t - 2, crc_info);
    if(crc_info ->CRC16_type != CRC16_CCITT_F)
    {
        _CRC_check_ = nuc ->TX_Navigation_INFO.TxFifo[_TXFIFO2NAVIGATION_SIZE_t - 1] << 8 | nuc ->TX_Navigation_INFO.TxFifo[_TXFIFO2NAVIGATION_SIZE_t - 2];
    }
    else{
        _CRC_check_ = nuc ->TX_Navigation_INFO.TxFifo[_TXFIFO2NAVIGATION_SIZE_t - 2] << 8 | nuc ->TX_Navigation_INFO.TxFifo[_TXFIFO2NAVIGATION_SIZE_t - 1];
    }

    if(_CRC_Calc_ != _CRC_check_)
    {
        return _MINIPC_ERROR;
    }

    if(HAL_UART_Transmit_DMA(huart, nuc ->TX_Navigation_INFO.TxFifo, 38) != HAL_OK)
    {
        return _MINIPC_ERROR;
    }else{
        return _MINIPC_OK;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////Version 2.0 函数 //////////////////////////////////////////////////////////////////

/**
 * @brief Get the crc16 from nuc fifo 2 0 object
 * 
 * @param nuc 
 * @param data_Sec2L 
 * @param data_Last 
 * @param crc_info 
 * @return _MINIPC_FCN 
 */
_MINIPC_FCN GET_CRC16_from_NUC_Fifo_2_0_(_NUC_UART_INFO_2_0_t *nuc, uint8_t data_Sec2L, uint8_t data_Last,  CRC16_INFO_t *crc_info)
{
    if(crc_info ->CRC16_type != CRC16_CCITT_F)
    {
        nuc ->RX_INFO_2_0.crc16_check = (data_Last  << 8) | data_Sec2L;
        return _MINIPC_OK;
    }if(crc_info ->CRC16_type == CRC16_CCITT_F)
    {
        nuc ->RX_INFO_2_0.crc16_check = (data_Sec2L << 8) | data_Last;
        return _MINIPC_OK;
    }else{
        return _MINIPC_ERROR;
    }
}

/**
 * @brief Set the data2minipc 2 0 object
 * 
 * @param nuc 
 * @param fdb_vel_x 
 * @param fdb_vel_y 
 * @param fdb_omega 
 * @param chassis_yaw 
 * @param chassis_pitch 
 * @param chassis_omega_yaw 
 * @param chassis_omega_pitch 
 * @param odo 
 * @param sentry_status 
 * @param gimbal_req_mode      //新增反馈视觉模式
 * @param gimbal_eular_angle 
 * @return _MINIPC_FCN 
 */
_MINIPC_FCN SET_DATA2MINIPC_2_0_(_NUC_UART_INFO_2_0_t *nuc,  float   fdb_vel_x, 
                                                             float   fdb_vel_y, 
                                                             float   fdb_omega, 
                                                             float   chassis_yaw, 
                                                             float   chassis_pitch, 
                                                             float   chassis_omega_yaw, 
                                                             float   chassis_omega_pitch, 
                                                             float   odo, 
                                                             uint8_t sentry_status, 
                                                             uint8_t gimbal_req_mode,
                                                             float  *gimbal_eular_angle
                                                            ) 
{
    nuc ->TX_INFO_2_0.real_Vel_x__           = fdb_vel_x;
    nuc ->TX_INFO_2_0.real_Vel_y__           = fdb_vel_y;

    nuc ->TX_INFO_2_0.real_Omega_z           = fdb_omega;

    nuc ->TX_INFO_2_0.Chassis_Eular_Angle[0] = chassis_yaw;
    nuc ->TX_INFO_2_0.Chassis_Eular_Angle[1] = chassis_pitch;

    nuc ->TX_INFO_2_0.Omrga_Chassis[0]       = chassis_omega_yaw;
    nuc ->TX_INFO_2_0.Omrga_Chassis[1]       = chassis_omega_pitch;

    nuc ->TX_INFO_2_0.ODO                    = odo;

    nuc ->TX_INFO_2_0.sentry_status          = sentry_status;

    nuc ->TX_INFO_2_0.gimbal_req_mode        = gimbal_req_mode;

    nuc ->TX_INFO_2_0.Gimbal_Eular_Angle[0]  = gimbal_eular_angle[0];
    nuc ->TX_INFO_2_0.Gimbal_Eular_Angle[1]  = gimbal_eular_angle[1];
    nuc ->TX_INFO_2_0.Gimbal_Eular_Angle[2]  = gimbal_eular_angle[2];

    return _MINIPC_OK;
}

/**
 * @brief 
 * 
 * @param nuc 
 * @param crc_info 
 * @param huart 
 * @param mode 
 * @return _MINIPC_FCN 
 */
_MINIPC_FCN NUC_SendMsg2MINIPC_2_0_(_NUC_UART_INFO_2_0_t *nuc, CRC16_INFO_t *crc_info, UART_HandleTypeDef *huart, uint8_t mode) {
    CRC16_t _CRC_check_ = 0;
    CRC16_t _CRC_Calc_  = 0;

    nuc ->TX_INFO_2_0.frame_Header[0] = __Msg2MINIPC_Frame_Header_1;
    nuc ->TX_INFO_2_0.frame_Header[1] = __Msg2MINIPC_Frame_Header_2;

    nuc ->TX_INFO_2_0.frame_id        = mode;

    /* Frame Header */
    nuc ->TX_INFO_2_0.TxFifo[0] = nuc ->TX_INFO_2_0.frame_Header[0];
    nuc ->TX_INFO_2_0.TxFifo[1] = nuc ->TX_INFO_2_0.frame_Header[1];

    /* Frame ID */
    nuc ->TX_INFO_2_0.TxFifo[2] = nuc ->TX_INFO_2_0.frame_id;

    /* Data Frame */
    /**
     * @attention : Little-endian transmission!!!
    */
    //vel_x_fdb
    nuc ->TX_INFO_2_0.TxFifo[3]  = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.real_Vel_x__      );
    nuc ->TX_INFO_2_0.TxFifo[4]  = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.real_Vel_x__ >> 8 );
    nuc ->TX_INFO_2_0.TxFifo[5]  = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.real_Vel_x__ >> 16);
    nuc ->TX_INFO_2_0.TxFifo[6]  = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.real_Vel_x__ >> 24);

    //vel_y_fdb
    nuc ->TX_INFO_2_0.TxFifo[7]  = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.real_Vel_y__      );
    nuc ->TX_INFO_2_0.TxFifo[8]  = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.real_Vel_y__ >> 8 );
    nuc ->TX_INFO_2_0.TxFifo[9]  = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.real_Vel_y__ >> 16);
    nuc ->TX_INFO_2_0.TxFifo[10] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.real_Vel_y__ >> 24);

    //omega_fdb
    nuc ->TX_INFO_2_0.TxFifo[11] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.real_Omega_z      );
    nuc ->TX_INFO_2_0.TxFifo[12] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.real_Omega_z >> 8 );
    nuc ->TX_INFO_2_0.TxFifo[13] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.real_Omega_z >> 16);
    nuc ->TX_INFO_2_0.TxFifo[14] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.real_Omega_z >> 24);

    //chassis_yaw
    nuc ->TX_INFO_2_0.TxFifo[15] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Chassis_Eular_Angle[0]      );
    nuc ->TX_INFO_2_0.TxFifo[16] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Chassis_Eular_Angle[0] >> 8 );
    nuc ->TX_INFO_2_0.TxFifo[17] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Chassis_Eular_Angle[0] >> 16);
    nuc ->TX_INFO_2_0.TxFifo[18] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Chassis_Eular_Angle[0] >> 24);

    //chassis_pitch
    nuc ->TX_INFO_2_0.TxFifo[19] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Chassis_Eular_Angle[1]      );
    nuc ->TX_INFO_2_0.TxFifo[20] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Chassis_Eular_Angle[1] >> 8 );
    nuc ->TX_INFO_2_0.TxFifo[21] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Chassis_Eular_Angle[1] >> 16);
    nuc ->TX_INFO_2_0.TxFifo[22] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Chassis_Eular_Angle[1] >> 24);

    //chassis_omega_yaw
    nuc ->TX_INFO_2_0.TxFifo[23] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Omrga_Chassis[0]           );
    nuc ->TX_INFO_2_0.TxFifo[24] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Omrga_Chassis[0]      >> 8 );
    nuc ->TX_INFO_2_0.TxFifo[25] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Omrga_Chassis[0]      >> 16);
    nuc ->TX_INFO_2_0.TxFifo[26] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Omrga_Chassis[0]      >> 24);

    //chassis_omega_pitch
    nuc ->TX_INFO_2_0.TxFifo[27] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Omrga_Chassis[1]           );
    nuc ->TX_INFO_2_0.TxFifo[28] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Omrga_Chassis[1]      >> 8 );
    nuc ->TX_INFO_2_0.TxFifo[29] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Omrga_Chassis[1]      >> 16);
    nuc ->TX_INFO_2_0.TxFifo[30] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Omrga_Chassis[1]      >> 24);

    //ODO
    nuc ->TX_INFO_2_0.TxFifo[31] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.ODO                        );
    nuc ->TX_INFO_2_0.TxFifo[32] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.ODO                   >> 8 );
    nuc ->TX_INFO_2_0.TxFifo[33] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.ODO                   >> 16);
    nuc ->TX_INFO_2_0.TxFifo[34] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.ODO                   >> 24);

    //sentry_status
    nuc ->TX_INFO_2_0.TxFifo[35] =                         nuc ->TX_INFO_2_0.sentry_status               ; 

    nuc ->TX_INFO_2_0.TxFifo[36] =                         nuc ->TX_INFO_2_0.gimbal_req_mode             ;

    //gimbal_yaw
    nuc ->TX_INFO_2_0.TxFifo[37] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Gimbal_Eular_Angle[0]      );
    nuc ->TX_INFO_2_0.TxFifo[38] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Gimbal_Eular_Angle[0] >> 8 );
    nuc ->TX_INFO_2_0.TxFifo[39] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Gimbal_Eular_Angle[0] >> 16);
    nuc ->TX_INFO_2_0.TxFifo[40] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Gimbal_Eular_Angle[0] >> 24);

    //gimbal_pitch
    nuc ->TX_INFO_2_0.TxFifo[41] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Gimbal_Eular_Angle[1]      );
    nuc ->TX_INFO_2_0.TxFifo[42] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Gimbal_Eular_Angle[1] >> 8 );
    nuc ->TX_INFO_2_0.TxFifo[43] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Gimbal_Eular_Angle[1] >> 16);
    nuc ->TX_INFO_2_0.TxFifo[44] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Gimbal_Eular_Angle[1] >> 24);

    //gimbal_roll
    nuc ->TX_INFO_2_0.TxFifo[45] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Gimbal_Eular_Angle[2]   );
    nuc ->TX_INFO_2_0.TxFifo[46] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Gimbal_Eular_Angle[2] >> 8);
    nuc ->TX_INFO_2_0.TxFifo[47] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Gimbal_Eular_Angle[2] >> 16);
    nuc ->TX_INFO_2_0.TxFifo[48] = (uint8_t)( *(uint32_t*)&nuc ->TX_INFO_2_0.Gimbal_Eular_Angle[2] >> 24);

    /* Calc CRC16/X25 */
    /* CRC Frame */
    uint32_t CRC_state = Append_CRC16(nuc ->TX_INFO_2_0.TxFifo, _TXFIFO2MINIPC_SIZE_t, crc_info);
    if(CRC_state == __CRC16_False){
        return _MINIPC_ERROR;
    }

    _CRC_Calc_ = Get_CRC16(nuc ->TX_INFO_2_0.TxFifo, _TXFIFO2MINIPC_SIZE_t - 2, crc_info);
    if(crc_info ->CRC16_type != CRC16_CCITT_F)
    {
        _CRC_check_ = nuc ->TX_INFO_2_0.TxFifo[_TXFIFO2MINIPC_SIZE_t - 1] << 8 | nuc ->TX_INFO_2_0.TxFifo[_TXFIFO2MINIPC_SIZE_t - 2];
    }
    else{
        _CRC_check_ = nuc ->TX_INFO_2_0.TxFifo[_TXFIFO2MINIPC_SIZE_t - 2] << 8 | nuc ->TX_INFO_2_0.TxFifo[_TXFIFO2MINIPC_SIZE_t - 1];
    }

    if(_CRC_Calc_ != _CRC_check_)
    {
        return _MINIPC_ERROR;
    }

    if(HAL_UART_Transmit_DMA(huart, nuc ->TX_INFO_2_0.TxFifo, _TXFIFO2MINIPC_SIZE_t) != HAL_OK)
    {
        return _MINIPC_ERROR;
    }else{
        return _MINIPC_OK;
    }
}

_MINIPC_FCN NUC_UnpackMsgfromMiniPC(_NUC_UART_INFO_2_0_t *nuc, CRC16_INFO_t *crc_info, uint8_t len) {
    if(Verify_CRC16(nuc ->RX_INFO_2_0.useful_info, len, crc_info) != __CRC16_True)    
    {
        memset(nuc ->rxbuffer, 0, _RxFifofromNUC_SIZE_t);
        return _MINIPC_ERROR;
    }else{
        nuc ->RX_INFO_2_0.gimbal_mode               =              nuc ->RX_INFO_2_0.useful_info[2];

        nuc ->RX_INFO_2_0.Gimbal_Eular_Angle_enc[0] =   (uint32_t)(nuc ->RX_INFO_2_0.useful_info[3]        ) | 
                                                        (uint32_t)(nuc ->RX_INFO_2_0.useful_info[4]   <<  8) | 
                                                        (uint32_t)(nuc ->RX_INFO_2_0.useful_info[5]   << 16) |
                                                        (uint32_t)(nuc ->RX_INFO_2_0.useful_info[6]   << 24);

        nuc ->RX_INFO_2_0.Gimbal_Eular_Angle_enc[1] =   (uint32_t)(nuc ->RX_INFO_2_0.useful_info[7]        ) |
                                                        (uint32_t)(nuc ->RX_INFO_2_0.useful_info[8]   <<  8) |
                                                        (uint32_t)(nuc ->RX_INFO_2_0.useful_info[9]   << 16) |
                                                        (uint32_t)(nuc ->RX_INFO_2_0.useful_info[10]  << 24);

        nuc ->RX_INFO_2_0.Chassis_Vel_enc[0]        =   (uint32_t)(nuc ->RX_INFO_2_0.useful_info[11]       ) |
                                                        (uint32_t)(nuc ->RX_INFO_2_0.useful_info[12]  <<  8) |
                                                        (uint32_t)(nuc ->RX_INFO_2_0.useful_info[13]  << 16) |
                                                        (uint32_t)(nuc ->RX_INFO_2_0.useful_info[14]  << 24);

        nuc ->RX_INFO_2_0.Chassis_Vel_enc[1]        =   (uint32_t)(nuc ->RX_INFO_2_0.useful_info[15]       ) |
                                                        (uint32_t)(nuc ->RX_INFO_2_0.useful_info[16]  <<  8) |
                                                        (uint32_t)(nuc ->RX_INFO_2_0.useful_info[17]  << 16) |
                                                        (uint32_t)(nuc ->RX_INFO_2_0.useful_info[18]  << 24);
                                                
        nuc ->RX_INFO_2_0.Chassis_Omega_enc         =   (uint32_t)(nuc ->RX_INFO_2_0.useful_info[19]       ) |
                                                        (uint32_t)(nuc ->RX_INFO_2_0.useful_info[20]  <<  8) |
                                                        (uint32_t)(nuc ->RX_INFO_2_0.useful_info[21]  << 16) |
                                                        (uint32_t)(nuc ->RX_INFO_2_0.useful_info[22]  << 24);

        memset(nuc ->rxbuffer, 0, _RxFifofromNUC_SIZE_t);

        nuc ->RX_INFO_2_0.Gimbal_Eular_Angle[0]     =   uint_to_float(nuc ->RX_INFO_2_0.Gimbal_Eular_Angle_enc[0], -PI, PI, 32);
        nuc ->RX_INFO_2_0.Gimbal_Eular_Angle[1]     =   uint_to_float(nuc ->RX_INFO_2_0.Gimbal_Eular_Angle_enc[1], -PI, PI, 32);
        nuc ->RX_INFO_2_0.Chassis_Vel[0]            =   uint_to_float(nuc ->RX_INFO_2_0.Chassis_Vel_enc[0]       ,  -1,  1, 32);
        nuc ->RX_INFO_2_0.Chassis_Vel[1]            =   uint_to_float(nuc ->RX_INFO_2_0.Chassis_Vel_enc[1]       ,  -1,  1, 32);
        nuc ->RX_INFO_2_0.Chassis_Omega             =   uint_to_float(nuc ->RX_INFO_2_0.Chassis_Omega_enc        ,  -1,  1, 32);

        nuc ->RX_INFO_2_0.Gimbal_Eular_Angle_Degree[0] =   nuc ->RX_INFO_2_0.Gimbal_Eular_Angle[0] * 180 / PI;
        nuc ->RX_INFO_2_0.Gimbal_Eular_Angle_Degree[1] =   nuc ->RX_INFO_2_0.Gimbal_Eular_Angle[1] * 180 / PI;

        return _MINIPC_OK;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
