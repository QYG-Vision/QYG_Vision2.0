#ifndef __MINIPC_H_
#define __MINIPC_H_

#ifdef __cplusplus
extern "C" {
#endif
	
#include "stm32f4xx.h"
#include "math.h"
#include "arm_math.h"
#include "usart.h"
#include "dma.h"
#include "CRC16.h"
#include "IEEE754.h"

#define _IDLE_MODE_TX_              0x00          //1.0版本属性参数
#define _VISUAL_MODE_TX_            0x01          //1.0版本属性参数
#define _NAVIGATION_MODE_TX_        0x02          //1.0版本属性参数

#define _IDLE_MODE_RX_              0xFA          //1.0版本属性参数
#define _VISUAL_MODE_RX_            0xFB          //1.0版本属性参数
#define _NAVIGATION_MODE_RX_        0xFC          //1.0版本属性参数

#define _MINIPC_FCN                 uint8_t

#define _MINIPC_OK                  0x01
#define _MINIPC_ERROR               0x00

#define _MINIPC_FIFO_t              uint8_t 

#define _RxFifofromNUC_SIZE_t       256

//////////////////////////////////////////////////////////////Version 1.0 宏定义///////////////////////////////////////////////////////////

/* VISUAL PARAM */
#define _TXFIFO2VISUAL_SIZE_t       33       //随便设的，会面会改，hahaha…… !(^o^)! ……好吧，确定是33个元素
#define _RXFIFOofVisual_SIZE_t      128
#define _FIFO_USEFULofvisual_SIZE_t 13

/* NAVIGATION PARAM */
#define _TXFIFO2NAVIGATION_SIZE_t   38
#define _RXFIFOofNavigation_SIZE_t  128
#define _FIFO_USEFULofNAVI_SIZE_t   25

/* frame header */
#define _Visual_Frame_Header_1      0x51     //"Q" 庆
#define _Visual_Frame_Header_2      0x59     //"Y" 园

#define _Navigation_Frame_Header_1  0x47     //"G" 广
#define _Navigation_Frame_Header_2  0x44     //"D" 大

/* EC to visual mode selector */
#define _None_Mode                  0x10     //无模式
#define _Auto_Aim_Mode              0x11     //自动瞄准模式（目标机器人、前哨战、基地）
#define _L_E_D                      0x12     //小符自动瞄准
#define _H_E_D                      0x13     //大符自动瞄准

/* Visual to EC mode selector */
#define _DISABLE_CTRL               0x10     //禁用控制
#define _NO_FIRE_MODE               0x11     //不射击模式
#define _FIRE_MODE                  0x12     //自动射击模式

/* EC to navigation mode selector */
#define _None_Mode_Navi_FDB         0x00     //无模式(回传)
#define _Navigation_Mode_FDB        0x01     //导航模式(回传)
#define _Visaul_First_Mode_FDB      0x02     //视觉优先模式(回传)
#define _OASS_Mode_FDB              0x03     //避障模式(回传)

/* navigation to EC mode selector */
#define _None_Mode_Navi             0x00     //无模式
#define _Navigation_Mode            0x01     //导航模式
#define _Visaul_First_Mode          0x02     //视觉优先模式
#define _OASS_Mode                  0x03     //避障模式

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////Version 2.0 宏定义////////////////////////////////////////////////////////////////

/* MINIPC PARAM */
#define _TXFIFO2MINIPC_SIZE_t       51
#define _RXFIFOofMINIPC_SIZE_t      128
#define _FIFO_USEFULofMINIPC_SIZE_t 25

/* frame header */
#define __Msg2MINIPC_Frame_Header_1 0x47     //"G" 广
#define __Msg2MINIPC_Frame_Header_2 0x44     //"D" 大

#define __Msg2EC_Frame_Header_1     0x51     //"Q" 庆
#define __Msg2EC_Frame_Header_2     0x59     //"Y" 园

/* EC to miniPC mode selector */
//To be perfected...

/* miniPC to EC mode selector */
#define _DISABLE_CTRL_MINIPC       0x00     //禁用控制
#define _NO_FIRE_MODE_MINIPC       0x01     //不射击模式
#define _FIRE_MODE_MINIPC          0x02     //自动射击模式

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////Version 1.0 枚举与结构体///////////////////////////////////////////////////////////

typedef struct
{
    _MINIPC_FIFO_t TxFifo[_TXFIFO2VISUAL_SIZE_t];
    _MINIPC_FIFO_t frame_Header[2];
    _MINIPC_FIFO_t frame_id;
    _MINIPC_FIFO_t frame_CRC16[2];

    /* data frame */
    float quaternion[4];
    float yaw;
    float pitch;
    float bullet_speed;
}_NUC_TX_Visual_INFO_t;

typedef struct
{
    _MINIPC_FIFO_t TxFifo[_TXFIFO2NAVIGATION_SIZE_t];
    _MINIPC_FIFO_t frame_Header[2];
    _MINIPC_FIFO_t frame_id;
    _MINIPC_FIFO_t frame_CRC16[2];

    /* data frame */
    float real_Vel_x__;
    float real_Vel_y__;
    float real_Omega_z;

    float IMU_yaw;
    float IMU_pitch;

    float Omrga_yaw;
    float Omrga_pitch;

    float ODO;

    uint8_t sentry_status;
}_NUC_TX_Navigation_INFO_t;

typedef struct
{
    _MINIPC_FIFO_t RxFifo[_RXFIFOofVisual_SIZE_t];
    _MINIPC_FIFO_t useful_info[_FIFO_USEFULofvisual_SIZE_t];

    /* data frame */
    uint8_t        gimbal_mode;

    uint32_t       dyaw_enc;
    uint32_t       dpitch_enc;

    uint32_t       yaw_enc;
    uint32_t       pitch_enc;

    float             dYaw;
    float             dPitch;

    ieee754_32_info_t dyaw;    
    ieee754_32_info_t dpitch;

    float             yaw_f;
    float             pitch_f;
    
    ieee754_32_info_t yaw;
    ieee754_32_info_t pitch;

    /* CRC frame */
    uint16_t       crc16_value;

}_NUC_RX_Visual_INFO_t;

typedef struct
{
    _MINIPC_FIFO_t RxFifo[_RXFIFOofNavigation_SIZE_t];
    _MINIPC_FIFO_t useful_info[_FIFO_USEFULofNAVI_SIZE_t];

    /* data frame */
    uint8_t        running_mode;

    uint32_t       ctrl_vel_x_enc;
    uint32_t       ctrl_vel_y_enc;
    uint32_t       ctrl_omega_enc;

    uint32_t       max_vel_enc; 
    uint32_t       max_omega_enc;

    ieee754_32_info_t ctrl_vel_x;
    ieee754_32_info_t ctrl_vel_y;
    ieee754_32_info_t ctrl_omega;

    ieee754_32_info_t max_vel;
    ieee754_32_info_t max_omega;

    /* CRC frame */
    uint16_t       crc16_value;

}_NUC_RX_Navigation_INFO_t;

typedef struct 
{
   _NUC_TX_Visual_INFO_t      TX_Visual_INFO;
   _NUC_TX_Navigation_INFO_t  TX_Navigation_INFO;
   _NUC_RX_Visual_INFO_t      RX_Visual_INFO;
   _NUC_RX_Navigation_INFO_t  RX_Navigation_INFO;

   uint8_t                    rxbuffer[_RxFifofromNUC_SIZE_t];
   
   uint8_t                    test[256];
}_NUC_UART_INFO_t;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////Version 2.0 枚举与结构体///////////////////////////////////////////////////////////
/**
 * 此版本将视觉与导航的数据结合
 * 2025 . 03 . 09
*/
typedef struct
{
    _MINIPC_FIFO_t TxFifo[_TXFIFO2MINIPC_SIZE_t];
    _MINIPC_FIFO_t frame_Header[2];
    _MINIPC_FIFO_t frame_id;
    _MINIPC_FIFO_t frame_CRC16[2];

    /* data frame */
    float real_Vel_x__;
    float real_Vel_y__;
    float real_Omega_z;

    float Chassis_Eular_Angle[2]; //[0]yaw [1]pitch

    float Omrga_Chassis[2];       //[0]yaw [1]pitch

    float ODO;

    uint8_t sentry_status;

    uint8_t gimbal_req_mode;

    float Gimbal_Eular_Angle[3];  //[0]yaw [1]pitch [2]roll
}_NUC_TX_INFO_2_0_t;

typedef struct
{
    _MINIPC_FIFO_t RxFifo[_RXFIFOofMINIPC_SIZE_t];
    _MINIPC_FIFO_t useful_info[_FIFO_USEFULofMINIPC_SIZE_t];

    /* data frame */
    uint8_t        gimbal_mode;

    uint32_t       Gimbal_Eular_Angle_enc[2];    //[0]yaw [1]pitch

    uint32_t       Chassis_Vel_enc[2];           //[0]vel_x [1]vel_y

    uint32_t       Chassis_Omega_enc;            //[0]omega_z

    float          Gimbal_Eular_Angle[2];        //[0]yaw [1]pitch

    float          Gimbal_Eular_Angle_Degree[2]; //[0]yaw [1]pitch

    float          Chassis_Vel[2];               //[0]vel_x [1]vel_y

    float          Chassis_Omega;                //[0]omega_z

    /* CRC16 frame */
    CRC16_t crc16_check;
}_NUC_RX_INFO_2_0_t;

typedef struct 
{
   _NUC_TX_INFO_2_0_t      TX_INFO_2_0;
   _NUC_RX_INFO_2_0_t      RX_INFO_2_0;

    _MINIPC_FIFO_t rxbuffer[_RxFifofromNUC_SIZE_t];
}_NUC_UART_INFO_2_0_t;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////Version 1.0 函数声明////////////////////////////////////////////////////////////////

extern _MINIPC_FCN GET_CRC16_from_NUC_Fifo(_NUC_UART_INFO_t *nuc, uint8_t data_Sec2L, uint8_t data_Last, uint8_t mode, CRC16_INFO_t *crc_info);

extern _MINIPC_FCN SET_DATA2TX2Visual(_NUC_UART_INFO_t *nuc, float *quaternion, float yaw, float pitch, float shoot_vel);
extern _MINIPC_FCN SET_DATA2NAVI(_NUC_UART_INFO_t *nuc, float real_vel_x, float real_vel_y, float real_omega, float imu_yaw, float imu_pitch, float omega_yaw, float omega_pitch, float odo, uint8_t sentry_status);

extern _MINIPC_FCN NUC_UnpackMsgfromVisual(_NUC_UART_INFO_t *nuc, CRC16_INFO_t *crc_info, uint8_t len);
extern _MINIPC_FCN NUC_UnpackMsgfromVisual_DM(_NUC_UART_INFO_t *nuc, CRC16_INFO_t *crc_info, uint8_t len);
extern _MINIPC_FCN NUC_UnpackMsgfromNavigation(_NUC_UART_INFO_t *nuc, CRC16_INFO_t *crc_info, uint8_t len);

extern _MINIPC_FCN NUC_SendMsg2Visual(_NUC_UART_INFO_t *nuc, UART_HandleTypeDef *huart, uint8_t mode, CRC16_INFO_t *crc_info);
extern _MINIPC_FCN NUC_SendMsg2Navigation(_NUC_UART_INFO_t *nuc, UART_HandleTypeDef *huart, uint8_t mode, CRC16_INFO_t *crc_info);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////Version 2.0 函数声明////////////////////////////////////////////////////////////////

/**
 * @brief Get the crc16 from nuc fifo 2 0 object
 * 
 * @param nuc 
 * @param data_Sec2L 
 * @param data_Last 
 * @param crc_info 
 * @return _MINIPC_FCN 
 */
extern _MINIPC_FCN GET_CRC16_from_NUC_Fifo_2_0_(_NUC_UART_INFO_2_0_t *nuc, uint8_t data_Sec2L, uint8_t data_Last,  CRC16_INFO_t *crc_info);

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
extern _MINIPC_FCN SET_DATA2MINIPC_2_0_(_NUC_UART_INFO_2_0_t *nuc,  float fdb_vel_x, float fdb_vel_y, float fdb_omega, float chassis_yaw, float chassis_pitch, float chassis_omega_yaw, float chassis_omega_pitch, float odo, uint8_t sentry_status, uint8_t gimbal_req_mode, float *gimbal_eular_angle);

/**
 * @brief Sent the msg to minipc 2 0 object
 * 
 * @param nuc 
 * @param crc_info 
 * @param huart 
 * @param mode 
 * @return _MINIPC_FCN 
 */
extern _MINIPC_FCN NUC_SendMsg2MINIPC_2_0_(_NUC_UART_INFO_2_0_t *nuc, CRC16_INFO_t *crc_info, UART_HandleTypeDef *huart, uint8_t mode);

/**
 * @brief Unpack the msg from minipc 2 0 object
 * 
 * @param nuc 
 * @param crc_info 
 * @param len 
 * @return _MINIPC_FCN 
 */
extern _MINIPC_FCN NUC_UnpackMsgfromMiniPC(_NUC_UART_INFO_2_0_t *nuc, CRC16_INFO_t *crc_info, uint8_t len);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#ifdef __cplusplus
}
#endif

#endif
