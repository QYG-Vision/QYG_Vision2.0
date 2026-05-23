#include "Serial_node.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "SerialPlot.h"
#include "IEEE754.h"

#include "bsp_can.h"
#include "Flysky_bsp.h"
#include "bsp_wit.h"
#include "MiniPC.h"
#include "auto_mode.h"

#include "ctrl_node.h"
#include "master_node.h"

#include "NLADRC_ver1.h"

extern Motor_info_t  Pitch_Motor;
extern Motor_info_t  Fire_Motor[2];
extern Motor_info_t  Yaw_Motor;
extern Motor_info_t  Pluck_Motor;
extern int16_t       current_test;
extern int32_t       test_tick;
extern nladrc_t      GM6020_nladrc;
extern sentry_remote sentry_Remote;
extern float         yaw_angle_set;

extern JiuXian_Logic_info_t       Soul_of_JiuXian;
extern XuanHuoTypedef             XuanHuo;

extern CRC16_INFO_t               crc16_X25;


extern _sentry_normal_auto_data_t Auto_Mode;
extern _sentry_remote_data_t      RemoteCtrl;
extern _sentry_data_t             sentry_RM2026;
extern wit_imu_t                  wit;
extern uint8_t                    debug_node_tick;

extern uint8_t                    record_master_tick;
extern uint8_t                    record_debug_tick;
extern uint8_t                    record_wit_tick;   
extern uint8_t                    record_remote_tick; 

extern float                      Vel_x_fdb;
extern float                      Vel_y_fdb;
extern float                      Omega_fdb;

extern float                      HWT_wit_yaw;
extern int16_t                    HWT_wit_enc;

extern float                      Eular_angle[4];

extern _NUC_UART_INFO_t           NUC;
extern _NUC_UART_INFO_2_0_t       NUC_ver_2_0;

ieee754_32_info_t                 check_float_ieee;

void Serial_node(void *argument)
{
  /* USER CODE BEGIN Serial_node */
  int num = 100;
  check_float_ieee = unpack_ieee754_32(0x40400000);
  /* Infinite loop */
  for(;;)
  {
		switch(num / 100)
		{
			case 1: HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,0);                                                                             break;
			case 2: HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,0);                                                                             break;
			case 3: HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,0);                                                                             break;
			case 4: HAL_GPIO_WritePin(GPIOG,GPIO_PIN_4,0);                                                                             break;
			case 5: HAL_GPIO_WritePin(GPIOG,GPIO_PIN_5,0);                                                                             break;
			case 6: HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,0);                                                                             break;
			case 7: HAL_GPIO_WritePin(GPIOG,GPIO_PIN_7,0);                                                                             break;
			case 8: HAL_GPIO_WritePin(GPIOG,GPIO_PIN_8,0);                                                                             break;
			default:HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8,1);break;
		}
		num += 5;
		if(num > 900)
		{
			num = 0;			
		}
		// sentMultiData2SerialPlot(           									NULL,
		// 																		NULL,
		// 																		NULL,
		// 																		NULL,
		// 																		wit.euler_angle[1],
		// 																		wit.euler_angle[2],
		// 																		wit.quaternion[0],
		// 																		wit.quaternion[1],
		// 																		wit.quaternion[2],
		// 																		wit.quaternion[3]
		// );
		VofaTenDataChannel(           									        NUC_ver_2_0.RX_INFO_2_0.gimbal_mode,
																				NUC_ver_2_0.RX_INFO_2_0.Gimbal_Eular_Angle_Degree[0],
																				NUC_ver_2_0.RX_INFO_2_0.Gimbal_Eular_Angle_Degree[1],
																				NUC_ver_2_0.RX_INFO_2_0.Chassis_Vel[0],
																				NUC_ver_2_0.RX_INFO_2_0.Chassis_Vel[1],
																				NUC_ver_2_0.RX_INFO_2_0.Chassis_Omega,
																				XuanHuo.XuanHuo_Now,
																				Eular_angle[0],
																				Eular_angle[1],
																				Yaw_Motor.real_info.angle
		);
		debug_node_tick++;
    	osDelay(0);
  }
  /* USER CODE END Serial_node */
}
