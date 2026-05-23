#include "master_node.h"
#include "dma.h"
#include "usart.h"
#include "Flysky_bsp.h"
#include "bsp_wit.h"
#include "MiniPC.h"
#include "tim.h"

#include "bsp_can.h"

#include "CRC16.h"

#include "SerialPlot.h"

#define USART3_TEST                       0
#define USART3_NORMAL                     1
#define USART3_NORMAL_ver_2_0             1

#define MINIPC_COMMUNICATION_VERSION_1_0  0
#define MINIPC_COMMUNICATION_VERSION_2_0  1

#define sentry_gyro_max      9.6f
#define sentry_vel_max       1.6f

#define K_Weight_min         0.3f             

extern DMA_HandleTypeDef   hdma_usart1_rx;
extern DMA_HandleTypeDef   hdma_usart2_rx;
extern DMA_HandleTypeDef   hdma_usart3_rx;

extern sentry_status_t     sentry_info;

JiuXian_Logic_info_t       Soul_of_JiuXian;
XuanHuoTypedef             XuanHuo;

sentry_remote              sentry_Remote;
_sentry_remote_data_t      RemoteCtrl;

_sentry_normal_auto_data_t Auto_Mode;
_sentry_data_t             sentry_RM2026;
_NUC_UART_INFO_t           NUC;                        //MiniPC库1.0版本数据
_NUC_UART_INFO_2_0_t       NUC_ver_2_0;                //MiniPC库2.0版本数据
wit_imu_t                  wit;

uint8_t                    master_node_tick = 0;
uint8_t                    debug_node_tick = 0;
uint8_t                    wit_tick = 0;
uint8_t                    remote_tick = 0;

uint8_t                    record_master_tick;
uint8_t                    record_debug_tick;
uint8_t                    record_wit_tick;
uint8_t                    record_remote_tick;

float                      Vel_x_fdb = 0.0f;
float                      Vel_y_fdb = 0.0f;
float                      Omega_fdb = 0.0f;

Gimbal_Rot_to_World_t      TF;

uint8_t                    yaw_init_switch = 0;

extern int16_t             chassis_msg_fdb[3];
extern float               IMU_init_yaw_angle;

extern Motor_info_t        Pitch_Motor;
extern int16_t             HWT_wit_enc;

float                      HWT_wit_yaw;

float                      delta_yaw = 0.0f;

CRC16_INFO_t               crc16_X25;

uint16_t                   Visual_CRC_CKeck;
uint16_t                   NAVI_CRC_CKeck;

uint16_t                   Visual_CRC_CKeck_;
uint16_t                   NAVI_CRC_CKeck_;

float                      Eular_angle[4] = {0.0f};
float                      Eular_angle_2_0[3] = {0.0f};	

//
uint32_t                   Auto_Period = 0;
//

static void Gimbal_Rot_Matrix_xOy(Gimbal_Rot_to_World_t *tf, float vel_X_gimbal, float vel_Y_gimbal, float yaw);
static void Gimbal_inv_Rot_Matrix_xOy(Gimbal_Rot_to_World_t *tf, float vel_X_world, float vel_Y_world, float yaw);

void Master_node(void *argument)
{
  /* USER CODE BEGIN Master_node */
  uint32_t NUC_send_mode = 0;

  JiuXian_Thoughts_Init(&Soul_of_JiuXian, SET_TICK_MS(20));
  JiuXian_Fire_Heat_status_Init(&XuanHuo, 21, 10, XUANHUO_Calc_1KHz, 1000);

  auto_mode_init(&Auto_Mode);

  Flysky_init(&sentry_Remote, sentry_vel_max, sentry_vel_max, 0.0001f, 3.0f);

  auto_mode_init(&Auto_Mode);

  CRC16_Init(&crc16_X25, CRC16_X25);

  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

  HAL_UART_Receive_DMA(&huart1,sentry_Remote.channel.sentry_sbus_rx, BUFF_SIZE);
  HAL_UART_Receive_DMA(&huart2,wit.rx_info.Rxfifo                  , 11       );

  #if MINIPC_COMMUNICATION_VERSION_1_0

  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, NUC.rxbuffer, _RxFifofromNUC_SIZE_t);

  #endif

  #if  MINIPC_COMMUNICATION_VERSION_2_0

  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, NUC_ver_2_0.rxbuffer, _RxFifofromNUC_SIZE_t);
  
  #endif 
  /* Infinite loop */
  for(;;)
  {
	Vel_x_fdb   = (float)chassis_msg_fdb[0] * 0.001f;
	Vel_y_fdb   = (float)chassis_msg_fdb[1] * 0.001f;
	Omega_fdb   = (float)chassis_msg_fdb[2] * 0.001f;
	delta_yaw   = HWT_wit_enc * 360.0f / (8192.0f);

	Eular_angle_2_0[0] = Eular_angle[0];
	Eular_angle_2_0[1] = Eular_angle[1];
	Eular_angle_2_0[2] = Eular_angle[2];

	#if MINIPC_COMMUNICATION_VERSION_1_0

	NUC_UnpackMsgfromVisual_DM(&NUC, &crc16_X25, _FIFO_USEFULofvisual_SIZE_t);

	NUC_UnpackMsgfromNavigation(&NUC, &crc16_X25, _FIFO_USEFULofNAVI_SIZE_t);
	
	SET_DATA2TX2Visual(&NUC, Eular_angle, wit.euler_angle[2], wit.euler_angle[0], sentry_RM2026.shoot_vel);
	SET_DATA2NAVI(&NUC, Vel_x_fdb, Vel_y_fdb, Omega_fdb, wit.euler_angle[2], wit.euler_angle[0], wit.omega[2], wit.omega[0], 0, 0);

	switch(NUC_send_mode)
	{
		case 2 :
		{
			NUC_SendMsg2Visual(&NUC, &huart3, _Auto_Aim_Mode, &crc16_X25);
			break;
		}
		case 4 :
		{
			NUC_SendMsg2Navigation(&NUC, &huart3, NUC.TX_Navigation_INFO.frame_id, &crc16_X25);
			break;
		}
		default :
		{
			break;
		}
	}
	
	#endif

	#if  MINIPC_COMMUNICATION_VERSION_2_0

	NUC_UnpackMsgfromMiniPC(&NUC_ver_2_0, &crc16_X25, _FIFO_USEFULofMINIPC_SIZE_t);
	if(SET_DATA2MINIPC_2_0_(&NUC_ver_2_0, Vel_x_fdb, 
									      Vel_y_fdb, 
										  Omega_fdb, 
										  0, 
										  0, 
										  0, 
										  0, 
										  0, 
										  (sentry_info.game_status == 0x04 ? 1 : 0), 
										  NUC_ver_2_0.RX_INFO_2_0.gimbal_mode, 
										  Eular_angle_2_0
								) == _MINIPC_OK) 
	{
		NUC_SendMsg2MINIPC_2_0_(&NUC_ver_2_0, &crc16_X25, &huart3, 0x00);
	}

	#endif
	
	/*-------------------------------------------------------MiniPC Communication Zone--------------------------------------------------*/
	
	/*-------------------------------------------------------MiniPC Communication Zone--------------------------------------------------*/

	/*-----------------------------------------------------Auto Zone Begin----------------------------------------------------------*/
	/**
	 * @brief 
	 * if you choose Navigation Mode, MiniPC will sent some data to the development board, these data will be clear when you choose Remote Control Mode.
	 */
	if(sentry_Remote.ctrl.mode_switch != AUTOMATION_MODE)
	{
	    auto_mode_clear(&Auto_Mode);
	}else{
		// Gimbal_Rot_Matrix_xOy(&TF, NUC_ver_2_0.RX_INFO_2_0.Chassis_Vel[0], NUC_ver_2_0.RX_INFO_2_0.Chassis_Vel[1], (Eular_angle[0] * PI / 180.0f));
		sentry_JiuXian_chassis_running_set(&Auto_Mode,  NUC_ver_2_0.RX_INFO_2_0.Chassis_Vel[0],
											  		    NUC_ver_2_0.RX_INFO_2_0.Chassis_Vel[1],
											  			NUC_ver_2_0.RX_INFO_2_0.Chassis_Omega
													);
		JiuXian_yaw_Logic(&Soul_of_JiuXian, &Auto_Mode, 0.3f);
		JiuXian_pitch_Logic(&Soul_of_JiuXian, &Auto_Mode);	
		Auto_Mode.shoot_vel = (Soul_of_JiuXian.Mode == STRICKER ? 500.0f : 0.0f);
		Auto_Mode.pluck_vel = (Soul_of_JiuXian.Mode == STRICKER ?  12.0f : 0.0f);
	}
	/*------------------------------------------------------Auto Zone End-----------------------------------------------------------*/
	
	/*------------------------------------------------Remote Comtrol Area Begin-----------------------------------------------------*/
	/*
		@attention : Some data should send to the development board, such as the angle of yaw, omega of chassis,
		             speed of X axis, speed of Y axis and speed of pluck. 
	*/
	/*---------------------------------------------------!!!GIMBAL!!!--------------------------------------------------*/
	/*-------------------------------Fire Control Begin---------------------------------*/
	RemoteCtrl.shoot_vel = (sentry_Remote.ctrl.friction_wheel_switch && 
							XuanHuo.HeatStatus == NORMAL_HEAT                                         ) ? 
							500.0f                                         : 0.0f;
	RemoteCtrl.pluck_vel = (sentry_Remote.ctrl.friction_wheel_switch && 
							sentry_Remote.ctrl.pluck_wheel_switch    && 
							XuanHuo.HeatStatus == NORMAL_HEAT                                         ) ?  
							18.0f * sentry_Remote.ctrl.gimbal_sensitivity  : 0.0f;

	/*--------------------------------Fire Control End----------------------------------*/

	/*-------------------------------Pitch Control Begin--------------------------------*/
	RemoteCtrl.pitch_angle = 100.0f * sentry_Remote.ctrl.gimbal_factor.pitch_factor;
	/*--------------------------------Pitch Control End---------------------------------*/

	/*-------------------------------Yaw Control Begin---------------------------------*/
	RemoteCtrl.yaw_add_angle = sentry_Remote.ctrl.gimbal_factor.yaw_add;
	/*--------------------------------Yaw Control End----------------------------------*/
	/*---------------------------------------------------!!!GIMBAL!!!--------------------------------------------------*/

	/*--------------------------------------------------!!!CHASSIS!!!--------------------------------------------------*/
	/*----------You Can Take The Value Of Chassis's Omega Directly!---------------------*/
	if(sentry_Remote.ctrl.gyro_switch == REMOTE_MODE)
	{
			RemoteCtrl.chassis_omega = sentry_Remote.ctrl.chassis_omega = sentry_gyro_max * sentry_Remote.ctrl.gimbal_sensitivity;//rad per sec.
	}else{
			RemoteCtrl.chassis_omega = sentry_Remote.ctrl.chassis_omega = 0.0f;
	}

	RemoteCtrl.chassis_vel_x = sentry_Remote.ctrl.spd_X.spd;
	RemoteCtrl.chassis_vel_y = sentry_Remote.ctrl.spd_Y.spd;
	/*----------You Can Take The Value Of Chassis's Omega Directly!---------------------*/
	/*--------------------------------------------------!!!CHASSIS!!!--------------------------------------------------*/
	/*-----------------------------------------------Remote Comtrol Area End------------------------------------------------------*/

	float K_speed_weight = Speed_Weight_Allocation(sentry_vel_max, Auto_Mode.chassis_vel_x + RemoteCtrl.chassis_vel_x, Auto_Mode.chassis_vel_y + RemoteCtrl.chassis_vel_y, K_Weight_min);
	
	sentry_RM2026.chassis_omega = (Auto_Mode.chassis_omega + RemoteCtrl.chassis_omega) * (1.0f - K_speed_weight);

	sentry_RM2026.chassis_vel_x = (sentry_RM2026.chassis_omega != 0 ? (Auto_Mode.chassis_vel_x + RemoteCtrl.chassis_vel_x) * K_speed_weight : Auto_Mode.chassis_vel_x + RemoteCtrl.chassis_vel_x);
	sentry_RM2026.chassis_vel_y = (sentry_RM2026.chassis_omega != 0 ? (Auto_Mode.chassis_vel_y + RemoteCtrl.chassis_vel_y) * K_speed_weight : Auto_Mode.chassis_vel_y + RemoteCtrl.chassis_vel_y);
	
	sentry_RM2026.pluck_vel     = Auto_Mode.pluck_vel     + RemoteCtrl.pluck_vel;
	sentry_RM2026.pluck_vel     =(sentry_info.heat > 160 ? 0 : sentry_RM2026.pluck_vel);
	sentry_RM2026.shoot_vel     = Auto_Mode.shoot_vel     + RemoteCtrl.shoot_vel;


	if (Soul_of_JiuXian.Mode != STRICKER) sentry_RM2026.pitch_angle   = Auto_Mode.pitch_angle   + RemoteCtrl.pitch_angle + sentry_pitch_min;
	else sentry_RM2026.pitch_angle = GM6020_angle_Rot_inv(NUC_ver_2_0.RX_INFO_2_0.Gimbal_Eular_Angle_Degree[1], sentry_abs_zero_pitch);
	// sentry_RM2026.pitch_angle   = Auto_Mode.pitch_angle   + RemoteCtrl.pitch_angle + sentry_pitch_min;
	sentry_RM2026.pitch_angle = (sentry_RM2026.pitch_angle > sentry_pitch_max ? sentry_pitch_max : (sentry_RM2026.pitch_angle < sentry_pitch_min ? sentry_pitch_min : sentry_RM2026.pitch_angle));

	sentry_RM2026.yaw_add_angle = Auto_Mode.yaw_add_angle + RemoteCtrl.yaw_add_angle;



	HAL_UART_Receive_DMA(&huart1,sentry_Remote.channel.sentry_sbus_rx, BUFF_SIZE    );
	HAL_UART_Receive_DMA(&huart2,wit.rx_info.Rxfifo                  , 11           );

	#if MINIPC_COMMUNICATION_VERSION_1_0
	
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, NUC.rxbuffer, _RxFifofromNUC_SIZE_t       );

	(NUC_send_mode > 5 ? NUC_send_mode = 0 : NUC_send_mode++);

	#endif

	#if MINIPC_COMMUNICATION_VERSION_2_0

	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, NUC_ver_2_0.rxbuffer, _RxFifofromNUC_SIZE_t);
	
	#endif 

	master_node_tick++;

    osDelay(1);
  }
  /* USER CODE END Master_node */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
	if(sentry_Remote.channel.sentry_sbus_rx[0] == 0x0f && sentry_Remote.channel.sentry_sbus_rx[24] == 0x00)
	{
		/**
		 * @brief 
		 * In fact, this function has made it very clear that when it is not in remote control mode, the data from the remote control is all zero.
		 */
		sbus_data_unpacking(&sentry_Remote, sentry_Remote.channel.sentry_sbus_rx);

		sentry_remote_mapping(&sentry_Remote);
	}
    HAL_UART_Receive_DMA(&huart1,sentry_Remote.channel.sentry_sbus_rx, BUFF_SIZE);
  }
  if (huart->Instance == USART2)
  {
	static const float OMEGA_SCALE = 2000.0f / 32768.0f;
    static const float EULER_SCALE =  180.0f / 32768.0f;
    static const float QUAT_SCALE  =    1.0f / 32768.0f;

	wit.rx_info.Check_Sum = wit.rx_info.Rxfifo[0] +
							wit.rx_info.Rxfifo[1] +
							wit.rx_info.Rxfifo[2] +
							wit.rx_info.Rxfifo[3] +
							wit.rx_info.Rxfifo[4] +
							wit.rx_info.Rxfifo[5] +
							wit.rx_info.Rxfifo[6] +
							wit.rx_info.Rxfifo[7] +
							wit.rx_info.Rxfifo[8] +
							wit.rx_info.Rxfifo[9];

	if(wit.rx_info.Rxfifo[0] == 0x55)
	{
			if(wit.rx_info.Check_Sum == wit.rx_info.Rxfifo[10])
			{
				switch(wit.rx_info.Rxfifo[1])
				{
					case Omega       :
					{
						wit.omega[0] = (float)((int16_t)(wit.rx_info.Rxfifo[3] << 8) | (int16_t)wit.rx_info.Rxfifo[2]) * OMEGA_SCALE;//x
						wit.omega[1] = (float)((int16_t)(wit.rx_info.Rxfifo[5] << 8) | (int16_t)wit.rx_info.Rxfifo[4]) * OMEGA_SCALE;//y
						wit.omega[2] = (float)((int16_t)(wit.rx_info.Rxfifo[7] << 8) | (int16_t)wit.rx_info.Rxfifo[6]) * OMEGA_SCALE;//z
						break;
					}	
					case Euler_Angle :
					{
						wit.euler_angle[0] = (float)((int16_t)(wit.rx_info.Rxfifo[3] << 8) | (int16_t)wit.rx_info.Rxfifo[2]) * EULER_SCALE;//roll
						wit.euler_angle[1] = (float)((int16_t)(wit.rx_info.Rxfifo[5] << 8) | (int16_t)wit.rx_info.Rxfifo[4]) * EULER_SCALE;//pitch
						wit.euler_angle[2] = (float)((int16_t)(wit.rx_info.Rxfifo[7] << 8) | (int16_t)wit.rx_info.Rxfifo[6]) * EULER_SCALE;//yaw
						break;
					}
					case Quaternion  :
					{
						wit.quaternion[0] = (float)((int16_t)(wit.rx_info.Rxfifo[3] << 8) | (int16_t)wit.rx_info.Rxfifo[2]) * QUAT_SCALE;//q0
						wit.quaternion[1] = (float)((int16_t)(wit.rx_info.Rxfifo[5] << 8) | (int16_t)wit.rx_info.Rxfifo[4]) * QUAT_SCALE;//q1
						wit.quaternion[2] = (float)((int16_t)(wit.rx_info.Rxfifo[7] << 8) | (int16_t)wit.rx_info.Rxfifo[6]) * QUAT_SCALE;//q2
						wit.quaternion[3] = (float)((int16_t)(wit.rx_info.Rxfifo[9] << 8) | (int16_t)wit.rx_info.Rxfifo[8]) * QUAT_SCALE;//q3
						break;
					}
					default          :
					{
					    break;
					}
				}
			}
	}
	if(yaw_init_switch == 0)
	{
		IMU_init_yaw_angle = wit.euler_angle[2];
		yaw_init_switch = 1;
	}
	HAL_UART_Receive_DMA(&huart2,wit.rx_info.Rxfifo, 11);
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
  }
  else{
	return;
  }
}

/**
 * @brief 
 * Because I do not know the length of receive data, so I use the interrupt method to receive data.
 * @param huart 
 * @param Size 
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
	if(huart->Instance == USART3)
	{
	    #if USART3_TEST
			HAL_UART_Transmit_DMA(&huart3, NUC.rxbuffer, _RxFifofromNUC_SIZE_t);
		#endif

		#if MINIPC_COMMUNICATION_VERSION_1_0

		    if(NUC.rxbuffer[0] == _Visual_Frame_Header_1 && NUC.rxbuffer[1] == _Visual_Frame_Header_2)
			{
				memcpy(NUC.RX_Visual_INFO.useful_info, NUC.rxbuffer, _RXFIFOofVisual_SIZE_t);
			}else if(NUC.rxbuffer[0] == _Navigation_Frame_Header_1 && NUC.rxbuffer[1] == _Navigation_Frame_Header_2)
			{
				memcpy(NUC.RX_Navigation_INFO.useful_info, NUC.rxbuffer, _RXFIFOofNavigation_SIZE_t);
			}else{
				memset(NUC.rxbuffer, 0, _RxFifofromNUC_SIZE_t);
			}

		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
	    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, NUC.rxbuffer, _RxFifofromNUC_SIZE_t);

		#endif 

		#if MINIPC_COMMUNICATION_VERSION_2_0

			if(NUC_ver_2_0.rxbuffer[0] == __Msg2EC_Frame_Header_1 && NUC_ver_2_0.rxbuffer[1] == __Msg2EC_Frame_Header_2)
			{
			    memcpy(NUC_ver_2_0.RX_INFO_2_0.useful_info, NUC_ver_2_0.rxbuffer, _RXFIFOofMINIPC_SIZE_t);
			}else{
				memset(NUC_ver_2_0.rxbuffer, 0, _RxFifofromNUC_SIZE_t);
			}
		
		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, NUC_ver_2_0.rxbuffer, _RxFifofromNUC_SIZE_t);
	
		#endif 
	}
	else{
		return;
	}
}

float GM6020_angle_Rot(float angle, float init_angle)
{
	float output_angle = angle - init_angle;
	if(output_angle >  180) output_angle -= 360;
	if(output_angle < -180) output_angle += 360;
	return output_angle;
}

float GM6020_angle_Rot_inv(float angle, float init_angle)
{
	float output_angle = angle + init_angle;
	if(output_angle >  180) output_angle -= 360;
	if(output_angle < -180) output_angle += 360;
	return output_angle;
}

static void Gimbal_Rot_Matrix_xOy(Gimbal_Rot_to_World_t *tf, float vel_X_gimbal, float vel_Y_gimbal, float yaw) {
	/* 云台逆时针转动角度为正 */
	tf->vel_for_World[0] = vel_X_gimbal * cos(yaw) + vel_Y_gimbal * sin(yaw);
	tf->vel_for_World[1] = vel_Y_gimbal * cos(yaw) - vel_X_gimbal * sin(yaw);
}

static void Gimbal_inv_Rot_Matrix_xOy(Gimbal_Rot_to_World_t *tf, float vel_X_world, float vel_Y_world, float yaw) {
	/* 云台逆时针转动角度为正 */
	tf->vel_for_Gimbal[0] = vel_X_world * cos(yaw) - vel_Y_world * sin(yaw);
	tf->vel_for_Gimbal[1] = vel_Y_world * cos(yaw) + vel_X_world * sin(yaw);
}

