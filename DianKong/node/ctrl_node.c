#include "ctrl_node.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

#include "bsp_can.h"
#include "Flysky_bsp.h"
#include "DJI_drv.h"
#include "bsp_wit.h"

#include "auto_mode.h"

#include "smc.h"
#include "pid.h"
#include "NLADRC_ver1.h"

#include "master_node.h"

#define MOTOR_TEST_MODE_INIT  0
#define MOTOR_TEST_MODE_START 0
#define NORMAL_MODE           1

/* IMU Selector */
#define WIT_HWT906             1
#define WIT_HWT906P            0

//哨兵PITCH轴活动范围
#define PITCH_MIN           sentry_pitch_min
#define PITCH_MID           sentry_pitch_mid
#define PITCH_MAX           sentry_pitch_max

#define YAW_INIT_ANGLE       49.115128f

int32_t test_tick    = 0;
uint8_t test_flag    = 1;
int16_t current_test = 0;

float   err_record_angle;

float   shoot_vel = 0;

extern JiuXian_Logic_info_t       Soul_of_JiuXian;
extern XuanHuoTypedef             XuanHuo;

extern _sentry_remote_data_t      RemoteCtrl;
extern Motor_info_t               Pitch_Motor;
extern Motor_info_t               Fire_Motor[2];
extern Motor_info_t               Pluck_Motor;
extern Motor_info_t               Yaw_Motor;
extern sentry_remote              sentry_Remote;
extern wit_imu_t                  wit;

extern _sentry_normal_auto_data_t Auto_Mode;
extern _sentry_remote_data_t      RemoteCtrl;
extern _sentry_data_t             sentry_RM2026;

PID_param_t                       FireCtrl[2];
PID_param_t                       PluckCtrl;
nladrc_t                          GM6020_nladrc;
nladrc_t                          Yaw_nladrc;
LowPassFilter1p_Info_TypeDef      yaw_adrc_lpf;
LowPassFilter1p_Info_TypeDef      adrc_lpf;
smc_param_t                       yaw_smc;

extern uint8_t                    master_node_tick;
extern uint8_t                    debug_node_tick;
extern uint8_t                    wit_tick;
extern uint8_t                    remote_tick;

extern uint8_t                    record_master_tick;
extern uint8_t                    record_debug_tick;
extern uint8_t                    record_wit_tick;   
extern uint8_t                    record_remote_tick;  

extern uint8_t                    can1_rx_flag;
extern uint8_t                    chassis_fdb_flag;

extern float                      delta_yaw;

extern _NUC_UART_INFO_t           NUC;
extern _NUC_UART_INFO_2_0_t       NUC_ver_2_0; 

extern osThreadId_t               ctrlTaskHandle;

extern float                      Eular_angle[4];

//拨弹电机状态机初始化
int16_t                           pluck_current        = 0;
tick_t                            pluck_err_tick       = 0;
tick_t                            pluck_doubt_err_tick = 0;
tick_t                            Yaw_init_tick        = 0;
float                             err_pluck_vel        = 6.28f;
float                             IMU_init_yaw_angle   = 0;

float                             yaw_angle_set        = 0.0f;

bool                              gimbal_Open_flag     = false;

void ctrl_node(void *argument)
{
  /* USER CODE BEGIN ctrl_node */  
  Pluck_Motor.motor_FSM = 1;
  Pitch_Motor.motor_FSM = Pitch_normal;
  yaw_angle_set         = YAW_INIT_ANGLE;
  PID_init(&FireCtrl[0],  140,  0, 40, 0.1, 1000,  6000);//right
  PID_init(&FireCtrl[1],  140,  0, 40, 0.1, 1000,  6000);//left
  PID_init(&PluckCtrl  , 1460, 45, 40, 0.1, 5000, 12000);

  LPF_Init(&adrc_lpf, 0.01);

  // LPF_Init(&yaw_adrc_lpf, 0.01);

  NLADRC_init(&GM6020_nladrc, 1.5, 400, 0.001, 560, 6, 0.5, 0.1);

  ntsmc_init(&yaw_smc.ntsmc_t); //初始化SMC参数

  smc_init(&yaw_smc, 14000); //初始化SMC输出幅值

//   osDelay(10000);

  #if MOTOR_TEST_MODE_INIT
      
  #endif
  /* Infinite loop */
  for(;;)
  {
    taskENTER_CRITICAL();

    #if WIT_HWT906

    ntsmc_param_set(&yaw_smc.ntsmc_t, 29500, 0.0045, 1.0, 1.9, 5, 45, 3, 5, 0.341, 0.5); //设置SMC参数

    #endif

    #if WIT_HWT906P

    ntsmc_param_set(&yaw_smc.ntsmc_t, 12500, 0.0045, 1.0, 1.9, 5, 41, 3, 5, 0.341, 0.6); //设置SMC参数

    #endif

    Pluck_FSM_fcn(); //拨弹电机状态机

    #if MOTOR_TEST_MODE_START      
    current_test = 1000 * sin(2 * PI * 0.5 * test_tick / 1000);
    PITCH_ctrl(current_test);
    test_tick++;
    if(test_tick > 2000) test_tick = 0;
    #endif

    #if NORMAL_MODE

    gimbal_Open_flag = (can1_rx_flag == 1 && chassis_fdb_flag == 1)? true : false;

    if (gimbal_Open_flag == true) {
        NLADRC(&GM6020_nladrc, sentry_RM2026.pitch_angle, Pitch_Motor.real_info.angle);                                       //GM6020非线性自抗扰控制参数初始化
        float u_limit = 12000;                                                                                                                //限制输出幅值设置
        GM6020_nladrc.NLSEF.u = (GM6020_nladrc.NLSEF.u > u_limit? u_limit : (GM6020_nladrc.NLSEF.u < -u_limit? -u_limit : GM6020_nladrc.NLSEF.u));  //输出限幅

        GM6020_nladrc.NLSEF.u = LPF_Calc(&adrc_lpf, GM6020_nladrc.NLSEF.u);                                            //对GM6020自抗扰控制器输出电流进行低通滤波

        shoot_vel = sentry_RM2026.shoot_vel;

        SMC_Update(&yaw_smc, yaw_angle_set - delta_yaw, Yaw_Motor.real_info.angle, Yaw_Motor.real_info.speed, NTSMC); 
    }
    
    #endif
    
    taskEXIT_CRITICAL();
    taskYIELD();
    //osDelay(1);
  }
  /* USER CODE END ctrl_node */
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if (htim->Instance == TIM1)
  {
    float pitch_motor_err = Pitch_Motor.real_info.angle - Auto_Mode.pitch_angle;

    /**
     * @brief 九玄——思绪
     * 
     */
    JiuXian_Thoughts_Update(&Soul_of_JiuXian, &sentry_Remote, NUC_ver_2_0.RX_INFO_2_0.gimbal_mode, 20, 30, 10, 100);
    /**
     * @brief 九玄——玄火
     * 
     */
    XuanHuo_Settlement(&XuanHuo, sentry_RM2026.shoot_vel, fminf(fabsf((float)Fire_Motor[0].enc_info.torque_enc), fabsf((float)Fire_Motor[1].enc_info.torque_enc)));
    
    Eular_angle[0] = GM6020_angle_Rot(Yaw_Motor.real_info.angle + delta_yaw, YAW_INIT_ANGLE);
    Eular_angle[1] = GM6020_angle_Rot(Pitch_Motor.real_info.angle, sentry_abs_zero_pitch);
    Eular_angle[2] = wit.euler_angle[1];
    Eular_angle[3] = 0.0f;

    if(PITCH_ctrl(GM6020_nladrc.NLSEF.u) == DJI_OK)//GM6020_nladrc.NLSEF.u
	{
        FIRE_ctrl((int16_t)PID_Calc(&FireCtrl[0], shoot_vel, Fire_Motor[0].real_info.speed), (int16_t)PID_Calc(&FireCtrl[1], -shoot_vel, Fire_Motor[1].real_info.speed));
	}
    set_sentry_chassis_vel(sentry_RM2026.chassis_vel_x, sentry_RM2026.chassis_vel_y, sentry_RM2026.chassis_omega, 0);

    if(YAW_ctrl(yaw_smc.u) == DJI_OK)//yaw_smc.u
    {
           GET_FIRE_CMD(pluck_current);
           if(Pluck_Motor.motor_FSM == 0)
           {
               pluck_err_tick++;
           }else{
               ;
           }
    }
    (Yaw_init_tick > 2000? 2000 : Yaw_init_tick++);

    if (Soul_of_JiuXian.Mode != STRICKER) {
        yaw_angle_set -= sentry_RM2026.yaw_add_angle;
        if(yaw_angle_set > 360) yaw_angle_set -= 360;
        if(yaw_angle_set <   0) yaw_angle_set += 360;
    }else {
        yaw_angle_set = GM6020_angle_Rot_inv(NUC_ver_2_0.RX_INFO_2_0.Gimbal_Eular_Angle_Degree[0], YAW_INIT_ANGLE);
    }
    // yaw_angle_set -= sentry_RM2026.yaw_add_angle;
    //     if(yaw_angle_set > 360) yaw_angle_set -= 360;
    //     if(yaw_angle_set <   0) yaw_angle_set += 360;
    

    record_master_tick = master_node_tick;
    record_debug_tick  = debug_node_tick;
	record_wit_tick    = wit_tick;
    record_remote_tick = remote_tick;
   
    master_node_tick   = 0;
    debug_node_tick    = 0;
    wit_tick           = 0;
    remote_tick        = 0;
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

ctrlNodeHandle_t set_sentry_chassis_vel(float vx, float vy, float w, int16_t yaw_current)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t             tx_data[8];

    tx_header.StdId = Cinderella;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = 8;

    vx         =  vx * 100;
    vy         =  vy * 100;
    w          =  w  * 100;

    tx_data[0] = ((int16_t)vx          >> 8) & 0xFF;
    tx_data[1] =  (int16_t)vx                & 0xFF;
    tx_data[2] = ((int16_t)vy          >> 8) & 0xFF;
    tx_data[3] =  (int16_t)vy                & 0xFF;
    tx_data[4] = ((int16_t)w           >> 8) & 0xFF;
    tx_data[5] =  (int16_t)w                 & 0xFF;
    tx_data[6] = ((int16_t)yaw_current >> 8) & 0xFF;
    tx_data[7] =  (int16_t)yaw_current       & 0xFF;

    if(HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
    {
        return CTRL_NODE_RUN_OK;
    }else{
        return CTRL_NODE_RUN_ERR;
    }
}

static ctrlNodeHandle_t Pluck_FSM_fcn(void)
{
    if(Pluck_Motor.motor_FSM == 1)
    {
      if(Pluck_Motor.real_info.torqueCurrent < - 12.3f)
      {
          Pluck_Motor.motor_FSM = 0;
      }else if(pluck_doubt_err_tick > 200)
      {
          Pluck_Motor.motor_FSM = 0;
          pluck_doubt_err_tick  = 0;
      }else{
          Pluck_Motor.motor_FSM = 1;
      }
    }else if(Pluck_Motor.motor_FSM == 0){
      if(Pluck_Motor.real_info.torqueCurrent > 27.0f)
      {
          Pluck_Motor.motor_FSM = 2;
          pluck_current = 0;
      }else if(pluck_doubt_err_tick > 200)
      {
          Pluck_Motor.motor_FSM = 2;
          pluck_current = 0;
          pluck_doubt_err_tick  = 0;
      }else{
          Pluck_Motor.motor_FSM = 0;
      }
    }
    if(Pluck_Motor.motor_FSM == 1) pluck_current = (int16_t)PID_Calc(&PluckCtrl, -sentry_RM2026.pluck_vel, Pluck_Motor.real_info.speed);
    else if(Pluck_Motor.motor_FSM == 0){
        if(pluck_err_tick < 90)
        {
            pluck_current = (int16_t)PID_Calc(&PluckCtrl, err_pluck_vel, Pluck_Motor.real_info.speed);
        }else{
            pluck_err_tick = 0;
            Pluck_Motor.motor_FSM = 1;
            pluck_current = (int16_t)PID_Calc(&PluckCtrl, -sentry_RM2026.pluck_vel, Pluck_Motor.real_info.speed);
        }
    }else{
        pluck_current = 0;
    }
    return CTRL_NODE_RUN_OK;
}
