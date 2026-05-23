#ifndef _AUTO_MODE_H_
#define _AUTO_MODE_H_

#ifdef __cplusplus
extern "C" {
#endif
	
#include "stm32f4xx.h"
#include "main.h"
#include "math.h"
#include "arm_math.h"
#include "master_node.h"
#include "MiniPC.h"
#include "Flysky_bsp.h"
#include "Ramp.h"

#define AUTO_MODE_OK               0x01
#define AUTO_MODE_ERR              0x00

#define SCAN_MODE                  0xAA

#define SET_TICK_MS(X)   (uint32_t)(X)

//#define 

#define AUTO_MODE_Typedef          uint8_t
#define AUTO_MODE_SstParamTypedef  float
#define AUTO_MODE_FSMTypedef       void

#define XUANHUO_FSMTypedef         void

/**
 * @brief : 九玄——思绪滴答宏
 * 
 */
#define Logic_tik                  uint32_t

/**
 * @brief : 九玄——玄火滴答宏
 * 
 */
#define Xuanhuo_tik                uint32_t

/**
 * @brief : 九玄——玄火属性数值宏
 * 
 */
#define XUANHUO_CD                 3.0f
#define XUANHUO_MAX                26.0f

/**
 * @brief : 九玄——玄火量计算频率
 * 
 */
#define XUANHUO_Calc_1Hz              1.0f
#define XUANHUO_Calc_2Hz              2.0f
#define XUANHUO_Calc_4Hz              4.0f
#define XUANHUO_Calc_8Hz              8.0f
#define XUANHUO_Calc_16Hz             16.0f
#define XUANHUO_Calc_32Hz             32.0f
#define XUANHUO_Calc_50Hz             50.0f
#define XUANHUO_Calc_100Hz            100.0f
#define XUANHUO_Calc_200Hz            200.0f
#define XUANHUO_Calc_500Hz            500.0f
#define XUANHUO_Calc_1KHz             1000.0f
#define XUANHUO_Calc_2KHz             2000.0f

/**
 * @brief : 九玄——思绪枚举
 * 
 */
typedef enum {
    IDLE_MODE     = 0x00U,
    REMOTE_DEBUG  = 0x01U,
    AUTO_SCAN     = 0x02U,
    STRICKER      = 0x03U
}JiuXIan_StatusTypedef;

/**
 * @brief : 九玄——玄火量状态枚举
 * 
 */
typedef enum {
    NORMAL_HEAT   = 0x00U,
    WARMING_HEAT  = 0x01U,
    OVER_HEAT     = 0x02U
}JiuXian_FireCtrl_HeatTpedef;

/**
 * @brief : 九玄——玄火使用状态枚举
 * 
 */
typedef enum {
    XUANHUO_START = 0x00U,
    XUANHUO_OPEN  = 0x01U,
    XUANHUO_STOP  = 0x02U,
    XUANHUO_CLOSE = 0x03U
}Xuan_Huo_UseStateTypedef;

typedef enum {
    XUANHUO_USE_IDLE      = 0x00U,
    XUANHUO_USE_DOUBT     = 0x01U,
    XUANHUO_USED          = 0x02U,
    XUANHUO_USE_ERROR     = 0x03U
}XuanHuo_Whether2UseTypedef; 

typedef enum {
    XUANHUO_USE_PERMISSIONS    = 0x00U,
    XUANHUO_USE_NO_PERMISSIONS = 0x01U
}XuanHuo_Usage_PermissionsTypedef;

typedef struct 
{
    float shoot_vel;
    float pluck_vel;
    
    float chassis_vel_x;
    float chassis_vel_y;

    float chassis_omega;

    float yaw_add_angle;
    float yaw_angle;
    float pitch_angle;

    uint8_t AutoHandler;
}_sentry_normal_auto_data_t;

/**
 * @brief : 九玄——玄火属性面板结构体
 * 
 */
typedef struct
{
    float                            XuanHuo_Max;
    float                            XuanHuo_Warming;
    float                            XuanHuo_Safe;
    float                            XuanHuo_Now;
    float                            XuanHuo_Calc_Period;

    float                            shoot_set_now;       //射击速度
    float                            shoot_set_prev;      //射击速度(先前)

    float                            XuanHuo_CD;          //玄火冷却时间

    float                            maturity;            //玄火火候

    float                            XuanHuo_CD_Calc_tik; //玄火冷却时间计算滴答
    uint32_t                         XuanHuo_TickTock;

    XuanHuo_Whether2UseTypedef       XuanHuo_Whether2Use;
    Xuan_Huo_UseStateTypedef         XuanHuo_State;
    Xuan_Huo_UseStateTypedef         XuanHuo_State_prev;
    JiuXian_FireCtrl_HeatTpedef      HeatStatus;
    XuanHuo_Usage_PermissionsTypedef Usage_Permissions;
}XuanHuoTypedef;

/**
 * @brief : 九玄——思绪属性面板结构体
 * 
 */
typedef struct
{
    Logic_tik             tick;
    Logic_tik             StatePeriod;
    JiuXIan_StatusTypedef Mode;
    JiuXIan_StatusTypedef Mode_prev;
}JiuXian_Logic_info_t;

extern AUTO_MODE_Typedef auto_mode_init(_sentry_normal_auto_data_t *sentry);
extern AUTO_MODE_Typedef auto_mode_clear(_sentry_normal_auto_data_t *sentry);

extern AUTO_MODE_SstParamTypedef Speed_Weight_Allocation(float spd_limit, float spdX, float spdY, float K_min);

/////////////////////////////////////////////////////////////////她的思绪之海……///////////////////////////////////////////////////////////////
/**
 * @brief : 九玄之思初始化
*/
extern AUTO_MODE_FSMTypedef JiuXian_Thoughts_Init(JiuXian_Logic_info_t *JiuXian_Thoughts, Logic_tik idle_period);

/**
 * @brief : 九玄之思更新
 * 
 * @param remote 
 * @param Visual_mode 
 * @return JiuXIan_StatusTypedef 
 */
extern JiuXIan_StatusTypedef JiuXian_Logic_Update(sentry_remote *remote, uint8_t Visual_mode);

/**
 * @brief : 九玄之思周期更新
 * 
 * @param JiuXian_Thoughts 
 * @param idle_period 
 * @param remote_period 
 * @param scan_period 
 * @param striker_period 
 * @return AUTO_MODE_FSMTypedef 
 */
extern AUTO_MODE_FSMTypedef JiuXian_Logic_Period_Update(JiuXian_Logic_info_t *JiuXian_Thoughts, Logic_tik idle_period, Logic_tik remote_period, Logic_tik scan_period, Logic_tik striker_period);

/**
 * @brief : 九玄之思状态更新【九玄之思状态机】
 * 
 * @param JiuXian_Thoughts 
 * @param remote 
 * @param Visual_mode 
 * @param idle_period 
 * @param remote_period 
 * @param scan_period 
 * @param striker_period 
 * @return AUTO_MODE_FSMTypedef 
 */
extern AUTO_MODE_FSMTypedef JiuXian_Thoughts_Update(JiuXian_Logic_info_t *JiuXian_Thoughts, sentry_remote *remote, uint8_t Visual_mode, Logic_tik idle_period, Logic_tik remote_period, Logic_tik scan_period, Logic_tik striker_period);

/**
 * @brief : 九玄之思底盘状态更新（全自动模式）
 * 
 * @param auto_sentry 
 * @param chassis_vel_x 
 * @param chassis_vel_y 
 * @param chassis_omega 
 * @return AUTO_MODE_Typedef 
 */
extern AUTO_MODE_Typedef sentry_JiuXian_chassis_running_set(_sentry_normal_auto_data_t *auto_sentry, float chassis_vel_x, float chassis_vel_y, float chassis_omega);

/**
 * @brief : 九玄之思云台偏航角状态更新（扫描模式）
 * 
 * @param JiuXian_Thoughts 
 * @param auto_sentry 
 * @param yaw_err 
 * @return AUTO_MODE_FSMTypedef 
 */
extern AUTO_MODE_FSMTypedef JiuXian_yaw_Logic(JiuXian_Logic_info_t *JiuXian_Thoughts, _sentry_normal_auto_data_t *auto_sentry, float yaw_err);

/**
 * @brief : 九玄之思云台俯仰角状态更新（扫描模式）
 * 
 * @param JiuXian_Thoughts 
 * @param auto_sentry 
 * @return AUTO_MODE_FSMTypedef 
 */
extern AUTO_MODE_FSMTypedef JiuXian_pitch_Logic(JiuXian_Logic_info_t *JiuXian_Thoughts, _sentry_normal_auto_data_t *auto_sentry);

///////////////////////////////////////////////////////////////////她的玄火之海……///////////////////////////////////////////////////////////////////
/**
 * @brief : 九玄——玄火属性初始化
 */  
extern XUANHUO_FSMTypedef JiuXian_Fire_Heat_status_Init(XuanHuoTypedef *xuanhuo, float Heat_Warming, float Heat_Safe, float Update_Freq, float Torque_threshold);

/**
 * @brief : 九玄——玄火状态更新
 * 
 * @param xuanhuo 
 * @param sentry 
 * @return Xuan_Huo_UseStateTypedef 
 */
extern Xuan_Huo_UseStateTypedef XuanHuo_State_Judgment(XuanHuoTypedef *xuanhuo, float shoot_set);

/**
 * @brief : 九玄——玄火使用判断
 * 
 * @param xuanhuo 
 * @param torque 
 * @return XuanHuo_Whether2UseTypedef 
 */
extern XuanHuo_Whether2UseTypedef XuanHuo_Whether2Use_Judgment(XuanHuoTypedef *xuanhuo, float torque);

/**
 * @brief : 九玄——玄火异常状态更新
 * 
 * @param xuanhuo 
 * @return JiuXian_FireCtrl_HeatTpedef 
 */
extern JiuXian_FireCtrl_HeatTpedef XuanHuo_Heat_Judgment(XuanHuoTypedef *xuanhuo);

/**
 * @brief : 九玄——玄火使用权限判断
 * 
 * @param xuanhuo 
 * @return XuanHuo_Usage_PermissionsTypedef 
 */
extern XuanHuo_Usage_PermissionsTypedef XuanHuo_Usage_Permissions_Judgment(XuanHuoTypedef *xuanhuo);

/**
 * @brief : 九玄——玄火更新
 * 
 * @param xuanhuo 
 * @param shoot_set 
 * @param torque 
 * @return XUANHUO_FSMTypedef 
 */
extern XUANHUO_FSMTypedef XuanHuoUpdate(XuanHuoTypedef *xuanhuo, float shoot_set, float torque);

/**
 * @brief : 九玄——玄火结算
 * 
 * @param xuanhuo 
 * @param shoot_set 
 * @param torque 
 * @return XUANHUO_FSMTypedef 
 */
extern XUANHUO_FSMTypedef XuanHuo_Settlement(XuanHuoTypedef *xuanhuo, float shoot_set, float torque);

#ifdef __cplusplus
}
#endif


#endif
