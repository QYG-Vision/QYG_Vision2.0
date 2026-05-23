/**
 * @file auto_mode.c
 * @author Guangzhi Tao
 * @brief 
 * @version 0.1
 * @date 2026-03-08
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "auto_mode.h"

AUTO_MODE_Typedef auto_mode_init(_sentry_normal_auto_data_t *sentry)
{
    sentry ->chassis_omega = 0;
    sentry ->chassis_vel_x = 0;
    sentry ->chassis_vel_y = 0;
    sentry ->pitch_angle   = 0;
    sentry ->pluck_vel     = 0;
    sentry ->shoot_vel     = 0;
    sentry ->yaw_add_angle = 0;

    return AUTO_MODE_OK;
}

AUTO_MODE_Typedef auto_mode_clear(_sentry_normal_auto_data_t *sentry)
{
    sentry ->chassis_omega = 0;
    sentry ->chassis_vel_x = 0;
    sentry ->chassis_vel_y = 0;
    sentry ->pitch_angle   = 0;
    sentry ->pluck_vel     = 0;
    sentry ->shoot_vel     = 0;
    sentry ->yaw_add_angle = 0;
    sentry ->yaw_angle     = 0;

    return AUTO_MODE_OK;
}

//////////////////////////////////////////////////////////玄火////////////////////////////////////////////////////////
/**
 * @brief ： 初始化玄火属性面板
 */
XUANHUO_FSMTypedef JiuXian_Fire_Heat_status_Init(XuanHuoTypedef *xuanhuo, float Heat_Warming, float Heat_Safe, float Update_Freq, float Torque_threshold) {
    xuanhuo ->XuanHuo_Now           = 0;
    xuanhuo ->XuanHuo_Max           = XUANHUO_MAX;
    xuanhuo ->XuanHuo_Warming       = Heat_Warming;
    xuanhuo ->XuanHuo_Safe          = Heat_Safe;
    xuanhuo ->XuanHuo_Calc_Period   = Update_Freq;
    xuanhuo ->HeatStatus            = NORMAL_HEAT;

    xuanhuo ->XuanHuo_CD            = XUANHUO_CD;
    xuanhuo ->XuanHuo_CD_Calc_tik   = 1.0f / xuanhuo ->XuanHuo_Calc_Period;

    xuanhuo ->XuanHuo_TickTock      = 0;

    xuanhuo ->maturity              = Torque_threshold;

    xuanhuo ->XuanHuo_State         = XUANHUO_CLOSE;
    xuanhuo ->XuanHuo_State_prev    = XUANHUO_CLOSE;

    xuanhuo ->shoot_set_now         = 0.0f;
    xuanhuo ->shoot_set_prev        = 0.0f;

    xuanhuo ->XuanHuo_Whether2Use   = XUANHUO_USE_IDLE;

    xuanhuo ->Usage_Permissions     = XUANHUO_USE_PERMISSIONS;
}

/**
 * @brief ： 玄火状态判断
 * 
 * @param xuanhuo 
 * @param shoot_set 
 * @return Xuan_Huo_UseStateTypedef 
 */
Xuan_Huo_UseStateTypedef XuanHuo_State_Judgment(XuanHuoTypedef *xuanhuo, float shoot_set) {
    xuanhuo ->shoot_set_prev = xuanhuo ->shoot_set_now;
    xuanhuo ->shoot_set_now  = shoot_set;
    
    if (xuanhuo ->shoot_set_now != xuanhuo ->shoot_set_prev) {
        if (xuanhuo ->shoot_set_now > 0) {
            return XUANHUO_START;
        }else {
            return XUANHUO_STOP;
        }
    }else {
        if (xuanhuo ->shoot_set_now > 0) {
            return XUANHUO_OPEN;
        }else {
            return XUANHUO_CLOSE;
        }
    }
}

XuanHuo_Whether2UseTypedef XuanHuo_Whether2Use_Judgment(XuanHuoTypedef *xuanhuo, float torque) {
    if (torque > xuanhuo ->maturity) {
        if (xuanhuo ->XuanHuo_Whether2Use == XUANHUO_USE_IDLE) {
            return XUANHUO_USE_DOUBT;
        }else if (xuanhuo ->XuanHuo_Whether2Use == XUANHUO_USE_DOUBT){
            return XUANHUO_USED;
        }else {
            return XUANHUO_USE_ERROR;
        }
    }else {
        return XUANHUO_USE_IDLE;
    }
}

/**
 * @brief ： 玄火热量异常状态判断
 * 
 * @param xuanhuo 
 * @return JiuXian_FireCtrl_HeatTpedef 
 */
JiuXian_FireCtrl_HeatTpedef XuanHuo_Heat_Judgment(XuanHuoTypedef *xuanhuo) {
    if (xuanhuo ->XuanHuo_Now >= xuanhuo ->XuanHuo_Max) {
        return OVER_HEAT;
    }else if (xuanhuo ->XuanHuo_Now >= xuanhuo ->XuanHuo_Warming) {
        return WARMING_HEAT;
    }else if (xuanhuo ->XuanHuo_Now >= xuanhuo ->XuanHuo_Safe) {
        return xuanhuo->HeatStatus;  // 保持当前状态
    }else {
        return NORMAL_HEAT;
    }
}

/**
 * @brief ： 玄火使用权限判断
 * 
 * @param xuanhuo 
 * @return XuanHuo_Usage_PermissionsTypedef 
 */
XuanHuo_Usage_PermissionsTypedef XuanHuo_Usage_Permissions_Judgment(XuanHuoTypedef *xuanhuo) {
    if (xuanhuo ->HeatStatus == NORMAL_HEAT) {
        return XUANHUO_USE_PERMISSIONS;
    }else {
        return XUANHUO_USE_NO_PERMISSIONS;
    }
}

/**
 * @brief ： 玄火状态更新
 * 
 * @param xuanhuo 
 * @param shoot_set 
 * @param torque 
 * @return XUANHUO_FSMTypedef 
 */
XUANHUO_FSMTypedef XuanHuoUpdate(XuanHuoTypedef *xuanhuo, float shoot_set, float torque) {
    xuanhuo ->XuanHuo_State       = XuanHuo_State_Judgment(xuanhuo, shoot_set);

    xuanhuo ->XuanHuo_Whether2Use = XuanHuo_Whether2Use_Judgment(xuanhuo, torque);

    switch(xuanhuo ->XuanHuo_State) {
    case XUANHUO_START:
    case XUANHUO_OPEN:
        if(xuanhuo ->XuanHuo_State != xuanhuo ->XuanHuo_State_prev) {
            xuanhuo ->XuanHuo_Now += 1.0f;
        } else if(xuanhuo ->XuanHuo_Whether2Use == XUANHUO_USED) {
            xuanhuo ->XuanHuo_Now += 1.0f;
        }
        break;
    case XUANHUO_STOP:
    case XUANHUO_CLOSE:
        if(xuanhuo ->XuanHuo_State != xuanhuo ->XuanHuo_State_prev) {
            xuanhuo ->XuanHuo_Now -= 1.0f;
        }
        break;
    }
    xuanhuo ->HeatStatus          = XuanHuo_Heat_Judgment(xuanhuo);
    xuanhuo ->Usage_Permissions   = XuanHuo_Usage_Permissions_Judgment(xuanhuo);
    xuanhuo ->XuanHuo_State_prev  = xuanhuo ->XuanHuo_State;
}

/**
 * @brief ： 玄火结算
 * 
 * @param xuanhuo 
 * @param shoot_set 
 * @param torque 
 * @return XUANHUO_FSMTypedef 
 */
XUANHUO_FSMTypedef XuanHuo_Settlement(XuanHuoTypedef *xuanhuo, float shoot_set, float torque) {
    float cooling = xuanhuo ->XuanHuo_CD * xuanhuo ->XuanHuo_CD_Calc_tik;
    xuanhuo ->XuanHuo_Now = fmaxf(0, xuanhuo ->XuanHuo_Now - cooling);
    xuanhuo ->XuanHuo_Now = fminf(xuanhuo ->XuanHuo_Max, xuanhuo ->XuanHuo_Now);

    if(xuanhuo ->XuanHuo_TickTock < xuanhuo ->XuanHuo_Calc_Period) {
        xuanhuo ->XuanHuo_TickTock += 1;
    }else {
        xuanhuo ->XuanHuo_TickTock  = 0;
        XuanHuoUpdate(xuanhuo, shoot_set, torque);
    }
}

/// //////////////////////////////////////////////////////New Generation!!!/////////////////////////////////////////////////////
//

/**
 * @brief 
 * 
 * @param auto_sentry 
 * @param chassis_vel_x 
 * @param chassis_vel_y 
 * @param chassis_omega 
 * @return AUTO_MODE_Typedef 
 */
AUTO_MODE_Typedef sentry_JiuXian_chassis_running_set(_sentry_normal_auto_data_t *auto_sentry, float chassis_vel_x, 
                                                                                              float chassis_vel_y, 
                                                                                              float chassis_omega
                                                                                            )
{
    auto_sentry ->chassis_vel_x = chassis_vel_x;
    auto_sentry ->chassis_vel_y = chassis_vel_y;
    auto_sentry ->chassis_omega = chassis_omega;

    return AUTO_MODE_OK;
}

AUTO_MODE_FSMTypedef JiuXian_yaw_Logic(JiuXian_Logic_info_t *JiuXian_Thoughts, _sentry_normal_auto_data_t *auto_sentry, float yaw_err)
{
    auto_sentry ->yaw_add_angle = (JiuXian_Thoughts ->Mode == AUTO_SCAN ? yaw_err : 0);
}

AUTO_MODE_FSMTypedef JiuXian_pitch_Logic(JiuXian_Logic_info_t *JiuXian_Thoughts, _sentry_normal_auto_data_t *auto_sentry)
{
    if (JiuXian_Thoughts ->Mode == AUTO_SCAN) {
        auto_sentry ->pitch_angle = ((sentry_pitch_max - sentry_pitch_min) / 2.0f) * sinf(HAL_GetTick() / 300.0f) + (sentry_pitch_mid - sentry_pitch_min);
    }else {
        auto_sentry ->pitch_angle = 0;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 
 * 
 * @param spd_limit 
 * @param spdX 
 * @param spdY 
 * @param K_min 
 * @return float 
 */
AUTO_MODE_SstParamTypedef Speed_Weight_Allocation(float spd_limit, float spdX, float spdY, float K_min)
{
    float K;

    float vel = (fabs(spdX) > fabs(spdY) ? fabs(spdX) : fabs(spdY));

    K = vel / spd_limit + K_min;

    K = (K > 0.7f ? 0.7f : K);

    return K;
}

/////////////////////////////////////////////////////////////////她的思绪，飘荡在青春的海洋上/////////////////////////////////////////////////////////////////

/**
 * @brief 思绪初始化
 * 
 * @param JiuXian_Thoughts 
 * @param idle_period 
 * @return AUTO_MODE_FSMTypedef 
 */
AUTO_MODE_FSMTypedef JiuXian_Thoughts_Init(JiuXian_Logic_info_t *JiuXian_Thoughts, Logic_tik idle_period)
{
    JiuXian_Thoughts ->StatePeriod = idle_period;
    JiuXian_Thoughts ->Mode        = IDLE_MODE;
    JiuXian_Thoughts ->Mode_prev   = IDLE_MODE;
    JiuXian_Thoughts ->tick        = 0;
}

/**
 * @brief 根据视觉模式以及遥控模式，返回当前模式
 * 
 * @param remote 
 * @param Visual_mode 
 * @return JiuXIan_StatusTypedef 
 */
JiuXIan_StatusTypedef JiuXian_Logic_Update(sentry_remote *remote, uint8_t Visual_mode)
{
    if (remote ->ctrl.mode_switch == DISABLE_MODE){
        return IDLE_MODE;
    }else {
        if (Visual_mode == _NO_FIRE_MODE_MINIPC || Visual_mode == _FIRE_MODE_MINIPC)
        {
            return STRICKER;
        }else {
            if (remote->ctrl.mode_switch == REMOTE_MODE)
            {
                return REMOTE_DEBUG;
            }else {
                return AUTO_SCAN;
            }
        }
    }
}

/**
 * @brief 根据当前模式，更新当前状态周期
 * 
 * @param JiuXian_Thoughts 
 * @param idle_period 
 * @param remote_period 
 * @param scan_period 
 * @param striker_period 
 * @return AUTO_MODE_FSMTypedef 
 */
AUTO_MODE_FSMTypedef JiuXian_Logic_Period_Update(JiuXian_Logic_info_t *JiuXian_Thoughts, Logic_tik idle_period, Logic_tik remote_period, Logic_tik scan_period, Logic_tik striker_period)
{
    if(JiuXian_Thoughts ->Mode == IDLE_MODE){
        JiuXian_Thoughts ->StatePeriod = idle_period;
    }else if(JiuXian_Thoughts ->Mode == REMOTE_DEBUG){
        JiuXian_Thoughts ->StatePeriod = remote_period;
    }else if(JiuXian_Thoughts ->Mode == AUTO_SCAN){
        JiuXian_Thoughts ->StatePeriod = scan_period;
    }else if(JiuXian_Thoughts ->Mode == STRICKER){
        JiuXian_Thoughts ->StatePeriod = striker_period;
    }
}    

/**
 * @brief 思绪更新
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
AUTO_MODE_FSMTypedef JiuXian_Thoughts_Update(JiuXian_Logic_info_t *JiuXian_Thoughts, sentry_remote *remote, uint8_t Visual_mode, Logic_tik idle_period, Logic_tik remote_period, Logic_tik scan_period, Logic_tik striker_period)
{
    JiuXian_Thoughts ->Mode = JiuXian_Logic_Update(remote, Visual_mode);
    
    if(JiuXian_Thoughts ->Mode != JiuXian_Thoughts ->Mode_prev){
        JiuXian_Logic_Period_Update(JiuXian_Thoughts, idle_period, remote_period, scan_period, striker_period);
        JiuXian_Thoughts ->tick = 0;
    }else if (JiuXian_Thoughts ->tick >= JiuXian_Thoughts ->StatePeriod) {
        JiuXian_Logic_Period_Update(JiuXian_Thoughts, idle_period, remote_period, scan_period, striker_period);
        JiuXian_Thoughts ->tick = 0;
    }else {
        JiuXian_Thoughts ->tick += 1;
    }
    
    JiuXian_Thoughts ->Mode_prev = JiuXian_Thoughts ->Mode;
}
