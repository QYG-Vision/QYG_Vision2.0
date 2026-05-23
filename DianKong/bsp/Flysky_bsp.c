#include "Flysky_bsp.h"

extern DMA_HandleTypeDef hdma_usart1_rx;

void Flysky_init(sentry_remote *sentry_Rm,float spd_X_max, float spd_Y_max, float eps, float gimbal_param)
{
   sentry_Rm ->ctrl.spd_X.spd_limit = spd_X_max;
   sentry_Rm ->ctrl.spd_Y.spd_limit = spd_Y_max;
   sentry_Rm ->epsilon              = eps;

   sentry_Rm ->ctrl.gimbal_factor.gimbal_ctrl_param = gimbal_param;
}

void sbus_data_unpacking(sentry_remote *rxdata,  uint8_t *inbuffer)
{
   if(inbuffer[24] == 0x00){
      rxdata ->channel.channel[0] = ( inbuffer[1]         | (inbuffer[2]   << 8))                        & 0x07FF;
      rxdata ->channel.channel[1] = ((inbuffer[2]   >> 3) | (inbuffer[3]   << 5))                        & 0x07FF;
      rxdata ->channel.channel[2] = ((inbuffer[3]   >> 6) | (inbuffer[4]   << 2)  | (inbuffer[5] << 10)) & 0x07FF;
      rxdata ->channel.channel[3] = ((inbuffer[5]   >> 1) | (inbuffer[6]   << 7))                        & 0x07FF;
      rxdata ->channel.channel[4] = ((inbuffer[6]   >> 4) | (inbuffer[7]   << 4))                        & 0x07FF;
      rxdata ->channel.channel[5] = ((inbuffer[7]   >> 7) | (inbuffer[8]   << 1)  | (inbuffer[9] <<  9)) & 0x07FF;
      rxdata ->channel.channel[6] = ((inbuffer[9]   >> 2) | (inbuffer[10]  << 6))                        & 0x07FF;
      rxdata ->channel.channel[7] = ((inbuffer[10]  >> 5) | (inbuffer[11]  << 3))                        & 0x07FF;
      rxdata ->channel.channel[8] =  (inbuffer[12]        | (inbuffer[13]  << 8))                        & 0x07FF;
      rxdata ->channel.channel[9] = ((inbuffer[13]  >> 3) | (inbuffer[14]  << 5))                        & 0x07FF;
      rxdata ->channel.sbus_check = sbus_OK;
   }
   else{
      rxdata ->channel.sbus_check = sbus_ERR;
   }
}

void sentry_remote_mapping(sentry_remote *RM)
{
   int16_t range = Flysky_max - Flysky_min;

   RM ->ctrl.mode_switch         = ((RM ->channel.channel[6] > Flysky_mid) ? AUTOMATION_MODE : ((RM ->channel.channel[6] < Flysky_mid) ? DISABLE_MODE : REMOTE_MODE));

   if(RM ->ctrl.mode_switch == REMOTE_MODE){
      RM ->ctrl.spd_X.spd_factor    = (float)(((float)(RM ->channel.channel[1]) - Flysky_mid) / (float)(range)) * 2;
      RM ->ctrl.spd_Y.spd_factor    = (float)(((float)(RM ->channel.channel[3]) - Flysky_mid) / (float)(range)) * 2;

      RM ->ctrl.gimbal_factor.yaw_add_factor   = (float)(((float)(RM ->channel.channel[0]) - Flysky_mid) / (float)(range)) * 2.00f;
      RM ->ctrl.gimbal_factor.pitch_factor     = (float)(((float)(RM ->channel.channel[2])             ) / (float)(range)) - 0.15f;

      RM ->ctrl.friction_wheel_switch          = (RM ->channel.channel[4] > Flysky_mid) ? switch_ON : switch_OFF;
   
      if(RM ->ctrl.friction_wheel_switch == 1){
         RM ->ctrl.pluck_wheel_switch          = (RM ->channel.channel[5] > Flysky_mid) ? switch_ON : switch_OFF;
      }

      RM ->ctrl.gyro_switch                    = (RM ->channel.channel[7] > Flysky_mid) ? switch_ON : switch_OFF;
   }else{
      RM ->ctrl.spd_X.spd_factor               = 0;
      RM ->ctrl.spd_Y.spd_factor               = 0;
      RM ->ctrl.gimbal_factor.yaw_add_factor   = 0;
      RM ->ctrl.gimbal_factor.pitch_factor     = 0;
      RM ->ctrl.friction_wheel_switch          = 0;
      RM ->ctrl.pluck_wheel_switch             = 0;
      RM ->ctrl.gyro_switch                    = 0;
   }
   
   RM ->ctrl.chassis_sensitivity = (float)(((float)(RM ->channel.channel[8]) - Flysky_min) / (float)(range));
   RM ->ctrl.gimbal_sensitivity  = (float)(((float)(RM ->channel.channel[9]) - Flysky_min) / (float)(range));

   RM ->ctrl.spd_X.spd               = RM ->ctrl.spd_X.spd_factor * RM ->ctrl.spd_X.spd_limit * RM ->ctrl.chassis_sensitivity;
   RM ->ctrl.spd_Y.spd               = RM ->ctrl.spd_Y.spd_factor * RM ->ctrl.spd_Y.spd_limit * RM ->ctrl.chassis_sensitivity;
   if(fabs(RM ->ctrl.spd_X.spd) < RM ->epsilon){
      RM ->ctrl.spd_X.spd = 0;
   }else{
      RM ->ctrl.spd_X.spd               = RM ->ctrl.spd_X.spd_factor * RM ->ctrl.spd_X.spd_limit * RM ->ctrl.chassis_sensitivity;
   }
   if(fabs(RM ->ctrl.spd_Y.spd) < RM ->epsilon){
      RM ->ctrl.spd_Y.spd = 0;
   }else{
      RM ->ctrl.spd_Y.spd               = RM ->ctrl.spd_Y.spd_factor * RM ->ctrl.spd_Y.spd_limit * RM ->ctrl.chassis_sensitivity;
   }
   RM ->ctrl.gimbal_factor.yaw_add   = RM ->ctrl.gimbal_factor.yaw_add_factor * RM ->ctrl.gimbal_factor.gimbal_ctrl_param * RM ->ctrl.gimbal_sensitivity;
}
