#include "bsp_can.h"

sentry_status_t sentry_info;

Motor_info_t Pitch_Motor;
Motor_info_t Fire_Motor[2];
Motor_info_t Yaw_Motor;
Motor_info_t Pluck_Motor;

int16_t      HWT_wit_enc;

int16_t      chassis_msg_fdb[3];

int8_t       can1_rx_flag = 0;
int8_t       can2_rx_flag = 0;
int8_t       chassis_fdb_flag = 0;

void can1_filter_init(void)
{
    CAN_FilterTypeDef can_filter;

    can_filter.FilterBank = 0;
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh = 0x0000;
    can_filter.FilterIdLow = 0x0000;
    can_filter.FilterMaskIdHigh = 0x0000;
    can_filter.FilterMaskIdLow = 0x0000;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter.FilterActivation = ENABLE;
    can_filter.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan1, &can_filter);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

}

void can2_filter_init(void)
{
    CAN_FilterTypeDef can_filter;
		
	can_filter.FilterBank = 14;                  
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh = 0x0000;       
    can_filter.FilterIdLow = 0x0000;
    can_filter.FilterMaskIdHigh = 0x0000;
    can_filter.FilterMaskIdLow = 0x0000;
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    can_filter.FilterActivation = ENABLE;
	
    HAL_CAN_ConfigFilter(&hcan2, &can_filter);  
    HAL_CAN_Start(&hcan2);                       
	//   HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if(hcan->Instance == CAN1)
    {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

        can1_rx_flag = 1;
        
        switch(rx_header.StdId)
        {
            case Yaw            :
            {
                Yaw_Motor.motorType = GM6020;
                Motor_info_Updata(&Yaw_Motor, rx_data);
                break;
            }
            case Pluck          :
            {
                Pluck_Motor.motorType = M2006;
                Motor_info_Updata(&Pluck_Motor, rx_data);
                break;
            }
            case chassis        :
            {
                sentry_info.id          =           rx_data[0];
								sentry_info.game_status =           rx_data[1];
								sentry_info.HP          = ((int16_t)rx_data[3] << 8) | rx_data[2];
								sentry_info.heat        = ((int16_t)rx_data[5] << 8) | rx_data[4];
                break;
            }
            case chassis1       :
            {
                chassis_fdb_flag = 1;
								chassis_msg_fdb[0] = ((int16_t)rx_data[0] << 8) | rx_data[1];
                chassis_msg_fdb[1] = ((int16_t)rx_data[2] << 8) | rx_data[3];
                chassis_msg_fdb[2] = ((int16_t)rx_data[4] << 8) | rx_data[5];
                HWT_wit_enc        = ((int16_t)rx_data[6] << 8) | rx_data[7];
                break;
            }
            default             :
            {
                break;
            }
        }
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if(hcan->Instance == CAN2)
    {
        HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14);
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);

        can2_rx_flag = 1;

        switch(rx_header.StdId){
            case Pitch  : {
                             Pitch_Motor.motorType   = GM6020;
                             Motor_info_Updata(&Pitch_Motor  , rx_data);
                             break;
            }
            case r_fric : {
                             Fire_Motor[0].motorType = M3508;
                             Motor_info_Updata(&Fire_Motor[0], rx_data);
                             break;
            }
            case l_fric : {
                             Fire_Motor[1].motorType = M3508;
                             Motor_info_Updata(&Fire_Motor[1], rx_data);
                             break;
            }
            default     : {
                             break;
            }
        }
    }
}

static void Motor_info_Updata(Motor_info_t *motor, uint8_t data[8])
{
    motor ->enc_info.angle_enc    = (data[0] << 8) | data[1];
    motor ->enc_info.speed_enc    = (data[2] << 8) | data[3];
    motor ->enc_info.torque_enc   = (data[4] << 8) | data[5];
    motor ->enc_info.temp         =  data[6];
    enc2real(motor);
}

static void enc2real(Motor_info_t *motor)
{
    switch (motor ->motorType)
    {
        case M3508  : {
                        motor ->real_info.angle     = motor ->enc_info.angle_enc * 360.f / 8191.f;
                        motor ->real_info.speed     = motor ->enc_info.speed_enc         / 9.55f;

                        motor ->real_info.temp      = motor ->enc_info.temp;
                        break;
        }
        case GM6020 : {
                        motor ->real_info.angle     = motor ->enc_info.angle_enc * 360.f / 8191.f;
                        motor ->real_info.speed     = motor ->enc_info.speed_enc         / 9.55f;

                        motor ->real_info.temp      = motor ->enc_info.temp;
                        break;
        }
        case M2006  : {
                        motor ->real_info.angle         = motor ->enc_info.angle_enc  *      360.0f /         8191.0f ;
                        motor ->real_info.speed         = motor ->enc_info.speed_enc  /(      36.0f *           9.55f);
                        motor ->real_info.torqueCurrent = motor ->enc_info.torque_enc * Current_MAX / Current_MAX_Enc ;
                        motor ->real_info.temp          = motor ->enc_info.temp       *        1.0f /            1.0f ;
                        break;
        }
    }
}


