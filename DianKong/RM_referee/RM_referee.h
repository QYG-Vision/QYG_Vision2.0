#ifndef __RM_REFEREE_H_
#define __RM_REFEREE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define REFEREE_FRAME_HEADER_SOF  0xA5   

#define HERO                      0x01
#define ENGINEER                  0x02
#define INFANTRY_3                0x03
#define INFANTRY_4                0x04
#define INFANTRY_5                0x05
#define AERIAL                    0x06
#define SENTRY                    0x07
#define DARTS                     0x08
#define RADAR                     0x09

#define CHASSIS_POS               0xAA
#define GIMBAL_POS                0xBB
#define RADAR_POS                 0xCC


typedef enum
{
    Competition_status_info                                         = 0x0001,     //Freq :  1Hz
    Competition_results_info                                        = 0x0002,     //Freq : Trigger
    Robot_Health_info                                               = 0x0003,     //Freq :  3Hz

    Venue_Event_info                                                = 0x0101,     //Freq :  1Hz
    Referee_warning_info                                            = 0x0104,     //Freq :  1Hz
    Dart_Launch_info                                                = 0x0105,     //Freq :  1Hz

    Robot_Performance_Coefficient_info                              = 0x0201,     //Freq : 10Hz
    RT_chassis_buffer_energy_and_firing_heat_info                   = 0x0202,     //Freq : 10Hz
    Robot_Position_info                                             = 0x0203,     //Freq :  1Hz
    Robot_Gains_and_Chassis_Energy_info                             = 0x0204,     //Freq :  3Hz
    Injured_state_info                                              = 0x0206,     //Freq : Trigger
    RT_shooting_info                                                = 0x0207,     //Freq : Trigger
    Allowed_firing_volume_info                                      = 0x0208,     //Freq : 10Hz
    Robot_RFID_Module_Status_info                                   = 0x0209,     //Freq :  3Hz
    Darts_Client_issues_cmd_info                                    = 0x020A,     //Freq :  3Hz
    Ground_Robot_Position_info                                      = 0x020B,     //Freq :  1Hz
    Radar_mark_progress_info                                        = 0x020C,     //Freq :  1Hz
    Sentinel_Autonomous_Decision_info                               = 0x020D,     //Freq :  1Hz
    Radar_Autonomous_Decision_Making_info                           = 0x020E,     //Freq :  1Hz

    Robot_Interaction_info                                          = 0x0301,     //Freq : 30Hz(max)
    Client_mini_map_interaction_info                                = 0x0303,     //Freq : Trigger
    Number_of_radars_received_on_the_player_s_minimap_info          = 0x0305,     //Freq :  5Hz(max)
    Custom_controller_interaction_with_Client_info                  = 0x0306,     //Freq : 30Hz(max)
    Number_of_paths_received_on_the_minimap_info                    = 0x0307,     //Freq :  1Hz(max)
    Client_minimap_receiving_robot                                  = 0x0308      //Freq :  3Hz(max)
}Regular_link_cmd_ID_info_t;

typedef enum
{
    Number_of_interactions_between_custom_controller_cnd_robot_info = 0x0302,     //Freq : 30Hz(max)
    Keyboard_and_mouse_remote_control_info                          = 0x0304,     //Freq : 30Hz
    Custom_controller_receives_robot_info                           = 0x0309,     //Freq : 10Hz(max)
    robot_to_the_custom_client_info                                 = 0x0310,     //Freq : 50Hz(max)

    Set_Image_Transmission_output_channel_info                      = 0x0F01,     //Freq :  1Hz(max)
    Check_the_current_Image_Transmission_output_channel_info        = 0x0F02      //Freq :  2Hz(max)
}Image_Transmission_Link_cmd_ID_info_t;

typedef struct
{
    uint8_t SOF           ;    // 0xA5
    uint8_t data_length[2];    // 数据长度
    uint8_t seq           ;    // 数据包序号
    uint8_t crc8          ;    // frame header CRC校验
}referee_frame_header_t;    


/*---------------------------------常规链路----------------------------------*/
/*
  0x0001 
*/
typedef struct 
{ 
    uint8_t  game_type : 4; 
    uint8_t  game_progress : 4; 
    uint16_t stage_remain_time; 
    uint64_t SyncTimeStamp; 
}game_status_t; 

/*
  0x0002
*/
typedef struct 
{ 
    uint8_t winner; 
}game_result_t; 

/*
  0x0003
*/
typedef struct 
{  
uint16_t ally_1_robot_HP;  
uint16_t ally_2_robot_HP;  
uint16_t ally_3_robot_HP;  
uint16_t ally_4_robot_HP;  

uint16_t ally_7_robot_HP;  
uint16_t ally_outpost_HP;  
uint16_t ally_base_HP; 
}game_robot_HP_t; 

/*
  0x0101
*/
typedef struct 
{ 
  uint32_t event_data; 
}event_data_t; 

/*
  0x0104
*/
typedef struct 
{ 
  uint8_t level; 
  uint8_t offending_robot_id; 
  uint8_t count; 
}referee_warning_t; 

/*
  0x0105
*/
typedef struct 
{ 
  uint8_t  dart_remaining_time; 
  uint16_t dart_info; 
}dart_info_t;

/*
  0x0201
*/
typedef struct 
{ 
  uint8_t  robot_id; 
  uint8_t  robot_level; 
  uint16_t current_HP;  
  uint16_t maximum_HP; 
  uint16_t shooter_barrel_cooling_value; 
  uint16_t shooter_barrel_heat_limit; 
  uint16_t chassis_power_limit;  
  uint8_t  power_management_gimbal_output : 1; 
  uint8_t  power_management_chassis_output : 1;  
  uint8_t  power_management_shooter_output : 1; 
}robot_status_t; 

/*
  0x0202
*/
typedef struct 
{ 
  uint16_t buffer_energy; 
  uint16_t shooter_17mm_1_barrel_heat; 
  uint16_t shooter_42mm_barrel_heat; 
}power_heat_data_t; 

/*
  0x0203
*/
typedef struct 
{ 
  float x; 
  float y; 
  float angle; 
}robot_pos_t; 

/*
  0x0204
*/
typedef struct 
{ 
  uint8_t  recovery_buff;  
  uint16_t cooling_buff;  
  uint8_t  defence_buff;  
  uint8_t  vulnerability_buff; 
  uint16_t attack_buff; 
  uint8_t  remaining_energy; 
}buff_t; 

/*
  0x0206
*/
typedef struct 
{ 
  uint8_t armor_id : 4; 
  uint8_t HP_deduction_reason : 4; 
}hurt_data_t; 

/*
  0x0207
*/
typedef struct 
{ 
  uint8_t bullet_type;  
  uint8_t shooter_number; 
  uint8_t launching_frequency;  
  float   initial_speed;  
}shoot_data_t; 

/*
  0x0208
*/
typedef struct 
{ 
  uint16_t projectile_allowance_17mm; 
  uint16_t projectile_allowance_42mm;  
  uint16_t remaining_gold_coin; 
  uint16_t projectile_allowance_fortress; 
}projectile_allowance_t; 

/*
  0x0209
*/
typedef struct 
{ 
  uint32_t rfid_status;  
  uint8_t  rfid_status_2; 
}rfid_status_t; 

/*
  0x020A
*/
typedef struct 
{ 
  uint8_t  dart_launch_opening_status;  
  uint8_t  reserved;  
  uint16_t target_change_time;  
  uint16_t latest_launch_cmd_time; 
}dart_client_cmd_t; 

/*
  0x020B
*/
typedef struct 
{ 
  float hero_x;  
  float hero_y;  
  float engineer_x;  
  float engineer_y;  
  float standard_3_x;  
  float standard_3_y;  
  float standard_4_x;  
  float standard_4_y;
}ground_robot_position_t; 

/*
  0x020C
*/
typedef struct 
{ 
  uint16_t mark_progress;  
}radar_mark_data_t; 

/*
  0x020D
*/
typedef struct 
{  
  uint32_t sentry_info; 
  uint16_t sentry_info_2; 
}sentry_info_t; 

/*
  0x020E
*/
typedef struct 
{ 
  uint8_t radar_info; 
}radar_info_t; 

/*
  0x0301
*/
typedef struct 
{ 
  uint16_t  data_cmd_id; 
  uint16_t  sender_id; 
  uint16_t  receiver_id; 
  uint8_t  *user_data; 
}robot_interaction_data_t; 

/*
  0x0303
*/
typedef struct 
{ 
float    target_position_x; 
float    target_position_y; 
uint8_t  cmd_keyboard; 
uint8_t  target_robot_id; 
uint16_t cmd_source; 
}map_command_t; 

/*
  0x0305
*/
typedef struct 
{  
  uint16_t hero_position_x; 
  uint16_t hero_position_y; 
  uint16_t engineer_position_x; 
  uint16_t engineer_position_y; 
  uint16_t infantry_3_position_x; 
  uint16_t infantry_3_position_y; 
  uint16_t infantry_4_position_x; 
  uint16_t infantry_4_position_y; 
  uint16_t infantry_5_position_x; 
  uint16_t infantry_5_position_y; 
  uint16_t sentry_position_x; 
  uint16_t sentry_position_y; 
}map_robot_data_t; 

/*
  0x0307
*/
typedef struct 
{ 
  uint8_t  intention; 
  uint16_t start_position_x; 
  uint16_t start_position_y; 
  int8_t   delta_x[49]; 
  int8_t   delta_y[49]; 
  uint16_t sender_id; 
}map_data_t; 

/*
  0x0308
*/
typedef struct 
{  
  uint16_t sender_id; 
  uint16_t receiver_id; 
  uint8_t  user_data[30]; 
}custom_info_t; 

//////////////////////////////////////////////////////////////////////////////////////////
typedef struct
{
    game_status_t                   game_status;
    game_result_t                   game_result;
    game_robot_HP_t                 game_robot_HP;
    event_data_t                    event_data;
    referee_warning_t               referee_warning;
    dart_info_t                     dart_info;
    robot_status_t                  robot_status;
    power_heat_data_t               power_heat_data;
    robot_pos_t                     robot_pos;
    buff_t                          buff;
    hurt_data_t                     hurt_data;
    shoot_data_t                    shoot_data;
    projectile_allowance_t          projectile_allowance;
    rfid_status_t                   rfid_status;
    dart_client_cmd_t               dart_client_cmd;
    ground_robot_position_t         ground_robot_position;
    radar_mark_data_t               radar_mark_data;
    sentry_info_t                   sentry_info;
    radar_info_t                    radar_info;
    robot_interaction_data_t        robot_interaction_data;
    map_command_t                   map_command;
    map_robot_data_t                map_robot_data;
    map_data_t                      map_data;
    custom_info_t                   custom_info;
}Regular_link_INFO_t;
//////////////////////////////////////////////////////////////////////////////////////////

/*----------------------------------------图传链路---------------------------------------*/
/*
  0x0302
*/
typedef struct 
{ 
  uint8_t *data; 
}custom_robot_data_t; 

/*
  0x0304
*/
typedef struct 
{ 
  int16_t  mouse_x; 
  int16_t  mouse_y; 
  int16_t  mouse_z; 
  int8_t   left_button_down; 
  int8_t   right_button_down; 
  uint16_t keyboard_value;
}remote_control_t; 

/*
  0x0309
*/
typedef struct 
{ 
  uint8_t *data; 
}robot_custom_data_t; 

/*
  0x0310
*/
typedef struct 
{  
  uint8_t *data;  
}robot_custom_data_2_t; 

//////////////////////////////////////////////////////////////////////////////////////////
typedef struct
{
    custom_info_t                   custom_info;
    remote_control_t                remote_control;
    custom_robot_data_t             custom_robot_data;
    robot_custom_data_t             robot_custom_data;
}Image_Transmission_link_INFO_t;
//////////////////////////////////////////////////////////////////////////////////////////

/*------------------------------------------Totol---------------------------------------*/
//////////////////////////////////////////////////////////////////////////////////////////
typedef struct
{
    uint16_t                        Robot_ID;
    uint8_t                         Referee_Pos;

    uint8_t                         Referee_FIFO_Buffer[256];

    Regular_link_INFO_t             Regular_link_INFO;
    Image_Transmission_link_INFO_t  Image_Transmission_link_INFO;
}__RoboMaster_Referee_system_info_t;
//////////////////////////////////////////////////////////////////////////////////////////

/*-----------------------------------Referee function-----------------------------------*/

#ifdef __cplusplus
}
#endif

#endif
