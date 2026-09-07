#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

#include <cstdint>

namespace io
{
struct Command
{
  bool control;              // 是否控制云台；false 会让协议 mode=0
  bool shoot;                // 是否开火；control=true 且 shoot=true 会让协议 mode=2
  double yaw;                // yaw 控制量，单位 rad，发送时转为 float32
  double pitch;              // pitch 控制量，单位 rad，发送时转为 float32
  double horizon_distance = 0; // 无人机专有；当前 PC 串口协议暂不发送这个字段
  uint8_t target_id = 0;     // 装甲板目标 ID：0=无装甲板元数据，1-8=机器人/建筑
  uint8_t target_valid = 0;  // 严格布尔：1=有装甲板元数据；0=无装甲板元数据/停止
  double yaw_vel = 0.0;      // yaw 目标角速度，单位 rad/s
  double pitch_vel = 0.0;    // pitch 目标角速度，单位 rad/s
  bool bypass_pitch_limit = false; // 仅诊断程序使用；正式控制默认保留机械角度保护
};

}  // namespace io

#endif  // IO__COMMAND_HPP
