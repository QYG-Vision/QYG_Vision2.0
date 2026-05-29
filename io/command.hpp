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
  uint8_t target_id = 0;     // 目标 ID：0=无目标，1-8=机器人/建筑
  uint8_t target_valid = 0;  // 目标是否有效：0=无效，非0=有效
};

}  // namespace io

#endif  // IO__COMMAND_HPP
