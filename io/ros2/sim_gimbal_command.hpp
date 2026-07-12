#ifndef IO_ROS2_SIM_GIMBAL_COMMAND_HPP_
#define IO_ROS2_SIM_GIMBAL_COMMAND_HPP_

#include <cmath>

namespace io::sim_bridge
{

struct SimGimbalCommandValues
{
  double pitch_deg = 90.0;
  double yaw_deg = 0.0;
  double yaw_diff = 0.0;
  double pitch_diff = 0.0;
  double distance = -1.0;
  bool fire_advice = false;
};

inline SimGimbalCommandValues make_gimbal_cmd_values(
  bool control, bool fire, double yaw_rad, double pitch_rad,
  double yaw_offset_deg, double pitch_offset_deg)
{
  constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

  SimGimbalCommandValues values;
  values.yaw_deg = yaw_rad * kRadToDeg + yaw_offset_deg;
  values.pitch_deg = pitch_rad * kRadToDeg + 90.0 + pitch_offset_deg;
  values.distance = control ? 0.0 : -1.0;
  values.fire_advice = control && fire;
  return values;
}

}  // namespace io::sim_bridge

#endif  // IO_ROS2_SIM_GIMBAL_COMMAND_HPP_
