#ifndef IO_ROS2_ROS_TIME_HPP_
#define IO_ROS2_ROS_TIME_HPP_

#include <chrono>
#include <cstdint>

namespace io::sim_bridge
{

using SteadyTimePoint = std::chrono::steady_clock::time_point;

inline bool has_ros_stamp(std::int32_t sec, std::uint32_t nanosec)
{
  return sec != 0 || nanosec != 0;
}

inline SteadyTimePoint ros_stamp_to_steady(std::int32_t sec, std::uint32_t nanosec)
{
  struct ClockAnchor
  {
    ClockAnchor()
    {
      const auto steady_before = std::chrono::steady_clock::now();
      system = std::chrono::system_clock::now();
      const auto steady_after = std::chrono::steady_clock::now();
      steady = steady_before + (steady_after - steady_before) / 2;
    }

    std::chrono::system_clock::time_point system;
    SteadyTimePoint steady;
  };

  static const ClockAnchor anchor;
  const auto ros_duration =
    std::chrono::seconds(sec) + std::chrono::nanoseconds(nanosec);
  const auto ros_time = std::chrono::system_clock::time_point(
    std::chrono::duration_cast<std::chrono::system_clock::duration>(ros_duration));
  return anchor.steady +
         std::chrono::duration_cast<std::chrono::steady_clock::duration>(ros_time - anchor.system);
}

}  // namespace io::sim_bridge

#endif  // IO_ROS2_ROS_TIME_HPP_
