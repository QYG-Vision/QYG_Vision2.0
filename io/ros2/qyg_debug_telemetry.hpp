#ifndef IO__ROS2__QYG_DEBUG_TELEMETRY_HPP
#define IO__ROS2__QYG_DEBUG_TELEMETRY_HPP

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace io
{
class QygDebugTelemetryPublisher : public rclcpp::Node
{
public:
  QygDebugTelemetryPublisher();
  void publish_planner_angles(
    const Eigen::Vector2d & raw_yaw_pitch_rad,
    const Eigen::Vector2d & tx_yaw_pitch_rad);

private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr raw_yaw_deg_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr raw_pitch_deg_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tx_yaw_deg_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tx_pitch_deg_;
};
}  // namespace io

#endif  // IO__ROS2__QYG_DEBUG_TELEMETRY_HPP
