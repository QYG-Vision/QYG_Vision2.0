#include "qyg_debug_telemetry.hpp"

#include "tools/math_tools.hpp"

namespace
{
constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;
}

namespace io
{
QygDebugTelemetryPublisher::QygDebugTelemetryPublisher() : Node("qyg_debug_telemetry_publisher")
{
  raw_yaw_deg_ = create_publisher<std_msgs::msg::Float64>("/qyg_debug/planner/raw/yaw_deg", 10);
  raw_pitch_deg_ = create_publisher<std_msgs::msg::Float64>("/qyg_debug/planner/raw/pitch_deg", 10);
  tx_yaw_deg_ = create_publisher<std_msgs::msg::Float64>("/qyg_debug/planner/tx/yaw_deg", 10);
  tx_pitch_deg_ = create_publisher<std_msgs::msg::Float64>("/qyg_debug/planner/tx/pitch_deg", 10);
}

void QygDebugTelemetryPublisher::publish_planner_angles(
  const Eigen::Vector2d & raw_yaw_pitch_rad,
  const Eigen::Vector2d & tx_yaw_pitch_rad)
{
  auto make_message = [](double value) {
    std_msgs::msg::Float64 message;
    message.data = value;
    return message;
  };
  raw_yaw_deg_->publish(make_message(tools::limit_rad(raw_yaw_pitch_rad.x()) * RAD_TO_DEG));
  raw_pitch_deg_->publish(make_message(raw_yaw_pitch_rad.y() * RAD_TO_DEG));
  tx_yaw_deg_->publish(make_message(tools::limit_rad(tx_yaw_pitch_rad.x()) * RAD_TO_DEG));
  tx_pitch_deg_->publish(make_message(tx_yaw_pitch_rad.y() * RAD_TO_DEG));
}
}  // namespace io
