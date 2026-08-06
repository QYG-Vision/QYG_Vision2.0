#ifndef IO_ROS2_AIM2NAV_HPP_
#define IO_ROS2_AIM2NAV_HPP_

#include <Eigen/Dense>
#include <deque>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "io/gimbal/gimbal.hpp"

namespace io
{

struct ChassisData
{
  Eigen::Vector3d imu_euler; // roll, pitch, yaw (rad)
  uint8_t current_mode;
  uint16_t sentry_state;
  rclcpp::Time stamp;
};

class Aim2Nav
{
public:
  Aim2Nav();
  ~Aim2Nav();

  // 发布云台 yaw/pitch 关节位姿到 /serial/gimbal_joint_state
  void publish(const GimbalState & state);

  GimbalState get_latest_state();
  io::GimbalMode get_mode();
  Eigen::Vector3d get_imu_euler();
  float get_imu_yaw();
  float get_imu_pitch();
  float get_imu_roll();
  // 获取插值后的姿态和模式
  std::optional<ChassisData> get_chassis_data(rclcpp::Time target_time);

private:
  void update_buffer(const GimbalState & state);

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gimbal_joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_real_pub_;
  
  GimbalState latest_state_;
  std::deque<ChassisData> data_buffer_;
  const double max_buffer_duration_ = 1.0; // 缓存1秒数据
  std::mutex mutex_;
};

}  // namespace io

#endif  // IO_ROS2_AIM2NAV_HPP_
