#ifndef IO_ROS2_SIM_GIMBAL_HPP_
#define IO_ROS2_SIM_GIMBAL_HPP_

#include <Eigen/Geometry>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rm_interfaces/msg/gimbal_cmd.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thread>

#include "io/gimbal/gimbal.hpp"
#include "io/ros2/sim_gimbal_command.hpp"
#include "io/ros2/sim_gimbal_pose.hpp"

namespace io
{

class SimGimbal
{
public:
  explicit SimGimbal(const std::string & config_path);
  ~SimGimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond orientation(std::chrono::steady_clock::time_point t);
  Eigen::Vector3d euler(std::chrono::steady_clock::time_point t);

  void send(
    bool control, bool fire, float yaw, float pitch,
    float linear_x = 0.0f, float linear_y = 0.0f, float angular_z = 0.0f);

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr cmd_pub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;

  mutable std::mutex mutex_;
  GimbalState state_;
  std::deque<sim_bridge::TimedQuaternion> orientation_buffer_;
  double yaw_cmd_offset_deg_ = 0.0;
  double pitch_cmd_offset_deg_ = 0.0;
};

}  // namespace io

#endif  // IO_ROS2_SIM_GIMBAL_HPP_
