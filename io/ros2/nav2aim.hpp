#ifndef IO_ROS2_NAV2AIM_HPP_
#define IO_ROS2_NAV2AIM_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <thread>

namespace io
{

class Nav2Aim
{
public:
  Nav2Aim();
  ~Nav2Aim();

  /**
   * @brief 启动 ROS 2 自旋线程，持续接收 /cmd_vel
   */
  void start();

  /**
   * @brief 获取最近一次收到的 /cmd_vel 消息
   * @return geometry_msgs::msg::Twist
   */
  geometry_msgs::msg::Twist get_latest_state();

private:
  /**
   * @brief 缓存最新的 /cmd_vel 数据，供视觉主线程读取
   * @param msg ROS 2 传入的速度指令消息
   */
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  
  geometry_msgs::msg::Twist latest_state_;
  std::mutex mutex_;
  std::unique_ptr<std::thread> spin_thread_;
};

}  // namespace io

#endif  // IO_ROS2_NAV2AIM_HPP_
