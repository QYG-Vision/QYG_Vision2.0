#include "nav2aim.hpp"

namespace io
{

Nav2Aim::Nav2Aim()
{
  // 允许该类被单独测试或在主程序中复用
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  
  node_ = std::make_shared<rclcpp::Node>("nav2aim");

  // 这里只负责接收导航侧发来的速度指令，不再做额外消息转换或转发
  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, std::bind(&Nav2Aim::cmd_vel_callback, this, std::placeholders::_1));
}

Nav2Aim::~Nav2Aim()
{
  if (spin_thread_ && spin_thread_->joinable()) {
    spin_thread_->join();
  }
}

void Nav2Aim::start()
{
  // 独立线程自旋，避免阻塞视觉主循环
  spin_thread_ = std::make_unique<std::thread>([this]() {
    rclcpp::spin(node_);
  });
}

void Nav2Aim::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // 直接缓存最后一帧速度指令，外部按需读取
  std::lock_guard<std::mutex> lock(mutex_);
  latest_state_ = *msg;
}

geometry_msgs::msg::Twist Nav2Aim::get_latest_state()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_state_;
}

}  // namespace io
