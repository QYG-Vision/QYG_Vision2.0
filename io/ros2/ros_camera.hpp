#ifndef IO_ROS2_ROS_CAMERA_HPP_
#define IO_ROS2_ROS_CAMERA_HPP_

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>

namespace io
{

class RosCamera
{
public:
  RosCamera();
  ~RosCamera();

  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;

  cv::Mat latest_img_;
  std::chrono::steady_clock::time_point latest_timestamp_;
  bool has_image_ = false;
  std::mutex mutex_;
  std::condition_variable image_cv_;
};

}  // namespace io

#endif  // IO_ROS2_ROS_CAMERA_HPP_
