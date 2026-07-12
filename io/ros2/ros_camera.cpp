#include "ros_camera.hpp"

#include <stdexcept>

#include "io/ros2/ros_time.hpp"
#include "tools/logger.hpp"

namespace io
{

RosCamera::RosCamera()
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  node_ = std::make_shared<rclcpp::Node>("qyg_sim_ros_camera");
  image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&RosCamera::image_callback, this, std::placeholders::_1));
  executor_.add_node(node_);
  spin_thread_ = std::thread([this]() { executor_.spin(); });
  tools::logger()->info("[RosCamera] Subscribed to /image_raw.");
}

RosCamera::~RosCamera()
{
  executor_.cancel();
  if (spin_thread_.joinable()) spin_thread_.join();
  if (node_) executor_.remove_node(node_);
}

void RosCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  std::unique_lock<std::mutex> lock(mutex_);
  image_cv_.wait_for(lock, std::chrono::milliseconds(100), [this]() { return has_image_; });
  if (!has_image_) {
    img.release();
    timestamp = std::chrono::steady_clock::now();
    return;
  }
  img = latest_img_.clone();
  timestamp = latest_timestamp_;
  has_image_ = false;
}

void RosCamera::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv::Mat converted;

  if (msg->encoding == "rgb8") {
    cv::Mat rgb(msg->height, msg->width, CV_8UC3, const_cast<uint8_t *>(msg->data.data()));
    cv::cvtColor(rgb, converted, cv::COLOR_RGB2BGR);
  } else if (msg->encoding == "bgr8") {
    cv::Mat bgr(msg->height, msg->width, CV_8UC3, const_cast<uint8_t *>(msg->data.data()));
    converted = bgr.clone();
  } else {
    static bool warned = false;
    if (!warned) {
      tools::logger()->warn("[RosCamera] Unsupported image encoding: {}", msg->encoding);
      warned = true;
    }
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_img_ = converted.clone();
    const auto & stamp = msg->header.stamp;
    latest_timestamp_ = sim_bridge::has_ros_stamp(stamp.sec, stamp.nanosec)
                          ? sim_bridge::ros_stamp_to_steady(stamp.sec, stamp.nanosec)
                          : std::chrono::steady_clock::now();
    has_image_ = true;
  }
  image_cv_.notify_all();
}

}  // namespace io
