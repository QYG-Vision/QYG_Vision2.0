#ifndef IO_ROS2_ARMOR_TF_PUBLISHER_HPP_
#define IO_ROS2_ARMOR_TF_PUBLISHER_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_ros/transform_broadcaster.h>

#include "tasks/auto_aim/armor.hpp"

namespace io
{

class ArmorTfPublisher
{
public:
  explicit ArmorTfPublisher(
    std::shared_ptr<rclcpp::Node> node,
    std::string parent_frame = "front_industrial_camera_optical_frame",
    std::string child_prefix = "detected_armor");

  static geometry_msgs::msg::TransformStamped make_transform(
    const auto_aim::Armor & armor, const rclcpp::Time & stamp, int index,
    const std::string & parent_frame = "front_industrial_camera_optical_frame",
    const std::string & child_prefix = "detected_armor");

  void publish(const auto_aim::Armor & armor, const rclcpp::Time & stamp, int index);

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::string parent_frame_;
  std::string child_prefix_;
  tf2_ros::TransformBroadcaster broadcaster_;
};

}  // namespace io

#endif  // IO_ROS2_ARMOR_TF_PUBLISHER_HPP_
