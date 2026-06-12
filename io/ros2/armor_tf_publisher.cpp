#include "armor_tf_publisher.hpp"

#include <Eigen/Geometry>

namespace io
{

ArmorTfPublisher::ArmorTfPublisher(
  std::shared_ptr<rclcpp::Node> node, std::string parent_frame, std::string child_prefix)
: node_(std::move(node)),
  parent_frame_(std::move(parent_frame)),
  child_prefix_(std::move(child_prefix)),
  broadcaster_(node_)
{
}

geometry_msgs::msg::TransformStamped ArmorTfPublisher::make_transform(
  const auto_aim::Armor & armor, const rclcpp::Time & stamp, int index,
  const std::string & parent_frame, const std::string & child_prefix)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = parent_frame;
  transform.child_frame_id = child_prefix + "_" + std::to_string(index);

  transform.transform.translation.x = armor.xyz_in_camera.x();
  transform.transform.translation.y = armor.xyz_in_camera.y();
  transform.transform.translation.z = armor.xyz_in_camera.z();

  Eigen::Quaterniond q(armor.R_armor2camera);
  q.normalize();
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();

  return transform;
}

void ArmorTfPublisher::publish(
  const auto_aim::Armor & armor, const rclcpp::Time & stamp, int index)
{
  broadcaster_.sendTransform(make_transform(armor, stamp, index, parent_frame_, child_prefix_));
}

}  // namespace io
