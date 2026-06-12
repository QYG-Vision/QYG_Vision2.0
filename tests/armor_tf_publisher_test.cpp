#include "io/ros2/armor_tf_publisher.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "tasks/auto_aim/armor.hpp"

namespace
{

bool nearly_equal(double a, double b, double eps = 1e-9)
{
  return std::abs(a - b) < eps;
}

void require(bool condition, const char * message)
{
  if (!condition) {
    std::cerr << "[FAIL] " << message << '\n';
    std::exit(1);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  if (!rclcpp::ok()) rclcpp::init(argc, argv);

  std::vector<cv::Point2f> points{
    {0.0F, 0.0F}, {1.0F, 0.0F}, {1.0F, 1.0F}, {0.0F, 1.0F}};
  auto_aim::Armor armor(0, 1.0F, cv::Rect(0, 0, 1, 1), points);
  armor.xyz_in_camera = Eigen::Vector3d(1.0, -2.0, 3.0);
  armor.R_armor2camera = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const auto stamp = rclcpp::Time(123, 456);
  const auto transform = io::ArmorTfPublisher::make_transform(armor, stamp, 2);

  require(
    transform.header.frame_id == "front_industrial_camera_optical_frame",
    "parent frame should be the camera optical frame");
  require(transform.child_frame_id == "detected_armor_2", "child frame should include armor index");
  require(transform.header.stamp == stamp, "stamp should be preserved");
  require(nearly_equal(transform.transform.translation.x, 1.0), "translation.x should use xyz_in_camera");
  require(nearly_equal(transform.transform.translation.y, -2.0), "translation.y should use xyz_in_camera");
  require(nearly_equal(transform.transform.translation.z, 3.0), "translation.z should use xyz_in_camera");
  require(
    nearly_equal(transform.transform.rotation.z, std::sqrt(0.5)),
    "rotation should come from R_armor2camera");
  require(
    nearly_equal(transform.transform.rotation.w, std::sqrt(0.5)),
    "rotation should come from R_armor2camera");

  rclcpp::shutdown();
  std::cout << "[PASS] armor_tf_publisher_test\n";
  return 0;
}
