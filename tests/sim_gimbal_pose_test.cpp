#include "io/ros2/ros_time.hpp"
#include "io/ros2/sim_gimbal_pose.hpp"

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <iostream>

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-6;

void expect_near(double actual, double expected, const char * label)
{
  if (std::abs(actual - expected) > kEpsilon) {
    std::cerr << "[FAIL] " << label << ": actual=" << actual
              << " expected=" << expected << std::endl;
    std::exit(1);
  }
}

void expect_true(bool value, const char * label)
{
  if (!value) {
    std::cerr << "[FAIL] " << label << std::endl;
    std::exit(1);
  }
}

Eigen::Quaterniond yaw(double degrees)
{
  return Eigen::Quaterniond(
    Eigen::AngleAxisd(degrees * kPi / 180.0, Eigen::Vector3d::UnitZ()));
}

}  // namespace

int main()
{
  const auto ros_t0 = io::sim_bridge::ros_stamp_to_steady(100, 200000000);
  const auto ros_t1 = io::sim_bridge::ros_stamp_to_steady(100, 210000000);
  expect_near(
    std::chrono::duration<double>(ros_t1 - ros_t0).count(), 0.010,
    "ROS timestamp delta is preserved in steady clock");

  const auto t0 = std::chrono::steady_clock::time_point{};
  const auto t1 = t0 + std::chrono::milliseconds(10);
  std::deque<io::sim_bridge::TimedQuaternion> samples;
  io::sim_bridge::append_orientation_sample(samples, yaw(179.0), t0, 30);
  io::sim_bridge::append_orientation_sample(samples, yaw(-179.0), t1, 30);

  const auto midpoint = io::sim_bridge::interpolate_orientation(
    samples, t0 + std::chrono::milliseconds(5));
  const auto rotated_x = midpoint * Eigen::Vector3d::UnitX();
  expect_true(rotated_x.x() < -0.999, "SLERP crosses the +/-180 boundary by the shortest path");
  expect_near(rotated_x.y(), 0.0, "SLERP midpoint remains at 180 degrees");

  const auto before = io::sim_bridge::interpolate_orientation(
    samples, t0 - std::chrono::milliseconds(1));
  const auto after = io::sim_bridge::interpolate_orientation(
    samples, t1 + std::chrono::milliseconds(1));
  expect_true(std::abs(before.dot(yaw(179.0))) > 1.0 - kEpsilon, "query clamps before buffer");
  expect_true(std::abs(after.dot(yaw(-179.0))) > 1.0 - kEpsilon, "query clamps after buffer");

  std::cout << "[PASS] sim timestamp mapping and quaternion SLERP" << std::endl;
  return 0;
}
