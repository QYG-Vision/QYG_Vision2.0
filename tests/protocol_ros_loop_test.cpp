#include "io/gimbal/gimbal_protocol.hpp"
#include "io/ros2/aim2nav.hpp"
#include "io/ros2/nav2aim.hpp"

#include <cmath>
#include <cstring>
#include <cstdint>
#include <iostream>
#include <optional>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace
{

constexpr double kEpsilon = 1e-4;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

[[noreturn]] void fail(const char * stage, const char * check, const char * hint)
{
  std::cerr << "\n[FAIL] " << stage << "\n"
            << "  Check: " << check << "\n"
            << "  Hint : " << hint << std::endl;
  std::exit(1);
}

void pass(const char * stage)
{
  std::cout << "[PASS] " << stage << std::endl;
}

void expect_near(
  double actual, double expected, double tolerance,
  const char * stage, const char * check, const char * hint)
{
  if (std::abs(actual - expected) > tolerance) {
    std::cerr << "\n[FAIL] " << stage << "\n"
              << "  Check    : " << check << "\n"
              << "  Actual   : " << actual << "\n"
              << "  Expected : " << expected << "\n"
              << "  Tolerance: " << tolerance << "\n"
              << "  Hint     : " << hint << std::endl;
    std::exit(1);
  }
}

void expect_true(bool value, const char * stage, const char * check, const char * hint)
{
  if (!value) {
    fail(stage, check, hint);
  }
}

void expect_equal_hex(
  uint16_t actual, uint16_t expected, const char * stage, const char * check, const char * hint)
{
  if (actual != expected) {
    std::cerr << "\n[FAIL] " << stage << "\n"
              << "  Check   : " << check << "\n"
              << "  Actual  : 0x" << std::hex << std::uppercase << actual << "\n"
              << "  Expected: 0x" << expected << std::dec << "\n"
              << "  Hint    : " << hint << std::endl;
    std::exit(1);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const uint8_t crc_sample[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  expect_equal_hex(
    io::gimbal_protocol::crc16_x25(crc_sample, sizeof(crc_sample)), 0x6F91,
    "CRC16 reference",
    "CRC16 uses RoboMaster referee-style no-final-xor result for 123456789",
    "Check the CRC16 final xor; X25 with final xor would produce 0x906E.");
  pass("CRC16 reference");

  io::ReceiveFrame rx{};
  rx.current_mode = 0x01;
  rx.actual_vx = 0.35f;
  rx.actual_vy = -0.20f;
  rx.actual_wz = 0.15f;
  rx.imu_yaw = 12.0f;
  rx.imu_pitch = -3.0f;
  rx.yaw_angular = 0.7f;
  rx.pitch_angular = -0.4f;
  rx.odom_x = 2.5f;
  rx.chassis_state = 0x02;
  rx.mode = 0x11;
  rx.vyaw = 30.0f;
  rx.vpitch = -10.0f;
  rx.vroll = 1.5f;
  rx.crc16 = io::gimbal_protocol::crc16_x25(
    reinterpret_cast<const uint8_t *>(&rx), sizeof(rx) - sizeof(rx.crc16));

  std::vector<uint8_t> bytes(sizeof(rx));
  std::memcpy(bytes.data(), &rx, sizeof(rx));

  const auto parsed_state = io::gimbal_protocol::parse_receive_frame(bytes);
  expect_true(
    parsed_state.has_value(),
    "GD frame parse",
    "valid GD frame should parse into GimbalState",
    "Check GD header bytes, ReceiveFrame size/offset static_asserts, and CRC16.");
  expect_near(
    parsed_state->actual_vx, rx.actual_vx, kEpsilon,
    "GD frame parse", "actual_vx copied from ReceiveFrame",
    "Check ReceiveFrame.actual_vx offset and GimbalState assignment.");
  expect_near(
    parsed_state->actual_vy, rx.actual_vy, kEpsilon,
    "GD frame parse", "actual_vy copied from ReceiveFrame",
    "Check ReceiveFrame.actual_vy offset and GimbalState assignment.");
  expect_near(
    parsed_state->actual_wz, rx.actual_wz, kEpsilon,
    "GD frame parse", "actual_wz copied from ReceiveFrame",
    "Check ReceiveFrame.actual_wz offset and GimbalState assignment.");
  expect_near(
    parsed_state->vyaw, rx.vyaw, kEpsilon,
    "GD frame parse", "vyaw copied from ReceiveFrame",
    "Check whether the gimbal yaw field was renamed or moved without updating parser/tests.");
  expect_near(
    parsed_state->vpitch, rx.vpitch, kEpsilon,
    "GD frame parse", "vpitch copied from ReceiveFrame",
    "Check whether the gimbal pitch field was renamed or moved without updating parser/tests.");
  expect_near(
    parsed_state->vroll, rx.vroll, kEpsilon,
    "GD frame parse", "vroll copied from ReceiveFrame",
    "Check whether the gimbal roll field was renamed or moved without updating parser/tests.");

  auto bad_bytes = bytes;
  bad_bytes[10] ^= 0x01;
  expect_true(
    !io::gimbal_protocol::parse_receive_frame(bad_bytes).has_value(),
    "GD CRC guard",
    "corrupted GD frame should be rejected",
    "Check CRC16 implementation and CRC field offset; bad frames must not update vision state.");
  pass("GD parse and CRC guard");

  auto observer = std::make_shared<rclcpp::Node>("protocol_ros_loop_observer");
  std::optional<geometry_msgs::msg::Twist> cmd_vel_real;
  std::optional<sensor_msgs::msg::JointState> joint_state;
  auto cmd_vel_real_sub = observer->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_real", 10,
    [&](const geometry_msgs::msg::Twist::SharedPtr msg) { cmd_vel_real = *msg; });
  auto joint_state_sub = observer->create_subscription<sensor_msgs::msg::JointState>(
    "/serial/gimbal_joint_state", 10,
    [&](const sensor_msgs::msg::JointState::SharedPtr msg) { joint_state = *msg; });

  io::Aim2Nav aim2nav;
  aim2nav.publish(*parsed_state);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline &&
         (!cmd_vel_real.has_value() || !joint_state.has_value())) {
    rclcpp::spin_some(observer);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  expect_true(
    cmd_vel_real.has_value(),
    "Aim2Nav ROS publish",
    "received /cmd_vel_real",
    "Check Aim2Nav::publish(), topic name /cmd_vel_real, and ROS2 executor spinning.");
  expect_true(
    joint_state.has_value(),
    "Aim2Nav ROS publish",
    "received /serial/gimbal_joint_state",
    "Check Aim2Nav::publish(), topic name /serial/gimbal_joint_state, and sensor_msgs dependency.");
  expect_near(
    cmd_vel_real->linear.x, rx.actual_vx, kEpsilon,
    "Aim2Nav ROS publish", "/cmd_vel_real.linear.x equals actual_vx",
    "Check actual_vx -> Twist.linear.x mapping.");
  expect_near(
    cmd_vel_real->linear.y, rx.actual_vy, kEpsilon,
    "Aim2Nav ROS publish", "/cmd_vel_real.linear.y equals actual_vy",
    "Check actual_vy -> Twist.linear.y mapping.");
  expect_near(
    cmd_vel_real->angular.z, rx.actual_wz, kEpsilon,
    "Aim2Nav ROS publish", "/cmd_vel_real.angular.z equals actual_wz",
    "Check actual_wz -> Twist.angular.z mapping.");
  expect_true(
    joint_state->position.size() >= 2,
    "Aim2Nav ROS publish",
    "JointState has yaw and pitch positions",
    "Check JointState.name/position population in Aim2Nav::publish().");
  expect_near(
    joint_state->position[0], rx.vyaw * kDegToRad, kEpsilon,
    "Aim2Nav ROS publish", "joint yaw converted degree -> rad",
    "Check degree/radian conversion for gimbal yaw.");
  expect_near(
    joint_state->position[1], rx.vpitch * kDegToRad, kEpsilon,
    "Aim2Nav ROS publish", "joint pitch converted degree -> rad",
    "Check degree/radian conversion for gimbal pitch.");
  pass("Aim2Nav ROS publish");

  io::Nav2Aim nav2aim;
  nav2aim.start();
  auto nav_node = std::make_shared<rclcpp::Node>("protocol_ros_loop_fake_nav");
  auto cmd_vel_pub = nav_node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.42;
  cmd_vel.linear.y = -0.18;
  cmd_vel.angular.z = 0.09;

  const auto cmd_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  geometry_msgs::msg::Twist latest_cmd;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < cmd_deadline) {
    cmd_vel_pub->publish(cmd_vel);
    rclcpp::spin_some(nav_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    latest_cmd = nav2aim.get_latest_state();
    if (std::abs(latest_cmd.linear.x - cmd_vel.linear.x) < kEpsilon &&
        std::abs(latest_cmd.linear.y - cmd_vel.linear.y) < kEpsilon &&
        std::abs(latest_cmd.angular.z - cmd_vel.angular.z) < kEpsilon) {
      break;
    }
  }
  expect_near(
    latest_cmd.linear.x, cmd_vel.linear.x, kEpsilon,
    "Nav2Aim ROS subscribe", "received /cmd_vel.linear.x",
    "Check /cmd_vel topic name, Nav2Aim::start(), and callback/executor thread.");
  expect_near(
    latest_cmd.linear.y, cmd_vel.linear.y, kEpsilon,
    "Nav2Aim ROS subscribe", "received /cmd_vel.linear.y",
    "Check /cmd_vel topic name and Nav2Aim::cmd_vel_callback().");
  expect_near(
    latest_cmd.angular.z, cmd_vel.angular.z, kEpsilon,
    "Nav2Aim ROS subscribe", "received /cmd_vel.angular.z",
    "Check /cmd_vel angular.z mapping.");
  pass("Nav2Aim ROS subscribe");

  const auto tx = io::gimbal_protocol::make_send_frame(
    true, false, 0.25f, -0.12f, latest_cmd.linear.x, latest_cmd.linear.y, latest_cmd.angular.z);
  expect_true(
    io::gimbal_protocol::crc16_x25(
      reinterpret_cast<const uint8_t *>(&tx), sizeof(tx) - sizeof(tx.crc16)) == tx.crc16,
    "QY frame pack",
    "QY CRC matches packed frame",
    "Check SendFrame size/offsets and CRC16 range.");
  expect_near(
    io::gimbal_protocol::decode_angle(tx.yaw), 0.25, kEpsilon,
    "QY frame pack", "yaw command encodes and decodes correctly",
    "Check yaw unit: MiniPC sends radians in [-pi, pi].");
  expect_near(
    io::gimbal_protocol::decode_angle(tx.pitch), -0.12, kEpsilon,
    "QY frame pack", "pitch command encodes and decodes correctly",
    "Check pitch unit: MiniPC sends radians in [-pi, pi].");
  expect_near(
    io::gimbal_protocol::decode_chassis_command(tx.linear_x), -cmd_vel.linear.x, kEpsilon,
    "QY frame pack", "linear_x command encodes negated /cmd_vel.linear.x",
    "Check chassis command sign, range [-1, 1], and field order.");
  expect_near(
    io::gimbal_protocol::decode_chassis_command(tx.linear_y), -cmd_vel.linear.y, kEpsilon,
    "QY frame pack", "linear_y command encodes negated /cmd_vel.linear.y",
    "Check chassis command sign, range [-1, 1], and field order.");
  expect_near(
    io::gimbal_protocol::decode_chassis_command(tx.angular_z), -cmd_vel.angular.z, kEpsilon,
    "QY frame pack", "angular_z command encodes negated /cmd_vel.angular.z",
    "Check /cmd_vel angular.z sign and mapping.");
  pass("QY frame pack");

  rclcpp::shutdown();
  std::cout << "[PASS] protocol_ros_loop_test finished: GD -> Aim2Nav -> ROS -> Nav2Aim -> QY"
            << std::endl;
  return 0;
}
