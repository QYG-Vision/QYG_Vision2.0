#include "io/gimbal/gimbal_protocol.hpp"
#include "io/ros2/aim2nav.hpp"
#include "io/ros2/nav2aim.hpp"
#include "tools/math_tools.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdint>
#include <iostream>
#include <limits>
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

bool same_gimbal_state(const io::GimbalState & lhs, const io::GimbalState & rhs)
{
  return
    lhs.current_mode == rhs.current_mode &&
    lhs.actual_vx == rhs.actual_vx &&
    lhs.actual_vy == rhs.actual_vy &&
    lhs.actual_wz == rhs.actual_wz &&
    lhs.imu_yaw == rhs.imu_yaw &&
    lhs.imu_pitch == rhs.imu_pitch &&
    lhs.roll_imu == rhs.roll_imu &&
    lhs.yaw_angular == rhs.yaw_angular &&
    lhs.pitch_angular == rhs.pitch_angular &&
    lhs.odom_x == rhs.odom_x &&
    lhs.sentry_state == rhs.sentry_state &&
    lhs.vyaw == rhs.vyaw &&
    lhs.vpitch == rhs.vpitch &&
    lhs.vroll == rhs.vroll &&
    lhs.yaw_imu == rhs.yaw_imu &&
    lhs.pitch_imu == rhs.pitch_imu &&
    lhs.bullet_speed == rhs.bullet_speed &&
    lhs.yaw == rhs.yaw &&
    lhs.pitch == rhs.pitch;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const uint8_t crc_sample[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  expect_equal_hex(
    io::gimbal_protocol::crc16_x25(crc_sample, sizeof(crc_sample)), 0x6F91,
    "CRC16 reference",
    "CRC16 uses the agreed no-final-xor result for 123456789",
    "Remove the legacy final xor; it would produce 0x906E.");
  pass("CRC16 reference");

  expect_equal_hex(
    io::gimbal_protocol::pack_sentry_state(0x1234, io::GimbalMode::BIG_BUFF), 0xD234,
    "Sentry-state packing",
    "status uses bits 13:0 and visual mode uses bits 15:14",
    "Check the status mask and mode left shift.");
  expect_equal_hex(
    io::gimbal_protocol::sentry_status(0x8002), 0x0002,
    "Observed sentry-state regression",
    "0x8002 keeps low-14-bit status value 2",
    "Do not shift the status right by two bits.");
  expect_true(
    io::gimbal_protocol::sentry_mode(0x8002) == io::GimbalMode::SMALL_BUFF,
    "Observed sentry-state regression",
    "0x8002 decodes high bits 10 as small-buff mode",
    "Extract visual mode from bits 15:14.");
  expect_true(
    io::gimbal_protocol::sentry_mode(0x1234) == io::GimbalMode::IDLE &&
    io::gimbal_protocol::sentry_mode(0x5234) == io::GimbalMode::AUTO_AIM &&
    io::gimbal_protocol::sentry_mode(0x9234) == io::GimbalMode::SMALL_BUFF &&
    io::gimbal_protocol::sentry_mode(0xD234) == io::GimbalMode::BIG_BUFF,
    "Sentry-state mode mapping",
    "all four high-bit encodings map to 00/01/10/11",
    "Check the high-two-bit mode mapping.");
  pass("Sentry-state helpers");

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
  rx.sentry_state = io::gimbal_protocol::pack_sentry_state(0x1234, io::GimbalMode::AUTO_AIM);
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
  const auto parsed_state_view =
    io::gimbal_protocol::parse_receive_frame(bytes.data(), bytes.size());
  expect_true(
    parsed_state_view.has_value() && same_gimbal_state(*parsed_state_view, *parsed_state),
    "GD frame parse",
    "pointer/size and vector parser overloads produce the same GimbalState",
    "The vector overload should delegate to the no-copy pointer/size parser.");
  expect_equal_hex(
    parsed_state->sentry_state, 0x5234,
    "GD sentry-state parse",
    "packed sentry state preserves all 16 wire bits",
    "Check ReceiveFrame and GimbalState sentry_state assignment.");
  expect_equal_hex(
    io::gimbal_protocol::sentry_status(parsed_state->sentry_state), 0x1234,
    "GD sentry-state parse",
    "packed state exposes the original 14-bit status",
    "Check the low-14-bit status mask.");
  expect_true(
    io::gimbal_protocol::sentry_mode(parsed_state->sentry_state) == io::GimbalMode::AUTO_AIM,
    "GD sentry-state parse",
    "packed state exposes auto-aim from the high two bits",
    "Check the high-two-bit mode extraction.");
  expect_true(
    bytes[35] == 0x34 && bytes[36] == 0x52,
    "GD sentry-state wire order",
    "0x5234 is serialized as little-endian bytes 34 52",
    "Check the agreed little-endian uint16_t layout.");
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
    parsed_state->vpitch, -rx.vpitch, kEpsilon,
    "GD frame parse", "EC wire pitch converted to the vision sign",
    "ReceiveFrame keeps the EC wire sign; GimbalState must expose the opposite vision sign.");
  expect_near(
    parsed_state->vroll, rx.vroll, kEpsilon,
    "GD frame parse", "vroll copied from ReceiveFrame",
    "Check whether the gimbal roll field was renamed or moved without updating parser/tests.");
  expect_near(
    parsed_state->imu_pitch, rx.imu_pitch, kEpsilon,
    "GD frame parse", "chassis IMU pitch keeps its wire sign",
    "Only the gimbal encoder vpitch field should be converted at the protocol boundary.");

  // Match the orientation path in the latest QYG_sentry_debug entry point.
  const Eigen::Vector3d debug_rpy_rad(
    parsed_state->vroll * kDegToRad,
    parsed_state->vpitch * kDegToRad,
    parsed_state->vyaw * kDegToRad);
  const Eigen::Vector3d debug_ypr_rad(
    debug_rpy_rad.z(), debug_rpy_rad.y(), debug_rpy_rad.x());
  const auto recovered_ypr =
    tools::eulers(tools::rotation_matrix(debug_ypr_rad), 2, 1, 0);
  expect_near(
    recovered_ypr.x(), parsed_state->vyaw * kDegToRad, 1e-9,
    "QYG_sentry_debug orientation", "yaw sign and degree-to-radian conversion",
    "Check the debug entry point's yaw conversion.");
  expect_near(
    recovered_ypr.y(), parsed_state->vpitch * kDegToRad, 1e-9,
    "QYG_sentry_debug orientation", "canonical vision pitch",
    "Pitch-sign correction belongs in the gimbal protocol boundary, not in consumers.");
  expect_near(
    recovered_ypr.z(), parsed_state->vroll * kDegToRad, 1e-9,
    "QYG_sentry_debug orientation", "roll sign and RPY/YPR ordering",
    "Check the solver's Rz(yaw) * Ry(pitch) * Rx(roll) convention.");
  pass("QYG_sentry_debug gimbal orientation convention");

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
    joint_state->position[1], -rx.vpitch * kDegToRad, kEpsilon,
    "Aim2Nav ROS publish", "canonical joint pitch converted degree -> rad",
    "Aim2Nav must consume GimbalState directly without another pitch-sign conversion.");
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
  const auto * const tx_bytes = reinterpret_cast<const uint8_t *>(&tx);
  const uint8_t expected_yaw_bytes[] = {0x00, 0x00, 0x80, 0x3E};
  expect_true(
    sizeof(tx) == 25 && offsetof(io::SendFrame, yaw) == 3 && offsetof(io::SendFrame, crc16) == 23,
    "QY frame layout",
    "QY remains 25 bytes with unchanged yaw and CRC offsets",
    "Check SendFrame packing and field declarations.");
  expect_true(
    io::gimbal_protocol::crc16_x25(
      reinterpret_cast<const uint8_t *>(&tx), sizeof(tx) - sizeof(tx.crc16)) == tx.crc16,
    "QY frame pack",
    "QY CRC matches packed frame",
    "Check SendFrame size/offsets and CRC16 range.");
  expect_true(
    std::memcmp(tx_bytes + offsetof(io::SendFrame, yaw), expected_yaw_bytes, sizeof(expected_yaw_bytes)) == 0,
    "QY float32 wire format",
    "yaw=0.25f is serialized as little-endian IEEE-754 bytes 00 00 80 3E",
    "QY yaw must be a direct float32 field at offset 3.");
  expect_near(
    tx.yaw, 0.25, kEpsilon,
    "QY frame pack", "yaw command is direct float32",
    "Check yaw unit: MiniPC sends radians in [-pi, pi].");
  expect_near(
    tx.pitch, 0.12, kEpsilon,
    "QY frame pack", "pitch command is converted to the EC wire sign",
    "The planner command is -0.12 rad; the current wire convention sends +0.12 rad.");
  expect_near(
    tx.linear_x, -cmd_vel.linear.x, kEpsilon,
    "QY frame pack", "linear_x command is negated on wire",
    "Check chassis command sign, range [-1, 1], and field order.");
  expect_near(
    tx.linear_y, -cmd_vel.linear.y, kEpsilon,
    "QY frame pack", "linear_y command is negated on wire",
    "Check chassis command sign, range [-1, 1], and field order.");
  expect_near(
    tx.angular_z, -cmd_vel.angular.z, kEpsilon,
    "QY frame pack", "angular_z command is negated on wire",
    "Check /cmd_vel angular.z sign and mapping.");

  const auto clamped_tx = io::gimbal_protocol::make_send_frame(
    true, false, 10.0f, -10.0f, 2.0f, -2.0f, 3.0f);
  expect_near(clamped_tx.yaw, M_PI, kEpsilon, "QY frame clamp", "yaw clamps to +pi",
    "Check yaw range before serialization.");
  expect_near(clamped_tx.pitch, M_PI, kEpsilon, "QY frame clamp", "converted pitch clamps to +pi",
    "Apply the vision-to-EC pitch sign before limiting the wire value.");
  expect_near(clamped_tx.linear_x, -1.0, kEpsilon, "QY frame clamp", "linear_x negates and clamps",
    "Check QY velocity sign and [-1, 1] limit.");
  expect_near(clamped_tx.linear_y, 1.0, kEpsilon, "QY frame clamp", "linear_y negates and clamps",
    "Check QY velocity sign and [-1, 1] limit.");
  expect_near(clamped_tx.angular_z, -1.0, kEpsilon, "QY frame clamp", "angular_z negates and clamps",
    "Check QY velocity sign and [-1, 1] limit.");

  const auto invalid_tx = io::gimbal_protocol::make_send_frame(
    true, true, std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f, 0.0f,
    std::numeric_limits<float>::infinity());
  expect_true(
    invalid_tx.mode == 0 && invalid_tx.yaw == 0.0f && invalid_tx.pitch == 0.0f &&
    invalid_tx.linear_x == 0.0f && invalid_tx.linear_y == 0.0f && invalid_tx.angular_z == 0.0f,
    "QY non-finite safety",
    "NaN or infinity produces a zero-valued stop frame",
    "Validate every command with std::isfinite before packing.");
  expect_true(
    io::gimbal_protocol::crc16_x25(
      reinterpret_cast<const uint8_t *>(&invalid_tx), sizeof(invalid_tx) - sizeof(invalid_tx.crc16)) ==
      invalid_tx.crc16,
    "QY non-finite safety",
    "stop frame CRC matches its zero-valued payload",
    "Calculate CRC after replacing invalid input with a stop frame.");
  const auto negative_inf_tx = io::gimbal_protocol::make_send_frame(
    true, false, 0.0f, 0.0f, 0.0f, 0.0f,
    -std::numeric_limits<float>::infinity());
  expect_true(
    negative_inf_tx.mode == 0 && negative_inf_tx.yaw == 0.0f &&
    negative_inf_tx.pitch == 0.0f && negative_inf_tx.linear_x == 0.0f &&
    negative_inf_tx.linear_y == 0.0f && negative_inf_tx.angular_z == 0.0f,
    "QY non-finite safety",
    "negative infinity also produces a zero-valued stop frame",
    "Validate negative infinity with std::isfinite.");
  pass("QY frame pack");

  rclcpp::shutdown();
  std::cout << "[PASS] protocol_ros_loop_test finished: GD -> Aim2Nav -> ROS -> Nav2Aim -> QY"
            << std::endl;
  return 0;
}
