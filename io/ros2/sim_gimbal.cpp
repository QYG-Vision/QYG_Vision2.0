#include "sim_gimbal.hpp"

#include <algorithm>

#include "io/gimbal/gimbal_protocol.hpp"
#include "io/ros2/ros_time.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{

namespace
{

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

template <typename T>
T read_or(const YAML::Node & yaml, const std::string & key, T fallback)
{
  if (yaml[key]) return yaml[key].as<T>();
  return fallback;
}

}  // namespace

SimGimbal::SimGimbal(const std::string & config_path)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto yaml = tools::load(config_path);
  yaw_cmd_offset_deg_ = read_or<double>(yaml, "sim_yaw_cmd_offset_deg", 0.0);
  pitch_cmd_offset_deg_ = read_or<double>(yaml, "sim_pitch_cmd_offset_deg", 0.0);
  state_.bullet_speed = read_or<float>(yaml, "sim_bullet_speed", 25.0f);
  state_.sentry_state = gimbal_protocol::pack_sentry_state(0, GimbalMode::AUTO_AIM);

  node_ = std::make_shared<rclcpp::Node>("qyg_sim_gimbal");
  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/gimbal_pose", rclcpp::SensorDataQoS(),
    std::bind(&SimGimbal::pose_callback, this, std::placeholders::_1));
  cmd_pub_ = node_->create_publisher<rm_interfaces::msg::GimbalCmd>(
    "/rm_gimbal/cmd", rclcpp::SensorDataQoS());
  executor_.add_node(node_);
  spin_thread_ = std::thread([this]() { executor_.spin(); });

  tools::logger()->warn(
    "[SimGimbal] Using first-pass Daedalus camera/gimbal extrinsics from config; "
    "verify R_camera2gimbal and t_camera2gimbal before tuning planner offsets.");
  tools::logger()->info(
    "[SimGimbal] Subscribed /gimbal_pose, publishing /rm_gimbal/cmd, bullet_speed={:.2f}, yaw_offset={:.2f}deg, pitch_offset={:.2f}deg.",
    state_.bullet_speed, yaw_cmd_offset_deg_, pitch_cmd_offset_deg_);
}

SimGimbal::~SimGimbal()
{
  executor_.cancel();
  if (spin_thread_.joinable()) spin_thread_.join();
  if (node_) executor_.remove_node(node_);
}

GimbalMode SimGimbal::mode() const
{
  return GimbalMode::AUTO_AIM;
}

GimbalState SimGimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string SimGimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE: return "IDLE";
    case GimbalMode::AUTO_AIM: return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF: return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF: return "BIG_BUFF";
    default: return "INVALID";
  }
}

Eigen::Quaterniond SimGimbal::orientation(std::chrono::steady_clock::time_point t)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return sim_bridge::interpolate_orientation(orientation_buffer_, t);
}

Eigen::Vector3d SimGimbal::euler(std::chrono::steady_clock::time_point t)
{
  const auto ypr_rad = tools::eulers(orientation(t), 2, 1, 0);
  return {
    ypr_rad[2] * kRadToDeg,
    ypr_rad[1] * kRadToDeg,
    ypr_rad[0] * kRadToDeg};
}

void SimGimbal::send(
  bool control, bool fire, float yaw, float pitch,
  float, float, float)
{
  const auto values = sim_bridge::make_gimbal_cmd_values(
    control, fire, yaw, pitch, yaw_cmd_offset_deg_, pitch_cmd_offset_deg_);

  rm_interfaces::msg::GimbalCmd msg;
  msg.header.stamp = node_->now();
  msg.pitch = values.pitch_deg;
  msg.yaw = values.yaw_deg;
  msg.yaw_diff = values.yaw_diff;
  msg.pitch_diff = values.pitch_diff;
  msg.distance = values.distance;
  msg.fire_advice = values.fire_advice;
  cmd_pub_->publish(msg);
}

void SimGimbal::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  const auto & q_msg = msg->pose.orientation;
  Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  if (q.norm() < 1e-9) return;
  q.normalize();

  const auto ypr_rad = tools::eulers(q, 2, 1, 0);
  Eigen::Vector3d rpy_deg(ypr_rad[2] * kRadToDeg, ypr_rad[1] * kRadToDeg, ypr_rad[0] * kRadToDeg);
  const auto & stamp = msg->header.stamp;
  const auto timestamp = sim_bridge::has_ros_stamp(stamp.sec, stamp.nanosec)
                           ? sim_bridge::ros_stamp_to_steady(stamp.sec, stamp.nanosec)
                           : std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> lock(mutex_);
  state_.current_mode = 0x01;
  state_.sentry_state = gimbal_protocol::pack_sentry_state(0, GimbalMode::AUTO_AIM);
  state_.vyaw = static_cast<float>(rpy_deg[2]);
  state_.vpitch = static_cast<float>(rpy_deg[1]);
  state_.vroll = static_cast<float>(rpy_deg[0]);
  state_.imu_yaw = state_.vyaw;
  state_.imu_pitch = state_.vpitch;
  state_.yaw_imu = state_.imu_yaw;
  state_.pitch_imu = state_.imu_pitch;

  sim_bridge::append_orientation_sample(orientation_buffer_, q, timestamp, 30);
}

}  // namespace io
