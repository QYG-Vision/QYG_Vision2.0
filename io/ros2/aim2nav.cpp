#include "aim2nav.hpp"

namespace io
{

namespace
{

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

}  // namespace

Aim2Nav::Aim2Nav()
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  
  node_ = std::make_shared<rclcpp::Node>("aim2nav");

  gimbal_joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    "serial/gimbal_joint_state", 10);
  cmd_vel_real_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel_real", 10);
  RCLCPP_INFO(
    node_->get_logger(),
    "Aim2Nav node initialized: publishing /serial/gimbal_joint_state and /cmd_vel_real.");
}

Aim2Nav::~Aim2Nav()
{
}

void Aim2Nav::publish(const GimbalState & state)
{
  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp = node_->now();
  joint_state_msg.name = {"gimbal_yaw_joint", "gimbal_pitch_joint"};
  joint_state_msg.position = {
    static_cast<double>(state.vyaw) * kDegToRad,
    static_cast<double>(state.vpitch) * kDegToRad
  };
  gimbal_joint_state_pub_->publish(joint_state_msg);

  geometry_msgs::msg::Twist cmd_vel_real_msg;
  cmd_vel_real_msg.linear.x = state.actual_vx;
  cmd_vel_real_msg.linear.y = state.actual_vy;
  cmd_vel_real_msg.linear.z = 0.0;
  cmd_vel_real_msg.angular.x = 0.0;
  cmd_vel_real_msg.angular.y = 0.0;
  cmd_vel_real_msg.angular.z = state.actual_wz;
  cmd_vel_real_pub_->publish(cmd_vel_real_msg);

  update_buffer(state);
}

void Aim2Nav::update_buffer(const GimbalState & state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  latest_state_ = state;
  
  ChassisData data;
  data.imu_euler = Eigen::Vector3d(
    state.vroll * M_PI / 180.0,
    state.vpitch * M_PI / 180.0,
    state.vyaw * M_PI / 180.0
  );
  data.current_mode = state.current_mode;
  data.mode = state.mode;
  data.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();

  data_buffer_.push_back(data);

  // 保持缓冲区在 1 秒时长
  while (!data_buffer_.empty() && 
         (data.stamp - data_buffer_.front().stamp).seconds() > max_buffer_duration_) {
    data_buffer_.pop_front();
  }
}

GimbalState Aim2Nav::get_latest_state()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_state_;
}

io::GimbalMode Aim2Nav::get_mode()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (data_buffer_.empty()) return io::GimbalMode::IDLE;
  uint8_t m = data_buffer_.back().mode;
  switch (m) {
    case 1: return io::GimbalMode::AUTO_AIM;
    case 2: return io::GimbalMode::SMALL_BUFF;
    case 3: return io::GimbalMode::BIG_BUFF;
    default: return io::GimbalMode::IDLE;
  }
}

Eigen::Vector3d Aim2Nav::get_imu_euler()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (data_buffer_.empty()) return Eigen::Vector3d::Zero();
  return data_buffer_.back().imu_euler;
}

float Aim2Nav::get_imu_yaw()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (data_buffer_.empty()) return 0.0f;
  return static_cast<float>(data_buffer_.back().imu_euler[2]); 
}

float Aim2Nav::get_imu_pitch()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (data_buffer_.empty()) return 0.0f;
  return static_cast<float>(data_buffer_.back().imu_euler[1]); 
}

float Aim2Nav::get_imu_roll()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (data_buffer_.empty()) return 0.0f;
  return static_cast<float>(data_buffer_.back().imu_euler[0]); 
}

std::optional<ChassisData> Aim2Nav::get_chassis_data(rclcpp::Time target_time)
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (data_buffer_.empty()) return std::nullopt;

  if (target_time >= data_buffer_.back().stamp) return data_buffer_.back();
  if (target_time <= data_buffer_.front().stamp) return data_buffer_.front();

  auto it = std::lower_bound(data_buffer_.begin(), data_buffer_.end(), target_time,
    [](const ChassisData& d, const rclcpp::Time& t) {
      return d.stamp < t;
    });

  if (it == data_buffer_.begin()) return *it;

  auto& d_after = *it;
  auto& d_before = *(--it);

  double t_diff = (d_after.stamp - d_before.stamp).seconds();
  if (t_diff < 1e-6) return d_before;

  double ratio = (target_time - d_before.stamp).seconds() / t_diff;

  ChassisData interpolated;
  interpolated.imu_euler = d_before.imu_euler + ratio * (d_after.imu_euler - d_before.imu_euler);
  interpolated.current_mode = d_before.current_mode;
  interpolated.mode = d_before.mode; 
  interpolated.stamp = target_time;

  return interpolated;
}

}  // namespace io
