#ifndef IO__GIMBAL_ORIENTATION_HPP
#define IO__GIMBAL_ORIENTATION_HPP

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <deque>
#include <optional>

namespace io
{
using GimbalClock = std::chrono::steady_clock;

struct TimedGimbalFeedback {
  Eigen::Quaterniond raw_q;
  double feedback_yaw_deg = 0.0;
  GimbalClock::time_point timestamp;
};

struct GimbalFeedbackSample {
  Eigen::Quaterniond raw_q;
  double feedback_yaw_deg = 0.0;
  GimbalClock::time_point query_timestamp;
  GimbalClock::time_point newest_feedback_timestamp;
};

struct FusedGimbalOrientation {
  Eigen::Matrix3d R_gimbal2world;
  Eigen::Quaterniond q_gimbal2world;
  Eigen::Vector3d ypr_rad;
};

std::optional<GimbalFeedbackSample> interpolate_gimbal_feedback(
  const std::deque<TimedGimbalFeedback> & feedback, GimbalClock::time_point query_timestamp);

Eigen::Matrix3d load_R_gimbal2imubody(const YAML::Node & yaml);

double load_imu_feedback_timeout_ms(const YAML::Node & yaml, double default_value = 1000.0);

std::chrono::microseconds load_gimbal_feedback_delay_us(
  const YAML::Node & yaml, std::chrono::microseconds default_value = std::chrono::microseconds{0});

GimbalClock::time_point image_feedback_query_time(
  GimbalClock::time_point image_timestamp, std::chrono::microseconds feedback_delay);

bool feedback_stream_is_live(
  const GimbalFeedbackSample & sample, GimbalClock::time_point now, double timeout_ms);

FusedGimbalOrientation fuse_gimbal_orientation(
  const GimbalFeedbackSample & sample, const Eigen::Matrix3d & R_gimbal2imubody);
}  // namespace io

#endif  // IO__GIMBAL_ORIENTATION_HPP
