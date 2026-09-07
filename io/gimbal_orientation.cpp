#include "gimbal_orientation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "tools/math_tools.hpp"

namespace
{
constexpr double PI = 3.14159265358979323846;
constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double ROTATION_TOLERANCE = 1e-6;

bool is_finite(const Eigen::Quaterniond & q)
{
  return q.coeffs().allFinite();
}

Eigen::Quaterniond normalized_quaternion(const Eigen::Quaterniond & q)
{
  if (!is_finite(q) || !std::isfinite(q.squaredNorm()) || q.squaredNorm() <= 0.0) {
    throw std::invalid_argument("gimbal feedback quaternion must be finite and nonzero");
  }
  return q.normalized();
}

void validate_feedback(const io::TimedGimbalFeedback & feedback)
{
  static_cast<void>(normalized_quaternion(feedback.raw_q));
  if (!std::isfinite(feedback.feedback_yaw_deg)) {
    throw std::invalid_argument("gimbal feedback yaw must be finite");
  }
}

void validate_feedback(const io::GimbalFeedbackSample & feedback)
{
  static_cast<void>(normalized_quaternion(feedback.raw_q));
  if (!std::isfinite(feedback.feedback_yaw_deg)) {
    throw std::invalid_argument("gimbal feedback yaw must be finite");
  }
}

void validate_rotation(const Eigen::Matrix3d & rotation)
{
  if (!rotation.allFinite() ||
      (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > ROTATION_TOLERANCE ||
      std::abs(rotation.determinant() - 1.0) > ROTATION_TOLERANCE) {
    throw std::invalid_argument("R_gimbal2imubody must be a proper rotation matrix");
  }
}

double milliseconds_between(io::GimbalClock::time_point first, io::GimbalClock::time_point second)
{
  return std::chrono::duration<double, std::milli>(first - second).count();
}

io::GimbalFeedbackSample make_sample(
  const io::TimedGimbalFeedback & feedback, io::GimbalClock::time_point query_timestamp,
  io::GimbalClock::time_point newest_feedback_timestamp)
{
  return {normalized_quaternion(feedback.raw_q), feedback.feedback_yaw_deg, query_timestamp,
          newest_feedback_timestamp};
}
}  // namespace

namespace io
{
std::optional<GimbalFeedbackSample> interpolate_gimbal_feedback(
  const std::deque<TimedGimbalFeedback> & feedback, GimbalClock::time_point query_timestamp)
{
  if (feedback.empty()) return std::nullopt;

  for (auto item = feedback.begin(); item != feedback.end(); ++item) {
    validate_feedback(*item);
    if (item != feedback.begin() && item->timestamp <= std::prev(item)->timestamp) {
      throw std::invalid_argument("gimbal feedback timestamps must be strictly increasing");
    }
  }
  const auto newest_feedback_timestamp = feedback.back().timestamp;

  const auto upper = std::lower_bound(
    feedback.begin(), feedback.end(), query_timestamp,
    [](const TimedGimbalFeedback & item, GimbalClock::time_point timestamp) {
      return item.timestamp < timestamp;
  });
  if (upper == feedback.begin()) {
    return make_sample(*upper, query_timestamp, newest_feedback_timestamp);
  }
  if (upper == feedback.end()) {
    const auto & nearest = feedback.back();
    return make_sample(nearest, query_timestamp, newest_feedback_timestamp);
  }
  if (upper->timestamp == query_timestamp) {
    return make_sample(*upper, query_timestamp, newest_feedback_timestamp);
  }

  const auto lower = std::prev(upper);
  const double interval_ms = milliseconds_between(upper->timestamp, lower->timestamp);
  const double fraction = milliseconds_between(query_timestamp, lower->timestamp) / interval_ms;
  const Eigen::Quaterniond raw_q = normalized_quaternion(lower->raw_q)
                                       .slerp(fraction, normalized_quaternion(upper->raw_q))
                                       .normalized();
  const double yaw_rad = tools::interpolate_angle(
    lower->feedback_yaw_deg * DEG_TO_RAD, upper->feedback_yaw_deg * DEG_TO_RAD, fraction);
  return GimbalFeedbackSample{
    raw_q, yaw_rad * RAD_TO_DEG, query_timestamp, newest_feedback_timestamp};
}

Eigen::Matrix3d load_R_gimbal2imubody(const YAML::Node & yaml)
{
  try {
    const auto values = yaml["R_gimbal2imubody"].as<std::vector<double>>();
    if (values.size() != 9 ||
        !std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); })) {
      throw std::invalid_argument("R_gimbal2imubody must contain nine finite values");
    }
    const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> row_major(values.data());
    const Eigen::Matrix3d rotation = row_major;
    validate_rotation(rotation);
    return rotation;
  } catch (const YAML::Exception & error) {
    throw std::invalid_argument(std::string("invalid R_gimbal2imubody: ") + error.what());
  }
}

double load_imu_feedback_timeout_ms(const YAML::Node & yaml, double default_value)
{
  if (!std::isfinite(default_value) || default_value <= 0.0) {
    throw std::invalid_argument("default IMU feedback timeout must be finite and positive");
  }
  try {
    const YAML::Node value = yaml["gimbal"] ? yaml["gimbal"]["imu_feedback_timeout_ms"]
                                             : yaml["imu_feedback_timeout_ms"];
    if (!value) return default_value;
    const double timeout_ms = value.as<double>();
    if (!std::isfinite(timeout_ms) || timeout_ms <= 0.0) {
      throw std::invalid_argument("IMU feedback timeout must be finite and positive");
    }
    return timeout_ms;
  } catch (const YAML::Exception & error) {
    throw std::invalid_argument(std::string("invalid IMU feedback timeout: ") + error.what());
  }
}

std::chrono::microseconds load_gimbal_feedback_delay_us(
  const YAML::Node & yaml, std::chrono::microseconds default_value)
{
  try {
    const YAML::Node value = yaml["gimbal"] ? yaml["gimbal"]["feedback_delay_us"]
                                             : yaml["gimbal_feedback_delay_us"];
    if (!value) return default_value;
    return std::chrono::microseconds{value.as<long long>()};
  } catch (const YAML::Exception & error) {
    throw std::invalid_argument(std::string("invalid gimbal_feedback_delay_us: ") + error.what());
  }
}

GimbalClock::time_point image_feedback_query_time(
  GimbalClock::time_point image_timestamp, std::chrono::microseconds feedback_delay)
{
  return image_timestamp + feedback_delay;
}

bool feedback_stream_is_live(
  const GimbalFeedbackSample & sample, GimbalClock::time_point now, double timeout_ms)
{
  validate_feedback(sample);
  if (!std::isfinite(timeout_ms) || timeout_ms <= 0.0) {
    throw std::invalid_argument("IMU feedback timeout must be finite and positive");
  }
  return milliseconds_between(now, sample.newest_feedback_timestamp) <= timeout_ms;
}

FusedGimbalOrientation fuse_gimbal_orientation(
  const GimbalFeedbackSample & sample, const Eigen::Matrix3d & R_gimbal2imubody)
{
  validate_feedback(sample);
  validate_rotation(R_gimbal2imubody);

  const Eigen::Quaterniond raw_q = normalized_quaternion(sample.raw_q);
  const Eigen::Matrix3d orientation =
    R_gimbal2imubody.transpose() * raw_q.toRotationMatrix() * R_gimbal2imubody;
  Eigen::Vector3d ypr_rad = tools::eulers(orientation, 2, 1, 0);
  ypr_rad[0] = tools::limit_rad(sample.feedback_yaw_deg * DEG_TO_RAD);
  const Eigen::Matrix3d R_gimbal2world = tools::rotation_matrix(ypr_rad);
  Eigen::Quaterniond q_gimbal2world(R_gimbal2world);
  q_gimbal2world.normalize();
  if (q_gimbal2world.w() < 0.0) q_gimbal2world.coeffs() *= -1.0;

  return {R_gimbal2world, q_gimbal2world, ypr_rad};
}
}  // namespace io
