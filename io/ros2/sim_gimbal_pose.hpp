#ifndef IO_ROS2_SIM_GIMBAL_POSE_HPP_
#define IO_ROS2_SIM_GIMBAL_POSE_HPP_

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <deque>

namespace io::sim_bridge
{

struct TimedQuaternion
{
  Eigen::Quaterniond orientation;
  std::chrono::steady_clock::time_point timestamp;
};

inline Eigen::Quaterniond interpolate_orientation(
  const std::deque<TimedQuaternion> & samples,
  std::chrono::steady_clock::time_point timestamp)
{
  if (samples.empty()) return Eigen::Quaterniond::Identity();
  if (samples.size() == 1 || timestamp <= samples.front().timestamp) {
    return samples.front().orientation;
  }
  if (timestamp >= samples.back().timestamp) return samples.back().orientation;

  const auto after = std::lower_bound(
    samples.begin(), samples.end(), timestamp,
    [](const TimedQuaternion & sample, const auto & target) {
      return sample.timestamp < target;
    });
  const auto before = std::prev(after);
  const auto interval =
    std::chrono::duration<double>(after->timestamp - before->timestamp).count();
  if (interval <= 0.0) return before->orientation;

  const auto elapsed =
    std::chrono::duration<double>(timestamp - before->timestamp).count();
  const double ratio = std::clamp(elapsed / interval, 0.0, 1.0);
  return before->orientation.slerp(ratio, after->orientation).normalized();
}

inline void append_orientation_sample(
  std::deque<TimedQuaternion> & samples,
  Eigen::Quaterniond orientation,
  std::chrono::steady_clock::time_point timestamp,
  std::size_t max_samples)
{
  if (orientation.norm() < 1e-9) return;
  orientation.normalize();

  const auto position = std::lower_bound(
    samples.begin(), samples.end(), timestamp,
    [](const TimedQuaternion & sample, const auto & target) {
      return sample.timestamp < target;
    });
  samples.insert(position, TimedQuaternion{orientation, timestamp});
  while (samples.size() > max_samples) samples.pop_front();
}

}  // namespace io::sim_bridge

#endif  // IO_ROS2_SIM_GIMBAL_POSE_HPP_
