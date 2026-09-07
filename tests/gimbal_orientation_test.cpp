#include "io/gimbal_orientation.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "tools/math_tools.hpp"

namespace
{
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

constexpr double PI = 3.14159265358979323846;

template <typename T, typename = void>
struct has_temporal_distance_ms : std::false_type
{
};

template <typename T>
struct has_temporal_distance_ms<
  T, std::void_t<decltype(std::declval<T>().temporal_distance_ms)>> : std::true_type
{
};

static_assert(
  !has_temporal_distance_ms<io::GimbalFeedbackSample>::value,
  "GimbalFeedbackSample must not expose the retired temporal-distance diagnostic");

double deg(double value) { return value * PI / 180.0; }

Eigen::Quaterniond make_zyx(double yaw, double pitch, double roll)
{
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}

bool near(double actual, double expected, double tolerance = 1e-9)
{
  return std::abs(actual - expected) < tolerance;
}

bool same_rotation(const Eigen::Quaterniond & actual, const Eigen::Quaterniond & expected)
{
  return actual.angularDistance(expected) < 1e-9;
}

template <typename Function>
void assert_throws(Function && function)
{
  bool threw = false;
  try {
    function();
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  assert(threw);
}

void interpolates_feedback_with_shortest_yaw_path()
{
  const auto t0 = Clock::time_point{};
  assert(!io::interpolate_gimbal_feedback({}, t0));

  const std::deque<io::TimedGimbalFeedback> samples{
    {make_zyx(0.0, deg(10.0), deg(-3.0)), 179.0, t0},
    {make_zyx(0.0, deg(20.0), deg(5.0)), -179.0, t0 + 20ms}};

  const auto middle = io::interpolate_gimbal_feedback(samples, t0 + 10ms);
  assert(middle);
  assert(std::abs(std::abs(middle->feedback_yaw_deg) - 180.0) < 1e-9);
  assert(middle->newest_feedback_timestamp == t0 + 20ms);

  const Eigen::Quaterniond expected = samples[0].raw_q.slerp(0.5, samples[1].raw_q).normalized();
  assert(same_rotation(middle->raw_q, expected));

  const auto late = io::interpolate_gimbal_feedback(samples, t0 + 125ms);
  assert(late);
  assert(late->newest_feedback_timestamp == t0 + 20ms);
  assert(io::feedback_stream_is_live(*late, t0 + 1020ms, 1000.0));
  assert(!io::feedback_stream_is_live(*late, t0 + 1021ms, 1000.0));
}

void uses_nearest_sample_outside_feedback_range()
{
  const auto t0 = Clock::time_point{};
  const std::deque<io::TimedGimbalFeedback> samples{
    {make_zyx(deg(10.0), 0.0, 0.0), 10.0, t0 + 10ms},
    {make_zyx(deg(20.0), 0.0, 0.0), 20.0, t0 + 30ms}};

  const auto before = io::interpolate_gimbal_feedback(samples, t0);
  assert(before);
  assert(same_rotation(before->raw_q, samples.front().raw_q));

  const std::deque<io::TimedGimbalFeedback> one_sample{{samples.front().raw_q, 10.0, t0 + 10ms}};
  const auto one = io::interpolate_gimbal_feedback(one_sample, t0 + 35ms);
  assert(one);
  assert(same_rotation(one->raw_q, one_sample.front().raw_q));
}

void rejects_feedback_with_nonincreasing_timestamps()
{
  const auto t0 = Clock::time_point{};
  assert_throws([&] {
    io::interpolate_gimbal_feedback(
      {{make_zyx(0.0, 0.0, 0.0), 0.0, t0 + 20ms},
       {make_zyx(0.0, 0.0, 0.0), 0.0, t0 + 10ms}},
      t0 + 15ms);
  });
  assert_throws([&] {
    io::interpolate_gimbal_feedback(
      {{make_zyx(0.0, 0.0, 0.0), 0.0, t0},
       {make_zyx(0.0, 0.0, 0.0), 0.0, t0}},
      t0 + 1ms);
  });
}

void fuses_feedback_yaw_with_mounted_imu_pitch_and_roll()
{
  const auto t0 = Clock::time_point{};
  const io::GimbalFeedbackSample identity_sample{
    make_zyx(deg(30.0), deg(12.0), deg(-4.0)), 75.0, t0, t0};
  const auto identity = io::fuse_gimbal_orientation(identity_sample, Eigen::Matrix3d::Identity());
  assert(near(identity.ypr_rad[0], deg(75.0)));
  assert(near(identity.ypr_rad[1], deg(12.0)));
  assert(near(identity.ypr_rad[2], deg(-4.0)));
  assert(near(identity.q_gimbal2world.norm(), 1.0));
  assert(identity.q_gimbal2world.w() >= 0.0);
  assert((identity.R_gimbal2world - identity.q_gimbal2world.toRotationMatrix()).norm() < 1e-12);

  const Eigen::Matrix3d mount =
    (Eigen::AngleAxisd(deg(20.0), Eigen::Vector3d::UnitX()) *
     Eigen::AngleAxisd(deg(-15.0), Eigen::Vector3d::UnitY()))
      .toRotationMatrix();
  const io::GimbalFeedbackSample mounted_sample{
    make_zyx(deg(-25.0), deg(8.0), deg(6.0)), 120.0, t0, t0};
  const auto mounted = io::fuse_gimbal_orientation(mounted_sample, mount);
  Eigen::Vector3d expected_ypr =
    tools::eulers(mount.transpose() * mounted_sample.raw_q.toRotationMatrix() * mount, 2, 1, 0);
  expected_ypr[0] = deg(120.0);
  const Eigen::Matrix3d expected_rotation = tools::rotation_matrix(expected_ypr);
  assert((mounted.R_gimbal2world - expected_rotation).norm() < 1e-12);
}

void rejects_invalid_feedback_and_configuration()
{
  const auto t0 = Clock::time_point{};
  const io::GimbalFeedbackSample valid_sample{
    make_zyx(0.0, 0.0, 0.0), 0.0, t0, t0};

  assert_throws([&] {
    auto invalid = valid_sample;
    invalid.feedback_yaw_deg = std::numeric_limits<double>::quiet_NaN();
    io::fuse_gimbal_orientation(invalid, Eigen::Matrix3d::Identity());
  });
  assert_throws([&] {
    auto invalid = valid_sample;
    invalid.raw_q = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
    io::fuse_gimbal_orientation(invalid, Eigen::Matrix3d::Identity());
  });
  assert_throws([&] {
    auto invalid = valid_sample;
    invalid.raw_q.w() = std::numeric_limits<double>::infinity();
    io::fuse_gimbal_orientation(invalid, Eigen::Matrix3d::Identity());
  });
  assert_throws([&] {
    Eigen::Matrix3d invalid = Eigen::Matrix3d::Identity();
    invalid(0, 0) = 2.0;
    io::fuse_gimbal_orientation(valid_sample, invalid);
  });
  assert_throws([&] {
    Eigen::Matrix3d reflection = Eigen::Matrix3d::Identity();
    reflection(0, 0) = -1.0;
    io::fuse_gimbal_orientation(valid_sample, reflection);
  });

  const YAML::Node valid_yaml = YAML::Load(
    "R_gimbal2imubody: [1, 0, 0, 0, 1, 0, 0, 0, 1]\nimu_feedback_timeout_ms: 250");
  assert((io::load_R_gimbal2imubody(valid_yaml) - Eigen::Matrix3d::Identity()).norm() < 1e-12);
  assert(near(io::load_imu_feedback_timeout_ms(valid_yaml), 250.0));
  assert(near(io::load_imu_feedback_timeout_ms(YAML::Load("{}")), 1000.0));

  assert_throws([] { io::load_R_gimbal2imubody(YAML::Load("R_gimbal2imubody: [1, 0]")); });
  assert_throws([] {
    io::load_R_gimbal2imubody(YAML::Load("R_gimbal2imubody: [1, 0, 0, 0, 2, 0, 0, 0, 1]"));
  });
  assert_throws([] { io::load_imu_feedback_timeout_ms(YAML::Load("imu_feedback_timeout_ms: 0")); });
  assert_throws([] {
    io::load_imu_feedback_timeout_ms(YAML::Load("imu_feedback_timeout_ms: .nan"));
  });
  assert_throws([] { io::load_imu_feedback_timeout_ms(YAML::Load("not a map")); });
}

void loads_and_applies_image_feedback_delay()
{
  const auto configured =
    io::load_gimbal_feedback_delay_us(YAML::Load("gimbal_feedback_delay_us: -55600"));
  assert(configured == std::chrono::microseconds{-55600});
  assert(io::load_gimbal_feedback_delay_us(YAML::Load("{}")) == 0us);

  const auto image_timestamp = Clock::time_point{} + 100ms;
  assert(
    io::image_feedback_query_time(image_timestamp, configured) ==
    Clock::time_point{} + 44400us);

  assert_throws([] {
    io::load_gimbal_feedback_delay_us(YAML::Load("gimbal_feedback_delay_us: invalid"));
  });
}

void interpolates_angles_across_the_wrap_boundary()
{
  assert(near(std::abs(tools::interpolate_angle(deg(179.0), deg(-179.0), 0.5)), PI));
  assert(near(tools::interpolate_angle(deg(10.0), deg(30.0), 0.25), deg(15.0)));
}
}  // namespace

int main()
{
  interpolates_feedback_with_shortest_yaw_path();
  uses_nearest_sample_outside_feedback_range();
  rejects_feedback_with_nonincreasing_timestamps();
  fuses_feedback_yaw_with_mounted_imu_pitch_and_roll();
  rejects_invalid_feedback_and_configuration();
  loads_and_applies_image_feedback_delay();
  interpolates_angles_across_the_wrap_boundary();
  return 0;
}
