#include <chrono>
#include <cmath>
#include <fmt/core.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "io/gimbal/gimbal.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/QYG_sentry.yaml | 位置参数,yaml配置文件路径 }";

using namespace std::chrono_literals;

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kSendHz = 100.0;
constexpr double kWaveHz = 0.2;
constexpr double kPitchMinDeg = -7.0;
constexpr double kPitchMaxDeg = 3.0;
constexpr double kYawMinDeg = -5.0;
constexpr double kYawMaxDeg = 5.0;

enum class WaveMode
{
  PitchSine = 1,
  YawSine = 2,
  BothSine = 3,
  Hold = 4,
};

constexpr WaveMode kWaveMode = WaveMode::PitchSine;

double center(double min_deg, double max_deg) { return 0.5 * (min_deg + max_deg); }

double amplitude(double min_deg, double max_deg) { return 0.5 * (max_deg - min_deg); }

const char * mode_name(WaveMode mode)
{
  switch (mode) {
    case WaveMode::PitchSine: return "pitch_sine";
    case WaveMode::YawSine: return "yaw_sine";
    case WaveMode::BothSine: return "both_sine";
    case WaveMode::Hold: return "hold";
  }
  return "unknown";
}

double sine_value(double center_deg, double amplitude_deg, double t_s)
{
  return center_deg + amplitude_deg * std::sin(2.0 * kPi * kWaveHz * t_s);
}

geometry_msgs::msg::Vector3 vector3(double x, double y, double z)
{
  geometry_msgs::msg::Vector3 msg;
  msg.x = x;
  msg.y = y;
  msg.z = z;
  return msg;
}

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    rclcpp::shutdown();
    return 0;
  }

  tools::Exiter exiter;
  io::Gimbal gimbal(config_path);

  auto node = std::make_shared<rclcpp::Node>("gimbal_wave_test");
  auto target_pub =
    node->create_publisher<geometry_msgs::msg::Vector3>("/debug/gimbal_wave_target", 10);
  auto feedback_pub =
    node->create_publisher<geometry_msgs::msg::Vector3>("/debug/gimbal_wave_feedback", 10);
  auto error_pub =
    node->create_publisher<geometry_msgs::msg::Vector3>("/debug/gimbal_wave_error", 10);

  tools::logger()->info(
    "[GimbalWaveTest] mode={} send_hz={:.1f} wave_hz={:.2f} pitch=[{:.1f},{:.1f}]deg yaw=[{:.1f},{:.1f}]deg",
    mode_name(kWaveMode), kSendHz, kWaveHz, kPitchMinDeg, kPitchMaxDeg, kYawMinDeg, kYawMaxDeg);
  tools::logger()->info(
    "[GimbalWaveTest] rqt topics: /debug/gimbal_wave_target, /debug/gimbal_wave_feedback, /debug/gimbal_wave_error");

  const auto start_time = std::chrono::steady_clock::now();
  const auto period = std::chrono::duration<double>(1.0 / kSendHz);
  auto next_tick = start_time;

  while (rclcpp::ok() && !exiter.exit()) {
    const auto now = std::chrono::steady_clock::now();
    const auto t_s = std::chrono::duration<double>(now - start_time).count();

    auto target_yaw_deg = center(kYawMinDeg, kYawMaxDeg);
    auto target_pitch_deg = center(kPitchMinDeg, kPitchMaxDeg);

    if (kWaveMode == WaveMode::YawSine || kWaveMode == WaveMode::BothSine) {
      target_yaw_deg = sine_value(center(kYawMinDeg, kYawMaxDeg), amplitude(kYawMinDeg, kYawMaxDeg), t_s);
    }
    if (kWaveMode == WaveMode::PitchSine || kWaveMode == WaveMode::BothSine) {
      target_pitch_deg =
        sine_value(center(kPitchMinDeg, kPitchMaxDeg), amplitude(kPitchMinDeg, kPitchMaxDeg), t_s);
    }

    gimbal.send(
      true, false,
      static_cast<float>(target_yaw_deg * kDegToRad),
      static_cast<float>(target_pitch_deg * kDegToRad),
      0.0f, 0.0f, 0.0f);

    const auto state = gimbal.state();
    const auto rx_yaw_deg = static_cast<double>(state.vyaw);
    const auto rx_pitch_deg = static_cast<double>(state.vpitch);

    target_pub->publish(vector3(target_yaw_deg, target_pitch_deg, static_cast<int>(kWaveMode)));
    feedback_pub->publish(vector3(rx_yaw_deg, rx_pitch_deg, t_s));
    error_pub->publish(vector3(target_yaw_deg - rx_yaw_deg, target_pitch_deg - rx_pitch_deg, 0.0));

    {
      static auto last_log_time = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time).count() >= 500) {
        tools::logger()->info(
          "[GimbalWaveTest] target yaw={:.2f} pitch={:.2f} | rx yaw={:.2f} pitch={:.2f} | err yaw={:.2f} pitch={:.2f}",
          target_yaw_deg, target_pitch_deg,
          rx_yaw_deg, rx_pitch_deg,
          target_yaw_deg - rx_yaw_deg, target_pitch_deg - rx_pitch_deg);
        last_log_time = now;
      }
    }

    next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    std::this_thread::sleep_until(next_tick);
  }

  gimbal.send(false, false, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  rclcpp::shutdown();
  return 0;
}
