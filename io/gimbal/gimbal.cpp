#include "gimbal.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unistd.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
namespace
{
constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;

bool valid_command(const Command & command)
{
  const bool armor_metadata =
    command.target_valid == 1 && command.target_id >= 1 && command.target_id <= 8;
  const bool no_armor_metadata = command.target_valid == 0 && command.target_id == 0;
  return command.control && (armor_metadata || no_armor_metadata) &&
         std::isfinite(command.yaw) && std::isfinite(command.pitch) &&
         std::isfinite(command.yaw_vel) && std::isfinite(command.pitch_vel);
}

EnemyColor enemy_color_for(OwnColor own_color)
{
  if (own_color == OwnColor::red) return EnemyColor::blue;
  if (own_color == OwnColor::blue) return EnemyColor::red;
  return EnemyColor::unknown;
}

template<typename T>
T read_positive(const YAML::Node & node, const char * key, T default_value)
{
  if (!node[key]) return default_value;
  const T value = node[key].as<T>();
  if (!std::isfinite(static_cast<double>(value)) || value <= 0) {
    throw std::invalid_argument{std::string{"gimbal."} + key + " must be positive"};
  }
  return value;
}
}  // namespace

std::uint16_t Gimbal::modbus_crc16(const std::uint8_t * data, std::size_t length)
{
  std::uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (std::uint8_t bit = 0; bit < 8; ++bit) {
      crc = crc & 0x0001 ? static_cast<std::uint16_t>((crc >> 1) ^ 0xA001) : crc >> 1;
    }
  }
  return crc;
}

Gimbal::ControlPacket Gimbal::make_control_packet(const Command & command)
{
  ControlPacket packet{};
  if (valid_command(command)) {
    packet.mode = command.shoot ? 2 : 1;
    packet.yaw = static_cast<float>(command.yaw);
    packet.pitch = static_cast<float>(
      command.bypass_pitch_limit ? command.pitch : command_pitch_to_electrical(command.pitch));
    packet.yaw_vel = static_cast<float>(command.yaw_vel);
    packet.pitch_vel =
      static_cast<float>(command_pitch_velocity_to_electrical(command.pitch_vel));
    packet.target_id = command.target_id;
    packet.target_valid = command.target_valid;
  }
  packet.crc16 = modbus_crc16(reinterpret_cast<const std::uint8_t *>(&packet), 21);
  return packet;
}

std::optional<Gimbal::DecodedFeedback> Gimbal::decode_feedback_packet(
  const FeedbackPacket & packet)
{
  if (packet.header[0] != 'i' || packet.header[1] != 'm') return std::nullopt;
  const auto crc = modbus_crc16(
    reinterpret_cast<const std::uint8_t *>(&packet), sizeof(packet) - sizeof(packet.crc16));
  if (crc != packet.crc16 || packet.mode > static_cast<std::uint8_t>(Mode::big_buff) ||
      packet.own_color > static_cast<std::uint8_t>(OwnColor::blue) ||
      !std::isfinite(packet.bullet_speed) || !std::isfinite(packet.yaw)) {
    return std::nullopt;
  }

  const Eigen::Quaterniond q{packet.qw, packet.qx, packet.qy, packet.qz};
  if (!q.coeffs().allFinite() || !std::isfinite(q.squaredNorm()) ||
      std::abs(q.squaredNorm() - 1.0) > 1e-2) {
    return std::nullopt;
  }

  const auto own_color = static_cast<OwnColor>(packet.own_color);
  return DecodedFeedback{
    static_cast<Mode>(packet.mode), own_color, enemy_color_for(own_color), packet.bullet_speed,
    feedback_orientation_to_vision(q), packet.yaw};
}

Eigen::Quaterniond Gimbal::feedback_orientation_to_vision(
  const Eigen::Quaterniond & electrical_q)
{
  return electrical_q.normalized();
}

double Gimbal::command_pitch_to_electrical(double vision_pitch)
{
  constexpr double MIN_PITCH = -22.0 * DEG_TO_RAD;
  constexpr double MAX_PITCH = 9.8 * DEG_TO_RAD;
  return std::clamp(vision_pitch, MIN_PITCH, MAX_PITCH);
}

double Gimbal::command_pitch_velocity_to_electrical(double vision_pitch_velocity)
{
  return vision_pitch_velocity;
}

std::string Gimbal::format_feedback_summary(
  const Eigen::Quaterniond & vision_q, double feedback_yaw_deg, double age_ms)
{
  const auto normalized_q = vision_q.normalized();
  const Eigen::Vector3d ypr = tools::eulers(normalized_q, 2, 1, 0) * RAD_TO_DEG;
  const double displayed_feedback_yaw_deg = tools::yaw_with_zero_at_wrap_boundary(
                                              feedback_yaw_deg * DEG_TO_RAD) *
                                            RAD_TO_DEG;
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6) << "RX qw=" << normalized_q.w()
         << ", qx=" << normalized_q.x() << ", qy=" << normalized_q.y()
         << ", qz=" << normalized_q.z() << ", yaw_deg=" << ypr[0]
         << ", pitch_deg=" << ypr[1] << ", roll_deg=" << ypr[2]
         << ", feedback_yaw_deg=" << displayed_feedback_yaw_deg
         << ", age_ms=" << std::setprecision(1) << age_ms;
  return stream.str();
}

Gimbal::Gimbal(const std::string & config_path)
{
  const auto yaml = tools::load(config_path);
  const auto config = yaml["gimbal"];
  if (!config || !config.IsMap()) {
    throw std::runtime_error{"Missing 'gimbal' map in YAML configuration."};
  }
  if (!config["port"]) {
    throw std::runtime_error{"Missing 'gimbal.port' in YAML configuration."};
  }

  port_ = config["port"].as<std::string>();
  baudrate_ = read_positive<std::uint32_t>(config, "baudrate", 921600);
  state_.bullet_speed = read_positive<double>(config, "default_bullet_speed", 23.0);
  feedback_log_interval_ = std::chrono::milliseconds{
    read_positive<int>(config, "feedback_log_interval_ms", 200)};
  imu_feedback_timeout_ms_ =
    read_positive<double>(config, "imu_feedback_timeout_ms", 100.0);
  if (config["feedback_delay_us"]) {
    gimbal_feedback_delay_ =
      std::chrono::microseconds{config["feedback_delay_us"].as<long long>()};
  }

  if (port_.empty()) throw std::runtime_error{"gimbal.port must not be empty"};
  if (!reconnect()) {
    throw std::runtime_error{"Failed to open Gimbal serial port: " + port_};
  }
  rx_thread_ = std::thread(&Gimbal::receive_loop, this);
  tools::logger()->info(
    "[Gimbal] serial opened: {} @{}, feedback_delay_us={}", port_, baudrate_,
    gimbal_feedback_delay_.count());
}

Gimbal::~Gimbal()
{
  rx_quit_.store(true, std::memory_order_relaxed);
  if (rx_thread_.joinable()) rx_thread_.join();
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (serial_.isOpen()) serial_.close();
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  auto snapshot = state_;
  snapshot.feedback_valid =
    newest_feedback_timestamp_ != std::chrono::steady_clock::time_point{} &&
    std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - newest_feedback_timestamp_).count() <=
      imu_feedback_timeout_ms_;
  return snapshot;
}

std::optional<std::string> Gimbal::enemy_color_string() const
{
  const auto color = state().enemy_color;
  if (color == EnemyColor::red) return "red";
  if (color == EnemyColor::blue) return "blue";
  return std::nullopt;
}

void Gimbal::set_packet_debug_enabled(bool enabled)
{
  packet_debug_enabled_.store(enabled, std::memory_order_relaxed);
}

bool Gimbal::has_imu_feedback() const
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  return !imu_buffer_.empty();
}

std::optional<GimbalFeedbackSample> Gimbal::gimbal_feedback_at(
  std::chrono::steady_clock::time_point timestamp) const
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  return interpolate_gimbal_feedback(imu_buffer_, timestamp);
}

std::optional<GimbalFeedbackSample> Gimbal::gimbal_feedback_for_image_at(
  std::chrono::steady_clock::time_point image_timestamp) const
{
  return gimbal_feedback_at(image_feedback_query_time(image_timestamp, gimbal_feedback_delay_));
}

Eigen::Quaterniond Gimbal::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  const auto sample = gimbal_feedback_at(timestamp);
  if (sample) return sample->raw_q;
  return Eigen::Quaterniond::Identity();
}

double Gimbal::yaw_at(std::chrono::steady_clock::time_point timestamp)
{
  const auto sample = gimbal_feedback_at(timestamp);
  return sample ? sample->feedback_yaw_deg : 0.0;
}

void Gimbal::send(const Command & command) const
{
  const auto packet = make_control_packet(command);
  try {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (!serial_.isOpen()) {
      tools::logger()->warn("[Gimbal] serial is not open when sending");
      return;
    }
    if (packet_debug_enabled_.load(std::memory_order_relaxed)) {
      const auto now = std::chrono::steady_clock::now();
      if (now - last_tx_debug_log_ >= feedback_log_interval_) {
        last_tx_debug_log_ = now;
        tools::logger()->debug(
          "[Gimbal][TX] mode={} target_id={} target_valid={} yaw={:.6f} pitch={:.6f} "
          "yaw_vel={:.6f} pitch_vel={:.6f}",
          packet.mode, packet.target_id, packet.target_valid, packet.yaw, packet.pitch,
          packet.yaw_vel, packet.pitch_vel);
      }
    }
    serial_.write(reinterpret_cast<const std::uint8_t *>(&packet), sizeof(packet));
  } catch (const std::exception & error) {
    tools::logger()->warn("[Gimbal] serial write failed: {}", error.what());
  }
}

bool Gimbal::reconnect()
{
  constexpr int MAX_RETRY_COUNT = 10;
  for (int attempt = 1; attempt <= MAX_RETRY_COUNT; ++attempt) {
    try {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      if (serial_.isOpen()) serial_.close();
      serial_.setPort(port_);
      serial_.setBaudrate(baudrate_);
      serial_.setFlowcontrol(serial::flowcontrol_none);
      serial_.setParity(serial::parity_none);
      serial_.setStopbits(serial::stopbits_one);
      serial_.setBytesize(serial::eightbits);
      {
        auto timeout = serial::Timeout::simpleTimeout(20);
        serial_.setTimeout(timeout);
      }
      serial_.open();
      usleep(200000);
      return true;
    } catch (const std::exception & error) {
      tools::logger()->warn(
        "[Gimbal] open serial failed ({}/{}): {}", attempt, MAX_RETRY_COUNT, error.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
  return false;
}

void Gimbal::receive_loop()
{
  while (!rx_quit_.load(std::memory_order_relaxed)) {
    try {
      std::vector<std::uint8_t> bytes;
      {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (serial_.isOpen()) {
          const auto available = serial_.available();
          if (available > 0) {
            bytes.resize(available);
            bytes.resize(serial_.read(bytes.data(), bytes.size()));
          }
        }
      }
      if (bytes.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      } else {
        rx_buffer_.insert(rx_buffer_.end(), bytes.begin(), bytes.end());
        parse_rx_buffer();
      }
    } catch (const std::exception & error) {
      tools::logger()->warn("[Gimbal] serial receive failed: {}", error.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
}

void Gimbal::parse_rx_buffer()
{
  constexpr std::uint8_t HEADER[2] = {'i', 'm'};
  while (rx_buffer_.size() >= RX_PACKET_SIZE) {
    const auto header = std::search(
      rx_buffer_.begin(), rx_buffer_.end(), std::begin(HEADER), std::end(HEADER));
    if (header == rx_buffer_.end()) {
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.end() - 1);
      return;
    }
    rx_buffer_.erase(rx_buffer_.begin(), header);
    if (rx_buffer_.size() < RX_PACKET_SIZE) return;

    FeedbackPacket packet{};
    std::memcpy(&packet, rx_buffer_.data(), sizeof(packet));
    const auto decoded = decode_feedback_packet(packet);
    if (!decoded) {
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    const auto received_at = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (state_.mode != decoded->mode) ++state_.mode_generation;
      state_.mode = decoded->mode;
      state_.own_color = decoded->own_color;
      state_.enemy_color = decoded->enemy_color;
      if (decoded->bullet_speed > 0.0) state_.bullet_speed = decoded->bullet_speed;
      state_.feedback_valid = true;
      newest_feedback_timestamp_ = received_at;
    }
    push_imu(decoded->orientation, decoded->yaw_deg, received_at);

    const auto now = std::chrono::steady_clock::now();
    if (packet_debug_enabled_.load(std::memory_order_relaxed) &&
        now - last_rx_debug_log_ >= feedback_log_interval_) {
      last_rx_debug_log_ = now;
      const auto snapshot = state();
      tools::logger()->debug(
        "[Gimbal][RX] mode={} own_color={} bullet_speed={:.3f} {}",
        static_cast<unsigned>(snapshot.mode), static_cast<unsigned>(snapshot.own_color),
        snapshot.bullet_speed, latest_rx_summary());
    }
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + RX_PACKET_SIZE);
  }
}

std::string Gimbal::latest_rx_summary() const
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  if (imu_buffer_.empty()) return "RX none";
  const auto & latest = imu_buffer_.back();
  const auto age_ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - latest.timestamp).count();
  return format_feedback_summary(latest.raw_q, latest.feedback_yaw_deg, age_ms);
}

void Gimbal::push_imu(
  const Eigen::Quaterniond & q, double yaw, std::chrono::steady_clock::time_point timestamp)
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  if (!imu_buffer_.empty()) {
    timestamp = detail::strictly_increasing_feedback_timestamp(
      timestamp, imu_buffer_.back().timestamp);
  }
  imu_buffer_.push_back({q, yaw, timestamp});
  while (imu_buffer_.size() > imu_buffer_max_size_) imu_buffer_.pop_front();
}

}  // namespace io
