#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "io/command.hpp"
#include "io/gimbal_orientation.hpp"
#include "serial/serial.h"

namespace io
{

enum Mode : std::uint8_t
{
  idle = 0,
  auto_aim = 1,
  small_buff = 2,
  big_buff = 3
};

inline const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff"};

enum class OwnColor : std::uint8_t
{
  unknown = 0,
  red = 1,
  blue = 2
};

enum class EnemyColor : std::uint8_t
{
  unknown = 0,
  red = 1,
  blue = 2
};

struct GimbalState
{
  Mode mode = Mode::idle;
  std::uint64_t mode_generation = 0;
  OwnColor own_color = OwnColor::unknown;
  EnemyColor enemy_color = EnemyColor::unknown;
  double bullet_speed = 23.0;
  bool feedback_valid = false;
};

namespace detail
{
inline std::chrono::steady_clock::time_point strictly_increasing_feedback_timestamp(
  std::chrono::steady_clock::time_point timestamp,
  std::chrono::steady_clock::time_point previous_timestamp)
{
  if (timestamp > previous_timestamp) return timestamp;
  return previous_timestamp + std::chrono::steady_clock::duration{1};
}
}  // namespace detail

class Gimbal
{
public:
  static constexpr std::size_t RX_PACKET_SIZE = 30;
  static constexpr std::size_t TX_PACKET_SIZE = 23;

  struct __attribute__((packed)) FeedbackPacket
  {
    std::uint8_t header[2] = {'i', 'm'};
    std::uint8_t mode = 0;
    std::uint8_t own_color = 0;
    float bullet_speed = 0.0F;
    float qw = 1.0F;
    float qx = 0.0F;
    float qy = 0.0F;
    float qz = 0.0F;
    float yaw = 0.0F;
    std::uint16_t crc16 = 0;
  };

  struct __attribute__((packed)) ControlPacket
  {
    std::uint8_t header[2] = {'p', 'c'};
    std::uint8_t mode = 0;
    float yaw = 0.0F;
    float pitch = 0.0F;
    float yaw_vel = 0.0F;
    float pitch_vel = 0.0F;
    std::uint8_t target_id = 0;
    std::uint8_t target_valid = 0;
    std::uint16_t crc16 = 0;
  };

  struct DecodedFeedback
  {
    Mode mode;
    OwnColor own_color;
    EnemyColor enemy_color;
    double bullet_speed;
    Eigen::Quaterniond orientation;
    double yaw_deg;
  };

  static_assert(sizeof(FeedbackPacket) == RX_PACKET_SIZE);
  static_assert(sizeof(ControlPacket) == TX_PACKET_SIZE);

  static std::uint16_t modbus_crc16(const std::uint8_t * data, std::size_t length);
  static ControlPacket make_control_packet(const Command & command);
  static std::optional<DecodedFeedback> decode_feedback_packet(const FeedbackPacket & packet);
  static Eigen::Quaterniond feedback_orientation_to_vision(const Eigen::Quaterniond & electrical_q);
  static double command_pitch_to_electrical(double vision_pitch);
  static double command_pitch_velocity_to_electrical(double vision_pitch_velocity);
  static std::string format_feedback_summary(
    const Eigen::Quaterniond & vision_q, double feedback_yaw_deg, double age_ms);

  explicit Gimbal(const std::string & config_path);
  ~Gimbal();

  Gimbal(const Gimbal &) = delete;
  Gimbal & operator=(const Gimbal &) = delete;

  GimbalState state() const;
  std::optional<std::string> enemy_color_string() const;
  void set_packet_debug_enabled(bool enabled);

  bool has_imu_feedback() const;
  std::optional<GimbalFeedbackSample> gimbal_feedback_at(
    std::chrono::steady_clock::time_point timestamp) const;
  std::optional<GimbalFeedbackSample> gimbal_feedback_for_image_at(
    std::chrono::steady_clock::time_point image_timestamp) const;
  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);
  double yaw_at(std::chrono::steady_clock::time_point timestamp);

  void send(const Command & command) const;

private:
  mutable serial::Serial serial_;
  mutable std::mutex serial_mutex_;
  std::thread rx_thread_;
  std::atomic<bool> rx_quit_{false};
  std::vector<std::uint8_t> rx_buffer_;

  mutable std::mutex state_mutex_;
  GimbalState state_;
  std::chrono::steady_clock::time_point newest_feedback_timestamp_{};
  double imu_feedback_timeout_ms_ = 100.0;

  mutable std::mutex imu_mutex_;
  std::deque<TimedGimbalFeedback> imu_buffer_;
  std::size_t imu_buffer_max_size_ = 500;
  std::chrono::microseconds gimbal_feedback_delay_{0};
  std::atomic<bool> packet_debug_enabled_{false};
  std::chrono::milliseconds feedback_log_interval_{200};
  mutable std::chrono::steady_clock::time_point last_tx_debug_log_{};
  std::chrono::steady_clock::time_point last_rx_debug_log_{};

  std::string port_;
  std::uint32_t baudrate_ = 921600;

  bool reconnect();
  void receive_loop();
  void parse_rx_buffer();
  std::string latest_rx_summary() const;
  void push_imu(
    const Eigen::Quaterniond & q, double yaw,
    std::chrono::steady_clock::time_point timestamp);
};

}  // namespace io

#endif  // IO__GIMBAL_HPP
