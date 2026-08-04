#include "io/gimbal/gimbal.hpp"
#include "io/gimbal/gimbal_protocol.hpp"
#include "serial/serial.h"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

#include <chrono>
#include <algorithm>
#include <cstring>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{send-fixed    | | 是否发送固定控制值}"
  "{yaw           | 5.0 | 固定 yaw, deg}"
  "{pitch         | -10.0 | 固定 pitch, deg}"
  "{period-ms     | 10 | 固定发送周期, ms}"
  "{@config-path  | configs/QYG_sentry.yaml | yaml配置文件路径}";

namespace
{
uint16_t crc16_x25(const uint8_t * data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : (crc >> 1);
    }
  }
  return crc;
}

uint32_t float_to_uint(float val, float min, float max, int bits)
{
  if (val < min) val = min;
  if (val > max) val = max;
  return static_cast<uint32_t>(
    (val - min) * static_cast<double>((1ULL << bits) - 1) / (max - min));
}

float uint_to_float(uint32_t val, float min, float max, int bits)
{
  return static_cast<float>(val) * (max - min) / static_cast<float>((1ULL << bits) - 1) + min;
}

std::string hex_bytes(const uint8_t * data, size_t len)
{
  std::ostringstream oss;
  oss << std::uppercase << std::hex << std::setfill('0');
  for (size_t i = 0; i < len; ++i) {
    oss << std::setw(2) << static_cast<int>(data[i]);
    if (i + 1 != len) oss << ' ';
  }
  return oss.str();
}

io::SendFrame make_send_frame(bool control, bool fire, float yaw, float pitch)
{
  io::SendFrame frame;
  frame.mode = control ? (fire ? 2 : 1) : 0;
  frame.yaw = float_to_uint(yaw, -M_PI, M_PI, 32);
  frame.pitch = float_to_uint(pitch, -M_PI, M_PI, 32);
  frame.linear_x = float_to_uint(0.0f, -1.0f, 1.0f, 32);
  frame.linear_y = float_to_uint(0.0f, -1.0f, 1.0f, 32);
  frame.angular_z = float_to_uint(0.0f, -1.0f, 1.0f, 32);
  frame.crc16 = crc16_x25(reinterpret_cast<const uint8_t *>(&frame), sizeof(frame) - 2);
  return frame;
}

std::string mode_name(io::GimbalMode mode)
{
  switch (mode) {
    case io::GimbalMode::AUTO_AIM: return "AUTO_AIM";
    case io::GimbalMode::SMALL_BUFF: return "SMALL_BUFF";
    case io::GimbalMode::BIG_BUFF: return "BIG_BUFF";
    default: return "IDLE";
  }
}
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  const auto config_path = cli.get<std::string>("@config-path");
  const bool send_fixed = cli.has("send-fixed");
  const float yaw = cli.get<float>("yaw") * CV_PI / 180.0f;
  const float pitch = cli.get<float>("pitch") * CV_PI / 180.0f;
  const int period_ms = cli.get<int>("period-ms");

  auto yaml = tools::load(config_path);
  const auto port = tools::read<std::string>(yaml, "com_port");

  serial::Serial serial;
  serial.setPort(port);
  serial.setBaudrate(921600);
  serial.setFlowcontrol(serial::flowcontrol_none);
  serial.setParity(serial::parity_none);
  serial.setStopbits(serial::stopbits_one);
  serial.setBytesize(serial::eightbits);
  auto timeout = serial::Timeout::simpleTimeout(20);
  serial.setTimeout(timeout);
  serial.open();

  tools::logger()->info(
    "[RawSerial] opened {}, listen GD frames{}",
    port, send_fixed ? ", and send fixed QY frames" : "");

  tools::Exiter exiter;
  std::vector<uint8_t> rx_buffer;
  auto last_log = std::chrono::steady_clock::now();
  auto last_send = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    if (send_fixed && std::chrono::steady_clock::now() - last_send >= std::chrono::milliseconds(period_ms)) {
      auto frame = make_send_frame(true, true, yaw, pitch);
      serial.write(reinterpret_cast<uint8_t *>(&frame), sizeof(frame));
      last_send = std::chrono::steady_clock::now();

      static auto last_tx_log = std::chrono::steady_clock::now();
      if (last_send - last_tx_log >= 200ms) {
        tools::logger()->info(
          "[QY TX] mode={} yaw={:.2f}deg pitch={:.2f}deg raw={}",
          frame.mode,
          uint_to_float(frame.yaw, -M_PI, M_PI, 32) * 57.3,
          uint_to_float(frame.pitch, -M_PI, M_PI, 32) * 57.3,
          hex_bytes(reinterpret_cast<uint8_t *>(&frame), sizeof(frame)));
        last_tx_log = last_send;
      }
    }

    const auto available = serial.available();
    if (available > 0) {
      std::vector<uint8_t> temp(available);
      serial.read(temp.data(), available);
      rx_buffer.insert(rx_buffer.end(), temp.begin(), temp.end());
    }

    const auto frame_size = sizeof(io::ReceiveFrame);
    while (rx_buffer.size() >= frame_size) {
      const uint8_t header[] = {'G', 'D'};
      auto it = std::search(rx_buffer.begin(), rx_buffer.end(), std::begin(header), std::end(header));
      if (it == rx_buffer.end()) {
        rx_buffer.clear();
        break;
      }
      rx_buffer.erase(rx_buffer.begin(), it);
      if (rx_buffer.size() < frame_size) break;

      io::ReceiveFrame frame;
      std::memcpy(&frame, rx_buffer.data(), frame_size);
      const auto crc_calc = crc16_x25(reinterpret_cast<uint8_t *>(&frame), frame_size - 2);
      const bool crc_ok = crc_calc == frame.crc16;
      const auto sentry_state = frame.sentry_state;
      const auto mode = io::gimbal_protocol::sentry_mode(sentry_state);
      const auto sentry_status = io::gimbal_protocol::sentry_status(sentry_state);
      const auto current_mode = frame.current_mode;
      const auto actual_vx = frame.actual_vx;
      const auto actual_vy = frame.actual_vy;
      const auto actual_wz = frame.actual_wz;
      const auto imu_yaw = frame.imu_yaw;
      const auto imu_pitch = frame.imu_pitch;
      const auto vyaw = frame.vyaw;
      const auto vpitch = frame.vpitch;
      const auto vroll = frame.vroll;

      const auto now = std::chrono::steady_clock::now();
      if (now - last_log >= 200ms) {
        tools::logger()->info(
          "[GD RX] crc={} sentry_state=0x{:04X} status=0x{:04X} mode={} current_mode=0x{:02X} vx={:.3f} vy={:.3f} wz={:.3f} imu_yaw={:.2f} imu_pitch={:.2f} vyaw={:.2f} vpitch={:.2f} vroll={:.2f} raw={}",
          crc_ok ? "OK" : "BAD",
          sentry_state, sentry_status, mode_name(mode),
          current_mode,
          actual_vx, actual_vy, actual_wz,
          imu_yaw, imu_pitch,
          vyaw, vpitch, vroll,
          hex_bytes(reinterpret_cast<uint8_t *>(&frame), frame_size));
        last_log = now;
      }

      rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + frame_size);
    }

    std::this_thread::sleep_for(1ms);
  }

  if (send_fixed) {
    auto stop = make_send_frame(false, false, 0.0f, 0.0f);
    serial.write(reinterpret_cast<uint8_t *>(&stop), sizeof(stop));
  }
  return 0;
}
