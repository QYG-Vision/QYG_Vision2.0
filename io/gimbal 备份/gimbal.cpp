#include "gimbal.hpp"
#include "io/ros2/aim2nav.hpp"

#include <iomanip>
#include <sstream>
#include <cstring>
#include <cmath>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{

static const uint16_t crc16_x25_table[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

uint16_t Gimbal::get_crc16(uint8_t* data, uint32_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc = (crc >> 8) ^ crc16_x25_table[(crc ^ *data++) & 0xFF];
    }
    return crc ^ 0xFFFF;
}

Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");

  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(921600);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(20);
    serial_.setTimeout(time_out);
    serial_.open();
    usleep(1000000); 
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);
  tools::logger()->info("[Gimbal] Initialized.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE: return "IDLE";
    case GimbalMode::AUTO_AIM: return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF: return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF: return "BIG_BUFF";
    default: return "INVALID";
  }
}

Eigen::Vector3d Gimbal::euler(std::chrono::steady_clock::time_point t)
{
  while (true) {
    if (queue_.empty()) return Eigen::Vector3d::Zero();
    auto [euler_a, t_a] = queue_.pop();
    if (queue_.empty()) return euler_a;
    auto [euler_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = (t_ab < 1e-6) ? 0.0 : (t_ac / t_ab);
    
    Eigen::Vector3d euler_c = euler_a + k * (euler_b - euler_a);
    
    if (t < t_a) return euler_c;
    if (!(t_a < t && t <= t_b)) continue;
    
    return euler_c;
  }
}

void Gimbal::send(bool control, bool fire, float yaw, float pitch, float linear_x, float linear_y, float angular_z)
{
  SendFrame frame;
  auto to_byte = [](float val) -> uint8_t {
    uint8_t b;
    std::memcpy(&b, &val, 1);
    return b;
  };

  frame.mode = (uint8_t)control ? (fire ? 2 : 1) : 0;
  frame.yaw = to_byte(yaw);
  frame.pitch = to_byte(pitch);
  frame.linear_x = to_byte(linear_x);
  frame.linear_y = to_byte(linear_y);
  frame.angular_z = to_byte(angular_z);
  frame.crc16 = get_crc16(reinterpret_cast<uint8_t*>(&frame), sizeof(frame) - 2);

  try {
    serial_.write(reinterpret_cast<uint8_t*>(&frame), sizeof(frame));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Send failed: {}", e.what());
  }
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread running.");
  const size_t target_size = sizeof(ReceiveFrame);

  while (!quit_) {
    try {
        size_t available = serial_.available();
        if (available > 0) {
            std::vector<uint8_t> temp(available);
            serial_.read(temp.data(), available);
            rx_buffer_.insert(rx_buffer_.end(), temp.begin(), temp.end());
        }
    } catch (const std::exception & e) {
        tools::logger()->warn("[Gimbal] Serial error: {}", e.what());
        reconnect();
        continue;
    }

    size_t start_idx = (size_t)-1;
    for (size_t i = 0; i < rx_buffer_.size(); ++i) {
        if (i + 1 < rx_buffer_.size() && rx_buffer_[i] == 0x47 && rx_buffer_[i+1] == 0x44) {
            start_idx = i;
            break;
        }
    }

    if (start_idx == (size_t)-1) {
        if (rx_buffer_.size() > 1024) rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.end() - 100);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
    }

    if (rx_buffer_.size() < start_idx + target_size) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
    }

    ReceiveFrame rx;
    std::memcpy(&rx, &rx_buffer_[start_idx], target_size);
    
    if (get_crc16(reinterpret_cast<uint8_t*>(&rx), target_size - 2) == rx.crc16) {
        auto t_now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(mutex_);
        
        auto from_byte = [](uint8_t b) -> float {
            float f = 0.0f;
            std::memcpy(&f, &b, 1);
            return f;
        };

        state_.actual_vx = from_byte(rx.actual_vx);
        state_.actual_vy = from_byte(rx.actual_vy);
        state_.actual_wz = from_byte(rx.actual_wz);
        state_.imu_yaw = from_byte(rx.imu_yaw);
        state_.imu_pitch = from_byte(rx.imu_pitch);
        state_.roll_imu = from_byte(rx.vroll); 
        state_.yaw_angular = from_byte(rx.yaw_angular);
        state_.pitch_angular = from_byte(rx.pitch_angular);
        state_.odom_x = from_byte(rx.odom_x);
        state_.chassis_state = rx.chassis_state;
        state_.mode = rx.mode;
        
        // 别名映射
        state_.yaw_imu = state_.imu_yaw;
        state_.pitch_imu = state_.imu_pitch;

        if (rx.mode == 1) mode_ = GimbalMode::AUTO_AIM;
        else if (rx.mode == 2) mode_ = GimbalMode::SMALL_BUFF;
        else if (rx.mode == 3) mode_ = GimbalMode::BIG_BUFF;
        else mode_ = GimbalMode::IDLE;

        
        Eigen::Vector3d euler_rad(rx.vroll * M_PI / 180.0, 
                                 rx.vpitch * M_PI / 180.0, 
                                 -rx.vyaw * M_PI / 180.0);
        queue_.push({euler_rad, t_now});

    }

    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + start_idx + target_size);
  }
}

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting... {}/{}", i + 1, max_retry_count);
    try {
      if (serial_.isOpen()) serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
      serial_.open();
      break;
    } catch (...) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io