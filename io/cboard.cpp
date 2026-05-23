#include "cboard.hpp"

#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>

#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
CBoard::CBoard(const std::string & config_path)
: mode(Mode::idle),
  shoot_mode(ShootMode::left_shoot),
  bullet_speed(0),
  queue_(5000),
  socketcan_(nullptr)
{
  // 读取配置
  auto yaml = tools::load(config_path);
  quaternion_canid_ = tools::read<int>(yaml, "quaternion_canid");
  bullet_speed_canid_ = tools::read<int>(yaml, "bullet_speed_canid");
  send_canid_ = tools::read<int>(yaml, "send_canid");

  std::string can_interface;
  std::string usb2can_device;
  uint32_t can_bitrate = 1000000; // 默认1Mbps


  if (yaml["can_interface"]) {
    can_interface = yaml["can_interface"].as<std::string>();
  }
  // if (yaml["usb2can_device"]) {
  //   usb2can_device = yaml["usb2can_device"].as<std::string>();
  //     // 读取 CAN 波特率配置（如果存在）
  //   if (yaml["can_bitrate"]) {
  //     can_bitrate = yaml["can_bitrate"].as<uint32_t>();
  //   }
  // } else {
  //   // 默认尝试常见的USB2CAN设备路径
  //   // 优先尝试 ttyACM0（达妙科技USB2CAN设备的常见路径）
  //   std::vector<std::string> default_devices = {"/dev/ttyACM0", "/dev/USB2CAN0", "/dev/ttyACM1"};
  //   for (const auto & dev : default_devices) {
  //     if (check_usb2can_available(dev)) {
  //       usb2can_device = dev;
  //       can_bitrate = 1000000;
  //       break;
  //     }
  //   }
  // }
  // 尝试使用SocketCAN
  if (!can_interface.empty() && check_socketcan_available(can_interface)) {
    try {
      socketcan_ = std::make_unique<SocketCAN>(
        can_interface, std::bind(&CBoard::callback, this, std::placeholders::_1));
      tools::logger()->info("[CBoard] Using SocketCAN interface: {}", can_interface);
    } catch (const std::exception & e) {
      tools::logger()->warn("[CBoard] Failed to initialize SocketCAN: {}", e.what());
      socketcan_.reset();
    }
  }

  // // 如果SocketCAN失败，尝试使用达妙USB2CAN
  // if (!socketcan_ && check_usb2can_available(usb2can_device)) {
  //   try {
  //     usb2can_ = std::make_unique<USB2CAN>(
  //       usb2can_device, std::bind(&CBoard::callback, this, std::placeholders::_1), can_bitrate);
  //     use_usb2can_ = true;
  //     tools::logger()->info(
  //       "[CBoard] Using USB2CAN device: {} with bitrate: {} bps", usb2can_device, can_bitrate);
  //   } catch (const std::exception & e) {
  //     tools::logger()->warn("[CBoard] Failed to initialize USB2CAN: {}", e.what());
  //     usb2can_.reset();
  //   }
  // }

  // 如果SocketCAN失败，抛出异常
  if (!socketcan_) {
    throw std::runtime_error(
      "Failed to initialize SocketCAN. Please check your CAN interface configuration.");
  }

  // 注意: callback的运行会早于Cboard构造函数的完成
  tools::logger()->info("[Cboard] Waiting for q...");
  queue_.pop(data_ahead_);
  queue_.pop(data_behind_);
  tools::logger()->info("[Cboard] Opened.");
}

Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  while (true) {
    // 检查队列是否为空，如果为空则定期输出日志提示
    if (queue_.empty()) {
      auto last_log_time = std::chrono::steady_clock::now();
      const auto log_interval = std::chrono::seconds(1);  // 每秒输出一次日志
      
      while (queue_.empty()) {
        auto now = std::chrono::steady_clock::now();
        if (now - last_log_time >= log_interval) {
          tools::logger()->warn(
            "[CBoard] IMU队列为空，等待IMU数据... 请检查CAN通信是否正常");
          last_log_time = now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 每100ms检查一次
      }
    }
    
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }

  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;
  auto t_c = timestamp;
  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = t_c - t_a;

  // 四元数插值
  auto k = t_ac / t_ab;
  Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

  return q_c;
}

void CBoard::send(Command command) const
{
  can_frame frame;
  frame.can_id = send_canid_;
  frame.can_dlc = 8;
  frame.data[0] = (command.control) ? 1 : 0;
  frame.data[1] = (command.shoot) ? 1 : 0;
  frame.data[2] = (int16_t)(command.yaw * 1e4) >> 8;
  frame.data[3] = (int16_t)(command.yaw * 1e4);
  frame.data[4] = (int16_t)(command.pitch * 1e4) >> 8;
  frame.data[5] = (int16_t)(command.pitch * 1e4);
  frame.data[6] = (int16_t)(command.horizon_distance * 1e4) >> 8;
  frame.data[7] = (int16_t)(command.horizon_distance * 1e4);

  // tools::logger()->info("Sending: {} {} {} {} ", frame.data[2], frame.data[3], frame.data[4], frame.data[5]);


  write_can_frame(frame);
}

void CBoard::write_can_frame(const can_frame & frame) const
{
  try {
    if (socketcan_) {
      socketcan_->write(const_cast<can_frame *>(&frame));
    } else {
      tools::logger()->warn("[CBoard] No CAN interface available for writing!");
    }
  } catch (const std::exception & e) {
    tools::logger()->warn("[CBoard] Write failed: {}", e.what());
  }
}

float CBoard::uint_to_float(int x_int,float x_min,float x_max,int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1<<bits) - 1)) + offset;
}

void CBoard::callback(const can_frame & frame)
{
  // 只处理四元数 CAN ID，其他 CAN ID 直接忽略
  if (frame.can_id != quaternion_canid_) {
    return;
  }
  
  auto timestamp = std::chrono::steady_clock::now();
  

  // if (frame.can_id == quaternion_canid_) {
    // auto x = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e4;
    // auto y = (int16_t)(frame.data[2] << 8 | frame.data[3]) / 1e4;
    // auto z = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e4;
    // auto w = (int16_t)(frame.data[6] << 8 | frame.data[7]) / 1e4;
  //   auto x = ((int16_t)(frame.data[0] << 8 | frame.data[1]) - 8192 )/ 16284.00;
  //   auto y = ((int16_t)(frame.data[2] << 8 | frame.data[3]) - 8192 )/ 16284.00;
  //   auto z = ((int16_t)(frame.data[4] << 8 | frame.data[5]) - 8192 )/ 16284.00;
  //   auto w = ((int16_t)(frame.data[6] << 8 | frame.data[7]) - 8192 )/ 16284.00;

  //   if (std::abs(x * x + y * y + z * z + w * w - 1) > 1e-2) {
  //     tools::logger()->warn("Invalid q: {} {} {} {}", w, x, y, z);
  //     // tools::logger()->warn("Invalid q: {} {} {} {} {} {} {} {}", frame.data[0]),frame.data[1],
  //     //    frame.data[2], frame.data[3], frame.data[4], frame.data[5], frame.data[6], frame.data[7];
  //     return;
  //   }

  if (frame.can_id == quaternion_canid_) {
    auto w = ((uint16_t)(frame.data[0] << 8 | frame.data[1]));
    auto x = ((uint16_t)(frame.data[2] << 8 | frame.data[3]));
    auto y = ((uint16_t)(frame.data[4] << 8 | frame.data[5]));
    auto z = ((uint16_t)(frame.data[6] << 8 | frame.data[7]));
    double x_d = static_cast<double>(uint_to_float(x, q_min, q_max, 16));
    double y_d = static_cast<double>(uint_to_float(y, q_min, q_max, 16));
    double z_d = static_cast<double>(uint_to_float(z, q_min, q_max, 16));
    double w_d = static_cast<double>(uint_to_float(w, q_min, q_max, 16));


    if (std::abs(x_d * x_d + y_d * y_d + z_d * z_d + w_d * w_d - 1) > 1e-2) {
      tools::logger()->warn("Invalid q: {} {} {} {}", w_d, x_d, y_d, z_d);
      return;
    }
    // tools::logger()->info("Invalid q: {} {} {} {}", w_d, x_d, y_d, z_d);
    // return;

    queue_.push({{w_d, x_d, y_d, z_d}, timestamp});

  }

  // else if (frame.can_id == bullet_speed_canid_) {
  //   bullet_speed = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e2;
  //   mode = Mode(frame.data[2]);
  //   shoot_mode = ShootMode(frame.data[3]);
  //   ft_angle = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e4;

  //   // 限制日志输出频率为1Hz
  //   static auto last_log_time = std::chrono::steady_clock::time_point::min();
  //   auto now = std::chrono::steady_clock::now();

  //   if (bullet_speed > 0 && tools::delta_time(now, last_log_time) >= 1.0) {
  //     tools::logger()->info(
  //       "[CBoard] Bullet speed: {:.2f} m/s, Mode: {}, Shoot mode: {}, FT angle: {:.2f} rad",
  //       bullet_speed, MODES[mode], SHOOT_MODES[shoot_mode], ft_angle);
  //     last_log_time = now;
  //   }
  // }
    bullet_speed = 15;
    mode = Mode(1);
    shoot_mode = ShootMode(1);
    ft_angle = 0;

    // tools::logger()->info(
    //   "[CBoard] Bullet speed: {:.2f} m/s, Mode: {}, Shoot mode: {}, FT angle: {:.2f} rad",
    //   bullet_speed, MODES[mode], SHOOT_MODES[shoot_mode], ft_angle);
}

bool CBoard::check_socketcan_available(const std::string & interface)
{
  if (interface.empty()) {
    return false;
  }

  int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock < 0) {
    return false;
  }

  ifreq ifr;
  std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
  bool available = (ioctl(sock, SIOCGIFINDEX, &ifr) >= 0);
  
  ::close(sock);
  return available;
}

// 实现方式有待改进
std::string CBoard::read_yaml(const std::string & config_path)
{
  auto yaml = tools::load(config_path);

  quaternion_canid_ = tools::read<int>(yaml, "quaternion_canid");
  bullet_speed_canid_ = tools::read<int>(yaml, "bullet_speed_canid");
  send_canid_ = tools::read<int>(yaml, "send_canid");

  if (!yaml["can_interface"] && !yaml["usb2can_device"]) {
    throw std::runtime_error(
      "Missing 'can_interface' or 'usb2can_device' in YAML configuration.");
  }

  if (yaml["can_interface"]) {
    return yaml["can_interface"].as<std::string>();
  }
  return "";
}

}  // namespace io