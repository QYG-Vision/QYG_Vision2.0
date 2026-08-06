#include "gimbal.hpp"
#include "io/gimbal/gimbal_protocol.hpp"
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

Gimbal::Gimbal(const std::string & config_path)
{
  // 从配置文件中加载YAML配置并读取串口设备路径
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");

  try {
    // 配置串口通信参数
    serial_.setPort(com_port);           // 设置串口设备路径
    serial_.setBaudrate(921600);         // 设置波特率为921600（高速通信）
    serial_.setFlowcontrol(serial::flowcontrol_none);  // 无流控
    serial_.setParity(serial::parity_none);            // 无奇偶校验
    serial_.setStopbits(serial::stopbits_one);         // 1位停止位
    serial_.setBytesize(serial::eightbits);            // 8位数据位
    
    // 设置串口超时参数（20毫秒超时）
    serial::Timeout time_out = serial::Timeout::simpleTimeout(20);
    serial_.setTimeout(time_out);
    
    // 打开串口连接
    serial_.open();
    
    // 等待1秒确保串口稳定连接
    usleep(1000000); 
  } catch (const std::exception & e) {
    // 串口打开失败时记录错误并退出程序
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  // 启动读取线程，用于持续接收云台数据
  thread_ = std::thread(&Gimbal::read_thread, this);
  
  // 记录云台初始化完成日志
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
  // 使用互斥锁确保线程安全访问模式状态
  // 防止在多线程环境下读取模式时发生数据竞争
  std::lock_guard<std::mutex> lock(mutex_);
  
  // 返回当前云台的工作模式
  // 可能的模式包括：IDLE(空闲)、AUTO_AIM(自动瞄准)、SMALL_BUFF(小符)、BIG_BUFF(大符)
  return mode_;
}

// // 完整的语义信息：
// GimbalMode   Gimbal::mode() const
// │           │         │     │
// │           │         │     └── "我不会修改对象状态"
// │           │         └──────── "我是mode函数"  
// │           └────────────────── "我属于Gimbal类"
// └────────────────────────────── "我返回GimbalMode类型"

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
  auto frame = gimbal_protocol::make_send_frame(
    control, fire, yaw, pitch, linear_x, -linear_y, angular_z);


    // //5.14
    // // 临时调试：每秒打印一次原始发送字节（16进制）
    // {
    //   static auto last_hex_log = std::chrono::steady_clock::now();
    //   auto now_hex = std::chrono::steady_clock::now();
    //   if (std::chrono::duration_cast<std::chrono::milliseconds>(now_hex - last_hex_log).count() >= 1000) {
    //     auto * bytes = reinterpret_cast<const uint8_t*>(&frame);
    //     std::ostringstream oss;
    //     for (size_t i = 0; i < sizeof(frame); ++i)
    //       oss << fmt::format("{:02X} ", bytes[i]);
    //     tools::logger()->info("[Gimbal] TX raw ({}B): {}", sizeof(frame), oss.str());
    //     last_hex_log = now_hex;
    //   }
    // }


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

    const auto parsed_state = gimbal_protocol::parse_receive_frame(
      rx_buffer_.data() + start_idx, target_size);
    if (parsed_state.has_value()) {
      const auto t_now = std::chrono::steady_clock::now();
      const auto latest_state = *parsed_state;

      {
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = latest_state;

        if (state_.mode == 0x11) mode_ = GimbalMode::AUTO_AIM;
        else if (state_.mode == 0x12) mode_ = GimbalMode::SMALL_BUFF;
        else if (state_.mode == 0x13) mode_ = GimbalMode::BIG_BUFF;
        else mode_ = GimbalMode::IDLE;
      }

      const Eigen::Vector3d euler_deg(
        latest_state.vroll, latest_state.vpitch, latest_state.vyaw);
      queue_.push({euler_deg, t_now});

      if (aim2nav_) {
        aim2nav_->publish(latest_state);
      }
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
