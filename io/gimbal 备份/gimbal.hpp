#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <memory>

#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"

namespace io
{

class Aim2Nav; // 前向声明

// --- 接收协议结构 (51字节, GD 帧头) ---
struct __attribute__((packed)) ReceiveFrame {
    uint8_t header[2] = {0x47, 0x44}; // 'G', 'D'
    uint8_t current_mode = 0;       
    float actual_vx = 0.0f;         
    float actual_vy = 0.0f;         
    float actual_wz = 0.0f;         
    float imu_yaw = 0.0f;           
    float imu_pitch = 0.0f;         
    float yaw_angular = 0.0f;       
    float pitch_angular = 0.0f;     
    float odom_x = 0.0f;            
    uint8_t chassis_state = 0;      
    uint8_t mode = 0;               
    float vyaw = 0.0f;              
    float vpitch = 0.0f;            
    float vroll = 0.0f;             
    uint16_t crc16 = 0;
};

// --- 发送协议结构 (QY 帧头) ---
struct __attribute__((packed)) SendFrame {
    uint8_t header[2] = {'Q', 'Y'};
    uint8_t mode = 0;
    float yaw = 0.0f;
    float pitch = 0.0f;
    float linear_x = 0.0f;
    float linear_y = 0.0f;
    float angular_z = 0.0f;
    uint16_t crc16 = 0;
};

enum class GimbalMode
{
  IDLE,        
  AUTO_AIM,    
  SMALL_BUFF,  
  BIG_BUFF     
};

struct GimbalState
{
  float actual_vx = 0;
  float actual_vy = 0;
  float actual_wz = 0;
  float imu_yaw = 0;    
  float imu_pitch = 0;  
  float roll_imu = 0;   
  float yaw_angular = 0;
  float pitch_angular = 0;
  float odom_x = 0;
  uint8_t chassis_state = 0;
  uint8_t mode = 0;

  // 算法兼容性别名
  float yaw_imu = 0; 
  float pitch_imu = 0;
  float bullet_speed = 15.0; 
  float yaw = 0;
  float pitch = 0;
};

class Gimbal
{
public:
  Gimbal(const std::string & config_path);
  ~Gimbal();

  // 关联通讯节点 (Aim2Nav)
  void set_aim2nav(std::shared_ptr<Aim2Nav> aim2nav) { aim2nav_ = aim2nav; }

  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  
  // 插值接口 (弧度)
  Eigen::Vector3d euler(std::chrono::steady_clock::time_point t);

  // 统一发送接口 (QY 帧头)
  void send(bool control, bool fire, float yaw, float pitch, 
            float linear_x = 0, float linear_y = 0, float angular_z = 0);

private:
  serial::Serial serial_;
  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;

  std::vector<uint8_t> rx_buffer_;

  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_;
  
  tools::ThreadSafeQueue<std::pair<Eigen::Vector3d, std::chrono::steady_clock::time_point>>
    queue_{1000};

  std::shared_ptr<Aim2Nav> aim2nav_;

  void read_thread();
  void reconnect();
  
  uint16_t get_crc16(uint8_t* data, uint32_t len);
};

}  // namespace io

#endif  // IO__GIMBAL_HPP