#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
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
    uint8_t header[2] = {0x47, 0x44}; // 'G', 'D'//1-2
    uint8_t current_mode = 0; //导航mode       //3
    float actual_vx = 0.0f;      //底盘的线速度   //4-7
    float actual_vy = 0.0f;       //底盘的线速度   //8-11
    float actual_wz = 0.0f;      //底盘的角速度   //12-15
    float imu_yaw = 0.0f;          //底盘的yaw角度   //16-19
    float imu_pitch = 0.0f;         //底盘的pitch角度   //20-23
    float yaw_angular = 0.0f;       //底盘的yaw加速度   //24-27
    float pitch_angular = 0.0f;     //底盘的pitch加速度   //28-31
    float odom_x = 0.0f;           //里程计累计值   //32-35
    uint16_t sentry_state = 0;      // 视觉模式(高2位) + 哨兵状态(低14位) //35-36
    float vyaw = 0.0f;       //云台的yaw角度       //38-41
    float vpitch = 0.0f;      // 电控线路原始云台 Pitch，degree //42-45
    float vroll = 0.0f;             //云台的roll角度       //46-49
    uint16_t crc16 = 0;//50-51
};

static_assert(sizeof(ReceiveFrame) == 51, "ReceiveFrame must stay 51 bytes for EC GD protocol");
static_assert(offsetof(ReceiveFrame, sentry_state) == 35, "GD sentry_state offset must stay byte 35");
static_assert(offsetof(ReceiveFrame, vyaw) == 37, "GD vyaw offset must stay byte 37");
static_assert(offsetof(ReceiveFrame, vpitch) == 41, "GD vpitch offset must stay byte 41");
static_assert(offsetof(ReceiveFrame, vroll) == 45, "GD vroll offset must stay byte 45");
static_assert(offsetof(ReceiveFrame, crc16) == 49, "GD CRC16 offset must stay byte 49");

// --- 发送协议结构 (QY 帧头) ---
struct __attribute__((packed)) SendFrame {
    uint8_t header[2] = {'Q', 'Y'};  // offsets 0-1
    uint8_t mode = 0;                // offset 2
    float yaw = 0.0f;                // offsets 3-6, little-endian IEEE-754 float32
    float pitch = 0.0f;              // offsets 7-10, little-endian IEEE-754 float32
    float linear_x = 0.0f;           // offsets 11-14, little-endian IEEE-754 float32
    float linear_y = 0.0f;           // offsets 15-18, little-endian IEEE-754 float32
    float angular_z = 0.0f;          // offsets 19-22, little-endian IEEE-754 float32
    uint16_t crc16 = 0;              // offsets 23-24
};

static_assert(sizeof(float) == 4, "QY protocol requires 32-bit float");
static_assert(std::numeric_limits<float>::is_iec559, "QY protocol requires IEEE-754 float");
static_assert(sizeof(SendFrame) == 25, "SendFrame must stay 25 bytes for EC QY protocol");
static_assert(offsetof(SendFrame, yaw) == 3, "QY yaw offset must stay byte 3");
static_assert(offsetof(SendFrame, pitch) == 7, "QY pitch offset must stay byte 7");
static_assert(offsetof(SendFrame, linear_x) == 11, "QY linear_x offset must stay byte 11");
static_assert(offsetof(SendFrame, linear_y) == 15, "QY linear_y offset must stay byte 15");
static_assert(offsetof(SendFrame, angular_z) == 19, "QY angular_z offset must stay byte 19");
static_assert(offsetof(SendFrame, crc16) == 23, "QY CRC16 offset must stay byte 23");

enum class GimbalMode : uint8_t
{
  IDLE = 0b00,
  AUTO_AIM = 0b01,
  SMALL_BUFF = 0b10,
  BIG_BUFF = 0b11
};

struct GimbalState
{
  uint8_t current_mode = 0;
  float actual_vx = 0;
  float actual_vy = 0;
  float actual_wz = 0;
  float imu_yaw = 0;    
  float imu_pitch = 0;  
  float roll_imu = 0;   
  float yaw_angular = 0;
  float pitch_angular = 0;
  float odom_x = 0;
  uint16_t sentry_state = 0;
  float vyaw = 0;
  float vpitch = 0;  // 视觉统一符号的云台 Pitch，degree；等于 -ReceiveFrame::vpitch
  float vroll = 0;

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
  
  // 插值接口：返回视觉统一符号的 {roll, pitch, yaw}，单位 degree
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
  
  tools::ThreadSafeQueue<std::pair<Eigen::Vector3d, std::chrono::steady_clock::time_point>, true>
    queue_{10};

  std::shared_ptr<Aim2Nav> aim2nav_;

  void read_thread();
  void reconnect();
};

}  // namespace io

#endif  // IO__GIMBAL_HPP
