#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>  // 提供 Eigen::Quaterniond，用于兼容上层 imu_at() 接口
#include <atomic>          // 提供 std::atomic，用于线程安全保存敌方颜色
#include <chrono>          // 提供 std::chrono::steady_clock::time_point
#include <cstdint>         // 提供 uint8_t/uint16_t/uint32_t 等定长整数
#include <mutex>           // 提供 std::mutex，保护串口写操作
#include <string>          // 提供 std::string，保存串口设备路径
#include <vector>          // 提供 std::vector，保存模式名字表

#include "io/command.hpp"  // 上层传给 CBoard::send() 的控制命令
#include "serial/serial.h" // 串口库，最终通信走这里

namespace io
{

enum Mode
{
  idle,        // 空闲模式
  auto_aim,    // 自瞄模式
  small_buff,  // 小符模式
  big_buff,    // 大符模式
  outpost      // 前哨站模式
};
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

enum class EnemyColor
{
  red,  // 敌方为红色
  blue  // 敌方为蓝色
};

enum ShootMode
{
  left_shoot,   // 左发射机构
  right_shoot,  // 右发射机构
  both_shoot    // 双发射机构
};
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

class CBoard
{
public:
  double bullet_speed;    // 协议当前没有弹速回传，这里使用配置默认值
  Mode mode;              // 协议当前没有模式回传，这里使用配置默认值
  ShootMode shoot_mode;   // 协议当前没有射击模式回传，这里保留旧接口默认值
  double ft_angle;        // 协议当前没有 FT 角回传，这里保留旧接口默认值

  EnemyColor enemy_color() const;         // 返回当前敌方颜色
  std::string enemy_color_string() const; // 返回敌方颜色字符串，给 tracker 使用

  CBoard(const std::string & config_path);  // 从 YAML 初始化串口参数
  ~CBoard();                                // 析构时关闭串口

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp); // 兼容旧 IMU 接口

  void send(Command command) const; // 按 pc_protocol_spec.md 组包并通过串口发送

private:
  struct __attribute__((packed)) PcControlPacket
  {
    uint8_t header[2] = {'p', 'c'}; // byte 0-1：固定帧头 0x70 0x63
    uint8_t mode = 0;               // byte 2：0=不控制，1=控制，2=控制且开火
    float yaw = 0.0f;               // byte 3-6：yaw，float32，小端，单位 rad
    float pitch = 0.0f;             // byte 7-10：pitch，float32，小端，单位 rad
    uint8_t target_id = 0;          // byte 11：目标 ID，0=无目标，1-8=机器人/建筑
    uint8_t target_valid = 0;       // byte 12：目标是否有效，0=无效，非0=有效
    uint16_t crc16 = 0;             // byte 13-14：Modbus CRC16，小端
  };

  static_assert(sizeof(PcControlPacket) == 15); // 协议要求固定 15 字节

  mutable serial::Serial serial_;      // 串口对象；send() 是 const，因此这里需要 mutable
  mutable std::mutex serial_mutex_;    // 防止多个线程同时写串口

  std::string com_port_;               // CBoard 串口设备路径，例如 /dev/cboard
  uint32_t baudrate_ = 921600;         // 协议指定默认波特率 921600
  std::atomic<EnemyColor> enemy_color_; // 敌方颜色，供 detector/tracker 使用

  static uint16_t modbus_crc16(const uint8_t * data, uint16_t len); // 协议指定 CRC 算法

  bool reconnect(); // 打开或重新打开串口
};

}  // namespace io

#endif  // IO__CBOARD_HPP
