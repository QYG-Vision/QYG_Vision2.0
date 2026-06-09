#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>  // 提供 Eigen::Quaterniond，用于兼容上层 imu_at() 接口
#include <atomic>          // 提供 std::atomic，用于线程安全保存敌方颜色
#include <chrono>          // 提供 std::chrono::steady_clock::time_point
#include <cstdint>         // 提供 uint8_t/uint16_t/uint32_t 等定长整数
#include <deque>           // 提供 IMU 环形缓存，供 imu_at() 按时间戳查询
#include <mutex>           // 提供 std::mutex，保护串口写操作
#include <string>          // 提供 std::string，保存串口设备路径
#include <thread>          // 提供串口接收线程
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
  double yaw_at(std::chrono::steady_clock::time_point timestamp);             // 查询同帧下板 yaw

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

  struct __attribute__((packed)) ImuFeedbackPacket
  {
    uint8_t header[2] = {'i', 'm'}; // byte 0-1：固定帧头，表示 IMU feedback
    float qw = 1.0f;                // byte 2-5：四元数 w，float32，小端
    float qx = 0.0f;                // byte 6-9：四元数 x，float32，小端
    float qy = 0.0f;                // byte 10-13：四元数 y，float32，小端
    float qz = 0.0f;                // byte 14-17：四元数 z，float32，小端
    float yaw = 0.0f;               // byte 18-21：下板 yaw 角度，float32，小端
    uint16_t crc16 = 0;             // byte 22-23：Modbus CRC16，小端
  };

  static_assert(sizeof(ImuFeedbackPacket) == 24); // 回传协议固定 24 字节

  struct IMUData
  {
    Eigen::Quaterniond q;                            // 接收到的云台/IMU 姿态四元数
    double yaw = 0.0;                                // 同帧下板 yaw
    std::chrono::steady_clock::time_point timestamp; // 视觉电脑接收到该帧数据的时间
  };

  mutable serial::Serial serial_;      // 串口对象；send() 是 const，因此这里需要 mutable
  mutable std::mutex serial_mutex_;    // 防止多个线程同时写串口
  std::thread rx_thread_;              // 后台串口接收线程，解析四元数 + yaw 回传包
  std::atomic<bool> rx_quit_{false};   // 通知接收线程退出
  std::vector<uint8_t> rx_buffer_;     // 串口流式缓存，用于处理半包/粘包

  mutable std::mutex imu_mutex_;       // 保护 IMU 缓存
  std::deque<IMUData> imu_buffer_;     // 保存最近一段 IMU 数据，供时间戳插值
  size_t imu_buffer_max_size_ = 500;   // 最多缓存 500 帧，避免长时间运行占内存

  std::string com_port_;               // CBoard 串口设备路径，例如 /dev/cboard
  uint32_t baudrate_ = 921600;         // 协议指定默认波特率 921600
  std::atomic<EnemyColor> enemy_color_; // 敌方颜色，供 detector/tracker 使用

  static uint16_t modbus_crc16(const uint8_t * data, uint16_t len); // 协议指定 CRC 算法

  bool reconnect(); // 打开或重新打开串口
  void receive_loop(); // 后台循环读取串口数据
  void parse_rx_buffer(); // 从串口缓存中解析完整四元数 + yaw 包
  std::string latest_rx_summary() const; // 获取最近一次接收数据的日志字符串
  void push_imu(
    const Eigen::Quaterniond & q, double yaw,
    std::chrono::steady_clock::time_point timestamp); // 写入 IMU 缓存
};

}  // namespace io

#endif  // IO__CBOARD_HPP
