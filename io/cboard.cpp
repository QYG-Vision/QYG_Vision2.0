#include "cboard.hpp"      // CBoard 类声明和协议包结构

#include <algorithm>        // std::clamp，用于 pitch 发送前限幅
#include <stdexcept>       // std::runtime_error，用于配置/串口初始化失败
#include <thread>          // std::this_thread::sleep_for，用于串口重试间隔
#include <unistd.h>        // usleep，用于串口打开后的短暂稳定等待

#include "tools/logger.hpp" // 项目统一日志
#include "tools/yaml.hpp"   // 项目统一 YAML 读取工具

namespace io
{
CBoard::CBoard(const std::string & config_path) // 构造函数：从配置文件初始化 CBoard 串口
: bullet_speed(23.0),                           // 默认弹速；协议当前没有弹速回传
  mode(Mode::auto_aim),                         // 默认模式；协议当前没有模式回传
  shoot_mode(ShootMode::left_shoot),            // 默认射击模式；协议当前没有射击模式回传
  ft_angle(0.0),                                // 默认 FT 角；协议当前没有 FT 角回传
  enemy_color_(EnemyColor::red)                 // 默认敌方颜色为红色，可由 YAML 覆盖
{
  auto yaml = tools::load(config_path); // 读取传入的 YAML 配置

  if (yaml["enemy_color"] && yaml["enemy_color"].as<std::string>() == "blue") { // 配置敌方为蓝色
    enemy_color_.store(EnemyColor::blue, std::memory_order_relaxed);            // 更新敌方颜色
  }

  if (yaml["cboard_com_port"]) {                         // 优先使用 CBoard 专用串口配置
    com_port_ = yaml["cboard_com_port"].as<std::string>(); // 读取 CBoard 串口路径
  } else if (yaml["com_port"]) {                         // 兼容旧配置或共用串口字段
    com_port_ = yaml["com_port"].as<std::string>();       // 读取通用串口路径
  }

  if (yaml["cboard_baudrate"]) {              // 如果配置里指定了波特率
    baudrate_ = yaml["cboard_baudrate"].as<uint32_t>(); // 使用配置波特率
  }

  if (yaml["cboard_default_bullet_speed"]) {                 // 如果配置里指定默认弹速
    bullet_speed = yaml["cboard_default_bullet_speed"].as<double>(); // 使用默认弹速
  }

  if (yaml["cboard_default_mode"]) {                         // 如果配置里指定默认模式
    const auto default_mode = yaml["cboard_default_mode"].as<std::string>(); // 读取模式字符串
    if (default_mode == "idle") mode = Mode::idle;           // idle -> 空闲
    else if (default_mode == "auto_aim") mode = Mode::auto_aim; // auto_aim -> 自瞄
    else if (default_mode == "small_buff") mode = Mode::small_buff; // small_buff -> 小符
    else if (default_mode == "big_buff") mode = Mode::big_buff;     // big_buff -> 大符
    else if (default_mode == "outpost") mode = Mode::outpost;       // outpost -> 前哨站
  }

  if (com_port_.empty()) {                                      // 串口路径不能为空
    throw std::runtime_error("Missing 'cboard_com_port' in YAML configuration."); // 配置错误直接报错
  }

  if (!reconnect()) {                                           // 尝试打开串口
    throw std::runtime_error("Failed to open CBoard serial port: " + com_port_); // 打不开就停止启动
  }

  tools::logger()->info("[CBoard] PC protocol serial opened: {} @{}", com_port_, baudrate_); // 打印串口信息
}

CBoard::~CBoard() // 析构函数：释放串口资源
{
  if (serial_.isOpen()) { // 如果串口还打开着
    serial_.close();      // 关闭串口
  }
}

EnemyColor CBoard::enemy_color() const // 获取敌方颜色枚举
{
  return enemy_color_.load(std::memory_order_relaxed); // 原子读取敌方颜色
}

std::string CBoard::enemy_color_string() const // 获取敌方颜色字符串
{
  return enemy_color() == EnemyColor::red ? "red" : "blue"; // 转成 tracker 需要的字符串
}

Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point) // 兼容旧的 IMU 查询接口
{
  static bool warned = false; // 只提示一次，避免刷日志
  if (!warned) {              // 第一次调用时提示当前协议没有 IMU 回传
    tools::logger()->warn("[CBoard] PC protocol has no IMU feedback; using identity quaternion."); // 说明兜底行为
    warned = true;            // 标记已经提示过
  }
  return Eigen::Quaterniond::Identity(); // 返回单位四元数，保证上层不阻塞
}

void CBoard::send(Command command) const // 将上层控制命令打成协议包并发送
{
  constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0; // 角度转弧度系数
  constexpr double MIN_PITCH = -9.8 * DEG_TO_RAD;               // pitch 下限：-9.8 degree
  constexpr double MAX_PITCH = 22.0 * DEG_TO_RAD;               // pitch 上限：22 degree
  const double limited_pitch = std::clamp(command.pitch, MIN_PITCH, MAX_PITCH); // 发送前保护云台 pitch

  PcControlPacket packet; // 创建 15 字节协议包，默认已带 'p''c' 帧头
  packet.mode = command.control ? (command.shoot ? 2 : 1) : 0; // 控制/开火状态映射到协议 mode
  packet.yaw = static_cast<float>(command.yaw);                // yaw 使用 float32，小端由本机内存布局给出
  packet.pitch = static_cast<float>(limited_pitch);            // pitch 使用限幅后的 float32，小端由本机内存布局给出
  packet.target_id = command.target_id;                        // 目标编号：0=无目标，1-8=有效目标
  packet.target_valid = command.target_valid;                  // 目标有效标志：0=无效，非0=有效
  packet.crc16 = modbus_crc16(                                 // 计算协议要求的 Modbus CRC16
    reinterpret_cast<const uint8_t *>(&packet),                // 将结构体视为连续字节
    sizeof(packet) - sizeof(packet.crc16));                    // CRC 覆盖 byte 0-12，不包含 crc16 本身

  try {                                                        // 串口写操作可能抛异常
    std::lock_guard<std::mutex> lock(serial_mutex_);           // 加锁，避免多线程同时写串口
    if (!serial_.isOpen()) {                                   // 写之前确认串口是否打开
      tools::logger()->warn("[CBoard] serial is not open when sending."); // 串口未打开时记录日志
      return;                                                  // 串口未打开则放弃本帧
    }
    serial_.write(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet)); // 写出完整 15 字节包
  } catch (const std::exception & e) {                         // 捕获串口库异常
    tools::logger()->warn("[CBoard] serial write failed: {}", e.what()); // 打印失败原因
  }
}

uint16_t CBoard::modbus_crc16(const uint8_t * data, uint16_t len) // Modbus CRC16，和协议文档一致
{
  uint16_t crc = 0xFFFF;                         // Modbus CRC 初始值
  for (uint16_t i = 0; i < len; ++i) {           // 遍历每个输入字节
    crc ^= data[i];                              // 当前字节先异或进 CRC 低位
    for (uint8_t j = 0; j < 8; ++j) {            // 每个字节处理 8 个 bit
      if (crc & 0x0001) {                        // 如果最低位为 1
        crc = (crc >> 1) ^ 0xA001;               // 右移并异或反射多项式 0xA001
      } else {                                   // 如果最低位为 0
        crc >>= 1;                               // 只右移一位
      }
    }
  }
  return crc;                                    // 返回小端写入结构体的 CRC 值
}

bool CBoard::reconnect() // 打开串口；失败时重试
{
  constexpr int max_retry_count = 10;                         // 最多重试 10 次
  for (int i = 0; i < max_retry_count; ++i) {                 // 逐次尝试打开串口
    try {                                                    // 串口配置/打开可能抛异常
      std::lock_guard<std::mutex> lock(serial_mutex_);       // 保护串口对象
      if (serial_.isOpen()) {                                // 如果串口此前已经打开
        serial_.close();                                     // 先关闭旧连接
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 给系统一点释放设备时间
      }

      serial_.setPort(com_port_);                            // 设置串口设备路径
      serial_.setBaudrate(baudrate_);                        // 设置波特率，协议默认 921600
      serial_.setFlowcontrol(serial::flowcontrol_none);      // 协议要求无流控
      serial_.setParity(serial::parity_none);                // 协议要求无校验位
      serial_.setStopbits(serial::stopbits_one);             // 协议要求 1 个停止位
      serial_.setBytesize(serial::eightbits);                // 协议要求 8 数据位
      auto timeout = serial::Timeout::simpleTimeout(20);     // 设置短超时，避免写异常长时间卡住
      serial_.setTimeout(timeout);                           // 应用串口超时参数
      serial_.open();                                        // 打开串口
      usleep(200000);                                        // 等待 200ms，让设备稳定
      return true;                                           // 打开成功
    } catch (const std::exception & e) {                     // 捕获打开串口失败
      tools::logger()->warn("[CBoard] open serial failed ({}/{}): {}", i + 1, max_retry_count, e.what()); // 记录失败
      std::this_thread::sleep_for(std::chrono::seconds(1));  // 等 1 秒后重试
    }
  }

  return false; // 多次重试仍失败
}

}  // namespace io
