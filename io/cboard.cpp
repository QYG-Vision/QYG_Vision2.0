#include "cboard.hpp"      // CBoard 类声明和协议包结构

#include <algorithm>        // std::clamp，用于 pitch 发送前限幅
#include <cmath>            // std::abs，用于检查四元数模长
#include <cstring>          // std::memcpy，用于从字节流安全拷贝协议包
#include <iomanip>          // std::setw/std::setfill，用于十六进制打印
#include <sstream>          // std::ostringstream，用于拼接发送字节日志
#include <stdexcept>       // std::runtime_error，用于配置/串口初始化失败
#include <thread>          // std::this_thread::sleep_for，用于串口重试间隔
#include <unistd.h>        // usleep，用于串口打开后的短暂稳定等待

#include "tools/logger.hpp" // 项目统一日志
#include "tools/yaml.hpp"   // 项目统一 YAML 读取工具

namespace io
{
namespace
{
std::string to_hex_string(const uint8_t * data, size_t size)
{
  std::ostringstream stream;
  stream << std::hex << std::uppercase << std::setfill('0');
  for (size_t i = 0; i < size; ++i) {
    if (i != 0) stream << ' ';
    stream << std::setw(2) << static_cast<int>(data[i]);
  }
  return stream.str();
}
}  // namespace

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

  rx_thread_ = std::thread(&CBoard::receive_loop, this);         // 打开串口后启动回传接收线程

  tools::logger()->info("[CBoard] PC protocol serial opened: {} @{}", com_port_, baudrate_); // 打印串口信息
}

CBoard::~CBoard() // 析构函数：释放串口资源
{
  rx_quit_.store(true, std::memory_order_relaxed); // 通知接收线程停止
  if (rx_thread_.joinable()) {                     // 如果接收线程已经启动
    rx_thread_.join();                             // 等待线程安全退出
  }
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

Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp) // 按图像时间戳查询 IMU
{
  std::lock_guard<std::mutex> lock(imu_mutex_); // 读取缓存时加锁，避免和接收线程同时修改

  if (imu_buffer_.empty()) { // 如果还没有收到任何四元数
    static bool warned = false; // 只提示一次，避免刷日志
    if (!warned) {              // 第一次调用时提示当前还没有 IMU 回传
      tools::logger()->warn("[CBoard] no IMU feedback received yet; using identity quaternion."); // 说明兜底行为
      warned = true;            // 标记已经提示过
    }
    return Eigen::Quaterniond::Identity(); // 没有数据时返回单位四元数，保证上层不阻塞
  }

  if (imu_buffer_.size() == 1 || timestamp <= imu_buffer_.front().timestamp) { // 缓存不足或请求时间早于缓存
    return imu_buffer_.front().q; // 返回最早一帧作为兜底
  }

  if (timestamp >= imu_buffer_.back().timestamp) { // 请求时间晚于最新 IMU
    return imu_buffer_.back().q; // 返回最新一帧作为兜底
  }

  for (size_t i = 1; i < imu_buffer_.size(); ++i) { // 查找时间戳前后两帧
    if (imu_buffer_[i].timestamp >= timestamp) {    // 找到第一帧不早于请求时间的 IMU
      const auto & data_a = imu_buffer_[i - 1];     // 请求时间之前的 IMU
      const auto & data_b = imu_buffer_[i];         // 请求时间之后的 IMU
      std::chrono::duration<double> t_ab = data_b.timestamp - data_a.timestamp; // 两帧间隔
      std::chrono::duration<double> t_ac = timestamp - data_a.timestamp;        // 请求点偏移
      double k = t_ab.count() > 1e-9 ? t_ac.count() / t_ab.count() : 0.0;       // 插值比例，避免除零
      return data_a.q.slerp(k, data_b.q).normalized(); // 四元数球面插值，保持旋转连续
    }
  }

  return imu_buffer_.back().q; // 理论上走不到这里，保底返回最新值
}

double CBoard::yaw_at(std::chrono::steady_clock::time_point timestamp) // 按图像时间戳查询下板 yaw
{
  std::lock_guard<std::mutex> lock(imu_mutex_); // yaw 和 IMU 同帧缓存，使用同一把锁

  if (imu_buffer_.empty()) { // 如果还没有收到任何回传
    static bool warned = false; // 只提示一次，避免刷日志
    if (!warned) {
      tools::logger()->warn("[CBoard] no yaw feedback received yet; using 0.");
      warned = true;
    }
    return 0.0;
  }

  if (imu_buffer_.size() == 1 || timestamp <= imu_buffer_.front().timestamp) {
    return imu_buffer_.front().yaw;
  }

  if (timestamp >= imu_buffer_.back().timestamp) {
    return imu_buffer_.back().yaw;
  }

  for (size_t i = 1; i < imu_buffer_.size(); ++i) {
    if (imu_buffer_[i].timestamp >= timestamp) {
      const auto & data_a = imu_buffer_[i - 1];
      const auto & data_b = imu_buffer_[i];
      std::chrono::duration<double> t_ab = data_b.timestamp - data_a.timestamp;
      std::chrono::duration<double> t_ac = timestamp - data_a.timestamp;
      const double k = t_ab.count() > 1e-9 ? t_ac.count() / t_ab.count() : 0.0;
      return data_a.yaw + (data_b.yaw - data_a.yaw) * k;
    }
  }

  return imu_buffer_.back().yaw;
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

    const auto * packet_bytes = reinterpret_cast<const uint8_t *>(&packet);
    const float packet_yaw = packet.yaw;
    const float packet_pitch = packet.pitch;
    tools::logger()->info(
      "[CBoard][TX] mode={}, yaw={:.6f}, pitch={:.6f}, target_id={}, target_valid={}, bytes={}",
      static_cast<int>(packet.mode), packet_yaw, packet_pitch, static_cast<int>(packet.target_id),
      static_cast<int>(packet.target_valid), to_hex_string(packet_bytes, sizeof(packet)));

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

void CBoard::receive_loop() // 串口接收线程：接收并解析 IMU 四元数 + 下板 yaw
{
  while (!rx_quit_.load(std::memory_order_relaxed)) {          // 未收到退出信号就持续运行
    try {                                                      // 串口读取可能抛异常
      std::vector<uint8_t> bytes;                              // 临时保存本轮读到的字节
      {
        std::lock_guard<std::mutex> lock(serial_mutex_);       // 和 send() 共用串口对象，读写前加锁
        if (serial_.isOpen()) {                                // 串口打开时才读取
          const size_t available = serial_.available();        // 查询当前可读字节数
          if (available > 0) {                                 // 有数据才真正 read，避免长时间阻塞发送
            bytes.resize(available);                           // 按可读长度分配缓存
            const size_t read_size = serial_.read(bytes.data(), bytes.size()); // 读出当前可读数据
            bytes.resize(read_size);                           // read 可能少于 requested，按实际长度截断
          }
        }
      }

      if (!bytes.empty()) {                                    // 本轮确实读到了数据
        rx_buffer_.insert(rx_buffer_.end(), bytes.begin(), bytes.end()); // 追加到流式缓存
        parse_rx_buffer();                                     // 尝试从缓存里解析完整协议包
      } else {                                                 // 没有数据时不要空转
        std::this_thread::sleep_for(std::chrono::milliseconds(2)); // 短暂休眠降低 CPU 占用
      }
    } catch (const std::exception & e) {                       // 捕获串口读取异常
      tools::logger()->warn("[CBoard] serial receive failed: {}", e.what()); // 打印异常
      std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 异常后稍微等一下再试
    }
  }
}

void CBoard::parse_rx_buffer() // 解析串口流中的 IMU + yaw 回传包
{
  constexpr size_t PACKET_SIZE = sizeof(ImuFeedbackPacket);    // IMU + yaw 回传包固定 24 字节
  constexpr uint8_t HEADER[2] = {'i', 'm'};                    // IMU 回传包帧头

  while (rx_buffer_.size() >= PACKET_SIZE) {                   // 缓存里至少有一个完整包长度才解析
    auto header_it = std::search(                              // 在缓存里搜索帧头 'i''m'
      rx_buffer_.begin(), rx_buffer_.end(),                    // 搜索范围：整个接收缓存
      std::begin(HEADER), std::end(HEADER));                   // 模板帧头

    if (header_it == rx_buffer_.end()) {                       // 如果没有找到帧头
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.end() - 1); // 保留最后 1 字节，防止半个帧头被丢
      return;                                                  // 等待后续字节
    }

    rx_buffer_.erase(rx_buffer_.begin(), header_it);           // 丢弃帧头之前的杂散字节
    if (rx_buffer_.size() < PACKET_SIZE) return;               // 找到帧头但包还不完整，等待下次读取

    ImuFeedbackPacket packet;                                  // 创建协议包对象
    std::memcpy(&packet, rx_buffer_.data(), PACKET_SIZE);      // 从字节缓存复制出完整包

    const auto crc = modbus_crc16(                             // 计算收到数据的 CRC
      reinterpret_cast<const uint8_t *>(&packet),              // 将包视为连续字节
      sizeof(packet) - sizeof(packet.crc16));                  // CRC 不包含最后两个 CRC 字节

    if (crc != packet.crc16) {                                 // CRC 不一致说明包损坏或误同步
      rx_buffer_.erase(rx_buffer_.begin());                    // 丢掉一个字节，继续寻找下一个可能帧头
      continue;                                                // 继续解析缓存
    }

    Eigen::Quaterniond q(packet.qw, packet.qx, packet.qy, packet.qz); // 按 w,x,y,z 构造四元数
    const double norm_error = std::abs(q.squaredNorm() - 1.0); // 检查四元数模长是否接近 1
    if (norm_error > 1e-2) {                                   // 模长偏差过大说明数据不可信
      const double qw = packet.qw;                              // packed 字段先复制出来，避免引用未对齐字段
      const double qx = packet.qx;                              // packed 字段先复制出来，避免引用未对齐字段
      const double qy = packet.qy;                              // packed 字段先复制出来，避免引用未对齐字段
      const double qz = packet.qz;                              // packed 字段先复制出来，避免引用未对齐字段
      tools::logger()->warn(                                  // 打印一次异常四元数内容
        "[CBoard] invalid IMU quaternion: {:.4f} {:.4f} {:.4f} {:.4f}",
        qw, qx, qy, qz);
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + PACKET_SIZE); // 丢弃该坏包
      continue;                                                // 继续尝试解析后续数据
    }

    push_imu(q.normalized(), packet.yaw, std::chrono::steady_clock::now()); // 使用视觉电脑接收时刻作为时间戳
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + PACKET_SIZE); // 移除已解析的数据
  }
}

void CBoard::push_imu(
  const Eigen::Quaterniond & q, double yaw, std::chrono::steady_clock::time_point timestamp)
{
  std::lock_guard<std::mutex> lock(imu_mutex_);                // 写缓存时加锁
  imu_buffer_.push_back({q, yaw, timestamp});                  // 写入最新四元数和 yaw
  while (imu_buffer_.size() > imu_buffer_max_size_) {          // 超过缓存上限时
    imu_buffer_.pop_front();                                   // 丢掉最旧数据
  }
}

}  // namespace io
