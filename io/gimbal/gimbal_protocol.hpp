#ifndef IO_GIMBAL_GIMBAL_PROTOCOL_HPP_
#define IO_GIMBAL_GIMBAL_PROTOCOL_HPP_

#include "io/gimbal/gimbal.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace io::gimbal_protocol
{

uint16_t crc16_x25(const uint8_t * data, size_t len);

std::optional<GimbalState> parse_receive_frame(const std::vector<uint8_t> & bytes);

SendFrame make_send_frame(
  bool control, bool fire, float yaw, float pitch,
  float linear_x = 0.0f, float linear_y = 0.0f, float angular_z = 0.0f);

float decode_angle(uint32_t value);
float decode_chassis_command(uint32_t value);

}  // namespace io::gimbal_protocol

#endif  // IO_GIMBAL_GIMBAL_PROTOCOL_HPP_
