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

uint16_t pack_sentry_state(uint16_t status, GimbalMode mode);
uint16_t sentry_status(uint16_t sentry_state);
GimbalMode sentry_mode(uint16_t sentry_state);

// Parse EC wire bytes and return angles in the canonical vision convention.
std::optional<GimbalState> parse_receive_frame(const uint8_t * bytes, std::size_t size);

// Compatibility overload; delegates to the pointer/size parser.
std::optional<GimbalState> parse_receive_frame(const std::vector<uint8_t> & bytes);

SendFrame make_send_frame(
  bool control, bool fire, float yaw, float pitch,
  float linear_x = 0.0f, float linear_y = 0.0f, float angular_z = 0.0f);

}  // namespace io::gimbal_protocol

#endif  // IO_GIMBAL_GIMBAL_PROTOCOL_HPP_
