#include "foxibot_protocol/framing.hpp"

#include <cstddef>
#include <cstdint>

#include "foxibot_protocol/crc.hpp"
#include "foxibot_protocol/messages.hpp"

namespace foxibot::protocol {
std::size_t encodeFrame(MessageType type, std::uint8_t version, const std::uint8_t * payload,
                        std::uint8_t payload_len, std::uint8_t * frame, std::size_t capacity) {
  const std::size_t frame_size = 6 + static_cast<size_t>(payload_len);
  if (capacity < frame_size) {
    return 0;
  }

  frame[0] = START_BYTE;
  frame[1] = static_cast<std::uint8_t>(type);
  frame[2] = version;
  frame[3] = payload_len;
  for (int i = 0; i < payload_len; i++) {
    frame[4 + i] = payload[i];
  }
  std::uint16_t crc = crc_16(frame + 1, 3 + payload_len);
  frame[4 + payload_len] = static_cast<uint8_t>(crc & 0xFF);
  frame[5 + payload_len] = static_cast<uint8_t>(crc >> 8 & 0xFF);
  return 6 + payload_len;
}

std::size_t encoder(const ArmStateV1 & message, std::uint8_t * frame, std::size_t capacity) {
  std::uint8_t payload[1] = {static_cast<std::uint8_t>(message.state)};
  return encodeFrame(MessageType::ARM_STATE, ARM_STATE_VERSION, payload, 1, frame, capacity);
}
}  // namespace foxibot::protocol