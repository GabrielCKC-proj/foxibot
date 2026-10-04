#include <cstddef>
#include <cstdint>

namespace foxibot::protocol {
std::uint16_t crc_16(const std::uint8_t * data, std::size_t length) {
  std::uint16_t crc = 0xFFFF;

  for (std::size_t i = 0; i < length; i++) {
    crc ^= data[i] << 8;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = crc << 1;
        crc ^= 0x1021;
      } else {
        crc = crc << 1;
      }
    }
  }
  return crc & 0xFFFF;
}
}  // namespace foxibot::protocol