#pragma once
#include <cstddef>
#include <cstdint>

namespace foxibot::protocol {
std::uint16_t crc_calcul(const std::uint8_t * data, std::size_t length);
}