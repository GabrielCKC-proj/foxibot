#pragma once

#include <cstddef>
#include <cstdint>

#include "foxibot_protocol/messages.hpp"

namespace foxibot::protocol {
constexpr std::uint8_t START_BYTE = 0x00;

std::size_t encodeFrame(MessageType type, std::uint8_t version, const std::uint8_t * payload,
                        std::uint8_t payload_len, std::uint8_t * frame, std::size_t capacity);

std::size_t encoder(const ArmStateV1 & message, std::uint8_t * frame, std::size_t capacity);
std::size_t encoder(const ServoCommandV1 & message, std::uint8_t * frame, std::size_t capacity);
std::size_t encoder(const StatusV1 & message, std::uint8_t * frame, std::size_t capacity);

}  // namespace foxibot::protocol