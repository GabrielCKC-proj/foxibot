#pragma once
#include <cstdint>

namespace foxibot::protocol {

enum class MessageType : std::uint8_t { SERVO_COMMAND, ARM_STATE, HEART_BEAT, STATUS };

constexpr std::uint8_t SERVO_COMMAND_VERSION = 1;
constexpr std::uint8_t ARM_STATE_VERSION = 1;
constexpr std::uint8_t HEARTBEAT = 1;
constexpr std::uint8_t STATUS = 1;

struct servo_command {
  std::uint8_t id_serv;
  std::uint8_t sens;
  std::int16_t angle;
};

struct arm_state {
  std::uint8_t arm_state;
};

struct heartbeat {};

struct status {
  std::uint8_t arm_state;
  std::uint8_t watchdog_state;
  std::uint16_t crc_error_count;
  std::uint8_t protocol_error_count;
  std::uint8_t protocol_error_code;
  std::uint8_t protocol_error_value;
};

}  // namespace foxibot::protocol
