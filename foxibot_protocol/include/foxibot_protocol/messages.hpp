#pragma once
#include <cstdint>

namespace foxibot::protocol {

enum class MessageType : std::uint8_t {
  SERVO_COMMAND = 0,
  ARM_STATE = 1,
  HEARTBEAT = 2,
  STATUS = 3
};

constexpr std::uint8_t SERVO_COMMAND_VERSION = 1;
constexpr std::uint8_t ARM_STATE_VERSION = 1;
constexpr std::uint8_t HEARTBEAT_VERSION = 1;
constexpr std::uint8_t STATUS_VERSION = 1;

enum class ArmState : std::uint8_t { DISARMED = 0, ARMED = 1 };

enum class WatchdogState : std::uint8_t { OK = 0, FAULT = 1 };

enum class ProtocolErrorCode : std::uint8_t { NONE = 0, UNKNOWN_TYPE = 1, UNKNOWN_VERSION = 2 };

struct ServoCommandV1 {
  std::uint8_t servo_id;
  std::uint8_t sens;
  std::int16_t angle;
};

struct ArmStateV1 {
  ArmState state;
};

struct StatusV1 {
  ArmState arm_state;
  WatchdogState watchdog_state;
  std::uint16_t crc_error_count;
  std::uint8_t protocol_error_count;
  ProtocolErrorCode protocol_error_code;
  std::uint8_t protocol_error_value;
};

}  // namespace foxibot::protocol
