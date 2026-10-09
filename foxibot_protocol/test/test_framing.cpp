#include <gtest/gtest.h>

#include "foxibot_protocol/crc.hpp"
#include "foxibot_protocol/framing.hpp"
#include "foxibot_protocol/messages.hpp"

namespace fp = foxibot::protocol;

TEST(Framing, EncodeArmStateV1) {
  fp::ArmStateV1 message{.state = fp::ArmState::ARMED};
  std::uint8_t frame[7];

  std::size_t size = fp::encoder(message, frame, 7);

  EXPECT_EQ(size, 7u);

  EXPECT_EQ(frame[0], 0x00);  // START
  EXPECT_EQ(frame[1], 0x01);  // ARM_STATE
  EXPECT_EQ(frame[2], 0x01);  // VERSION
  EXPECT_EQ(frame[3], 0x01);  // LENGTH
  EXPECT_EQ(frame[4], 0x01);  // ARMED
  EXPECT_EQ(frame[5], 0x54);  // CRC LOW
  EXPECT_EQ(frame[6], 0xE6);  // CRC HIGH
}
TEST(Framing, EncodeHalfAngleServoCommand) {
  fp::ServoCommandV1 message{.servo_id = 3, .sens = 0, .angle = 9000};
  std::uint8_t frame[10];

  std::size_t size = fp::encoder(message, frame, 10);

  EXPECT_EQ(size, 10u);

  EXPECT_EQ(frame[0], 0x00);  // START
  EXPECT_EQ(frame[1], 0x00);  // SERVO_COMMAND
  EXPECT_EQ(frame[2], 0x01);  // VERSION
  EXPECT_EQ(frame[3], 0x04);  // LENGTH
  EXPECT_EQ(frame[4], 0x03);  // SERVO ID
  EXPECT_EQ(frame[5], 0x00);  // SERVO SENS
  EXPECT_EQ(frame[6], 0x28);  // SERVO ANGLE LOW
  EXPECT_EQ(frame[7], 0x23);  // SERVO ANGLE HIGHs
  EXPECT_EQ(frame[8], 0xFA);  // CRC LOW
  EXPECT_EQ(frame[9], 0x3D);  // CRC HIGHs
}
TEST(Framing, EncodeStatusV1) {
  fp::StatusV1 message{.arm_state = fp::ArmState::ARMED,
                       .watchdog_state = fp::WatchdogState::OK,
                       .crc_error_count = 42,
                       .protocol_error_count = 3,
                       .protocol_error_code = fp::ProtocolErrorCode::UNKNOWN_TYPE,
                       .protocol_error_value = 0x42};

  std::uint8_t frame[13];

  std::size_t size = fp::encoder(message, frame, 13);

  EXPECT_EQ(size, 13u);

  EXPECT_EQ(frame[0], fp::START_BYTE);  // START
  EXPECT_EQ(frame[1], 0x03);            // STATUS
  EXPECT_EQ(frame[2], 0x01);            // VERSION
  EXPECT_EQ(frame[3], 0x07);            // LENGTH

  EXPECT_EQ(frame[4], 0x01);  // ARMED
  EXPECT_EQ(frame[5], 0x00);  // WATCHDOG OK

  EXPECT_EQ(frame[6], 0x2A);  // CRC_ERROR_COUNT LOW
  EXPECT_EQ(frame[7], 0x00);  // CRC_ERROR_COUNT HIGH

  EXPECT_EQ(frame[8], 0x03);   // PROTOCOL_ERROR_COUNT
  EXPECT_EQ(frame[9], 0x01);   // UNKNOWN_TYPE
  EXPECT_EQ(frame[10], 0x42);  // offending TYPE value

  EXPECT_EQ(frame[11], 0x51);  // CRC LOW
  EXPECT_EQ(frame[12], 0x88);  // CRC HIGH
}

TEST(Framing, EncodeHeartbeatV1) {
  std::uint8_t frame[6];

  std::size_t size =
      fp::encodeFrame(fp::MessageType::HEARTBEAT, fp::HEARTBEAT_VERSION, nullptr, 0, frame, 6);

  EXPECT_EQ(size, 6u);

  EXPECT_EQ(frame[0], fp::START_BYTE);  // START
  EXPECT_EQ(frame[1], 0x02);            // HEARTBEAT
  EXPECT_EQ(frame[2], 0x01);            // VERSION
  EXPECT_EQ(frame[3], 0x00);            // LENGTH

  EXPECT_EQ(frame[4], 0xCD);  // CRC LOW
  EXPECT_EQ(frame[5], 0x91);  // CRC HIGH
}

TEST(Framing, BufferSizeTest) {
  std::uint8_t frame[5] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
  std::uint8_t payload[1] = {1};
  std::size_t size =
      fp::encodeFrame(fp::MessageType::ARM_STATE, fp::ARM_STATE_VERSION, payload, 1, frame, 5);

  EXPECT_EQ(size, 0u);
  EXPECT_EQ(frame[0], 0xAA);
}