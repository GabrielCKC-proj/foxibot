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