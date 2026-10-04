#include <gtest/gtest.h>

#include <cstdint>

#include "foxibot_protocol/crc.hpp"

namespace fp = foxibot::protocol;

std::uint8_t data[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};

TEST(CRC, ValidCrc) { EXPECT_EQ(fp::crc_16(data, 9), 0x29B1); }