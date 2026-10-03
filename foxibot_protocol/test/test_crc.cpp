#include <gtest/gtest.h>

#include "foxibot_protocol/crc.hpp"

namespace fp = foxibot::protocol;
TEST(CRC, ValidCrc) { EXPECT_EQ(fp::crc_calcul("123456789", 9), 0x29B1); }