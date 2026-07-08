#include <gtest/gtest.h>
#include "foxibot_common/types.hpp"

TEST(Types, JointCountIsThree) {
  EXPECT_EQ(foxibot::kJointsPerLeg, 3u);
}
