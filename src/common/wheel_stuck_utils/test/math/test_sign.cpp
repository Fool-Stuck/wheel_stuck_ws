#include "wheel_stuck_utils/math/math.hpp"

#include <gtest/gtest.h>

TEST(sign, sign)
{
  using wheel_stuck_utils::math::sign;

  EXPECT_EQ(sign(0), 0);
  EXPECT_EQ(sign(1), 1);
  EXPECT_EQ(sign(-1), -1);
  EXPECT_EQ(sign(0.1), 1);
  EXPECT_EQ(sign(-0.1), -1);
  EXPECT_EQ(sign(0.0), 0);
  EXPECT_EQ(sign(-0.0), 0);
  EXPECT_EQ(sign(1.0), 1);
}
