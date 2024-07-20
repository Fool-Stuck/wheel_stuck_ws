#include "wheel_stuck_utils/math/math.hpp"

#include <gtest/gtest.h>

TEST(clamp01, clamp01)
{
  using wheel_stuck_utils::math::clamp01;

  EXPECT_DOUBLE_EQ(clamp01(0.5), 0.5);
  EXPECT_DOUBLE_EQ(clamp01(1.5), 1.0);
  EXPECT_DOUBLE_EQ(clamp01(-0.5), 0.0);
}
