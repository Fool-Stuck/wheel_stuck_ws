#include "wheel_stuck_utils/math/math.hpp"

#include <gtest/gtest.h>

TEST(lerp, lerp)
{
  using wheel_stuck_utils::math::lerp;

  EXPECT_DOUBLE_EQ(lerp(0.0, 10.0, 0.5), 5.0);
  EXPECT_DOUBLE_EQ(lerp(0.0, 10.0, 1.5), 10.0);
  EXPECT_DOUBLE_EQ(lerp(0.0, 10.0, -0.5), 0.0);
}
