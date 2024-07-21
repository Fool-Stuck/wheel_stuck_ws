#include "wheel_stuck_utils/math/math.hpp"

#include <gtest/gtest.h>

TEST(min, min)
{
  using wheel_stuck_utils::math::min;

  EXPECT_EQ(min(5), 5);
  EXPECT_EQ(min(5, 2, 8, -1), -1);
  EXPECT_DOUBLE_EQ(min(2.4, 5.6, 0.1), 0.1);
}
