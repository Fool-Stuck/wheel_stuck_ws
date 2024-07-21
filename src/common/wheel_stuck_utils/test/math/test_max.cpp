#include "wheel_stuck_utils/math/math.hpp"

#include <gtest/gtest.h>

TEST(max, max)
{
  using wheel_stuck_utils::math::max;

  EXPECT_EQ(max(5), 5);
  EXPECT_EQ(max(5, 2, 8, -1), 8);
  EXPECT_DOUBLE_EQ(max(2.4, 5.6, 0.1), 5.6);
}
