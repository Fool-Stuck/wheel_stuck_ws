#include "wheel_stuck_utils/math/math.hpp"

#include <gtest/gtest.h>

TEST(inverse_lerp, inverse_lerp)
{
  using wheel_stuck_utils::math::inverse_lerp;

  EXPECT_DOUBLE_EQ(inverse_lerp(0.0, 10.0, 5), 0.5);
  EXPECT_DOUBLE_EQ(inverse_lerp(0.0, 10.0, 15.0), 1.0);
  EXPECT_DOUBLE_EQ(inverse_lerp(0.0, 10.0, -0.5), 0.0);
  EXPECT_DOUBLE_EQ(inverse_lerp(10.0, 10.0, 10.0), 0.0);  // a == b のとき
}
