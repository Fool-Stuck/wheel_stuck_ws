// Copyright 2024 Fool Stuck Engineers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "wheel_stuck_utils/math/math.hpp"

#include <gtest/gtest.h>

TEST(clamp01, clamp01)
{
  using wheel_stuck_utils::math::clamp01;

  EXPECT_DOUBLE_EQ(clamp01(0.5), 0.5);
  EXPECT_DOUBLE_EQ(clamp01(1.5), 1.0);
  EXPECT_DOUBLE_EQ(clamp01(-0.5), 0.0);
}
