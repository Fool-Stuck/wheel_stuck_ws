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

#include "wheel_stuck_common_utils/math/math.hpp"

#include <gtest/gtest.h>

TEST(max, max)
{
  using wheel_stuck_common_utils::math::max;

  EXPECT_EQ(max(5), 5);
  EXPECT_EQ(max(5, 2, 8, -1), 8);
  EXPECT_DOUBLE_EQ(max(2.4, 5.6, 0.1), 5.6);
}
