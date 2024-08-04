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

#ifndef WHEEL_STUCK_ROBOT_UTILS__ROBOT_INFO_HPP_
#define WHEEL_STUCK_ROBOT_UTILS__ROBOT_INFO_HPP_

namespace wheel_stuck_robot_utils
{
struct RobotRawInfo
{
  double wheel_radius_m;
  double wheel_tread_m;
  double front_overhang_m;
  double rear_overhang_m;
  double left_overhang_m;
  double right_overhang_m;
  double robot_height_m;
};

struct RobotInfo
{
  RobotInfo() = default;
  explicit RobotInfo(RobotRawInfo raw_info)
  {
    this->raw_info = raw_info;
    robot_length = this->raw_info.front_overhang_m + this->raw_info.rear_overhang_m;
    robot_width = this->raw_info.wheel_tread_m + this->raw_info.left_overhang_m +
                  this->raw_info.right_overhang_m;
  }

  RobotRawInfo raw_info;
  double robot_length;
  double robot_width;
};
}  // namespace wheel_stuck_robot_utils

#endif  // WHEEL_STUCK_ROBOT_UTILS__ROBOT_INFO_HPP_
