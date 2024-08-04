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

#include "robot_info_user/robot_info_user.hpp"

#include <wheel_stuck_robot_utils/robot_info_utils.hpp>

namespace robot_info_user
{

using RobotInfoUtils = wheel_stuck_robot_utils::RobotInfoUtils;

RobotInfoUser::RobotInfoUser(const rclcpp::NodeOptions & options)
: rclcpp::Node("robot_info_user", options)
{
  auto robot_info = RobotInfoUtils(*this).get_info();
  RCLCPP_INFO(
    this->get_logger(),
    "\nRobotInfo: \n  wheel_radius: %f\n  wheel_tread: %f\n  front_overhang: %f\n  rear_overhang: "
    "%f\n  left_overhang: %f\n  right_overhang: %f\n  robot_height: %f",
    robot_info.raw_info.wheel_radius_m, robot_info.raw_info.wheel_tread_m,
    robot_info.raw_info.front_overhang_m, robot_info.raw_info.rear_overhang_m,
    robot_info.raw_info.left_overhang_m, robot_info.raw_info.right_overhang_m,
    robot_info.raw_info.robot_height_m);
}
}  // namespace robot_info_user

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_info_user::RobotInfoUser)
