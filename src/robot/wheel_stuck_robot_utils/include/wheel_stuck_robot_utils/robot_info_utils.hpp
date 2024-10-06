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

#ifndef WHEEL_STUCK_ROBOT_UTILS__ROBOT_INFO_UTILS_HPP_
#define WHEEL_STUCK_ROBOT_UTILS__ROBOT_INFO_UTILS_HPP_

#include "wheel_stuck_robot_utils/robot_info.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace wheel_stuck_robot_utils
{
class RobotInfoUtils
{
public:
  explicit RobotInfoUtils(rclcpp::Node & node)
  {
    RobotRawInfo raw_info;
    raw_info.wheel_radius_m = get_parameter<double>(node, "wheel_radius");
    raw_info.wheel_tread_m = get_parameter<double>(node, "wheel_tread");
    raw_info.front_overhang_m = get_parameter<double>(node, "front_overhang");
    raw_info.rear_overhang_m = get_parameter<double>(node, "rear_overhang");
    raw_info.left_overhang_m = get_parameter<double>(node, "left_overhang");
    raw_info.right_overhang_m = get_parameter<double>(node, "right_overhang");
    raw_info.robot_height_m = get_parameter<double>(node, "robot_height");

    robot_info_ = RobotInfo(raw_info);
  }
  RobotInfo get_info() const { return robot_info_; }

private:
  template <typename T>
  T get_parameter(rclcpp::Node & node, const std::string & name)
  {
    if (node.has_parameter(name)) return node.get_parameter(name).get_value<T>();
    try {
      return node.declare_parameter<T>(name);
    } catch (const rclcpp::ParameterTypeException & e) {
      RCLCPP_ERROR(node.get_logger(), "Failed to get parameter: %s", e.what());
      throw e;
    }
  }

  RobotInfo robot_info_;
};
}  // namespace wheel_stuck_robot_utils

#endif  // WHEEL_STUCK_ROBOT_UTILS__ROBOT_INFO_UTILS_HPP_
