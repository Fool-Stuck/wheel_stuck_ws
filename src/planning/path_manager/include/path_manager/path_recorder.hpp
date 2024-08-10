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

#ifndef PATH_MANAGER__PATH_RECORDER_HPP_
#define PATH_MANAGER__PATH_RECORDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <wheel_stuck_common_utils/ros/function_timer.hpp>

#include <wheel_stuck_planning_msgs/msg/path.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace path_manager
{

using Path = wheel_stuck_planning_msgs::msg::Path;

using FunctionTimer = wheel_stuck_common_utils::ros::FunctionTimer;

class PathRecorder : public rclcpp::Node
{
public:
  explicit PathRecorder(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  FunctionTimer::SharedPtr try_record_timer_;
};
}  // namespace path_manager

#endif  // PATH_MANAGER__PATH_RECORDER_HPP_
