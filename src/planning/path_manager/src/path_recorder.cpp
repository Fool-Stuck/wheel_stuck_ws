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

#include "path_manager/path_recorder.hpp"

#include <wheel_stuck_common_utils/geometry/conversion.hpp>

#include <tf2/utils.h>

namespace path_manager
{
PathRecorder::PathRecorder(const rclcpp::NodeOptions & options)
: Node("path_recorder", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  double update_rate = declare_parameter("update_rate", 10.0);
  map_frame_ = declare_parameter("map_frame", "map");
  robot_frame_ = declare_parameter("robot_frame", "base_link");

  try_record_timer_ = FunctionTimer::create_function_timer(
    this, update_rate, std::bind(&PathRecorder::try_record, this));

  is_recording_ = false;
}

void PathRecorder::try_record()
{
  if (!is_recording_) return;

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(robot_frame_, map_frame_, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    is_recording_ = false;
    return;
  }

  geometry_msgs::msg::Pose pose = wheel_stuck_common_utils::geometry::to_pose(transform.transform);

  double linear_distance_sqr;
  {
    geometry_msgs::msg::Vector3 linear_diff;
    linear_diff.x = pose.position.x - last_recorded_pose_.position.x;
    linear_diff.y = pose.position.y - last_recorded_pose_.position.y;
    linear_diff.z = pose.position.z - last_recorded_pose_.position.z;
    linear_distance_sqr =
      linear_diff.x * linear_diff.x + linear_diff.y * linear_diff.y + linear_diff.z * linear_diff.z;
  }
  if (linear_distance_sqr > linear_record_interval_sqr_) {
    record(pose);
    return;
  }

  double yaw_diff;
  {
    tf2::Quaternion q1, q2;
    tf2::fromMsg(last_recorded_pose_.orientation, q1);
    tf2::fromMsg(pose.orientation, q2);
    yaw_diff = tf2::getYaw(q2 * q1.inverse());
  }
  if (std::abs(yaw_diff) < angular_record_interval_) return;
  record(pose);
}

void PathRecorder::record(const geometry_msgs::msg::Pose & pose)
{
  last_recorded_pose_ = pose;
}
}  // namespace path_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(path_manager::PathRecorder)
