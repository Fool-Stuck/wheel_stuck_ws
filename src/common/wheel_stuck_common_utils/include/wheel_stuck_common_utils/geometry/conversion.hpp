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

#ifndef WHEEL_STUCK_COMMON_UTILS__GEOMETRY__CONVERSION_HPP_
#define WHEEL_STUCK_COMMON_UTILS__GEOMETRY__CONVERSION_HPP_

#include <Eigen/Core>
#include <tf2_eigen/tf2_eigen.hpp>

namespace wheel_stuck_common_utils::geometry
{

geometry_msgs::msg::Pose to_pose(const geometry_msgs::msg::Transform & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation.x;
  pose.position.y = transform.translation.y;
  pose.position.z = transform.translation.z;
  pose.orientation = transform.rotation;
  return pose;
}

Eigen::Matrix4f to_matrix4f(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Affine3d affine;
  tf2::fromMsg(pose, affine);
  return affine.cast<float>().matrix();
}

Eigen::Matrix4f to_matrix4f(const geometry_msgs::msg::Transform & transform)
{
  Eigen::Affine3d affine;
  tf2::fromMsg(to_pose(transform), affine);
  return affine.cast<float>().matrix();
}
}  // namespace wheel_stuck_common_utils::geometry

#endif  // WHEEL_STUCK_COMMON_UTILS__GEOMETRY__CONVERSION_HPP_
