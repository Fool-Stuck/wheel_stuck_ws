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

#ifndef WHEEL_STUCK_COMMON_UTILS__POINTCLOUD__TRANSFORM_POINTCLOUD_HPP_
#define WHEEL_STUCK_COMMON_UTILS__POINTCLOUD__TRANSFORM_POINTCLOUD_HPP_

#include <Eigen/Core>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>

namespace wheel_stuck_common_utils::pointcloud
{
template <typename PointT>
void try_transform_pointcloud(
  const pcl::PointCloud<PointT> & input, pcl::PointCloud<PointT> & output,
  const Eigen::Affine3d & transform)
{
  pcl::transformPointCloud(input, output, transform);
}

template <typename PointT>
pcl::PointCloud<PointT> transform_pointcloud(
  const pcl::PointCloud<PointT> & input, const Eigen::Affine3d & transform)
{
  pcl::PointCloud<PointT> output;
  pcl::transformPointCloud(input, output, transform);
  return output;
}
}  // namespace wheel_stuck_common_utils::pointcloud

#endif  // WHEEL_STUCK_COMMON_UTILS__POINTCLOUD__TRANSFORM_POINTCLOUD_HPP_
