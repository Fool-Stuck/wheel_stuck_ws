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

#ifndef MULTI_LAYERED_POINTCLOUD_FILTER__POINTCLOUD_FILTER_BASE_HPP_
#define MULTI_LAYERED_POINTCLOUD_FILTER__POINTCLOUD_FILTER_BASE_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>

namespace multi_layered_pointcloud_filter
{

using PointCloud2 = sensor_msgs::msg::PointCloud2;

class PointCloudFilterBase
{
public:
  using SharedPtr = std::shared_ptr<PointCloudFilterBase>;
  virtual bool is_ready() = 0;
  virtual bool filter(PointCloud2::SharedPtr points) = 0;
};
}  // namespace multi_layered_pointcloud_filter

#endif  // MULTI_LAYERED_POINTCLOUD_FILTER__POINTCLOUD_FILTER_BASE_HPP_
