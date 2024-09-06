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

#ifndef MULTI_LAYERED_POINTCLOUD_FILTER__MULTI_LAYERED_POINTCLOUD_FILTER_HPP_
#define MULTI_LAYERED_POINTCLOUD_FILTER__MULTI_LAYERED_POINTCLOUD_FILTER_HPP_

#include "multi_layered_pointcloud_filter/pointcloud_filter_base.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <vector>

namespace multi_layered_pointcloud_filter
{

using PointCloud2 = sensor_msgs::msg::PointCloud2;

class MultiLayeredPointCloudFilter
{
public:
  using SharedPtr = std::shared_ptr<MultiLayeredPointCloudFilter>;
  void add_filter_layer(PointCloudFilterBase::SharedPtr filter_layer);
  bool is_ready();
  bool filter(const PointCloud2::SharedPtr input_points, PointCloud2::SharedPtr output_points);

private:
  std::vector<PointCloudFilterBase::SharedPtr> filter_layers_;
};
}  // namespace multi_layered_pointcloud_filter

#endif  // MULTI_LAYERED_POINTCLOUD_FILTER__MULTI_LAYERED_POINTCLOUD_FILTER_HPP_
