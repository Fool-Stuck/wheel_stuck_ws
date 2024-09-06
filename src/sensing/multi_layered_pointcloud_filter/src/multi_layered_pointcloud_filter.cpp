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

#include "multi_layered_pointcloud_filter/multi_layered_pointcloud_filter.hpp"

namespace multi_layered_pointcloud_filter
{
void MultiLayeredPointCloudFilter::add_filter_layer(PointCloudFilterBase::SharedPtr filter_layer)
{
  filter_layers_.push_back(filter_layer);
}

bool MultiLayeredPointCloudFilter::is_ready()
{
  for (auto filter_layer : filter_layers_) {
    if (!filter_layer->is_ready()) return false;
  }
  return true;
}

bool MultiLayeredPointCloudFilter::filter(
  const PointCloud2::SharedPtr input_points, PointCloud2::SharedPtr output_points)
{
  PointCloud2::SharedPtr current_points = input_points;
  for (auto filter_layer : filter_layers_) {
    if (!filter_layer->filter(current_points)) return false;
  }
  output_points = current_points;
  return true;
}
}  // namespace multi_layered_pointcloud_filter
