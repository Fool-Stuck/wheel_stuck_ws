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

#include "ndt_localizer/ndt_localizer.hpp"

namespace ndt_localizer
{

NDTLocalizer::NDTLocalizer(const rclcpp::NodeOptions & options) : Node("ndt_localizer", options)
{
  pointcloud_sub_ = this->create_subscription<PointCloud2>(
    "pointcloud", 1, std::bind(&NDTLocalizer::pointcloud_callback, this, std::placeholders::_1));
}

void NDTLocalizer::pointcloud_callback(const PointCloud2::SharedPtr msg)
{
}

}  // namespace ndt_localizer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ndt_localizer::NDTLocalizer)
