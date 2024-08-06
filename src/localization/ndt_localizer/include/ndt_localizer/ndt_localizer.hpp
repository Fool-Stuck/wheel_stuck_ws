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

#ifndef NDT_LOCALIZER__NDT_LOCALIZER_HPP_
#define NDT_LOCALIZER__NDT_LOCALIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <wheel_stuck_common_utils/geometry/conversion.hpp>
#include <wheel_stuck_common_utils/pointcloud/transform_pointcloud.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace ndt_localizer
{

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2Subscription = rclcpp::Subscription<PointCloud2>;

class NDTLocalizer : public rclcpp::Node
{
public:
  explicit NDTLocalizer(const rclcpp::NodeOptions & options);

private:
  void pointcloud_callback(const PointCloud2::SharedPtr msg);

  PointCloud2Subscription::SharedPtr pointcloud_sub_;
};

}  // namespace ndt_localizer

#endif  // NDT_LOCALIZER__NDT_LOCALIZER_HPP_
