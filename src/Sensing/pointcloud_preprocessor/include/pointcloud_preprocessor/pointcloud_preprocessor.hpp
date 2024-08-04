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

#ifndef POINTCLOUD_PREPROCESSOR__POINTCLOUD_PREPROCESSOR_HPP_
#define POINTCLOUD_PREPROCESSOR__POINTCLOUD_PREPROCESSOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <wheel_stuck_common_utils/ros/function_timer.hpp>
#include <wheel_stuck_common_utils/ros/no_callback_subscription.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#ifndef LASERS_NUM
#define LASERS_NUM 32
#endif

namespace velodyne_cloud_separator
{

using FunctionTimer = wheel_stuck_common_utils::ros::FunctionTimer;

using PointXYZIR = pcl::PointXYZIR;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

using PointCloud2Subscription = wheel_stuck_common_utils::ros::NoCallbackSubscription<PointCloud2>;
using PointCloud2Publisher = rclcpp::Publisher<PointCloud2>;

class PoitnCloud_Preprocessor : public rclcpp::Node
{
private:
  // enum class PointType { UNKNOWN = 0, GROUND = 1, OBSTACLE = 2 };

public:
  explicit PointCloud_Preprocessor(const rclcpp::NodeOptions & options);

private:
  void update();
  void processCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  FunctionTimer::SharedPtr update_timer_;

  PointCloud2Subscription::SharedPtr pc_sub_;
  PointCloud2Subscription::SharedPtr pc_pub_;

  pcl::PointCloud<PointXYZIR>::Ptr pc_;
};
}  // namespace velodyne_cloud_separator

#endif  // POINTCLOUD_PREPROCESSOR__POINTCLOUD_PREPROCESSOR_HPP_
