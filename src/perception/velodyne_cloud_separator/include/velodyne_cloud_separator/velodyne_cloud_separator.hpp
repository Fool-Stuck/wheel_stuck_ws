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

#ifndef VELODYNE_CLOUD_SEPARATOR__VELODYNE_CLOUD_SEPARATOR_HPP_
#define VELODYNE_CLOUD_SEPARATOR__VELODYNE_CLOUD_SEPARATOR_HPP_

#include "velodyne_cloud_separator/point_types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <wheel_stuck_common_utils/ros/function_timer.hpp>
#include <wheel_stuck_common_utils/ros/no_callback_subscription.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>

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

class VelodyneCloudSeparator : public rclcpp::Node
{
private:
  enum class PointType { UNKNOWN = 0, GROUND = 1, OBSTACLE = 2 };

public:
  explicit VelodyneCloudSeparator(const rclcpp::NodeOptions & options);

private:
  void init(
    const double sensor_height, const double radius_coef_close, const double radius_coef_far);
  void update();
  bool try_subscribe_pc();

  FunctionTimer::SharedPtr update_timer_;

  PointCloud2Subscription::SharedPtr pc_sub_;
  PointCloud2Publisher::SharedPtr pc_ground_pub_;
  PointCloud2Publisher::SharedPtr pc_obstacle_pub_;

  pcl::PointCloud<PointXYZIR>::Ptr pc_;

  static const int height_ = LASERS_NUM;

  double radius_array_[LASERS_NUM];

  double limiting_ratio_;
  double gap_threshold_;
  int min_points_num_;
};
}  // namespace velodyne_cloud_separator

#endif  // VELODYNE_CLOUD_SEPARATOR__VELODYNE_CLOUD_SEPARATOR_HPP_
