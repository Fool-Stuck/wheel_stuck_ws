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
#include <tf2_eigen/tf2_eigen.hpp>
#include <wheel_stuck_common_utils/geometry/conversion.hpp>
#include <wheel_stuck_common_utils/pointcloud/transform_pointcloud.hpp>
#include <wheel_stuck_common_utils/ros/function_timer.hpp>
#include <wheel_stuck_common_utils/ros/no_callback_subscription.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#ifndef LASERS_NUM
#define LASERS_NUM 32
#endif

namespace pointcloud_preprocessor
{

using FunctionTimer = wheel_stuck_common_utils::ros::FunctionTimer;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

class PointCloudPreprocessor : public rclcpp::Node
{
private:
  // enum class PointType { UNKNOWN = 0, GROUND = 1, OBSTACLE = 2 };

public:
  explicit PointCloudPreprocessor(const rclcpp::NodeOptions & options);

private:
  void processCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  FunctionTimer::SharedPtr update_timer_;

  rclcpp::Subscription<PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pc_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  geometry_msgs::msg::TransformStamped transformStamped_;
  Eigen::Affine3d transform_;
  pcl::PointCloud<pcl::PointXYZ> pcl_input_;
  pcl::PointCloud<pcl::PointXYZ> pcl_output_;
  PointCloud2 output_msg_;
};

}  // namespace pointcloud_preprocessor

#endif  // POINTCLOUD_PREPROCESSOR__POINTCLOUD_PREPROCESSOR_HPP_
