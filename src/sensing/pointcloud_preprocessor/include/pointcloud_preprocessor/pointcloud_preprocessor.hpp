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

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <wheel_stuck_common_utils/geometry/conversion.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>

namespace pointcloud_preprocessor
{
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using wheel_stuck_common_utils::geometry::to_matrix4f;

class PointCloudPreprocessor : public rclcpp::Node
{
public:
  explicit PointCloudPreprocessor(const rclcpp::NodeOptions & options);

private:
  void process_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pc_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string target_frame_id_;
  double leaf_size_;
};

}  // namespace pointcloud_preprocessor

#endif  // POINTCLOUD_PREPROCESSOR__POINTCLOUD_PREPROCESSOR_HPP_
