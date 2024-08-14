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

#ifndef GROUND_SEGMENTATION__GROUND_FILTER_HPP_
#define GROUND_SEGMENTATION__GROUND_FILTER_HPP_

#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>

namespace ground_segmentation
{

using PointCloud2 = sensor_msgs::msg::PointCloud2;

class GroundFilter : public rclcpp::Node
{
public:
  explicit GroundFilter(const std::string & node_name, const rclcpp::NodeOptions & options);

protected:
  virtual bool filter(const PointCloud2 & input, PointCloud2 & output) = 0;
  [[nodiscard]] bool try_transform_pointcloud(
    const std::string & target_frame, const PointCloud2 & input, PointCloud2 & output,
    const double timeout = 1.0);

private:
  void input_callback(const PointCloud2::SharedPtr msg);

  rclcpp::Subscription<PointCloud2>::SharedPtr input_sub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr output_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};
}  // namespace ground_segmentation

#endif  // GROUND_SEGMENTATION__GROUND_FILTER_HPP_
