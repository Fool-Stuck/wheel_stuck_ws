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

#include "ground_segmentation/ground_filter.hpp"

namespace ground_segmentation
{
GroundFilter::GroundFilter(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  input_sub_ = create_subscription<PointCloud2>(
    "~/input/points", 1, std::bind(&GroundFilter::input_callback, this, std::placeholders::_1));
  output_pub_ = create_publisher<PointCloud2>("~/output/points", 1);
}

bool GroundFilter::try_transform_pointcloud(
  const std::string & target_frame, const PointCloud2 & input, PointCloud2 & output,
  const double timeout)
{
  if (input.header.frame_id == target_frame) {
    output = input;
    return true;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(
      target_frame, input.header.frame_id, input.header.stamp, tf2::durationFromSec(timeout));
  } catch (const tf2::TransformException & e) {
    return false;
  }

  Eigen::Matrix4f transform_matrix = tf2::transformToEigen(transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(transform_matrix, input, output);
  output.header.frame_id = target_frame;

  return true;
}

void GroundFilter::input_callback(const PointCloud2::SharedPtr msg)
{
  PointCloud2 output;
  if (!filter(*msg, output)) return;
  output_pub_->publish(output);
}
}  // namespace ground_segmentation
