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

#include "pointcloud_preprocessor/pointcloud_preprocessor.hpp"

namespace pointcloud_preprocessor
{
PointCloudPreprocessor::PointCloudPreprocessor(const rclcpp::NodeOptions & options)
: Node("pointcloud_preprocessor", options),
  tf_buffer_(this->get_clock()),  // tf2_ros::Bufferの初期化
  tf_listener_(tf_buffer_)        // tf2_ros::TransformListenerの初期化

{
  // 受信機を作る。
  pc_sub_ = this->create_subscription<PointCloud2>(
    "/velodyne_points", 10,
    std::bind(&PointCloudPreprocessor::processCloud, this, std::placeholders::_1));
  // 送信機を作る。
  pc_pub_ = this->create_publisher<PointCloud2>("/output/filtered_points", 10);
}

void PointCloudPreprocessor::processCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  try {
    // velodyne_linkからbase_linkへの変換を取得する。
    transformStamped_ =
      tf_buffer_.lookupTransform("base_link", msg->header.frame_id, tf2::TimePointZero);
    RCLCPP_INFO(this->get_logger(), "Transform obtained");

    // 座標変換行列の取得
    tf2::fromMsg(transformStamped_.transform, transform_);
    if (std::isnan(transform_.matrix().norm())) {
      throw std::runtime_error("Transform contains NaN values");
    }

    // 点群データの取得
    pcl::fromROSMsg(*msg, pcl_input_);
    RCLCPP_INFO(this->get_logger(), "Point cloud converted to PCL format");
    // 点群データの変換
    pcl_output_ = wheel_stuck_common_utils::pointcloud::transform_pointcloud<pcl::PointXYZIR>(
      pcl_input_, transform_);
    RCLCPP_INFO(this->get_logger(), "Point cloud transformed");

    // 座標変換したデータのパブリッシュ
    pcl::toROSMsg(pcl_output_, output_msg_);
    output_msg_.header = msg->header;
    output_msg_.header.frame_id = "base_link";  // 変換後のフレームIDを設定
    pc_pub_->publish(output_msg_);
    RCLCPP_INFO(this->get_logger(), "Filtered point cloud published");
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
  }
}

}  // namespace pointcloud_preprocessor
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::PointCloudPreprocessor)
