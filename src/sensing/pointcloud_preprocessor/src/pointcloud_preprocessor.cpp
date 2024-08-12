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
  target_frame_id_ = declare_parameter<std::string>("target_frame_id", "base_link");
  // 受信機を作る。
  pc_sub_ = this->create_subscription<PointCloud2>(
    "/velodyne_points", 10,
    std::bind(&PointCloudPreprocessor::process_pointcloud, this, std::placeholders::_1));
  // 送信機を作る。
  pc_pub_ = this->create_publisher<PointCloud2>("~/output/filtered_points", 10);
}

void PointCloudPreprocessor::process_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // std::cout << "Frame ID: " << msg->header.frame_id << std::endl;
  geometry_msgs::msg::TransformStamped transformStamped_;
  pcl::PointCloud<pcl::PointXYZ> pcl_input_;
  pcl::PointCloud<pcl::PointXYZ> pcl_output_;
  PointCloud2 output_msg_;
  Eigen::Matrix4f transform_matrix;

  try {
    // velodyne_linkからbase_linkへの変換を取得する。引数は（変換先のフレームID,変換元のフレームID,最新の変換データを使用）
    transformStamped_ =
      tf_buffer_.lookupTransform(target_frame_id_, msg->header.frame_id, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    return;
  }

  // 座標変換行列(表現行列)の取得
  transform_matrix = to_matrix4f(transformStamped_.transform);

  // ROSメッセージ形式の点群データをPCL形式に変換
  pcl::fromROSMsg(*msg, pcl_input_);

  // 点群データの座標変換
  pcl::transformPointCloud(pcl_input_, pcl_output_, transform_matrix);

  // 座標変換した点群データを再びROSメッセージ形式に変換
  pcl::toROSMsg(pcl_output_, output_msg_);
  // メッセージのヘッダーを設定
  output_msg_.header = msg->header;
  output_msg_.header.frame_id = target_frame_id_;  // 変換後のフレームIDを設定

  // 変換後の点群データをパブリッシュ
  pc_pub_->publish(output_msg_);
}

}  // namespace pointcloud_preprocessor
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::PointCloudPreprocessor)
