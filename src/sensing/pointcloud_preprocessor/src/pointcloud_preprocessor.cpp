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

#include "pointcloud_preprocessor/cropbox_filter.hpp"

#include <wheel_stuck_robot_utils/robot_info.hpp>

namespace pointcloud_preprocessor
{
PointCloudPreprocessor::PointCloudPreprocessor(const rclcpp::NodeOptions & options)
: Node("pointcloud_preprocessor", options),
  tf_buffer_(this->get_clock()),  // tf2_ros::Bufferの初期化
  tf_listener_(tf_buffer_)        // tf2_ros::TransformListenerの初期化

{
  // パラメーターの宣言
  target_frame_id_ = declare_parameter<std::string>("target_frame_id", "base_link");
  leaf_size_ = declare_parameter<double>("leaf_size", 0.1);

  // CropBoxfolterのインスタンスを作成。
  cropbox_filter::CropBoxFilter crop_box_filter(*this);
  // パラメーターを出力する関数を呼び出す
  print_parameters();

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
  geometry_msgs::msg::TransformStamped transform_stamped;

  try {
    // velodyne_linkからbase_linkへの変換を取得する。引数は（変換先のフレームID,変換元のフレームID,最新の変換データを使用）
    transform_stamped =
      tf_buffer_.lookupTransform(target_frame_id_, msg->header.frame_id, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    return;
  }

  // ROSメッセージ形式の点群データをPCL形式に変換
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *pcl_input);

  // 座標変換行列(表現行列)の取得
  Eigen::Matrix4f transform_matrix;
  transform_matrix = to_matrix4f(transform_stamped.transform);

  // 点群データの座標変換
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*pcl_input, *pcl_output, transform_matrix);

  // ダウンサンプリング
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(pcl_output);
  voxel_grid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  pcl::PointCloud<pcl::PointXYZ> downsampled_cloud;
  voxel_grid.filter(downsampled_cloud);

  // 座標変換した点群データを再びROSメッセージ形式に変換
  PointCloud2 output_msg;
  pcl::toROSMsg(downsampled_cloud, output_msg);
  // メッセージのヘッダーを設定
  output_msg.header = msg->header;
  output_msg.header.frame_id = target_frame_id_;  // 変換後のフレームIDを設定

  // 変換後の点群データをパブリッシュ
  pc_pub_->publish(output_msg);
}

void PointCloudPreprocessor::print_parameters()
{
  RCLCPP_INFO(this->get_logger(), "target_frame_id: %s", target_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "leaf_size: %f", leaf_size_);
}

}  // namespace pointcloud_preprocessor
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::PointCloudPreprocessor)
