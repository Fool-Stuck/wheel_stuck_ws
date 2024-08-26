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

#ifndef POINTCLOUD_PREPROCESSOR__CROPBOX_FILTER_HPP_
#define POINTCLOUD_PREPROCESSOR__CROPBOX_FILTER_HPP_

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <wheel_stuck_common_utils/geometry/conversion.hpp>
#include <wheel_stuck_robot_utils/robot_info.hpp>
#include <wheel_stuck_robot_utils/robot_info_utils.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>

#include <string>

namespace cropbox_filter
{

// クロッピングする処理をここに書く
class CropBoxFilter
{
public:
  explicit CropBoxFilter(rclcpp::Node & node)
  {
    // robot_infoからのパラメータの読み取りを行う。
    wheel_stuck_robot_utils::RobotInfoUtils robot_info_utils(node);
    robot_info_ = robot_info_utils.get_info();
    // パラメーターが読み込まれた後のログ出力
    RCLCPP_INFO(node.get_logger(), "Front Overhang: %f", robot_info_.raw_info.front_overhang_m);

    // 必要であれば、debug_parameters()を呼び出して詳細をログ出力
    debug_parameters();
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr crop_box(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcl_output,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & cropped_cloud)
  {
    Eigen::Vector4f self_min_pt(
      -robot_info_.raw_info.rear_overhang_m,   // x軸負方向（後方）
      -robot_info_.raw_info.right_overhang_m,  // y軸負方向（左側）
      -robot_info_.raw_info.robot_height_m,    // z軸負方向（下方）
      1.0);

    Eigen::Vector4f self_max_pt(
      robot_info_.raw_info.front_overhang_m,  // x軸正方向（前方）
      robot_info_.raw_info.left_overhang_m,   // y軸正方向（右側）
      robot_info_.raw_info.robot_height_m,    // z軸正方向（上方）
      1.0);

    pcl::CropBox<pcl::PointXYZ> self_crop_box_filter;

    self_crop_box_filter.setInputCloud(pcl_output);
    self_crop_box_filter.setMin(self_min_pt);
    self_crop_box_filter.setMax(self_max_pt);
    pcl::PointCloud<pcl::PointXYZ>::Ptr self_cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    self_crop_box_filter.setNegative(true);
    self_crop_box_filter.filter(*self_cropped_cloud);

    pcl::CropBox<pcl::PointXYZ> area_crop_box_filter;
    area_crop_box_filter.setInputCloud(self_cropped_cloud);
    Eigen::Vector4f area_min_pt(-5.0, -5.0, -5.0, 1.0);
    Eigen::Vector4f area_max_pt(5.0, 5.0, 5.0, 1.0);
    area_crop_box_filter.setMin(area_min_pt);
    area_crop_box_filter.setMax(area_max_pt);
    area_crop_box_filter.filter(*cropped_cloud);

    return cropped_cloud;
  }
  // デバッグ用。パラメーターの取得の確認。
  void debug_parameters() const
  {
    RCLCPP_INFO(
      rclcpp::get_logger("CropBoxFilter"), "Wheel Radius: %f", robot_info_.raw_info.wheel_radius_m);
    RCLCPP_INFO(
      rclcpp::get_logger("CropBoxFilter"), "Wheel Tread: %f", robot_info_.raw_info.wheel_tread_m);
    RCLCPP_INFO(
      rclcpp::get_logger("CropBoxFilter"), "Front Overhang: %f",
      robot_info_.raw_info.front_overhang_m);
    RCLCPP_INFO(
      rclcpp::get_logger("CropBoxFilter"), "Rear Overhang: %f",
      robot_info_.raw_info.rear_overhang_m);
    RCLCPP_INFO(
      rclcpp::get_logger("CropBoxFilter"), "Left Overhang: %f",
      robot_info_.raw_info.left_overhang_m);
    RCLCPP_INFO(
      rclcpp::get_logger("CropBoxFilter"), "Right Overhang: %f",
      robot_info_.raw_info.right_overhang_m);
    RCLCPP_INFO(
      rclcpp::get_logger("CropBoxFilter"), "Robot Height: %f", robot_info_.raw_info.robot_height_m);
  }

private:
  wheel_stuck_robot_utils::RobotInfo robot_info_;
};

}  // namespace cropbox_filter

#endif  // POINTCLOUD_PREPROCESSOR__CROPBOX_FILTER_HPP_
