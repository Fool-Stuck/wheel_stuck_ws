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

    self_min_pt_ = Eigen::Vector4f(
      -robot_info_.raw_info.rear_overhang_m,  // x軸負方向（後方）
      -robot_info_.raw_info.right_overhang_m -
        robot_info_.raw_info.wheel_tread_m / 2.0,  // y軸負方向（右側）
      -robot_info_.raw_info.robot_height_m -
        robot_info_.raw_info.wheel_radius_m,  // z軸負方向（下方）
      1.0);
    self_max_pt_ = Eigen::Vector4f(
      robot_info_.raw_info.front_overhang_m,  // x軸正方向（前方）
      robot_info_.raw_info.left_overhang_m +
        robot_info_.raw_info.wheel_tread_m / 2.0,  // y軸正方向（左側）
      robot_info_.raw_info.robot_height_m -
        robot_info_.raw_info.wheel_radius_m,  // z軸正方向（上方）
      1.0);

    min_x_ = node.declare_parameter<double>("crop_area_min_x", -1.0);
    min_y_ = node.declare_parameter<double>("crop_area_min_y", -1.0);
    min_z_ = node.declare_parameter<double>("crop_area_min_z", -1.0);
    max_x_ = node.declare_parameter<double>("crop_area_max_x", 1.0);
    max_y_ = node.declare_parameter<double>("crop_area_max_y", 1.0);
    max_z_ = node.declare_parameter<double>("crop_area_max_z", 1.0);
    crop_area_w_ = node.declare_parameter<double>("crop_area_w", 1.0);

    area_min_pt_ = Eigen::Vector4f(min_x_, min_y_, min_z_, crop_area_w_);
    area_max_pt_ = Eigen::Vector4f(max_x_, max_y_, max_z_, crop_area_w_);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr crop_box(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcl_input_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & pcl_cropped_cloud)
  {
    self_crop_box_filter.setInputCloud(pcl_input_cloud);
    self_crop_box_filter.setMin(self_min_pt_);
    self_crop_box_filter.setMax(self_max_pt_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr self_cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    self_crop_box_filter.setNegative(true);
    self_crop_box_filter.filter(*self_cropped_cloud);

    area_crop_box_filter.setInputCloud(self_cropped_cloud);
    area_crop_box_filter.setMin(area_min_pt_);
    area_crop_box_filter.setMax(area_max_pt_);
    area_crop_box_filter.filter(*pcl_cropped_cloud);

    return pcl_cropped_cloud;
  }

private:
  wheel_stuck_robot_utils::RobotInfo robot_info_;
  Eigen::Vector4f self_min_pt_;
  Eigen::Vector4f self_max_pt_;
  Eigen::Vector4f area_min_pt_;
  Eigen::Vector4f area_max_pt_;
  pcl::CropBox<pcl::PointXYZ> self_crop_box_filter;
  pcl::CropBox<pcl::PointXYZ> area_crop_box_filter;
  double min_x_;
  double min_y_;
  double min_z_;
  double max_x_;
  double max_y_;
  double max_z_;
  double crop_area_w_;
};

}  // namespace cropbox_filter

#endif  // POINTCLOUD_PREPROCESSOR__CROPBOX_FILTER_HPP_
