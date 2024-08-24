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
    auto robot_info = robot_info_utils.get_info();
    RCLCPP_INFO(node.get_logger(), "Wheel Radius: %f", robot_info.raw_info.wheel_radius_m);
    RCLCPP_INFO(node.get_logger(), "Wheel Tread: %f", robot_info.raw_info.wheel_tread_m);
    RCLCPP_INFO(node.get_logger(), "Front Overhang: %f", robot_info.raw_info.front_overhang_m);
    RCLCPP_INFO(node.get_logger(), "Rear Overhang: %f", robot_info.raw_info.rear_overhang_m);
    RCLCPP_INFO(node.get_logger(), "Left Overhang: %f", robot_info.raw_info.left_overhang_m);
    RCLCPP_INFO(node.get_logger(), "Right Overhang: %f", robot_info.raw_info.right_overhang_m);
    RCLCPP_INFO(node.get_logger(), "Robot Height: %f", robot_info.raw_info.robot_height_m);

    // 読み取ったパラメーターでセルフクロッピングをする

    // 指定した範囲をクロッピングする

    // クロッピングした値を返す
  }

private:
  wheel_stuck_robot_utils::RobotInfo robot_info_;
};

}  // namespace cropbox_filter

#endif  // POINTCLOUD_PREPROCESSOR__CROPBOX_FILTER_HPP_
