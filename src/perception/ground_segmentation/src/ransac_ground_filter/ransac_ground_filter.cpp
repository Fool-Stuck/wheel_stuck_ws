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

#include "ground_segmentation/ransac_ground_filter/ransac_ground_filter.hpp"

namespace ground_segmentation::ransac_ground_filter
{
RANSACGroundFilter::RANSACGroundFilter(const rclcpp::NodeOptions & options)
: GroundFilter("ransac_ground_filter", options)
{
  footprint_frame_ = declare_parameter<std::string>("footprint_frame", "base_link");
  leaf_size_ = declare_parameter<double>("leaf_size", 0.1);
  distance_threshold_ = declare_parameter<double>("distance_threshold", 0.1);
  max_iterations_ = declare_parameter<int>("max_iterations", 1000);
  slope_threshold_rad_ = declare_parameter<double>("slope_threshold", 10.0) * M_PI / 180.0;
}

bool RANSACGroundFilter::filter(const PointCloud2 & input, PointCloud2 & output)
{
  // Transform input to base_link
  PointCloud2::SharedPtr input_transformed(new PointCloud2);
  {
    if (!try_transform_pointcloud(footprint_frame_, input, *input_transformed)) return false;
  }

  // Convert to PCL
  pcl::PointCloud<PointType>::Ptr input_cloud(new pcl::PointCloud<PointType>);
  {
    pcl::fromROSMsg(*input_transformed, *input_cloud);
  }

  // Downsample the input cloud
  pcl::PointCloud<PointType>::Ptr downsampled_cloud(new pcl::PointCloud<PointType>);
  {
    pcl::VoxelGrid<PointType> voxel_grid;
    voxel_grid.setInputCloud(input_cloud);
    voxel_grid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    voxel_grid.filter(*downsampled_cloud);
  }

  // Apply RANSAC
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  {
    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(max_iterations_);
    seg.setDistanceThreshold(distance_threshold_);

    seg.setInputCloud(downsampled_cloud);
    seg.segment(*inliers, *coefficients);
  }

  // Return input as output if no ground plane is found
  if (coefficients->values.empty()) {
    RCLCPP_WARN(get_logger(), "Failed to find a ground plane");
    output = input;
    return true;
  }

  // Filter tilted plane
  {
    Eigen::Vector3d normal(
      coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    const auto slope_rad =
      std::abs(std::acos(normal.dot(unit_z_vec_) / (normal.norm() * unit_z_vec_.norm())));
    if (slope_rad > slope_threshold_rad_) {
      output = input;
      return true;
    }
  }

  // Extract ground points and obstacle points
  pcl::PointCloud<PointType>::Ptr ground_cloud(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr obstacle_cloud(new pcl::PointCloud<PointType>);
  {
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(downsampled_cloud);
    extract.setIndices(inliers);

    extract.setNegative(false);
    extract.filter(*ground_cloud);

    extract.setNegative(true);
    extract.filter(*obstacle_cloud);
  }

  // Convert obstacle cloud to ROS message
  {
    PointCloud2 obstacle_cloud_msg;
    pcl::toROSMsg(*obstacle_cloud, obstacle_cloud_msg);
    obstacle_cloud_msg.header = input_transformed->header;
    output = obstacle_cloud_msg;
  }

  return true;
}
}  // namespace ground_segmentation::ransac_ground_filter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ground_segmentation::ransac_ground_filter::RANSACGroundFilter)
