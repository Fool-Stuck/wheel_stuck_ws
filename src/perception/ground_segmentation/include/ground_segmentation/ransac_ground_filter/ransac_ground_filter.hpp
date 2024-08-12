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

#ifndef GROUND_SEGMENTATION__RANSAC_GROUND_FILTER__RANSAC_GROUND_FILTER_HPP_
#define GROUND_SEGMENTATION__RANSAC_GROUND_FILTER__RANSAC_GROUND_FILTER_HPP_

#include "ground_segmentation/ground_filter.hpp"

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>

namespace ground_segmentation::ransac_ground_filter
{

using PointType = pcl::PointXYZ;

class RANSACGroundFilter : public GroundFilter
{
public:
  explicit RANSACGroundFilter(const rclcpp::NodeOptions & options);

protected:
  bool filter(const PointCloud2 & input, PointCloud2 & output) override;

private:
  Eigen::Vector3d unit_z_vec_ = Eigen::Vector3d::UnitZ();

  std::string footprint_frame_;
  double leaf_size_;
  double distance_threshold_;
  int max_iterations_;
  double slope_threshold_rad_;
};

}  // namespace ground_segmentation::ransac_ground_filter

#endif  // GROUND_SEGMENTATION__RANSAC_GROUND_FILTER__RANSAC_GROUND_FILTER_HPP_
