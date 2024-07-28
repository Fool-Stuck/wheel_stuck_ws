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

#ifndef WHEEL_STUCK_PLANNING_UTILS__POINT_INDEX_CONVERTOR_HPP_
#define WHEEL_STUCK_PLANNING_UTILS__POINT_INDEX_CONVERTOR_HPP_

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

using PointXYZ = pcl::PointXYZ;
using PointCloudXYZ = pcl::PointCloud<PointXYZ>;

namespace point_index_convertor
{
int Point2GridIndex(const PointXYZ & input_point, const OccupancyGrid & dest_map)
{
  if (
    abs(input_point.x) < (dest_map.info.width / 2) * dest_map.info.resolution &&
    abs(input_point.y) < (dest_map.info.height / 2) * dest_map.info.resolution) {
    int x_index, y_index;
    x_index = (input_point.x / dest_map.info.resolution) + (dest_map.info.width / 2);
    y_index = (input_point.y / dest_map.info.resolution) + (dest_map.info.height / 2);
    return x_index + y_index * dest_map.info.width;
  } else {
    return -1;
  }
}

inline int GridIndex2XIndex(const int input_index, const OccupancyGrid & source_map)
{
  return input_index % source_map.info.width;
}

inline int GridIndex2YIndex(const int input_index, const OccupancyGrid & source_map)
{
  return input_index / source_map.info.width;
}

PointXYZ GridIndex2Coordinate(const int input_index, const OccupancyGrid & source_map)
{
  double x_index = static_cast<double>(GridIndex2XIndex(input_index, source_map));
  double y_index = static_cast<double>(GridIndex2YIndex(input_index, source_map));
  PointXYZ output_coordinate;
  output_coordinate.x = (x_index + 0.5 * (1 + source_map.info.width)) * source_map.info.resolution;
  output_coordinate.y = (y_index + 0.5 * (1 + source_map.info.height)) * source_map.info.resolution;
  output_coordinate.z = 0;
  return output_coordinate;
}

}  // namespace point_index_convertor
#endif  // WHEEL_STUCK_PLANNING_UTILS__POINT_INDEX_CONVERTOR_HPP_
