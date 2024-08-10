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

#ifndef GROUND_SEGMENTATION__RANSAC_GROUND_FILTER__RANASC_GROUND_FILTER_HPP_
#define GROUND_SEGMENTATION__RANSAC_GROUND_FILTER__RANASC_GROUND_FILTER_HPP_

#include "ground_segmentation/ground_filter.hpp"

namespace ground_segmentation::ransac_ground_filter
{

class RANSACGroundFilter : public GroundFilter
{
public:
  explicit RANSACGroundFilter(const rclcpp::NodeOptions & options);

protected:
  bool filter(const PointCloud2 & input, PointCloud2 & output) override;
};

}  // namespace ground_segmentation::ransac_ground_filter

#endif  // GROUND_SEGMENTATION__RANSAC_GROUND_FILTER__RANASC_GROUND_FILTER_HPP_
