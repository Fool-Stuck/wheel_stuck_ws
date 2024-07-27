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

#include "velodyne_cloud_separator/velodyne_cloud_separator.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <limits>
#include <vector>

namespace velodyne_cloud_separator
{
VelodyneCloudSeparator::VelodyneCloudSeparator(const rclcpp::NodeOptions & options)
: Node("velodyne_cloud_separator", options)
{
  // Declare Parameters
  double update_rate_hz = this->declare_parameter("update_rate_hz", 20.0);
  double sensor_height = this->declare_parameter("sensor_height_m", 1.0);
  double radius_coef_close = this->declare_parameter("radius_coef_close", 0.2);
  double radius_coef_far = this->declare_parameter("radius_coef_far", 0.7);
  float max_slope_deg = this->declare_parameter("max_slope_deg", 15.0);
  limiting_ratio_ = tan(max_slope_deg * M_PI / 180.0);
  gap_threshold_ = this->declare_parameter("gap_threshold_m", 0.1);
  min_points_num_ = this->declare_parameter("min_points_per_cluster", 2);

  // Declare Subscriptions
  {
    pc_sub_ = PointCloud2Subscription::create_subscription(this, "~/input/points");
  }

  // Declare Publishers
  {
    pc_ground_pub_ = this->create_publisher<PointCloud2>("~/output/ground_points", 1);
    pc_obstacle_pub_ = this->create_publisher<PointCloud2>("~/output/obstacle_points", 1);
  }

  // Declare timer for update function
  {
    auto update_callback = [this]() { this->update(); };
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / update_rate_hz));
    update_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(update_callback)>>(
      this->get_clock(), period, std::move(update_callback),
      this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(update_timer_, nullptr);
  }

  // Initialize
  init(sensor_height, radius_coef_close, radius_coef_far);
}

void VelodyneCloudSeparator::init(
  const double sensor_height, const double radius_coef_close, const double radius_coef_far)
{
  // HDL-32E unique parameters
  double angle_start = 92.0 / 3.0 * M_PI / 180.0;
  double angle_diff = 4.0 / 3.0 * M_PI / 180.0;

  radius_array_[0] = std::numeric_limits<double>::max();
  for (int i = 1; i < height_; i++) {
    if (i > 20) {
      radius_array_[i] = radius_array_[20];
      continue;
    }

    double angle = angle_start - angle_diff * i;
    radius_array_[i] = sensor_height * (1.0 / tan(angle) - 1.0 / tan(angle + angle_diff));
    radius_array_[i] *= (i <= 12 ? radius_coef_close : radius_coef_far);
  }
}

void VelodyneCloudSeparator::update()
{
  if (!try_subscribe_pc()) return;

  int points_size = pc_->points.size();

  const int width = static_cast<int>(points_size * 1.8 / height_);

  // Create index map
  std::vector<std::vector<int>> index_map(height_, std::vector<int>(width, -1));
  for (int i = 0; i < points_size; i++) {
    const auto & point = pc_->points[i];
    float angle = atan2(point.y, point.x) * 180.0 / M_PI;
    if (angle < 0.0) angle += 360.0;
    int row = height_ - 1 - point.ring;
    int col = width - 1 - static_cast<int>(static_cast<double>(width) * angle / 360.0);
    index_map[row][col] = i;
  }

  // Separate points
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_points(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < width; i++) {
    PointType point_types[height_] = {PointType::UNKNOWN};
    int point_indices[height_] = {0};
    int point_indices_size = 0;
    int unknown_indices[height_] = {0};
    int unknown_indices_size = 0;
    double z_ref = 0.0;
    double r_ref = 0.0;

    for (int j = height_ - 1; j >= 0; j--) {
      auto & point = pc_->points[index_map[j][i]];
      if (index_map[j][i] > -1 && point_types[j] == PointType::UNKNOWN) {
        const float x0 = point.x;
        const float y0 = point.y;
        const float z0 = point.z;
        const float r0 = sqrt(x0 * x0 + y0 * y0);
        const float r_diff = r0 - r_ref;
        const float z_diff = fabs(z0 - z_ref);
        const float angle = z_diff / r_diff;

        if (
          !(angle > 0 && angle < limiting_ratio_ && z_diff < gap_threshold_) ||
          point_indices_size != 0) {
          for (int k = 0; k < point_indices_size; k++) {
            int index = index_map[point_indices[k]][i];
            if (point_indices_size > min_points_num_) {
              auto & p = pc_->points[index];
              pcl::PointXYZ p_xyz;
              p_xyz.x = p.x;
              p_xyz.y = p.y;
              p_xyz.z = p.z;
              ground_points->push_back(p_xyz);
              point_types[point_indices[k]] = PointType::GROUND;
            } else {
              unknown_indices[unknown_indices_size++] = index;
            }
          }
          point_indices_size = 0;
        }
        point_indices[point_indices_size++] = j;
        z_ref = z0;
        r_ref = r0;
      }
      if (j != 0) continue;

      if (point_indices_size != 0) {
        for (int k = 0; k < point_indices_size; k++) {
          int index = index_map[point_indices[k]][i];
          if (point_indices_size > min_points_num_) {
            auto & p = pc_->points[index];
            pcl::PointXYZ p_xyz;
            p_xyz.x = p.x;
            p_xyz.y = p.y;
            p_xyz.z = p.z;
            ground_points->push_back(p_xyz);
            point_types[point_indices[k]] = PointType::GROUND;
          } else {
            unknown_indices[unknown_indices_size++] = index;
          }
        }
        point_indices_size = 0;
      }

      double centroid = 0;
      int centroid_ring = 0;
      int cluster_indices[height_] = {0};
      int cluster_indices_size = 0;
      for (int k = unknown_indices_size - 1; k >= 0; k--) {
        float x0 = pc_->points[unknown_indices[k]].x;
        float y0 = pc_->points[unknown_indices[k]].y;
        float r0 = sqrt(x0 * x0 + y0 * y0);
        float r_diff = fabs(r0 - centroid);
        float dist = radius_array_[centroid_ring];
        if (r_diff >= dist && cluster_indices_size != 0) {
          for (int l = 0; l < cluster_indices_size; l++) {
            auto & p = pc_->points[cluster_indices[l]];
            pcl::PointXYZ p_xyz;
            p_xyz.x = p.x;
            p_xyz.y = p.y;
            p_xyz.z = p.z;
            (cluster_indices_size > 1 ? obstacle_points : ground_points)->push_back(p_xyz);
          }
          cluster_indices_size = 0;
        }
        cluster_indices[cluster_indices_size++] = unknown_indices[k];
        centroid = r0;
        centroid_ring = pc_->points[unknown_indices[k]].ring;
        if (k != 0) continue;
        for (int l = 0; l < cluster_indices_size; l++) {
          auto & p = pc_->points[cluster_indices[l]];
          pcl::PointXYZ p_xyz;
          p_xyz.x = p.x;
          p_xyz.y = p.y;
          p_xyz.z = p.z;
          (cluster_indices_size > 1 ? obstacle_points : ground_points)->push_back(p_xyz);
        }
        cluster_indices_size = 0;
      }
    }
  }

  // Publish
  {
    sensor_msgs::msg::PointCloud2 pc_ground_msg;
    pcl::toROSMsg(*ground_points, pc_ground_msg);
    pc_ground_msg.header.frame_id = pc_->header.frame_id;
    pc_ground_msg.header.stamp = this->now();
    pc_ground_pub_->publish(pc_ground_msg);
  }
  {
    sensor_msgs::msg::PointCloud2 pc_obstacle_msg;
    pcl::toROSMsg(*obstacle_points, pc_obstacle_msg);
    pc_obstacle_msg.header.frame_id = pc_->header.frame_id;
    pc_obstacle_msg.header.stamp = this->now();
    pc_obstacle_pub_->publish(pc_obstacle_msg);
  }
}

bool VelodyneCloudSeparator::try_subscribe_pc()
{
  auto pc_msg = pc_sub_->get_data();
  if (!pc_msg) return false;
  try {
    pc_ = pcl::PointCloud<PointXYZIR>::Ptr(new pcl::PointCloud<PointXYZIR>);
    pcl::fromROSMsg(*pc_msg, *pc_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to convert PointCloud2 to pcl::PointCloud: %s", e.what());
    return false;
  }
  return true;
}
}  // namespace velodyne_cloud_separator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(velodyne_cloud_separator::VelodyneCloudSeparator)
