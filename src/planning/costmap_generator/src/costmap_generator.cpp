#include "costmap_generator/costmap_generator.hpp"

#include <pcl_conversions/pcl_conversions.h>

namespace costmap_generator
{

CostmapGenerator::CostmapGenerator(const rclcpp::NodeOptions & options)
: Node("costmap_generator", options)
{
  // Declare Parameters
  costmap_frame_id = this->declare_parameter("costmap_frame_id", "base_link");
  costmap_resolution = this->declare_parameter("costmap_resolution", 1.0);
  costmap_width = this->declare_parameter("costmap_width", 10);
  costmap_height = this->declare_parameter("costmap_height", 10);

  // Declare Subscribers
  {
    pc_sub_ = this->create_subscription<PointCloud2>(
      "~/input/pointcloud", 1, [this](const PointCloud2::SharedPtr msg) { this->callback(msg); });
  }

  // Declare Publishers
  {
    costmap_pub_ = this->create_publisher<OccupancyGrid>("~/output/costmap", 1);
  }
}

void CostmapGenerator::callback(const PointCloud2::SharedPtr msg)
{
  pc_ = PointCloudXYZ::Ptr(new PointCloudXYZ);
  pcl::fromROSMsg(*msg, *pc_);

  OccupancyGrid costmap;

  costmap.header.stamp = this->now();
  costmap.header.frame_id = costmap_frame_id;

  costmap.info.map_load_time = this->now();
  costmap.info.resolution = costmap_resolution;                                 // [m/cell]
  costmap.info.width = costmap_width;                                           // [cell]
  costmap.info.height = costmap_height;                                         // [cell]
  costmap.info.origin.position.x = -(costmap_width / 2) * costmap_resolution;   // [m]
  costmap.info.origin.position.y = -(costmap_height / 2) * costmap_resolution;  // [m]
  costmap.info.origin.position.z = 0.0;                                         // [m]
  costmap.info.origin.orientation.x = 0.0;
  costmap.info.origin.orientation.y = 0.0;
  costmap.info.origin.orientation.z = 0.0;
  costmap.info.origin.orientation.w = 1.0;

  costmap.data.resize(costmap_width * costmap_height);

  int index;
  for (const auto & point : pc_->points) {
    index = point_index_convertor::Point2GridIndex(point, costmap);
    if (index != -1) continue;

    costmap.data[index] += 10;
  }

  costmap_pub_->publish(costmap);
}
}  // namespace costmap_generator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(costmap_generator::CostmapGenerator)
