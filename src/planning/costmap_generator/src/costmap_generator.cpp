#include "costmap_generator/costmap_generator.hpp"

namespace costmap_generator
{

CostmapGenerator::CostmapGenerator(const rclcpp::NodeOptions & options)
: Node("costmap_generator", options)
{
  // Declare Subscribers
  {
    pc_sub_ = this->create_subscription<PointCloud2>(
      "~/input/pointcloud", 1, [this](const PointCloud2::SharedPtr msg) { this->callback(msg); });
  }
}

void CostmapGenerator::callback(const PointCloud2::SharedPtr msg)
{
  std::cout << "Received" << std::endl;
}

}  // namespace costmap_generator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(costmap_generator::CostmapGenerator)
