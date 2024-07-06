#ifndef COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_
#define COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace costmap_generator
{

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2Subscription = rclcpp::Subscription<PointCloud2>;

class CostmapGenerator : public rclcpp::Node
{
public:
  explicit CostmapGenerator(const rclcpp::NodeOptions & options);

private:
  PointCloud2Subscription::SharedPtr pc_sub_;
  void callback(const PointCloud2::SharedPtr msg);
};

}  // namespace costmap_generator

#endif  // COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_
