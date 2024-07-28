#ifndef COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_
#define COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <wheel_stuck_planning_utils/point_index_convertor.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

namespace costmap_generator
{

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2Subscription = rclcpp::Subscription<PointCloud2>;
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using OccupancyGridPublisher = rclcpp::Publisher<OccupancyGrid>;

using PointXYZ = pcl::PointXYZ;
using PointCloudXYZ = pcl::PointCloud<PointXYZ>;

class CostmapGenerator : public rclcpp::Node
{
public:
  explicit CostmapGenerator(const rclcpp::NodeOptions & options);

private:
  PointCloud2Subscription::SharedPtr pc_sub_;
  OccupancyGridPublisher::SharedPtr costmap_pub_;

  void callback(const PointCloud2::SharedPtr msg);

  PointCloudXYZ::Ptr pc_;

  std::string costmap_frame_id;
  double costmap_resolution;
  int costmap_width;
  int costmap_height;
};

}  // namespace costmap_generator

#endif  // COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_
