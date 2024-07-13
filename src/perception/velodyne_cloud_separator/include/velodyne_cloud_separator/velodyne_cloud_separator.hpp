#ifndef VELODYNE_CLOUD_SEPARATOR__VELODYNE_CLOUD_SEPARATOR_HPP_
#define VELODYNE_CLOUD_SEPARATOR__VELODYNE_CLOUD_SEPARATOR_HPP_

#include "point_types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <wheel_stuck_utils/ros/no_callback_subscription.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>

namespace velodyne_cloud_separator
{

using PointCloud2 = sensor_msgs::msg::PointCloud2;

using PointCloud2Subscription = wheel_stuck_utils::ros::NoCallbackSubscription<PointCloud2>;
using PointCloud2Publisher = rclcpp::Publisher<PointCloud2>;

class VelodyneCloudSeparator : public rclcpp::Node
{
public:
  explicit VelodyneCloudSeparator(const rclcpp::NodeOptions & options);
  void update();

  rclcpp::TimerBase::SharedPtr update_timer_;

private:
  bool try_subscribe_pc();

  PointCloud2Subscription::SharedPtr pc_sub_;
  PointCloud2Publisher::SharedPtr pc_ground_pub_;
  PointCloud2Publisher::SharedPtr pc_obstacle_pub_;

  pcl::PointCloud<PointXYZIR>::Ptr pc_;
};
}  // namespace velodyne_cloud_separator

#endif  // VELODYNE_CLOUD_SEPARATOR__VELODYNE_CLOUD_SEPARATOR_HPP_
