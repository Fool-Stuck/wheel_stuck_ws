#include "velodyne_cloud_separator/velodyne_cloud_separator.hpp"

#include <pcl_conversions/pcl_conversions.h>

namespace velodyne_cloud_separator
{
VelodyneCloudSeparator::VelodyneCloudSeparator(const rclcpp::NodeOptions & options)
: Node("velodyne_cloud_separator", options)
{
  // Declare Parameters
  double update_rate_hz = this->declare_parameter("update_rate_hz", 20.0);

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
}

void VelodyneCloudSeparator::update()
{
  if (!try_subscribe_pc()) return;
}

bool VelodyneCloudSeparator::try_subscribe_pc()
{
  auto pc_msg = pc_sub_->getData();
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
