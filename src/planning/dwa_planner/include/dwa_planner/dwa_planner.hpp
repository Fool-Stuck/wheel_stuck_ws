#ifndef DWA_PLANNER__DWA_PLANNER_HPP_
#define DWA_PLANNER__DWA_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>

namespace dwa_planner {
class DWAPlanner : public rclcpp::Node {
 public:
  explicit DWAPlanner(const rclcpp::NodeOptions& options);
};
}  // namespace dwa_planner

#endif