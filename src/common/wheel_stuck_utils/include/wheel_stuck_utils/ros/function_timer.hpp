#ifndef WHEEL_STUCK_UTILS__ROS__FUNCTION_TIMER_HPP_
#define WHEEL_STUCK_UTILS__ROS__FUNCTION_TIMER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <utility>

namespace wheel_stuck_utils::ros
{

class FunctionTimer
{
private:
  rclcpp::TimerBase::SharedPtr timer_;

public:
  using SharedPtr = std::shared_ptr<FunctionTimer>;

  static SharedPtr create_function_timer(
    rclcpp::Node * node, const double update_rate_hz, std::function<void()> callback)
  {
    return std::make_shared<FunctionTimer>(node, update_rate_hz, callback);
  }

  explicit FunctionTimer(
    rclcpp::Node * node, const double update_rate_hz, std::function<void()> callback)
  {
    const double dt = 1.0 / update_rate_hz;

    auto period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(dt));
    timer_ = std::make_shared<rclcpp::GenericTimer<decltype(callback)>>(
      node->get_clock(), period, std::move(callback),
      node->get_node_base_interface()->get_context());
    node->get_node_timers_interface()->add_timer(timer_, nullptr);
  }
};

}  // namespace wheel_stuck_utils::ros

#endif  // WHEEL_STUCK_UTILS__ROS__FUNCTION_TIMER_HPP_
