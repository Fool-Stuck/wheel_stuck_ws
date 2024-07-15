#ifndef WHEEL_STUCK_UTILS__ROS__TIMER_DRIVEN_NODE_HPP_
#define WHEEL_STUCK_UTILS__ROS__TIMER_DRIVEN_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>

namespace wheel_stuck_utils::ros
{
class TimerDrivenNode : public rclcpp::Node
{
protected:
  explicit TimerDrivenNode(
    const std::string & node_name, const rclcpp::NodeOptions & options, const double update_rate_hz)
  : Node(node_name, options)
  {
    // Declare timer for update function
    {
      const double dt = 1.0 / update_rate_hz;

      auto update_callback = [this]() { this->on_update(); };
      auto period =
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(dt));
      update_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(update_callback)>>(
        this->get_clock(), period, std::move(update_callback),
        this->get_node_base_interface()->get_context());
      this->get_node_timers_interface()->add_timer(update_timer_, nullptr);
    }

    this->on_init();
  }

  virtual void on_init() = 0;
  virtual void on_update() = 0;

private:
  rclcpp::TimerBase::SharedPtr update_timer_;
};
}  // namespace wheel_stuck_utils::ros

#endif  // WHEEL_STUCK_UTILS__ROS__TIMER_DRIVEN_NODE_HPP_
