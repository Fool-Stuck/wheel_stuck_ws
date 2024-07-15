#ifndef SIMPLE_INT_PUBLISHER__SIMPLE_INT_PUBLISHER_HPP_
#define SIMPLE_INT_PUBLISHER__SIMPLE_INT_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <wheel_stuck_utils/ros/timer_driven_node.hpp>

#include <std_msgs/msg/int32.hpp>

namespace simple_int_publisher
{

using TimerDrivenNode = wheel_stuck_utils::ros::TimerDrivenNode;

using Int32 = std_msgs::msg::Int32;

using Int32Publisher = rclcpp::Publisher<Int32>;

class SimpleIntPublisher : public TimerDrivenNode
{
public:
  explicit SimpleIntPublisher(const rclcpp::NodeOptions & options);

protected:
  void on_init() override;
  void on_update() override;

private:
  Int32Publisher::SharedPtr pub_;
  int counter_;
};

}  // namespace simple_int_publisher

#endif  // SIMPLE_INT_PUBLISHER__SIMPLE_INT_PUBLISHER_HPP_
