#ifndef SIMPLE_INT_PUBLISHER__SIMPLE_INT_PUBLISHER_HPP_
#define SIMPLE_INT_PUBLISHER__SIMPLE_INT_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <wheel_stuck_utils/ros/function_timer.hpp>

#include <std_msgs/msg/int32.hpp>

namespace simple_int_publisher
{

using Int32 = std_msgs::msg::Int32;

using FunctionTimer = wheel_stuck_utils::ros::FunctionTimer;
using Int32Publisher = rclcpp::Publisher<Int32>;

class SimpleIntPublisher : public rclcpp::Node
{
public:
  explicit SimpleIntPublisher(const rclcpp::NodeOptions & options);

protected:
  void update();

private:
  FunctionTimer::SharedPtr update_timer_;
  Int32Publisher::SharedPtr pub_;
  int counter_;
};

}  // namespace simple_int_publisher

#endif  // SIMPLE_INT_PUBLISHER__SIMPLE_INT_PUBLISHER_HPP_
