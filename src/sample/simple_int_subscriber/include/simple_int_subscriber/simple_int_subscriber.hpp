#ifndef SIMPLE_INT_SUBSCRIBER__SIMPLE_INT_SUBSCRIBER_HPP_
#define SIMPLE_INT_SUBSCRIBER__SIMPLE_INT_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int32.hpp>

namespace simple_int_subscriber
{

using Int32 = std_msgs::msg::Int32;

using Int32Subscription = rclcpp::Subscription<Int32>;

class SimpleIntSubscriber : public rclcpp::Node
{
public:
  explicit SimpleIntSubscriber(const rclcpp::NodeOptions & options);

private:
  void callback(const Int32::SharedPtr msg);

  Int32Subscription::SharedPtr sub_;
};

}  // namespace simple_int_subscriber

#endif  // SIMPLE_INT_SUBSCRIBER__SIMPLE_INT_SUBSCRIBER_HPP_
