#ifndef SIMPLE_INT_SUBSCRIBER__POLLING_INT_SUBSCRIBER_HPP_
#define SIMPLE_INT_SUBSCRIBER__POLLING_INT_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <wheel_stuck_utils/ros/no_callback_subscription.hpp>

#include <std_msgs/msg/int32.hpp>

namespace simple_int_subscriber
{

using Int32 = std_msgs::msg::Int32;

using Int32Subscription = wheel_stuck_utils::ros::NoCallbackSubscription<Int32>;

class PollingIntSubscriber : public rclcpp::Node
{
public:
  explicit PollingIntSubscriber(const rclcpp::NodeOptions & options);

private:
  void update();

  rclcpp::TimerBase::SharedPtr update_timer_;
  Int32Subscription::SharedPtr sub_;
};

}  // namespace simple_int_subscriber

#endif  // SIMPLE_INT_SUBSCRIBER__POLLING_INT_SUBSCRIBER_HPP_
