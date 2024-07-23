#ifndef JOY2TWIST__JOY2TWIST_HPP_
#define JOY2TWIST__JOY2TWIST_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace joy2twist
{
using Joy = sensor_msgs::msg::Joy;
using TwistStamped = geometry_msgs::msg::TwistStamped;

using JoySubscription = rclcpp::Subscription<Joy>;
using TwistStampedPublisher = rclcpp::Publisher<TwistStamped>;

class Joy2Twist : public rclcpp::Node
{
public:
  explicit Joy2Twist(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void joy_callback(const Joy::SharedPtr msg);
  int linear_x_axis_;
  int angular_z_axis_;

  JoySubscription::SharedPtr joy_sub_;
  TwistStampedPublisher::SharedPtr twist_pub_;
};

}  // namespace joy2twist

#endif  // JOY2TWIST__JOY2TWIST_HPP_
