#ifndef JOY2TWIST__JOY2TWIST_HPP_
#define JOY2TWIST__JOY2TWIST_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace joy2twist
{
using Joy = sensor_msgs::msg::Joy;
using Twist = geometry_msgs::msg::Twist;

using JoySubscription = rclcpp::Subscription<Joy>;
using TwistPublisher = rclcpp::Publisher<Twist>;

class Joy2Twist : public rclcpp::Node
{
public:
  explicit Joy2Twist(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void joy_callback(const Joy::SharedPtr msg);
  int linear_x_axis_;
  int angular_z_axis_;

  JoySubscription::SharedPtr joy_sub_;
  TwistPublisher::SharedPtr twist_pub_;
};

}  // namespace joy2twist

#endif  // JOY2TWIST__JOY2TWIST_HPP_
