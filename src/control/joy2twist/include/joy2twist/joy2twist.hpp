#ifndef JOY2TWIST__JOY2TWIST_HPP_
#define JOY2TWIST__JOY2TWIST_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace joy2twist
{
using Joymsg = sensor_msgs::msg::Joy;
using Twistmsg = geometry_msgs::msg::Twist;

class Joy2Twist : public rclcpp::Node
{
public:
  explicit Joy2Twist(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void joy_callback(const Joymsg::SharedPtr msg);

  rclcpp::Subscription<Joymsg>::SharedPtr joy_sub_;
  rclcpp::Publisher<Twistmsg>::SharedPtr twist_pub_;
};

}  // namespace joy2twist

#endif  // JOY2TWIST__JOY2TWIST_HPP_
