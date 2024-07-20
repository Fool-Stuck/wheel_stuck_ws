#include <joy2twist/joy2twist.hpp>

namespace joy2twist
{

Joy2Twist::Joy2Twist(const rclcpp::NodeOptions & options) : Node("joy2twist")
{
  // 受信機を作る。型はjoyで受け取る。10個まで値が保持される。
  joy_sub_ = this->create_subscription<Joymsg>(
    "joy", 10, std::bind(&Joy2Twist::joy_callback, this, std::placeholders::_1));
  // 送信機を作る。
  twist_pub_ = this->create_publisher<Twistmsg>("cmd_vel", 10);
}

void Joy2Twist::joy_callback(const Joymsg::SharedPtr msg)
{
  auto twist = Twistmsg();

  float linear_x = msg->axes[1];   // 前後方向の入力
  float angular_z = msg->axes[0];  // 左右方向の入力

  twist.linear.x = linear_x;
  twist.angular.z = angular_z;

  twist_pub_->publish(twist);
}

}  // namespace joy2twist
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(joy2twist::Joy2Twist)

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<joy2twist::Joy2Twist>());
  rclcpp::shutdown();
  return 0;
}
