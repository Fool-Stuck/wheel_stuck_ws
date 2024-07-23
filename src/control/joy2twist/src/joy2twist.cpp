#include <joy2twist/joy2twist.hpp>

namespace joy2twist
{

Joy2Twist::Joy2Twist(const rclcpp::NodeOptions & options) : Node("joy2twist", options)
{
  // パラメータの取得
  linear_x_axis_ = declare_parameter<int>("linear_x_axis", 1);
  angular_z_axis_ = declare_parameter<int>("angular_z_axis", 0);

  // 受信機を作る。型はjoyで受け取る。10個まで値が保持される。
  joy_sub_ = this->create_subscription<Joy>(
    "~/input/joy", 10, std::bind(&Joy2Twist::joy_callback, this, std::placeholders::_1));
  // 送信機を作る。
  twist_pub_ = this->create_publisher<Twist>("~/output/cmd_vel", 10);
}

void Joy2Twist::joy_callback(const Joy::SharedPtr msg)
{
  auto twist = Twist();

  float linear_x = msg->axes[linear_x_axis_];    // 前後方向の入力
  float angular_z = msg->axes[angular_z_axis_];  // 左右方向の入力

  twist.linear.x = linear_x;
  twist.angular.z = angular_z;

  twist_pub_->publish(twist);
}

}  // namespace joy2twist
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(joy2twist::Joy2Twist)
