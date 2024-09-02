// Copyright 2024 Fool Stuck Engineers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <joy2twist/joy2twist.hpp>

namespace joy2twist
{
Joy2Twist::Joy2Twist(const rclcpp::NodeOptions & options) : Node("joy2twist", options)
{
  // パラメータの取得。
  std::string linear_x_axis_str = declare_parameter<std::string>("linear_x_axis", "LEFT_STICK_Y");
  std::string angular_z_axis_str = declare_parameter<std::string>("angular_z_axis", "LEFT_STICK_X");

  // 軸名を対応する数値に変換
  linear_x_axis_ = axis_map.at(linear_x_axis_str);
  angular_z_axis_ = axis_map.at(angular_z_axis_str);

  // 受信機を作る。型はjoyで受け取る。10個まで値が保持される。
  joy_sub_ = this->create_subscription<Joy>(
    "~/input/joy", 10, std::bind(&Joy2Twist::joy_callback, this, std::placeholders::_1));
  // 送信機を作る。
  twist_pub_ = this->create_publisher<TwistStamped>("~/output/cmd_vel", 10);
}

void Joy2Twist::joy_callback(const Joy::SharedPtr msg)
{
  float linear_x = msg->axes[linear_x_axis_];    // 前後方向の入力
  float angular_z = msg->axes[angular_z_axis_];  // 左右方向の入力

  auto twist = TwistStamped();
  twist.header.stamp = this->now();
  twist.twist.linear.x = static_cast<double>(linear_x);
  twist.twist.angular.z = static_cast<double>(angular_z);

  twist_pub_->publish(twist);
}

}  // namespace joy2twist
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(joy2twist::Joy2Twist)
