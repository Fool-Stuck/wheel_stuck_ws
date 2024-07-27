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
