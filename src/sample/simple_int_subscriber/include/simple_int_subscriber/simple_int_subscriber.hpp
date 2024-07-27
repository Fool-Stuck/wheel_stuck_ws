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
