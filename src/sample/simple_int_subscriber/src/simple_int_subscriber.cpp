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

#include "simple_int_subscriber/simple_int_subscriber.hpp"

namespace simple_int_subscriber
{

SimpleIntSubscriber::SimpleIntSubscriber(const rclcpp::NodeOptions & options)
: Node("simple_int_subscriber", options)
{
  // Declare Subscribers
  {
    sub_ = this->create_subscription<Int32>(
      "~/input/int", 1, [this](const Int32::SharedPtr msg) { this->callback(msg); });
  }
}

void SimpleIntSubscriber::callback(const Int32::SharedPtr msg)
{
  std::cout << "Received: '" << msg->data << "'" << std::endl;
}

}  // namespace simple_int_subscriber

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(simple_int_subscriber::SimpleIntSubscriber)
