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
