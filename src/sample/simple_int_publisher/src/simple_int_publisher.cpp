#include "simple_int_publisher/simple_int_publisher.hpp"

namespace simple_int_publisher
{

SimpleIntPublisher::SimpleIntPublisher(const rclcpp::NodeOptions & options)
: TimerDrivenNode("simple_int_publisher", options, this->declare_parameter("update_rate_hz", 1.0))
{
}

void SimpleIntPublisher::on_init()
{
  // pub_ = this->create_publisher<Int32>("~/output/int", 1);
  counter_ = 0;
}

void SimpleIntPublisher::on_update()
{
  Int32 msg;
  msg.data = counter_;
  pub_->publish(msg);

  std::cout << "Published: '" << msg.data << "'" << std::endl;

  counter_++;
}

}  // namespace simple_int_publisher

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(simple_int_publisher::SimpleIntPublisher)
