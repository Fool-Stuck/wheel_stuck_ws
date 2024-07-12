#include "simple_int_publisher/simple_int_publisher.hpp"

namespace simple_int_publisher
{

SimpleIntPublisher::SimpleIntPublisher(const rclcpp::NodeOptions & options)
: Node("simple_int_publisher", options)
{
  // Declare Parameters
  double update_rate_hz = this->declare_parameter("update_rate_hz", 1.0);

  // Declare Publishers
  {
    pub_ = this->create_publisher<Int32>("~/output/int", 1);
  }

  // Declare timer for update function
  {
    int update_dt_ms = static_cast<int>(1000.0 / update_rate_hz);
    update_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(update_dt_ms), std::bind(&SimpleIntPublisher::update, this));
    // auto update_callback = [this]() { this->update(); };
    // auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    //   std::chrono::duration<double>(1.0 / update_rate_hz));
    // update_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(update_callback)>>(
    //   this->get_clock(), period, std::move(update_callback),
    //   this->get_node_base_interface()->get_context());
    // this->get_node_timers_interface()->add_timer(update_timer_, nullptr);
  }

  // Reset counter
  counter_ = 0;
}

void SimpleIntPublisher::update()
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
