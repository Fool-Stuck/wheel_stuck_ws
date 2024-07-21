#include "simple_int_subscriber/polling_int_subscriber.hpp"

namespace simple_int_subscriber
{

PollingIntSubscriber::PollingIntSubscriber(const rclcpp::NodeOptions & options)
: Node("polling_int_subscriber", options)
{
  double update_rate_hz = this->declare_parameter("update_rate_hz", 1.0);

  // Declare Subscribers
  {
    sub_ = Int32Subscription::create_subscription(this, "~/input/int");
  }

  // Declare timer for update function
  {
    int update_dt_ms = static_cast<int>(1000.0 / update_rate_hz);
    update_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(update_dt_ms), std::bind(&PollingIntSubscriber::update, this));
    // auto update_callback = [this]() { this->update(); };
    // auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    //   std::chrono::duration<double>(1.0 / update_rate_hz));
    // update_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(update_callback)>>(
    //   this->get_clock(), period, std::move(update_callback),
    //   this->get_node_base_interface()->get_context());
    // this->get_node_timers_interface()->add_timer(update_timer_, nullptr);
  }
}

void PollingIntSubscriber::update()
{
  const auto msg = sub_->get_data();
  if (msg) {
    std::cout << "Received: '" << msg->data << "'" << std::endl;
  } else {
    std::cout << "No message received" << std::endl;
  }
}
}  // namespace simple_int_subscriber

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(simple_int_subscriber::PollingIntSubscriber)
