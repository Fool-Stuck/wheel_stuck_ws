#ifndef WHEEL_STUCK_UTILS__ROS__NO_CALLBACK_SUBSCRIPTION_HPP_
#define WHEEL_STUCK_UTILS__ROS__NO_CALLBACK_SUBSCRIPTION_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace wheel_stuck_utils::ros
{

template <typename TMessage>
class NoCallbackSubscription
{
private:
  typename rclcpp::Subscription<TMessage>::SharedPtr subscription_;

public:
  using SharedPtr = std::shared_ptr<NoCallbackSubscription<TMessage>>;
  static SharedPtr create_subscription(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos = rclcpp::QoS{1})
  {
    return std::make_shared<NoCallbackSubscription<TMessage>>(node, topic_name, qos);
  }

  explicit NoCallbackSubscription(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos = rclcpp::QoS{1})
  {
    auto noexec_callback_group =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto noexec_subscription_options = rclcpp::SubscriptionOptions();
    noexec_subscription_options.callback_group = noexec_callback_group;

    subscription_ = node->create_subscription<TMessage>(
      topic_name, qos,
      [node]([[maybe_unused]] const typename TMessage::ConstSharedPtr msg) { assert(false); },
      noexec_subscription_options);
  }

  typename TMessage::ConstSharedPtr get_data()
  {
    auto data = std::make_shared<TMessage>();
    rclcpp::MessageInfo message_info;
    const bool is_received = subscription_->take(*data, message_info);
    return is_received ? data : nullptr;
  }
};

}  // namespace wheel_stuck_utils::ros

#endif  // WHEEL_STUCK_UTILS__ROS__NO_CALLBACK_SUBSCRIPTION_HPP_
