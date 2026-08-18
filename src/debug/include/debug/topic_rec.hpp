#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/serialized_message.hpp"

namespace debug
{

struct TopicSpec
{
  std::string name;
  std::string type;
};

class TopicRec : public rclcpp::Node
{
public:
  using MessageCallback = std::function<void(
    const std::string & topic_name,
    const std::string & topic_type,
    std::shared_ptr<rclcpp::SerializedMessage> message)>;

  explicit TopicRec(
    MessageCallback callback,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("topic_rec", options),
    callback_(std::move(callback))
  {
  }

  std::optional<std::string> addSubscriber(
    const std::string & topic_name,
    const std::string & topic_type,
    const rclcpp::QoS & qos = rclcpp::SystemDefaultsQoS())
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (topic_name.empty()) {
      return "topic name is empty";
    }

    if (topic_type.empty()) {
      return "topic type is empty";
    }

    if (subscriptions_.find(topic_name) != subscriptions_.end()) {
      return std::nullopt;
    }

    try {
      auto subscription = this->create_generic_subscription(
        topic_name,
        topic_type,
        qos,
        [this, topic_name, topic_type](std::shared_ptr<rclcpp::SerializedMessage> message) {
          if (callback_) {
            callback_(topic_name, topic_type, std::move(message));
          }
        });

      subscriptions_.emplace(topic_name, std::move(subscription));
      topic_types_.emplace(topic_name, topic_type);
    } catch (const std::exception & exception) {
      return std::string("failed to subscribe ") + topic_name + " (" + topic_type + "): " +
             exception.what();
    }

    return std::nullopt;
  }

  std::optional<std::string> removeSubscriber(const std::string & topic_name)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    const auto subscription_iter = subscriptions_.find(topic_name);
    if (subscription_iter == subscriptions_.end()) {
      return "topic is not monitored: " + topic_name;
    }

    topic_types_.erase(topic_name);
    subscriptions_.erase(subscription_iter);
    return std::nullopt;
  }

  std::optional<std::string> addSubscribers(
    const std::vector<TopicSpec> & topics,
    const rclcpp::QoS & qos = rclcpp::SystemDefaultsQoS())
  {
    for (const auto & topic : topics) {
      auto error = addSubscriber(topic.name, topic.type, qos);
      if (error.has_value()) {
        return error;
      }
    }

    return std::nullopt;
  }

  bool hasSubscriber(const std::string & topic_name) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return subscriptions_.find(topic_name) != subscriptions_.end();
  }

  std::vector<TopicSpec> subscriptions() const
  {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<TopicSpec> topics;
    topics.reserve(topic_types_.size());

    for (const auto & [name, type] : topic_types_) {
      topics.push_back(TopicSpec{name, type});
    }

    return topics;
  }

  std::vector<TopicSpec> broadcastingTopics() const
  {
    std::vector<TopicSpec> topics;
    const auto topic_names_and_types = this->get_topic_names_and_types();

    for (const auto & [topic_name, topic_types] : topic_names_and_types) {
      for (const auto & topic_type : topic_types) {
        topics.push_back(TopicSpec{topic_name, topic_type});
      }
    }

    return topics;
  }

private:
  MessageCallback callback_;
  mutable std::mutex mutex_;
  std::unordered_map<std::string, rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  std::unordered_map<std::string, std::string> topic_types_;
};

}  // namespace debug
