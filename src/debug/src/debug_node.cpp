#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "debug/cli.hpp"
#include "debug/shower.hpp"
#include "debug/topic_manager.hpp"
#include "debug/topic_rec.hpp"
#include "plugins/drawpoints.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

std::vector<debug::TopicSpec> parseTopicSpecs(const std::vector<std::string> & raw_topics)
{
  std::vector<debug::TopicSpec> topics;
  topics.reserve(raw_topics.size());

  for (const auto & raw_topic : raw_topics) {
    const auto separator = raw_topic.find(':');
    if (separator == std::string::npos || separator == 0 || separator == raw_topic.size() - 1) {
      throw std::invalid_argument(
        "topic spec must use format '<topic_name>:<topic_type>', got '" + raw_topic + "'");
    }

    topics.push_back(debug::TopicSpec{
      raw_topic.substr(0, separator),
      raw_topic.substr(separator + 1)});
  }

  return topics;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto topic_manager = std::make_shared<debug::TopicManager>();
  auto shower = std::make_shared<debug::Shower>(*topic_manager);
  shower->addPlugin(std::make_shared<plugins::DrawPoints>());
  auto topic_rec = std::make_shared<debug::TopicRec>(
    [topic_manager, shower](
      const std::string & topic_name,
      const std::string & topic_type,
      std::shared_ptr<rclcpp::SerializedMessage> message) {
      const auto timestamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
      topic_manager->onTopicMessage(topic_name, topic_type, timestamp, std::move(message));

      const auto latest_message = topic_manager->latestMessage(topic_name);
      if (latest_message.has_value()) {
        shower->onTopicMessage(*latest_message);
      }
    });

  topic_rec->declare_parameter<std::vector<std::string>>("topics", std::vector<std::string>{});

  try {
    const auto topic_specs = parseTopicSpecs(
      topic_rec->get_parameter("topics").as_string_array());
    const auto error = topic_rec->addSubscribers(topic_specs);
    if (error.has_value()) {
      RCLCPP_ERROR(topic_rec->get_logger(), "%s", error->c_str());
      rclcpp::shutdown();
      return 1;
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(topic_rec->get_logger(), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(topic_rec->get_logger(), "topic receiver started");

  auto cli = std::make_shared<debug::Cli>(
    [topic_rec]() {
      return topic_rec->broadcastingTopics();
    },
    [topic_rec](const debug::TopicSpec & topic) {
      return topic_rec->addSubscriber(topic.name, topic.type);
    },
    [topic_rec](const std::string & topic_name) {
      return topic_rec->removeSubscriber(topic_name);
    });
  shower->setDisplaySink([cli](std::vector<debug::DisplayItem> items) {
    cli->setDisplayItems(std::move(items));
  });

  std::thread cli_thread([cli]() {
    cli->run();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  });

  rclcpp::spin(topic_rec);
  cli->requestStop();
  if (cli_thread.joinable()) {
    cli_thread.join();
  }
  rclcpp::shutdown();
  return 0;
}
