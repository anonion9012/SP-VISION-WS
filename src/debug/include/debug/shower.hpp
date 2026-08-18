#pragma once

#include <algorithm>
#include <any>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/byte.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "debug/topic_manager.hpp"

namespace debug
{

enum class DisplayKind
{
  Image,
  Text,
  Number,
  Boolean,
  Extra,
};

struct DisplayItem
{
  DisplayKind kind = DisplayKind::Extra;
  std::string topic_name;
  std::string topic_type;
  std::string label;
  rclcpp::Time timestamp;
  cv::Mat image;
  std::string text;
  double number = 0.0;
  bool boolean = false;
  std::any extra;
  std::unordered_map<std::string, std::string> metadata;
};

struct ShowerContext
{
  const TopicMessage & message;
  const TopicManager & topic_manager;
};

class ShowerPlugin
{
public:
  virtual ~ShowerPlugin() = default;
  virtual void process(const ShowerContext & context, std::vector<DisplayItem> & items) = 0;
};

class Shower
{
public:
  using PluginCallback = std::function<void(const ShowerContext &, std::vector<DisplayItem> &)>;
  using DisplaySink = std::function<void(std::vector<DisplayItem>)>;

  explicit Shower(const TopicManager & topic_manager, std::size_t max_cli_rows = 80)
  : topic_manager_(topic_manager),
    max_cli_rows_(std::max<std::size_t>(1, max_cli_rows))
  {
  }

  void addPlugin(std::shared_ptr<ShowerPlugin> plugin)
  {
    if (!plugin) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    plugins_.push_back(std::move(plugin));
  }

  void addPluginCallback(PluginCallback callback)
  {
    if (!callback) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    plugin_callbacks_.push_back(std::move(callback));
  }

  void setDisplaySink(DisplaySink sink)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    display_sink_ = std::move(sink);
  }

  void onTopicMessage(const TopicMessage & message)
  {
    auto items = decodeMessage(message);
    if (items.empty()) {
      return;
    }

    const ShowerContext context{message, topic_manager_};
    runPlugins(context, items);
    displayItems(items);
  }

private:
  template<typename MessageT>
  static std::optional<MessageT> deserializeMessage(const TopicMessage & message)
  {
    if (!message.message) {
      return std::nullopt;
    }

    try {
      MessageT typed_message;
      rclcpp::Serialization<MessageT> serializer;
      serializer.deserialize_message(message.message.get(), &typed_message);
      return typed_message;
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }

  template<typename MessageT, typename ValueT>
  std::vector<DisplayItem> decodeNumberMessage(
    const TopicMessage & message,
    const std::string & label,
    ValueT value) const
  {
    auto typed_message = deserializeMessage<MessageT>(message);
    if (!typed_message.has_value()) {
      return {};
    }

    DisplayItem item;
    item.kind = DisplayKind::Number;
    item.topic_name = message.topic_name;
    item.topic_type = message.topic_type;
    item.label = label;
    item.timestamp = message.timestamp;
    item.number = static_cast<double>((*typed_message).*value);
    return {std::move(item)};
  }

  std::vector<DisplayItem> decodeMessage(const TopicMessage & message) const
  {
    if (message.topic_type == "sensor_msgs/msg/Image") {
      return decodeImageMessage(message);
    }

    if (message.topic_type == "std_msgs/msg/String") {
      auto typed_message = deserializeMessage<std_msgs::msg::String>(message);
      if (!typed_message.has_value()) {
        return {};
      }

      DisplayItem item;
      item.kind = DisplayKind::Text;
      item.topic_name = message.topic_name;
      item.topic_type = message.topic_type;
      item.label = "string";
      item.timestamp = message.timestamp;
      item.text = typed_message->data;
      return {std::move(item)};
    }

    if (message.topic_type == "std_msgs/msg/Bool") {
      auto typed_message = deserializeMessage<std_msgs::msg::Bool>(message);
      if (!typed_message.has_value()) {
        return {};
      }

      DisplayItem item;
      item.kind = DisplayKind::Boolean;
      item.topic_name = message.topic_name;
      item.topic_type = message.topic_type;
      item.label = "bool";
      item.timestamp = message.timestamp;
      item.boolean = typed_message->data;
      return {std::move(item)};
    }

    if (message.topic_type == "std_msgs/msg/Byte") {
      return decodeNumberMessage<std_msgs::msg::Byte>(message, "byte", &std_msgs::msg::Byte::data);
    }
    if (message.topic_type == "std_msgs/msg/Char") {
      return decodeNumberMessage<std_msgs::msg::Char>(message, "char", &std_msgs::msg::Char::data);
    }
    if (message.topic_type == "std_msgs/msg/Int8") {
      return decodeNumberMessage<std_msgs::msg::Int8>(message, "int8", &std_msgs::msg::Int8::data);
    }
    if (message.topic_type == "std_msgs/msg/UInt8") {
      return decodeNumberMessage<std_msgs::msg::UInt8>(message, "uint8", &std_msgs::msg::UInt8::data);
    }
    if (message.topic_type == "std_msgs/msg/Int16") {
      return decodeNumberMessage<std_msgs::msg::Int16>(message, "int16", &std_msgs::msg::Int16::data);
    }
    if (message.topic_type == "std_msgs/msg/UInt16") {
      return decodeNumberMessage<std_msgs::msg::UInt16>(
        message, "uint16", &std_msgs::msg::UInt16::data);
    }
    if (message.topic_type == "std_msgs/msg/Int32") {
      return decodeNumberMessage<std_msgs::msg::Int32>(message, "int32", &std_msgs::msg::Int32::data);
    }
    if (message.topic_type == "std_msgs/msg/UInt32") {
      return decodeNumberMessage<std_msgs::msg::UInt32>(
        message, "uint32", &std_msgs::msg::UInt32::data);
    }
    if (message.topic_type == "std_msgs/msg/Int64") {
      return decodeNumberMessage<std_msgs::msg::Int64>(message, "int64", &std_msgs::msg::Int64::data);
    }
    if (message.topic_type == "std_msgs/msg/UInt64") {
      return decodeNumberMessage<std_msgs::msg::UInt64>(
        message, "uint64", &std_msgs::msg::UInt64::data);
    }
    if (message.topic_type == "std_msgs/msg/Float32") {
      return decodeNumberMessage<std_msgs::msg::Float32>(
        message, "float32", &std_msgs::msg::Float32::data);
    }
    if (message.topic_type == "std_msgs/msg/Float64") {
      return decodeNumberMessage<std_msgs::msg::Float64>(
        message, "float64", &std_msgs::msg::Float64::data);
    }

    return {};
  }

  std::vector<DisplayItem> decodeImageMessage(const TopicMessage & message) const
  {
    auto typed_message = deserializeMessage<sensor_msgs::msg::Image>(message);
    if (!typed_message.has_value()) {
      return {};
    }

    try {
      const auto converted = cv_bridge::toCvCopy(*typed_message, displayEncoding(*typed_message));

      DisplayItem item;
      item.kind = DisplayKind::Image;
      item.topic_name = message.topic_name;
      item.topic_type = message.topic_type;
      item.label = "image";
      item.timestamp = message.timestamp;
      item.image = converted->image.clone();
      item.metadata["encoding"] = typed_message->encoding;
      item.metadata["frame_id"] = typed_message->header.frame_id;
      return {std::move(item)};
    } catch (const std::exception &) {
      return {};
    }
  }

  static std::string displayEncoding(const sensor_msgs::msg::Image & image)
  {
    if (image.encoding == sensor_msgs::image_encodings::RGB8 ||
        image.encoding == sensor_msgs::image_encodings::RGBA8 ||
        image.encoding == sensor_msgs::image_encodings::BGR8 ||
        image.encoding == sensor_msgs::image_encodings::BGRA8) {
      return sensor_msgs::image_encodings::BGR8;
    }

    if (image.encoding == sensor_msgs::image_encodings::MONO8 ||
        image.encoding == sensor_msgs::image_encodings::MONO16) {
      return image.encoding;
    }

    return {};
  }

  void runPlugins(const ShowerContext & context, std::vector<DisplayItem> & items)
  {
    std::vector<std::shared_ptr<ShowerPlugin>> plugins;
    std::vector<PluginCallback> callbacks;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      plugins = plugins_;
      callbacks = plugin_callbacks_;
    }

    for (const auto & plugin : plugins) {
      plugin->process(context, items);
    }

    for (const auto & callback : callbacks) {
      callback(context, items);
    }
  }

  void displayItems(const std::vector<DisplayItem> & items)
  {
    bool should_render_cli = false;
    DisplaySink display_sink;
    std::vector<DisplayItem> display_items;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto & item : items) {
        if (item.kind == DisplayKind::Image) {
          showImage(item);
        } else {
          cli_items_.push_back(item);
          should_render_cli = true;
        }
      }

      pruneCliItems();
      display_sink = display_sink_;
      display_items = cli_items_;
    }

    if (should_render_cli) {
      if (display_sink) {
        display_sink(std::move(display_items));
      } else {
        renderCli();
      }
    }

    cv::waitKey(1);
  }

  static void showImage(const DisplayItem & item)
  {
    if (item.image.empty()) {
      return;
    }

    cv::imshow(windowName(item), item.image);

    cv::waitKey(1000 / 60);
  }

  static std::string windowName(const DisplayItem & item)
  {
    return item.topic_name.empty() ? std::string("debug image") : item.topic_name;
  }

  void pruneCliItems()
  {
    if (cli_items_.size() <= max_cli_rows_) {
      return;
    }

    const auto remove_count = cli_items_.size() - max_cli_rows_;
    cli_items_.erase(cli_items_.begin(), cli_items_.begin() + static_cast<std::ptrdiff_t>(remove_count));
  }

  void renderCli() const
  {
    std::vector<DisplayItem> items;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      items = cli_items_;
    }

    std::printf("\033[2J\033[H");
    const auto table = makeTable(items);
    constexpr std::size_t timestamp_width = 19;
    constexpr std::size_t topic_width = 32;

    std::printf("%-32s", "topic");
    for (const auto & timestamp : table.timestamps) {
      std::printf(" | %-19s", timestampText(timestamp).c_str());
    }
    std::printf("\n%s", std::string(topic_width, '-').c_str());
    for (std::size_t index = 0; index < table.timestamps.size(); ++index) {
      std::printf("-+-%s", std::string(timestamp_width, '-').c_str());
    }
    std::printf("\n");

    for (std::size_t row = 0; row < table.rows.size(); ++row) {
      std::printf("%-32s", fitText(table.rows[row], topic_width).c_str());
      for (std::size_t column = 0; column < table.timestamps.size(); ++column) {
        std::printf(" | %-19s", fitText(table.values[row][column], timestamp_width).c_str());
      }
      std::printf("\n");
    }

    std::fflush(stdout);
  }

  struct DisplayTable
  {
    std::vector<std::string> rows;
    std::vector<rclcpp::Time> timestamps;
    std::vector<std::vector<std::string>> values;
  };

  static DisplayTable makeTable(const std::vector<DisplayItem> & items)
  {
    DisplayTable table;
    std::map<std::string, std::size_t> row_indices;
    std::map<std::int64_t, std::size_t> timestamp_indices;

    for (const auto & item : items) {
      if (item.kind == DisplayKind::Image) {
        continue;
      }

      const auto row_name = item.topic_name.empty() ? kindText(item.kind) : item.topic_name;
      if (row_indices.find(row_name) == row_indices.end()) {
        row_indices.emplace(row_name, table.rows.size());
        table.rows.push_back(row_name);
      }

      const auto timestamp_ns = item.timestamp.nanoseconds();
      if (timestamp_indices.find(timestamp_ns) == timestamp_indices.end()) {
        timestamp_indices.emplace(timestamp_ns, table.timestamps.size());
        table.timestamps.push_back(item.timestamp);
      }
    }

    std::sort(table.rows.begin(), table.rows.end());
    std::sort(table.timestamps.begin(), table.timestamps.end(), [](const rclcpp::Time & lhs, const rclcpp::Time & rhs) {
      return lhs.nanoseconds() < rhs.nanoseconds();
    });

    row_indices.clear();
    timestamp_indices.clear();
    for (std::size_t index = 0; index < table.rows.size(); ++index) {
      row_indices.emplace(table.rows[index], index);
    }
    for (std::size_t index = 0; index < table.timestamps.size(); ++index) {
      timestamp_indices.emplace(table.timestamps[index].nanoseconds(), index);
    }

    table.values.assign(
      table.rows.size(), std::vector<std::string>(table.timestamps.size()));
    for (const auto & item : items) {
      if (item.kind == DisplayKind::Image) {
        continue;
      }

      const auto row_name = item.topic_name.empty() ? kindText(item.kind) : item.topic_name;
      const auto row_iter = row_indices.find(row_name);
      const auto timestamp_iter = timestamp_indices.find(item.timestamp.nanoseconds());
      if (row_iter == row_indices.end() || timestamp_iter == timestamp_indices.end()) {
        continue;
      }

      auto & value = table.values[row_iter->second][timestamp_iter->second];
      if (!value.empty()) {
        value += "; ";
      }
      value += valueText(item);
    }

    return table;
  }

  static std::string timestampText(const rclcpp::Time & timestamp)
  {
    const auto ns = timestamp.nanoseconds();
    const auto sec = ns / 1000000000LL;
    const auto nsec = std::llabs(ns % 1000000000LL);

    std::ostringstream stream;
    stream << sec << "." << std::setw(9) << std::setfill('0') << nsec;
    return stream.str();
  }

  static std::string kindText(DisplayKind kind)
  {
    switch (kind) {
      case DisplayKind::Image:
        return "image";
      case DisplayKind::Text:
        return "text";
      case DisplayKind::Number:
        return "number";
      case DisplayKind::Boolean:
        return "bool";
      case DisplayKind::Extra:
        return "extra";
    }

    return "extra";
  }

  static std::string valueText(const DisplayItem & item)
  {
    switch (item.kind) {
      case DisplayKind::Text:
        return item.text;
      case DisplayKind::Number: {
        std::ostringstream stream;
        stream << item.number;
        return stream.str();
      }
      case DisplayKind::Boolean:
        return item.boolean ? "true" : "false";
      case DisplayKind::Extra:
        return item.text.empty() ? item.label : item.text;
      case DisplayKind::Image:
        return item.image.empty() ? "empty" : "opencv window";
    }

    return {};
  }

  static std::string fitText(const std::string & text, std::size_t width)
  {
    if (text.size() <= width) {
      return text;
    }

    if (width <= 3) {
      return text.substr(0, width);
    }

    return text.substr(0, width - 3) + "...";
  }

  const TopicManager & topic_manager_;
  std::size_t max_cli_rows_;
  mutable std::mutex mutex_;
  std::vector<std::shared_ptr<ShowerPlugin>> plugins_;
  std::vector<PluginCallback> plugin_callbacks_;
  std::vector<DisplayItem> cli_items_;
  DisplaySink display_sink_;
};

}  // namespace debug
