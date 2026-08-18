#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>

#include "debug/shower.hpp"

namespace plugins
{

class DrawPoints : public debug::ShowerPlugin
{
public:
  explicit DrawPoints(
    std::int64_t tolerance_ns = 5000000,
    cv::Scalar line_color = cv::Scalar(0, 255, 0),
    cv::Scalar point_color = cv::Scalar(0, 0, 255),
    int line_thickness = 2,
    int point_radius = 4)
  : tolerance_(tolerance_ns, RCL_SYSTEM_TIME),
    line_color_(line_color),
    point_color_(point_color),
    line_thickness_(line_thickness),
    point_radius_(point_radius)
  {
  }

  void process(const debug::ShowerContext & context, std::vector<debug::DisplayItem> & items) override
  {
    for (auto & item : items) {
      if (item.kind == debug::DisplayKind::Image && !item.image.empty()) {
        const auto point_stamp = latestPointTimestampAround(context, item.timestamp);
        if (point_stamp.has_value()) {
          drawPointsOnImage(context, item, *point_stamp);
        }
      }
    }

    if (context.message.topic_type != point_type_) {
      return;
    }

    auto image_items = imageItemsAroundPoint(context);
    for (auto & item : image_items) {
      drawPointsOnImage(context, item, pointTimestamp(context.message));
      if (!item.image.empty()) {
        items.push_back(std::move(item));
      }
    }
  }

private:
  struct OrderedPoint
  {
    std::string group;
    int order = 0;
    cv::Point point;
  };

  static constexpr const char * point_type_ = "geometry_msgs/msg/PointStamped";
  static constexpr const char * image_type_ = "sensor_msgs/msg/Image";

  void drawPointsOnImage(
    const debug::ShowerContext & context,
    debug::DisplayItem & image_item,
    std::int64_t point_timestamp_ns) const
  {
    const auto points = pointsAroundTimestamp(context, image_item.timestamp, point_timestamp_ns);
    if (points.empty()) {
      return;
    }

    auto grouped_points = groupPoints(points);
    for (const auto & [_, group] : grouped_points) {
      drawPointGroup(image_item.image, group);
    }
  }

  std::vector<debug::DisplayItem> imageItemsAroundPoint(const debug::ShowerContext & context) const
  {
    std::vector<debug::DisplayItem> image_items;
    const auto images = context.topic_manager.messagesAroundTimestamp(
      context.message.timestamp, tolerance_, image_type_);

    for (const auto & image_message : images) {
      auto typed_image = deserialize<sensor_msgs::msg::Image>(image_message);
      if (!typed_image.has_value()) {
        continue;
      }

      try {
        const auto converted = cv_bridge::toCvCopy(*typed_image, displayEncoding(*typed_image));

        debug::DisplayItem item;
        item.kind = debug::DisplayKind::Image;
        item.topic_name = image_message.topic_name;
        item.topic_type = image_message.topic_type;
        item.label = "image_with_points";
        item.timestamp = image_message.timestamp;
        item.image = converted->image.clone();
        item.metadata["encoding"] = typed_image->encoding;
        item.metadata["frame_id"] = typed_image->header.frame_id;
        image_items.push_back(std::move(item));
      } catch (const std::exception &) {
      }
    }

    return image_items;
  }

  std::vector<OrderedPoint> pointsAroundTimestamp(
    const debug::ShowerContext & context,
    const rclcpp::Time & timestamp,
    std::optional<std::int64_t> point_timestamp_ns = std::nullopt) const
  {
    std::vector<OrderedPoint> points;
    const auto messages = context.topic_manager.messagesAroundTimestamp(timestamp, tolerance_, point_type_);

    for (const auto & message : messages) {
      auto typed_point = deserialize<geometry_msgs::msg::PointStamped>(message);
      if (!typed_point.has_value()) {
        continue;
      }

      const auto message_timestamp_ns = pointTimestamp(*typed_point);
      if (point_timestamp_ns.has_value() && message_timestamp_ns != *point_timestamp_ns) {
        continue;
      }

      auto order = pointOrder(typed_point->header.frame_id);
      if (!order.has_value()) {
        continue;
      }

      points.push_back(OrderedPoint{
        pointGroup(typed_point->header.frame_id),
        *order,
        cv::Point{
          static_cast<int>(std::lround(typed_point->point.x)),
          static_cast<int>(std::lround(typed_point->point.y))}});
    }

    return points;
  }

  static std::int64_t pointTimestamp(const geometry_msgs::msg::PointStamped & point)
  {
    return static_cast<std::int64_t>(point.header.stamp.sec) * 1000000000LL +
           static_cast<std::int64_t>(point.header.stamp.nanosec);
  }

  static std::int64_t pointTimestamp(const debug::TopicMessage & message)
  {
    const auto point = deserialize<geometry_msgs::msg::PointStamped>(message);
    return point.has_value() ? pointTimestamp(*point) : 0;
  }

  std::optional<std::int64_t> latestPointTimestampAround(
    const debug::ShowerContext & context,
    const rclcpp::Time & timestamp) const
  {
    const auto messages = context.topic_manager.messagesAroundTimestamp(timestamp, tolerance_, point_type_);
    std::optional<std::int64_t> latest_timestamp;
    for (const auto & message : messages) {
      const auto point = deserialize<geometry_msgs::msg::PointStamped>(message);
      if (point.has_value()) {
        latest_timestamp = pointTimestamp(*point);
      }
    }

    return latest_timestamp;
  }

  static std::map<std::string, std::vector<OrderedPoint>> groupPoints(
    const std::vector<OrderedPoint> & points)
  {
    std::map<std::string, std::vector<OrderedPoint>> groups;
    for (const auto & point : points) {
      groups[point.group].push_back(point);
    }

    for (auto & [_, group] : groups) {
      std::stable_sort(group.begin(), group.end(), [](const OrderedPoint & lhs, const OrderedPoint & rhs) {
        return lhs.order < rhs.order;
      });
    }

    return groups;
  }

  void drawPointGroup(cv::Mat & image, const std::vector<OrderedPoint> & points) const
  {
    if (points.empty()) {
      return;
    }

    for (std::size_t index = 1; index < points.size(); ++index) {
      cv::line(
        image,
        points[index - 1].point,
        points[index].point,
        line_color_,
        line_thickness_,
        cv::LINE_AA);
    }

    if (points.size() > 2) {
      cv::line(
        image,
        points.back().point,
        points.front().point,
        line_color_,
        line_thickness_,
        cv::LINE_AA);
    }

    for (const auto & point : points) {
      cv::circle(image, point.point, point_radius_, point_color_, -1, cv::LINE_AA);
    }
  }

  template<typename MessageT>
  static std::optional<MessageT> deserialize(const debug::TopicMessage & message)
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

  static std::optional<int> pointOrder(const std::string & frame_id)
  {
    const auto separator = frame_id.find_last_of('_');
    if (separator == std::string::npos || separator + 1 >= frame_id.size()) {
      return std::nullopt;
    }

    const auto suffix = frame_id.substr(separator + 1);
    if (!std::all_of(suffix.begin(), suffix.end(), [](unsigned char ch) {
        return std::isdigit(ch) != 0;
      })) {
      return std::nullopt;
    }

    return std::stoi(suffix);
  }

  static std::string pointGroup(const std::string & frame_id)
  {
    const auto separator = frame_id.find_last_of('_');
    if (separator == std::string::npos) {
      return frame_id;
    }

    return frame_id.substr(0, separator);
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

  rclcpp::Duration tolerance_;
  cv::Scalar line_color_;
  cv::Scalar point_color_;
  int line_thickness_;
  int point_radius_;
};

}  // namespace plugins
