#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/time.hpp"

namespace debug
{

struct TopicMessage
{
  std::string topic_name;
  std::string topic_type;
  std::shared_ptr<rclcpp::SerializedMessage> message;
  rclcpp::Time timestamp;
  std::size_t receive_count = 0;
};

struct TopicFrame
{
  rclcpp::Time timestamp;
  std::unordered_map<std::string, TopicMessage> messages;

  bool hasTopic(const std::string & topic_name) const
  {
    return messages.find(topic_name) != messages.end();
  }

  std::optional<TopicMessage> message(const std::string & topic_name) const
  {
    const auto iter = messages.find(topic_name);
    if (iter == messages.end()) {
      return std::nullopt;
    }

    return iter->second;
  }
};

class TopicManager
{
public:
  explicit TopicManager(std::size_t max_cache_frames = 120)
  : max_cache_frames_(std::max<std::size_t>(1, max_cache_frames))
  {
  }

  void onTopicMessage(
    const std::string & topic_name,
    const std::string & topic_type,
    std::shared_ptr<rclcpp::SerializedMessage> message)
  {
    onTopicMessage(topic_name, topic_type, rclcpp::Clock(RCL_SYSTEM_TIME).now(), std::move(message));
  }

  void onTopicMessage(
    const std::string & topic_name,
    const std::string & topic_type,
    const rclcpp::Time & timestamp,
    std::shared_ptr<rclcpp::SerializedMessage> message)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    auto & receive_count = receive_counts_[topic_name];
    ++receive_count;

    TopicMessage topic_message{
      topic_name,
      topic_type,
      std::move(message),
      timestamp,
      receive_count};

    latest_messages_[topic_name] = topic_message;

    const auto timestamp_ns = timestamp.nanoseconds();
    auto & frame = cache_pool_[timestamp_ns];
    frame.timestamp = timestamp;
    frame.messages[topic_name] = std::move(topic_message);

    pruneCache();
  }

  std::optional<TopicMessage> latestMessage(const std::string & topic_name) const
  {
    std::lock_guard<std::mutex> lock(mutex_);

    const auto iter = latest_messages_.find(topic_name);
    if (iter == latest_messages_.end()) {
      return std::nullopt;
    }

    return iter->second;
  }

  std::size_t receiveCount(const std::string & topic_name) const
  {
    std::lock_guard<std::mutex> lock(mutex_);

    const auto iter = receive_counts_.find(topic_name);
    if (iter == receive_counts_.end()) {
      return 0;
    }

    return iter->second;
  }

  std::optional<TopicFrame> matchByTimestamp(
    const rclcpp::Time & timestamp,
    const rclcpp::Duration & tolerance) const
  {
    return matchByTimestamp(timestamp.nanoseconds(), tolerance.nanoseconds());
  }

  std::optional<TopicFrame> matchByTimestamp(
    std::int64_t timestamp_ns,
    std::int64_t tolerance_ns) const
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (cache_pool_.empty()) {
      return std::nullopt;
    }

    tolerance_ns = std::max<std::int64_t>(0, tolerance_ns);

    auto best_iter = cache_pool_.end();
    auto best_delta = std::numeric_limits<std::int64_t>::max();

    const auto lower = cache_pool_.lower_bound(timestamp_ns);
    considerMatchCandidate(timestamp_ns, lower, best_iter, best_delta);

    if (lower != cache_pool_.begin()) {
      considerMatchCandidate(timestamp_ns, std::prev(lower), best_iter, best_delta);
    }

    if (best_iter == cache_pool_.end() || best_delta > tolerance_ns) {
      return std::nullopt;
    }

    return best_iter->second;
  }

  std::optional<TopicFrame> latestMatchedFrame(
    const std::vector<std::string> & required_topics,
    const rclcpp::Duration & tolerance) const
  {
    return latestMatchedFrame(required_topics, tolerance.nanoseconds());
  }

  std::optional<TopicFrame> latestMatchedFrame(
    const std::vector<std::string> & required_topics,
    std::int64_t tolerance_ns) const
  {
    std::lock_guard<std::mutex> lock(mutex_);

    tolerance_ns = std::max<std::int64_t>(0, tolerance_ns);

    for (auto frame_iter = cache_pool_.rbegin(); frame_iter != cache_pool_.rend(); ++frame_iter) {
      const auto matched_frame = collectFrameAroundTimestamp(
        frame_iter->first, required_topics, tolerance_ns);
      if (matched_frame.has_value()) {
        return matched_frame;
      }
    }

    return std::nullopt;
  }

  std::vector<TopicMessage> messagesAroundTimestamp(
    const rclcpp::Time & timestamp,
    const rclcpp::Duration & tolerance,
    const std::string & topic_type = {}) const
  {
    return messagesAroundTimestamp(timestamp.nanoseconds(), tolerance.nanoseconds(), topic_type);
  }

  std::vector<TopicMessage> messagesAroundTimestamp(
    std::int64_t timestamp_ns,
    std::int64_t tolerance_ns,
    const std::string & topic_type = {}) const
  {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<TopicMessage> messages;
    tolerance_ns = std::max<std::int64_t>(0, tolerance_ns);

    for (const auto & [frame_timestamp, frame] : cache_pool_) {
      if (absDelta(frame_timestamp, timestamp_ns) > tolerance_ns) {
        continue;
      }

      for (const auto & [_, message] : frame.messages) {
        if (topic_type.empty() || message.topic_type == topic_type) {
          messages.push_back(message);
        }
      }
    }

    std::stable_sort(
      messages.begin(),
      messages.end(),
      [](const TopicMessage & lhs, const TopicMessage & rhs) {
        return lhs.timestamp.nanoseconds() < rhs.timestamp.nanoseconds();
      });

    return messages;
  }

  std::size_t cacheFrameCount() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return cache_pool_.size();
  }

  void setMaxCacheFrames(std::size_t max_cache_frames)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    max_cache_frames_ = std::max<std::size_t>(1, max_cache_frames);
    pruneCache();
  }

private:
  using CachePool = std::map<std::int64_t, TopicFrame>;
  using CacheIterator = CachePool::const_iterator;

  static std::int64_t absDelta(std::int64_t lhs, std::int64_t rhs)
  {
    return lhs >= rhs ? lhs - rhs : rhs - lhs;
  }

  void considerMatchCandidate(
    std::int64_t target_timestamp_ns,
    CacheIterator candidate,
    CacheIterator & best_iter,
    std::int64_t & best_delta) const
  {
    if (candidate == cache_pool_.end()) {
      return;
    }

    const auto delta = absDelta(candidate->first, target_timestamp_ns);
    if (delta < best_delta) {
      best_iter = candidate;
      best_delta = delta;
    }
  }

  std::optional<TopicMessage> findNearestTopicMessage(
    const std::string & topic_name,
    std::int64_t target_timestamp_ns,
    std::int64_t tolerance_ns) const
  {
    auto best_message = std::optional<TopicMessage>{};
    auto best_delta = std::numeric_limits<std::int64_t>::max();

    const auto lower = cache_pool_.lower_bound(target_timestamp_ns);
    findNearestTopicMessageFrom(topic_name, target_timestamp_ns, lower, best_message, best_delta);

    auto reverse_iter = std::make_reverse_iterator(lower);
    while (reverse_iter != cache_pool_.rend()) {
      const auto delta = absDelta(reverse_iter->first, target_timestamp_ns);
      if (delta > tolerance_ns && delta > best_delta) {
        break;
      }

      findNearestTopicMessageFrom(
        topic_name,
        target_timestamp_ns,
        std::prev(reverse_iter.base()),
        best_message,
        best_delta);
      ++reverse_iter;
    }

    auto forward_iter = lower;
    while (forward_iter != cache_pool_.end()) {
      const auto delta = absDelta(forward_iter->first, target_timestamp_ns);
      if (delta > tolerance_ns && delta > best_delta) {
        break;
      }

      findNearestTopicMessageFrom(
        topic_name,
        target_timestamp_ns,
        forward_iter,
        best_message,
        best_delta);
      ++forward_iter;
    }

    if (!best_message.has_value() || best_delta > tolerance_ns) {
      return std::nullopt;
    }

    return best_message;
  }

  void findNearestTopicMessageFrom(
    const std::string & topic_name,
    std::int64_t target_timestamp_ns,
    CacheIterator frame_iter,
    std::optional<TopicMessage> & best_message,
    std::int64_t & best_delta) const
  {
    if (frame_iter == cache_pool_.end()) {
      return;
    }

    const auto message_iter = frame_iter->second.messages.find(topic_name);
    if (message_iter == frame_iter->second.messages.end()) {
      return;
    }

    const auto delta = absDelta(message_iter->second.timestamp.nanoseconds(), target_timestamp_ns);
    if (delta < best_delta) {
      best_message = message_iter->second;
      best_delta = delta;
    }
  }

  std::optional<TopicFrame> collectFrameAroundTimestamp(
    std::int64_t target_timestamp_ns,
    const std::vector<std::string> & required_topics,
    std::int64_t tolerance_ns) const
  {
    TopicFrame matched_frame;
    matched_frame.timestamp = rclcpp::Time(target_timestamp_ns, RCL_SYSTEM_TIME);

    for (const auto & topic_name : required_topics) {
      auto message = findNearestTopicMessage(topic_name, target_timestamp_ns, tolerance_ns);
      if (!message.has_value()) {
        return std::nullopt;
      }

      matched_frame.messages[topic_name] = std::move(*message);
    }

    if (!required_topics.empty() && !isFrameWithinTolerance(matched_frame, tolerance_ns)) {
      return std::nullopt;
    }

    return matched_frame;
  }

  bool isFrameWithinTolerance(const TopicFrame & frame, std::int64_t tolerance_ns) const
  {
    auto min_timestamp = std::numeric_limits<std::int64_t>::max();
    auto max_timestamp = std::numeric_limits<std::int64_t>::min();

    for (const auto & [_, message] : frame.messages) {
      const auto timestamp_ns = message.timestamp.nanoseconds();
      min_timestamp = std::min(min_timestamp, timestamp_ns);
      max_timestamp = std::max(max_timestamp, timestamp_ns);
    }

    return frame.messages.empty() || max_timestamp - min_timestamp <= tolerance_ns;
  }

  void pruneCache()
  {
    while (cache_pool_.size() > max_cache_frames_) {
      cache_pool_.erase(cache_pool_.begin());
    }
  }

  mutable std::mutex mutex_;
  std::size_t max_cache_frames_;
  std::unordered_map<std::string, TopicMessage> latest_messages_;
  std::unordered_map<std::string, std::size_t> receive_counts_;
  CachePool cache_pool_;
};

}  // namespace debug
