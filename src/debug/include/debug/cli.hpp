#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <set>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include "debug/shower.hpp"
#include "debug/topic_rec.hpp"

namespace debug
{

class Cli
{
public:
  using TopicListProvider = std::function<std::vector<TopicSpec>()>;
  using AddMonitorCallback = std::function<std::optional<std::string>(const TopicSpec &)>;
  using RemoveMonitorCallback = std::function<std::optional<std::string>(const std::string &)>;

  Cli(
    TopicListProvider topic_list_provider,
    AddMonitorCallback add_monitor_callback,
    RemoveMonitorCallback remove_monitor_callback)
  : topic_list_provider_(std::move(topic_list_provider)),
    add_monitor_callback_(std::move(add_monitor_callback)),
    remove_monitor_callback_(std::move(remove_monitor_callback))
  {
  }

  void run()
  {
    TerminalMode terminal_mode;
    hideCursor();
    clearScreen();

    stop_requested_.store(false);
    bool running = true;
    auto last_refresh = Clock::now() - std::chrono::seconds(1);

    while (running && !stop_requested_.load()) {
      const auto now = Clock::now();
      if (now - last_refresh >= std::chrono::seconds(1) || redraw_requested_.exchange(false)) {
        refreshTopics();
        draw();
        last_refresh = now;
      }

      const auto key = readKey();
      if (key.has_value()) {
        running = handleKey(*key);
        draw();
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    resetStyle();
    showCursor();
    clearScreen();
  }

  void requestStop()
  {
    stop_requested_.store(true);
  }

  void setDisplayItems(std::vector<DisplayItem> items)
  {
    {
      std::lock_guard<std::mutex> lock(display_mutex_);
      display_items_ = std::move(items);
    }
    redraw_requested_.store(true);
  }

private:
  enum class Key
  {
    Add,
    Remove,
    Quit,
    Up,
    Down,
    MessageUp,
    MessageDown,
  };

  using Clock = std::chrono::steady_clock;

  class TerminalMode
  {
  public:
    TerminalMode()
    {
      if (!isatty(STDIN_FILENO)) {
        return;
      }

      if (tcgetattr(STDIN_FILENO, &original_) != 0) {
        return;
      }

      auto raw = original_;
      raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
      raw.c_cc[VMIN] = 0;
      raw.c_cc[VTIME] = 0;

      if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
        enabled_ = true;
      }
    }

    ~TerminalMode()
    {
      if (enabled_) {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_);
      }
    }

    TerminalMode(const TerminalMode &) = delete;
    TerminalMode & operator=(const TerminalMode &) = delete;

  private:
    bool enabled_ = false;
    termios original_{};
  };

  static void clearScreen()
  {
    std::printf("\033[2J\033[H");
    std::fflush(stdout);
  }

  static void hideCursor()
  {
    std::printf("\033[?25l");
    std::fflush(stdout);
  }

  static void showCursor()
  {
    std::printf("\033[?25h");
    std::fflush(stdout);
  }

  static void resetStyle()
  {
    std::printf("\033[0m");
    std::fflush(stdout);
  }

  static std::size_t terminalWidth()
  {
    winsize size{};
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &size) == 0 && size.ws_col > 0) {
      return size.ws_col;
    }

    return 100;
  }

  static std::size_t terminalHeight()
  {
    winsize size{};
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &size) == 0 && size.ws_row > 0) {
      return size.ws_row;
    }

    return 30;
  }

  static std::string fitText(const std::string & text, std::size_t width)
  {
    if (width == 0) {
      return {};
    }

    if (text.size() > width) {
      if (width <= 3) {
        return text.substr(0, width);
      }

      return text.substr(0, width - 3) + "...";
    }

    return text + std::string(width - text.size(), ' ');
  }

  static std::string topicKey(const TopicSpec & topic)
  {
    return topic.name + "|" + topic.type;
  }

  static std::optional<Key> readKey()
  {
    char input = 0;
    const auto read_count = ::read(STDIN_FILENO, &input, 1);
    if (read_count <= 0) {
      return std::nullopt;
    }

    if (input == 'a' || input == 'A') {
      return Key::Add;
    }

    if (input == 'r' || input == 'R') {
      return Key::Remove;
    }

    if (input == 'q' || input == 'Q') {
      return Key::Quit;
    }

    if (input == 'j' || input == 'J') {
      return Key::Down;
    }

    if (input == 'k' || input == 'K') {
      return Key::Up;
    }

    if (input == 'u' || input == 'U') {
      return Key::MessageUp;
    }

    if (input == 'd' || input == 'D') {
      return Key::MessageDown;
    }

    if (input == '\033') {
      char sequence[2] = {};
      if (::read(STDIN_FILENO, &sequence[0], 1) <= 0 ||
          ::read(STDIN_FILENO, &sequence[1], 1) <= 0) {
        return std::nullopt;
      }

      if (sequence[0] == '[' && sequence[1] == 'A') {
        return Key::Up;
      }

      if (sequence[0] == '[' && sequence[1] == 'B') {
        return Key::Down;
      }
    }

    return std::nullopt;
  }

  void refreshTopics()
  {
    if (!topic_list_provider_) {
      topics_.clear();
      return;
    }

    topics_ = topic_list_provider_();
    std::sort(topics_.begin(), topics_.end(), [](const TopicSpec & lhs, const TopicSpec & rhs) {
      if (lhs.name == rhs.name) {
        return lhs.type < rhs.type;
      }

      return lhs.name < rhs.name;
    });

    topics_.erase(
      std::unique(topics_.begin(), topics_.end(), [](const TopicSpec & lhs, const TopicSpec & rhs) {
        return lhs.name == rhs.name && lhs.type == rhs.type;
      }),
      topics_.end());

    if (topics_.empty()) {
      selected_index_ = 0;
      scroll_offset_ = 0;
      return;
    }

    if (selected_index_ >= topics_.size()) {
      selected_index_ = topics_.size() - 1;
    }
  }

  bool handleKey(Key key)
  {
    switch (key) {
      case Key::Add:
        addSelectedTopic();
        return true;
      case Key::Remove:
        removeSelectedTopic();
        return true;
      case Key::Quit:
        return false;
      case Key::Up:
        moveSelectionUp();
        return true;
      case Key::Down:
        moveSelectionDown();
        return true;
      case Key::MessageUp:
        moveMessageUp();
        return true;
      case Key::MessageDown:
        moveMessageDown();
        return true;
    }

    return true;
  }

  void moveSelectionUp()
  {
    if (topics_.empty() || selected_index_ == 0) {
      return;
    }

    --selected_index_;
    if (selected_index_ < scroll_offset_) {
      scroll_offset_ = selected_index_;
    }
  }

  void moveSelectionDown()
  {
    if (topics_.empty() || selected_index_ + 1 >= topics_.size()) {
      return;
    }

    ++selected_index_;
  }

  void moveMessageUp()
  {
    message_follow_latest_ = false;
    if (message_scroll_offset_ > 0) {
      --message_scroll_offset_;
    }
  }

  void moveMessageDown()
  {
    message_follow_latest_ = false;
    ++message_scroll_offset_;
  }

  void addSelectedTopic()
  {
    const auto topic = selectedTopic();
    if (!topic.has_value()) {
      status_message_ = "no topic selected";
      return;
    }

    if (add_monitor_callback_) {
      const auto error = add_monitor_callback_(*topic);
      if (error.has_value()) {
        status_message_ = *error;
        return;
      }
    }

    monitored_topics_.insert(topicKey(*topic));
    status_message_ = "monitoring " + topic->name;
  }

  void removeSelectedTopic()
  {
    const auto topic = selectedTopic();
    if (!topic.has_value()) {
      status_message_ = "no topic selected";
      return;
    }

    if (remove_monitor_callback_) {
      const auto error = remove_monitor_callback_(topic->name);
      if (error.has_value()) {
        status_message_ = *error;
        return;
      }
    }

    monitored_topics_.erase(topicKey(*topic));
    status_message_ = "stopped monitoring " + topic->name;
  }

  std::optional<TopicSpec> selectedTopic() const
  {
    if (topics_.empty() || selected_index_ >= topics_.size()) {
      return std::nullopt;
    }

    return topics_[selected_index_];
  }

  bool isMonitored(const TopicSpec & topic) const
  {
    return monitored_topics_.find(topicKey(topic)) != monitored_topics_.end();
  }

  void draw() const
  {
    const auto width = terminalWidth();
    const auto height = terminalHeight();
    const auto left_width = std::max<std::size_t>(28, width / 3);
    const auto right_width = width > left_width + 3 ? width - left_width - 3 : 40;
    const auto list_height = height > 16 ? std::max<std::size_t>(1, height / 2 - 4) : 1;
    const auto message_height = height > list_height + 5 ? height - list_height - 5 : 0;
    updateScrollOffset(list_height);

    std::vector<DisplayItem> display_items;
    {
      std::lock_guard<std::mutex> lock(display_mutex_);
      display_items = display_items_;
    }

    std::printf("\033[H");
    resetStyle();
    drawHeader(left_width, right_width);
    drawDivider(left_width, right_width);

    for (std::size_t row = 0; row < list_height; ++row) {
      drawRow(row, left_width, right_width);
    }

    drawDivider(left_width, right_width);
    drawMessages(display_items, message_height, width);
    drawDivider(left_width, right_width);
    drawStatus(left_width, right_width);
    std::fflush(stdout);
  }

  static std::string displayValue(const DisplayItem & item)
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
        return "opencv window";
    }

    return {};
  }

  static std::string displayKind(DisplayKind kind)
  {
    switch (kind) {
      case DisplayKind::Text:
        return "text";
      case DisplayKind::Number:
        return "number";
      case DisplayKind::Boolean:
        return "bool";
      case DisplayKind::Extra:
        return "extra";
      case DisplayKind::Image:
        return "image";
    }

    return "extra";
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

  void drawMessages(
    const std::vector<DisplayItem> & items,
    std::size_t message_height,
    std::size_t width) const
  {
    if (message_height == 0) {
      return;
    }

    const auto table = makeTable(items);
    const auto body_height = message_height > 2 ? message_height - 2 : 0;
    constexpr std::size_t timestamp_width = 19;
    const auto topic_width = table.rows.empty() ? std::size_t(16) : topicColumnWidth(table, width);
    const auto columns_per_page = width > topic_width + 3 ?
      std::max<std::size_t>(1, (width - topic_width - 3) / (timestamp_width + 3)) : 1;
    const auto first_column = table.timestamps.size() > columns_per_page ?
      table.timestamps.size() - columns_per_page : 0;

    std::printf("Text messages  (u/d scroll table, j/k scroll topics)\n");
    if (message_height == 1) {
      return;
    }

    std::printf("%s", fitText("topic", topic_width).c_str());
    for (std::size_t column = first_column; column < table.timestamps.size(); ++column) {
      std::printf(" | %-19s", timestampText(table.timestamps[column]).c_str());
    }
    std::printf("\n");

    if (message_height == 2) {
      return;
    }

    updateMessageScrollOffset(table.rows.size(), body_height);
    for (std::size_t row = 0; row < body_height; ++row) {
      const auto row_index = message_scroll_offset_ + row;
      if (row_index >= table.rows.size()) {
        std::printf("%s\n", std::string(width, ' ').c_str());
        continue;
      }

      std::printf("%s", fitText(table.rows[row_index], topic_width).c_str());
      const auto value_row = table.values[row_index];
      for (std::size_t column = first_column; column < table.timestamps.size(); ++column) {
        std::printf(" | %-19s", fitText(value_row[column], timestamp_width).c_str());
      }
      std::printf("\n");
    }
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
    std::set<std::string> row_names;
    std::set<std::int64_t> timestamp_values;
    for (const auto & item : items) {
      if (item.kind != DisplayKind::Image) {
        row_names.insert(item.topic_name.empty() ? displayKind(item.kind) : item.topic_name);
        timestamp_values.insert(item.timestamp.nanoseconds());
      }
    }

    table.rows.assign(row_names.begin(), row_names.end());
    for (const auto timestamp : timestamp_values) {
      table.timestamps.emplace_back(timestamp, RCL_SYSTEM_TIME);
    }

    std::map<std::string, std::size_t> row_indices;
    std::map<std::int64_t, std::size_t> timestamp_indices;
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

      const auto row_name = item.topic_name.empty() ? displayKind(item.kind) : item.topic_name;
      auto & value = table.values[row_indices.at(row_name)][timestamp_indices.at(item.timestamp.nanoseconds())];
      if (!value.empty()) {
        value += "; ";
      }
      value += displayValue(item);
    }

    return table;
  }

  static std::size_t topicColumnWidth(const DisplayTable & table, std::size_t width)
  {
    std::size_t desired_width = 16;
    for (const auto & row : table.rows) {
      desired_width = std::max(desired_width, row.size());
    }

    const auto max_width = width > 22 ? width - 22 : std::size_t(1);
    return std::min<std::size_t>(32, std::max<std::size_t>(1, std::min(desired_width, max_width)));
  }

  void updateMessageScrollOffset(
    std::size_t row_count,
    std::size_t body_height) const
  {
    const auto max_offset = row_count > body_height ? row_count - body_height : 0;
    if (message_follow_latest_) {
      message_scroll_offset_ = max_offset;
      return;
    }

    message_scroll_offset_ = std::min(message_scroll_offset_, max_offset);
    if (message_scroll_offset_ >= max_offset) {
      message_follow_latest_ = true;
    }
  }

  void drawHeader(std::size_t left_width, std::size_t right_width) const
  {
    std::printf(
      "%s | %s\n",
      fitText("Actions", left_width).c_str(),
      fitText("Broadcasting Topics", right_width).c_str());
  }

  void drawDivider(std::size_t left_width, std::size_t right_width) const
  {
    std::printf(
      "%s-+-%s\n",
      std::string(left_width, '-').c_str(),
      std::string(right_width, '-').c_str());
  }

  void drawRow(std::size_t row, std::size_t left_width, std::size_t right_width) const
  {
    const auto left = actionText(row);

    std::printf("%s | ", fitText(left, left_width).c_str());

    const auto topic_index = scroll_offset_ + row;
    if (topic_index < topics_.size()) {
      const auto & topic = topics_[topic_index];
      const auto selected = topic_index == selected_index_;
      const auto monitored = isMonitored(topic);
      const auto line = fitText(topic.name + "  " + topic.type, right_width);

      if (selected) {
        std::printf("\033[7m");
      }

      if (monitored) {
        std::printf("\033[31m");
      } else {
        std::printf("\033[37m");
      }

      std::printf("%s\033[0m", line.c_str());
    } else {
      std::printf("%s", fitText("", right_width).c_str());
    }

    std::printf("\n");
  }

  void updateScrollOffset(std::size_t list_height) const
  {
    if (topics_.empty() || list_height == 0) {
      scroll_offset_ = 0;
      return;
    }

    if (selected_index_ < scroll_offset_) {
      scroll_offset_ = selected_index_;
      return;
    }

    if (selected_index_ >= scroll_offset_ + list_height) {
      scroll_offset_ = selected_index_ - list_height + 1;
    }
  }

  std::string actionText(std::size_t row) const
  {
    switch (row) {
      case 0:
        return "a  add selected topic";
      case 1:
        return "r  remove selected topic";
      case 2:
        return "q  exit program";
      case 4:
        return "j/k or arrows move";
      case 5:
        return "u/d scroll messages";
      default:
        return "";
    }
  }

  void drawStatus(std::size_t left_width, std::size_t right_width) const
  {
    const auto monitored_count = monitored_topics_.size();
    const auto topic_count = topics_.size();
    const auto status = status_message_.empty() ? "ready" : status_message_;
    const auto left = "monitored: " + std::to_string(monitored_count);
    const auto right = "broadcasting: " + std::to_string(topic_count) + " | " + status;

    std::printf(
      "%s | %s\n",
      fitText(left, left_width).c_str(),
      fitText(right, right_width).c_str());
  }

  TopicListProvider topic_list_provider_;
  AddMonitorCallback add_monitor_callback_;
  RemoveMonitorCallback remove_monitor_callback_;
  std::vector<TopicSpec> topics_;
  std::set<std::string> monitored_topics_;
  std::string status_message_;
  std::size_t selected_index_ = 0;
  mutable std::size_t scroll_offset_ = 0;
  mutable std::size_t message_scroll_offset_ = 0;
  mutable bool message_follow_latest_ = true;
  std::atomic<bool> stop_requested_{false};
  mutable std::mutex display_mutex_;
  std::vector<DisplayItem> display_items_;
  std::atomic<bool> redraw_requested_{true};
};

}  // namespace debug
