# Debug 插件接口说明

`debug` 包当前的插件机制是源码级/编译期注册机制，不是 ROS 2 `pluginlib` 动态插件。插件通过继承
`debug::ShowerPlugin`，或通过 `debug::Shower::addPluginCallback()` 注册回调，接入 `Shower` 的显示流水线。

现有示例插件见 `plugins/drawpoints.hpp`：它会把时间戳附近的 `geometry_msgs/msg/PointStamped` 按
`frame_id` 分组并绘制到 `sensor_msgs/msg/Image` 上。

## 公开插件接口

插件作者主要依赖 `debug/shower.hpp` 和 `debug/topic_manager.hpp` 中的类型。

### `debug::DisplayKind`

`DisplayKind` 描述一个显示项的类型：

- `Image`：图像项，`Shower` 会用 OpenCV 窗口显示。
- `Text`：文本项，CLI 会显示 `text`。
- `Number`：数值项，CLI 会显示 `number`。
- `Boolean`：布尔项，CLI 会显示 `boolean`。
- `Extra`：扩展项，CLI 默认显示 `text`，为空时显示 `label`。

### `debug::DisplayItem`

`DisplayItem` 是插件输入和输出的统一载体。

```cpp
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
```

字段约定：

- `topic_name` / `topic_type`：该显示项来源 topic 的名称和类型。
- `label`：显示项语义标签，例如 `image`、`string`、`image_with_points`。
- `timestamp`：显示项对应的接收时间戳。
- `image`：仅 `DisplayKind::Image` 使用，非空才会被 `cv::imshow()` 显示。
- `text`：`Text` 和 `Extra` 常用。
- `number`：`Number` 使用。
- `boolean`：`Boolean` 使用。
- `extra`：给自定义插件传递复杂对象预留，当前默认显示链路不会解析它。
- `metadata`：字符串元数据，例如图片解码时会写入 `encoding`、`frame_id`。

### `debug::ShowerContext`

`ShowerContext` 是每次插件处理时的上下文。

```cpp
struct ShowerContext
{
  const TopicMessage & message;
  const TopicManager & topic_manager;
};
```

- `message`：触发本轮显示处理的原始序列化消息。
- `topic_manager`：消息缓存管理器，可查询最新消息、时间戳附近消息和跨 topic 匹配帧。

### `debug::ShowerPlugin`

类插件需要继承 `ShowerPlugin` 并实现 `process()`。

```cpp
class ShowerPlugin
{
public:
  virtual ~ShowerPlugin() = default;
  virtual void process(const ShowerContext & context, std::vector<DisplayItem> & items) = 0;
};
```

`process()` 的参数含义：

- `context` 是只读上下文，包含触发消息和全局 topic 缓存。
- `items` 是本轮消息被 `Shower` 内置解码后的显示项列表。
- 插件可以原地修改 `items` 中已有项，例如在图像上画标注。
- 插件可以向 `items` 追加新项，例如额外文本、数值、处理后的图像。
- 插件可以删除或清空项，但这会影响后续插件和最终显示，建议只在明确需要拦截默认显示时使用。

### `debug::Shower::PluginCallback`

如果逻辑很短，也可以不定义类，直接注册函数对象：

```cpp
using PluginCallback =
  std::function<void(const ShowerContext &, std::vector<DisplayItem> &)>;
```

它和 `ShowerPlugin::process()` 处在同一处理阶段。执行顺序是先运行通过 `addPlugin()` 注册的类插件，再运行
`addPluginCallback()` 注册的回调。

### `debug::TopicMessage`

`TopicMessage` 保存一条已接收的 ROS 消息。

```cpp
struct TopicMessage
{
  std::string topic_name;
  std::string topic_type;
  std::shared_ptr<rclcpp::SerializedMessage> message;
  rclcpp::Time timestamp;
  std::size_t receive_count = 0;
};
```

注意：`message` 是序列化消息。插件如果要读取具体字段，需要用 `rclcpp::Serialization<MessageT>` 自行反序列化。

### `debug::TopicFrame`

`TopicFrame` 表示同一个目标时间戳附近的一组 topic 消息。

```cpp
struct TopicFrame
{
  rclcpp::Time timestamp;
  std::unordered_map<std::string, TopicMessage> messages;

  bool hasTopic(const std::string & topic_name) const;
  std::optional<TopicMessage> message(const std::string & topic_name) const;
};
```

### `debug::TopicManager`

插件常用的只读查询接口：

```cpp
std::optional<TopicMessage> latestMessage(const std::string & topic_name) const;

std::size_t receiveCount(const std::string & topic_name) const;

std::optional<TopicFrame> matchByTimestamp(
  const rclcpp::Time & timestamp,
  const rclcpp::Duration & tolerance) const;

std::optional<TopicFrame> latestMatchedFrame(
  const std::vector<std::string> & required_topics,
  const rclcpp::Duration & tolerance) const;

std::vector<TopicMessage> messagesAroundTimestamp(
  const rclcpp::Time & timestamp,
  const rclcpp::Duration & tolerance,
  const std::string & topic_type = {}) const;
```

使用建议：

- 查询指定 topic 最新值：用 `latestMessage(topic_name)`。
- 统计某 topic 已收到多少条：用 `receiveCount(topic_name)`。
- 查找某个时间戳附近的一帧缓存：用 `matchByTimestamp(timestamp, tolerance)`。
- 查找包含一组指定 topic 的最近同步帧：用 `latestMatchedFrame(required_topics, tolerance)`。
- 查找时间窗口内所有消息，或按消息类型过滤：用 `messagesAroundTimestamp(timestamp, tolerance, topic_type)`。

## 内置解码和插件输入

`Shower::onTopicMessage()` 收到 `TopicMessage` 后，先做内置解码。如果当前消息类型不能被内置解码，
`items` 会为空，并且本轮插件不会运行。

当前内置支持：

- `sensor_msgs/msg/Image`：生成 `DisplayKind::Image`，`label = "image"`，并填充 `metadata["encoding"]`
  和 `metadata["frame_id"]`。
- `std_msgs/msg/String`：生成 `DisplayKind::Text`。
- `std_msgs/msg/Bool`：生成 `DisplayKind::Boolean`。
- `std_msgs/msg/Byte`、`Char`、`Int8`、`UInt8`、`Int16`、`UInt16`、`Int32`、`UInt32`、`Int64`、
  `UInt64`、`Float32`、`Float64`：生成 `DisplayKind::Number`。

这意味着：如果插件只依赖 `geometry_msgs/msg/PointStamped` 这类当前不会被内置解码的消息，它不能只等待
PointStamped 触发执行。常见做法是让图片或基础消息触发插件，然后通过 `context.topic_manager` 查询缓存里的
PointStamped。`DrawPoints` 代码中虽然保留了 PointStamped 触发时补画图像的分支，但以当前 `Shower`
实现来看，PointStamped 本身不会被内置解码成非空 `items`，因此不会单独触发插件执行。

## 数据流

整体流入流出如下：

1. `TopicRec` 通过 generic subscription 接收被监控 topic 的序列化 ROS 消息。
2. `debug_node` 为消息打接收时间戳，并调用 `TopicManager::onTopicMessage()` 写入最新消息和时间戳缓存。
3. `debug_node` 取出该 topic 的 `latestMessage()`，交给 `Shower::onTopicMessage()`。
4. `Shower` 根据 `topic_type` 把序列化消息解码成一个或多个 `DisplayItem`。
5. 如果 `DisplayItem` 列表非空，`Shower` 构造 `ShowerContext` 并依次执行插件。
6. 插件读取 `context.message`、查询 `context.topic_manager`，并修改或追加 `items`。
7. `Shower` 显示最终 `items`：图片项进入 OpenCV 窗口，非图片项进入 CLI 显示缓冲。
8. CLI 通过 `setDisplaySink()` 接收非图片显示项并刷新终端界面。

## 编写类插件

推荐在 `src/debug/include/plugins/` 下新增一个头文件，例如 `my_plugin.hpp`。

最小插件示例：

```cpp
#pragma once

#include <utility>
#include <vector>

#include "debug/shower.hpp"

namespace plugins
{

class MyPlugin : public debug::ShowerPlugin
{
public:
  void process(
    const debug::ShowerContext & context,
    std::vector<debug::DisplayItem> & items) override
  {
    if (context.message.topic_type != "std_msgs/msg/String") {
      return;
    }

    debug::DisplayItem item;
    item.kind = debug::DisplayKind::Extra;
    item.topic_name = context.message.topic_name;
    item.topic_type = context.message.topic_type;
    item.label = "my_plugin";
    item.timestamp = context.message.timestamp;
    item.text = "plugin processed message";
    items.push_back(std::move(item));
  }
};

}  // namespace plugins
```

注册插件：

```cpp
#include "plugins/my_plugin.hpp"

auto shower = std::make_shared<debug::Shower>(*topic_manager);
shower->addPlugin(std::make_shared<plugins::MyPlugin>());
```

当前 `debug_node.cpp` 中已有：

```cpp
shower->addPlugin(std::make_shared<plugins::DrawPoints>());
```

新增插件时，需要在 `debug_node.cpp` include 新插件头文件并调用 `addPlugin()`。如果新增插件依赖新的 ROS 消息包，
还需要在 `CMakeLists.txt` 和 `package.xml` 添加对应依赖。

## 编写回调插件

简单逻辑可以直接注册回调：

```cpp
shower->addPluginCallback(
  [](const debug::ShowerContext & context, std::vector<debug::DisplayItem> & items) {
    debug::DisplayItem item;
    item.kind = debug::DisplayKind::Extra;
    item.topic_name = context.message.topic_name;
    item.topic_type = context.message.topic_type;
    item.label = "callback";
    item.timestamp = context.message.timestamp;
    item.text = "callback processed message";
    items.push_back(std::move(item));
  });
```

回调适合少量无状态逻辑；需要配置、缓存或复杂算法时，优先写 `ShowerPlugin` 子类。

## 反序列化消息

插件收到的是 `rclcpp::SerializedMessage`。如果需要读取具体 ROS 消息字段，可以在插件内定义工具函数：

```cpp
template<typename MessageT>
std::optional<MessageT> deserialize(const debug::TopicMessage & message)
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
```

使用时要确保 `message.topic_type` 和 `MessageT` 匹配，例如：

```cpp
if (message.topic_type == "sensor_msgs/msg/Image") {
  auto image = deserialize<sensor_msgs::msg::Image>(message);
}
```

## 跨 topic 数据匹配

插件可以通过 `TopicManager` 读取缓存，实现图像、点、状态量等不同 topic 的时间对齐。

按时间窗口查找某类消息：

```cpp
const rclcpp::Duration tolerance(5000000, RCL_SYSTEM_TIME);
const auto points = context.topic_manager.messagesAroundTimestamp(
  image_item.timestamp,
  tolerance,
  "geometry_msgs/msg/PointStamped");
```

查找一组指定 topic 的最近同步帧：

```cpp
const auto frame = context.topic_manager.latestMatchedFrame(
  {"/camera/image_raw", "/target/point"},
  rclcpp::Duration(5000000, RCL_SYSTEM_TIME));

if (frame.has_value() && frame->hasTopic("/camera/image_raw")) {
  auto image_message = frame->message("/camera/image_raw");
}
```

`TopicManager` 的时间戳是接收时间戳，不是消息 header 中的原始采集时间。若插件需要使用 header 时间戳，需要先反序列化消息并自行读取。

## `DrawPoints` 示例数据流

`plugins::DrawPoints` 展示了一个典型的跨 topic 图像增强插件：

1. 输入图像消息先被 `Shower` 解码成 `DisplayKind::Image`。
2. 插件遍历 `items`，找到图像项。
3. 插件用 `messagesAroundTimestamp()` 查找图像接收时间戳附近的 `geometry_msgs/msg/PointStamped`。
4. 插件反序列化点消息，读取 `point.x`、`point.y` 作为像素坐标。
5. 插件从 `header.frame_id` 解析点组和顺序：最后一个下划线后的数字作为顺序，前缀作为分组。
6. 同组点按顺序连线，点本身画圆。
7. 修改后的 `DisplayItem::image` 继续流向 `Shower`，最终由 OpenCV 窗口显示。

`DrawPoints` 默认参数：

- 时间匹配容差：`5000000ns`。
- 连线颜色：绿色，`cv::Scalar(0, 255, 0)`。
- 点颜色：红色，`cv::Scalar(0, 0, 255)`。
- 线宽：`2`。
- 点半径：`4`。

PointStamped 的 `frame_id` 约定示例：

- `armor_0`
- `armor_1`
- `armor_2`
- `armor_3`

这些点会被分到 `armor` 组，并按 `0, 1, 2, 3` 连线；超过两个点时会闭合首尾。

## 编写注意事项

- 插件运行在显示处理链路中，避免长时间阻塞。
- `items` 会继续传给后续插件，修改已有项时要考虑插件顺序。
- `cv::Mat` 如果来自别的消息或临时对象，追加到 `DisplayItem` 前建议 `clone()`。
- 新增非图片输出时，优先设置 `kind`、`topic_name`、`topic_type`、`label`、`timestamp` 和对应值字段。
- 新增图片输出时，确保 `kind = DisplayKind::Image` 且 `image` 非空。
- 当前 `Shower` 内置反序列化工具是私有函数，插件需要自己实现反序列化辅助函数，或后续把公共工具抽到共享头文件。
- 当前没有运行时插件发现机制；新增插件后必须重新编译 `debug_node`。
