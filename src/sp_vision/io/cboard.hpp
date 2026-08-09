#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#ifdef SP_VISION_HAS_ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#endif

#include "io/command.hpp"
#include "io/socketcan.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
enum Mode
{
  idle,
  auto_aim,
  small_buff,
  big_buff,
  outpost
};
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

// 哨兵专有
enum ShootMode
{
  left_shoot,
  right_shoot,
  both_shoot
};
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

class CBoard
{
public:
  using ImuProvider = std::function<Eigen::Quaterniond(std::chrono::steady_clock::time_point)>;

  double bullet_speed;
  Mode mode;
  ShootMode shoot_mode;
  double ft_angle;  //无人机专有

  CBoard(const std::string & config_path, ImuProvider imu_provider = {});

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  void send(Command command) const;

private:
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  ImuProvider imu_provider_;
  tools::ThreadSafeQueue<IMUData> queue_;  // 必须在can_之前初始化，否则存在死锁的可能
  std::unique_ptr<SocketCAN> can_;
  IMUData data_ahead_;
  IMUData data_behind_;

  int quaternion_canid_, bullet_speed_canid_, send_canid_;
  bool ros_imu_connect_can_{true};
  std::string ros_command_topic_{"vision/command"};

#ifdef SP_VISION_HAS_ROS2
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr command_pub_;
#endif

  void callback(const can_frame & frame);

  std::string read_yaml(const std::string & config_path);
};

}  // namespace io

#endif  // IO__CBOARD_HPP
