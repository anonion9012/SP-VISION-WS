#include "cboard.hpp"

#include <utility>

#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
CBoard::CBoard(const std::string & config_path, ImuProvider imu_provider)
: mode(Mode::idle),
  shoot_mode(ShootMode::left_shoot),
  bullet_speed(0),
  imu_provider_(std::move(imu_provider)),
  queue_(5000)
// 注意: callback的运行会早于Cboard构造函数的完成
{
  auto can_interface = read_yaml(config_path);
  auto should_connect_can = !imu_provider_ || ros_imu_connect_can_;

  if (should_connect_can) {
    can_ = std::make_unique<SocketCAN>(
      can_interface, std::bind(&CBoard::callback, this, std::placeholders::_1));
  } else {
    tools::logger()->info("[Cboard] Skipped SocketCAN connection.");
  }

#ifdef SP_VISION_HAS_ROS2
  if (rclcpp::ok()) {
    ros_node_ = std::make_shared<rclcpp::Node>("cboard_command_publisher");
    command_pub_ =
      ros_node_->create_publisher<std_msgs::msg::UInt8MultiArray>(ros_command_topic_, 10);
    tools::logger()->info("[Cboard] ROS command publisher: {}", ros_command_topic_);
  }
#endif

  if (!imu_provider_) {
    tools::logger()->info("[Cboard] Waiting for q...");
    queue_.pop(data_ahead_);
    queue_.pop(data_behind_);
  } else {
    tools::logger()->info("[Cboard] Using external IMU provider.");
  }
  tools::logger()->info("[Cboard] Opened.");
}

Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (imu_provider_) {
    return imu_provider_(timestamp);
  }

  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  while (true) {
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }

  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;
  auto t_c = timestamp;
  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = t_c - t_a;

  // 四元数插值
  auto k = t_ac / t_ab;
  Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

  return q_c;
}

void CBoard::send(Command command) const
{
  can_frame frame;
  frame.can_id = send_canid_;
  frame.can_dlc = 8;
  frame.data[0] = (command.control) ? 1 : 0;
  frame.data[1] = (command.shoot) ? 1 : 0;
  frame.data[2] = (int16_t)(command.yaw * 1e4) >> 8;
  frame.data[3] = (int16_t)(command.yaw * 1e4);
  frame.data[4] = (int16_t)(command.pitch * 1e4) >> 8;
  frame.data[5] = (int16_t)(command.pitch * 1e4);
  frame.data[6] = (int16_t)(command.horizon_distance * 1e4) >> 8;
  frame.data[7] = (int16_t)(command.horizon_distance * 1e4);

#ifdef SP_VISION_HAS_ROS2
  if (command_pub_) {
    std_msgs::msg::UInt8MultiArray msg;
    msg.data.assign(frame.data, frame.data + frame.can_dlc);
    command_pub_->publish(msg);
  }
#endif

  if (!can_) return;

  try {
    can_->write(&frame);
  } catch (const std::exception & e) {
    tools::logger()->warn("{}", e.what());
  }
}

void CBoard::callback(const can_frame & frame)
{
  auto timestamp = std::chrono::steady_clock::now();

  if (frame.can_id == quaternion_canid_) {
    if (imu_provider_) {
      return;
    }

    auto x = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e4;
    auto y = (int16_t)(frame.data[2] << 8 | frame.data[3]) / 1e4;
    auto z = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e4;
    auto w = (int16_t)(frame.data[6] << 8 | frame.data[7]) / 1e4;

    if (std::abs(x * x + y * y + z * z + w * w - 1) > 1e-2) {
      tools::logger()->warn("Invalid q: {} {} {} {}", w, x, y, z);
      return;
    }

    queue_.push({{w, x, y, z}, timestamp});
  }

  else if (frame.can_id == bullet_speed_canid_) {
    bullet_speed = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e2;
    mode = Mode(frame.data[2]);
    shoot_mode = ShootMode(frame.data[3]);
    ft_angle = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e4;

    // 限制日志输出频率为1Hz
    static auto last_log_time = std::chrono::steady_clock::time_point::min();
    auto now = std::chrono::steady_clock::now();

    if (bullet_speed > 0 && tools::delta_time(now, last_log_time) >= 1.0) {
      tools::logger()->info(
        "[CBoard] Bullet speed: {:.2f} m/s, Mode: {}, Shoot mode: {}, FT angle: {:.2f} rad",
        bullet_speed, MODES[mode], SHOOT_MODES[shoot_mode], ft_angle);
      last_log_time = now;
    }
  }
}

// 实现方式有待改进
std::string CBoard::read_yaml(const std::string & config_path)
{
  auto yaml = tools::load(config_path);

  quaternion_canid_ = tools::read<int>(yaml, "quaternion_canid");
  bullet_speed_canid_ = tools::read<int>(yaml, "bullet_speed_canid");
  send_canid_ = tools::read<int>(yaml, "send_canid");
  if (yaml["ros_imu_connect_can"]) ros_imu_connect_can_ = yaml["ros_imu_connect_can"].as<bool>();
  if (yaml["ros_command_topic"]) ros_command_topic_ = yaml["ros_command_topic"].as<std::string>();

  if (!yaml["can_interface"]) {
    throw std::runtime_error("Missing 'can_interface' in YAML configuration.");
  }

  return yaml["can_interface"].as<std::string>();
}

}  // namespace io
