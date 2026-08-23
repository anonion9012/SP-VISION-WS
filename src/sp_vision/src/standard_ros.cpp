// 这是一个ROS2的主节点，承载AutoAim系统，是控制整个自瞄系统流程的子系统。

// c++ standard
#include <print>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>

// opencv
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// ros2
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "autoaim_msgs/msg/orienta.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

// sp_vision
#include "io/cboard.hpp"
#include "io/ros_imu/ros_imu.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

namespace
{
constexpr std::size_t ROS_QUEUE_DEPTH = 2000;

std::string default_config_path()
{
    return ament_index_cpp::get_package_share_directory("sp_vision") + "/configs/standard3.yaml";
}

bool load_force_match(const std::string & config_path)
{
    const auto yaml = tools::load(config_path);
    return yaml["ros_imu_force_match"] ? yaml["ros_imu_force_match"].as<bool>() : false;
}

int tracker_state_code(const std::string & state)
{
    if (state == "detecting") return 1;
    if (state == "tracking") return 2;
    if (state == "temp_lost") return 3;
    if (state == "switching") return 4;
    return 0;
}
}

//自瞄节点
class AutoAim : public rclcpp::Node {
    public:
    AutoAim()
        : Node("auto_aim"),
          config_path(declare_parameter<std::string>("config_path", default_config_path())),
          ros_imu_force_match(load_force_match(config_path)),
          imu(std::make_shared<io::ROSIMU>(ros_imu_force_match)),
          cboard(
              config_path,
              [this](std::chrono::steady_clock::time_point timestamp) {
                  return imu->try_imu_at(timestamp, std::chrono::milliseconds(100))
                      .value_or(Eigen::Quaterniond::Identity());
              })
    {
        std::println(">>>>>AutoAim node started<<<<<");
        std::println("config_path: {}", config_path);
        tools::logger()->info("config_path: {}", config_path);
        image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 
            rclcpp::QoS(rclcpp::KeepLast(ROS_QUEUE_DEPTH)).reliable().durability_volatile(),
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
                this->callback(msg);
            }
        );
        armor_img_pub = this->create_publisher<sensor_msgs::msg::Image>("debug/armor_img", 10);
        tracker_state_pub = this->create_publisher<std_msgs::msg::String>("debug/tracker_state", 10);
        tracker_state_code_pub = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("debug/tracker_state_code", 10);
        gimbal_yaw_pub = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("debug/gimbal_yaw", 10);
        target_yaw_pub = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("debug/target_yaw", 10);
        imu_count_pub = this->create_publisher<std_msgs::msg::UInt64>("debug/imu_count", 10);
        image_count_pub = this->create_publisher<std_msgs::msg::UInt64>("debug/image_count", 10);
        armor_point_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("debug/armor_point", 10);
        target_point_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("debug/target_point", 10);
        try{
            imu_thread = std::thread([this]() { rclcpp::spin(imu); });
        } catch (const std::exception & e) {
            std::println("[ERROR] Failed to start IMU node: {}", e.what());
            tools::logger()->error("[ERROR] Failed to start IMU node: {}", e.what());
            std::exit(1);
        }
        
    }

    ~AutoAim() {
        if (imu_thread.joinable()) {
            // rclcpp::spin(imu) only returns after the context is shut down.
            if (rclcpp::ok()) {
                rclcpp::shutdown();
            }
            imu_thread.join();
        }
        tools::logger()->info("ROS_imu stopped.");
        std::println(">>>>>AutoAim node stopped<<<<<");
    }

    bool exit_requested() const { return exiter.exit(); }

    private:
    // Must precede cboard/detector/solver declarations: C++ initializes members
    // in declaration order, not initializer-list order.
    std::string config_path;
    bool ros_imu_force_match{false};

    // @msg 接收图像信息
    void callback(sensor_msgs::msg::Image::ConstSharedPtr msg) {
        ++image_count_;

        cv::Mat img{cv_bridge::toCvShare(msg, "bgr8")->image};
        const auto stamp_ns =
            static_cast<int64_t>(msg->header.stamp.sec) * 1'000'000'000LL +
            static_cast<int64_t>(msg->header.stamp.nanosec);
        if (!fst_frame_initialized_) {
            fst_frame_time = std::chrono::steady_clock::now();
            fst_frame_stamp = std::chrono::nanoseconds(stamp_ns);
            fst_frame_initialized_ = true;
        }
        
        const auto t = fst_frame_time + (std::chrono::nanoseconds(stamp_ns) - fst_frame_stamp);
        // Orienta 没有时间戳，bag 契约保证图像和姿态同帧同序。
        // 强制模式按序取姿态；非强制模式才按本机接收时间插值。
        const auto imu_query_time =
            ros_imu_force_match ? t : std::chrono::steady_clock::now();
        const auto imu_q = imu->try_imu_at(imu_query_time, std::chrono::milliseconds(100));
        if (!imu_q) {
            static auto last_timeout_log = std::chrono::steady_clock::time_point::min();
            const auto now = std::chrono::steady_clock::now();
            if (tools::delta_time(now, last_timeout_log) >= 1.0) {
                std::println("[WARN] IMU data timeout, dropping unpaired image.");
                last_timeout_log = now;
            }
            return;
        }
        const auto & q = *imu_q;

        std_msgs::msg::UInt64 imu_count_msg{};
        imu_count_msg.data = imu->imu_count();
        imu_count_pub->publish(imu_count_msg);
        std_msgs::msg::UInt64 image_count_msg{};
        image_count_msg.data = image_count_;
        image_count_pub->publish(image_count_msg);

        const auto mode = cboard.mode;
        
        if (mode != last_mode) { 
            tools::logger()->info("Switch to {}", io::MODES[mode]);
            std::println("Switch to {}", io::MODES[mode]);
            last_mode = mode;
        }

        solver.set_R_gimbal2world(q);
        const auto gimbal_ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

        geometry_msgs::msg::Vector3Stamped gimbal_ypr_msg{};
        gimbal_ypr_msg.header.stamp = msg->header.stamp;
        gimbal_ypr_msg.header.frame_id = "world";
        gimbal_ypr_msg.vector.x = gimbal_ypr[0] * 180.0 / CV_PI;
        gimbal_ypr_msg.vector.y = gimbal_ypr[1] * 180.0 / CV_PI;
        gimbal_ypr_msg.vector.z = gimbal_ypr[2] * 180.0 / CV_PI;
        gimbal_yaw_pub->publish(gimbal_ypr_msg);

        auto armors = detector.detect(img);

        auto solved_armors = armors;
        for (auto & armor : solved_armors) {
            solver.solve(armor);
        }

        auto targets = tracker.track(armors, t);

        std_msgs::msg::String tracker_state{};
        tracker_state.data = tracker.state();
        tracker_state_pub->publish(tracker_state);
        geometry_msgs::msg::Vector3Stamped tracker_state_code_msg{};
        tracker_state_code_msg.header.stamp = msg->header.stamp;
        tracker_state_code_msg.header.frame_id = "world";
        tracker_state_code_msg.vector.x = tracker_state_code(tracker_state.data);
        tracker_state_code_pub->publish(tracker_state_code_msg);

        // 使用迭代器遍历
        auto armor_it = solved_armors.begin();
        auto target_it = targets.begin();
        for (; armor_it != solved_armors.end(); ++armor_it)
        {
            const auto &armor = *armor_it;

            if (armor.points.empty()) {
                continue;
            }

            // armor_points
            const auto name_index = static_cast<std::size_t>(armor.name);

            tools::draw_points(img, armor.points);

            geometry_msgs::msg::PointStamped armor_point{};
            armor_point.header.stamp = msg->header.stamp;
            armor_point.header.frame_id = "world";
            armor_point.point.set__x(armor.xyz_in_world.x());
            armor_point.point.set__y(armor.xyz_in_world.y());
            armor_point.point.set__z(armor.xyz_in_world.z());
            armor_point_pub->publish(armor_point);

            if (name_index < auto_aim::ARMOR_NAMES.size()) {
                tools::draw_text(img, auto_aim::ARMOR_NAMES[name_index], armor.points[0], {255, 255, 255}, 2);
            }

            cv::drawFrameAxes(img, solver.camera_matrix(), solver.distort_coeffs(), armor.rvec, armor.tvec, 1);

        }

        for (; target_it != targets.end(); ++target_it)
        {
            // target
            const auto &target = *target_it;
            const auto target_x = target.ekf_x();
            geometry_msgs::msg::PointStamped target_point{};
            target_point.header.stamp = msg->header.stamp;
            target_point.header.frame_id = "world";
            target_point.point.set__x(target_x[0]);
            target_point.point.set__y(target_x[2]);
            target_point.point.set__z(target_x[4]);
            target_point_pub->publish(target_point);

            geometry_msgs::msg::Vector3Stamped target_yaw_msg{};
            target_yaw_msg.header.stamp = msg->header.stamp;
            target_yaw_msg.header.frame_id = "world";
            target_yaw_msg.vector.x = target_x[6] * 180.0 / CV_PI;
            target_yaw_pub->publish(target_yaw_msg);
        }

        // Debug output is optional and published once per input frame. A
        // missing debug viewer does not affect the main auto-aim pipeline.
        if (armor_img_pub && armor_img_pub->get_subscription_count() > 0)
        {
            const auto debug_image = cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg();
            armor_img_pub->publish(*debug_image);
        }

        auto command = aimer.aim(targets, t, cboard.bullet_speed);

        cboard.send(command);
    }

    std::chrono::steady_clock::time_point fst_frame_time;
    std::chrono::nanoseconds fst_frame_stamp;
    bool fst_frame_initialized_{false};

    tools::Exiter exiter;
    tools::Plotter plotter;
    tools::Recorder recorder;

    std::shared_ptr<io::ROSIMU> imu;
    std::thread imu_thread;
    io::CBoard cboard;

    auto_aim::YOLO detector{config_path, false};
    auto_aim::Solver solver{config_path};
    auto_aim::Tracker tracker{config_path, solver};
    auto_aim::Aimer aimer{config_path};
    auto_aim::Shooter shooter{config_path};

    io::Mode last_mode{io::Mode::idle};
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr armor_img_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tracker_state_pub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr tracker_state_code_pub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gimbal_yaw_pub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr target_yaw_pub;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr armor_point_pub;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_point_pub;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr imu_count_pub;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr image_count_pub;
    std::uint64_t image_count_{0};

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoAim>();
    while (rclcpp::ok() && !node->exit_requested()) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    rclcpp::shutdown();
    node.reset();
    return 0;
}
