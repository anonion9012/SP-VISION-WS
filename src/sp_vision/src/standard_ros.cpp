// 这是一个ROS2的主节点，承载AutoAim系统，是控制整个自瞄系统流程的子系统。

// c++ standard
#include <print>
#include <chrono>
#include <cstddef>
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
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

namespace
{
std::string default_config_path()
{
    return ament_index_cpp::get_package_share_directory("sp_vision") + "/configs/standard3.yaml";
}
}

//自瞄节点
class AutoAim : public rclcpp::Node {
    public:
    AutoAim()
        : Node("auto_aim"),
          imu(std::make_shared<io::ROSIMU>()),
          cboard(
              config_path,
              [this](std::chrono::steady_clock::time_point timestamp) {
                  return imu->try_imu_at(timestamp, std::chrono::milliseconds(100))
                      .value_or(Eigen::Quaterniond::Identity());
              })
    {
        set_params();
        std::println(">>>>>AutoAim node started<<<<<");
        std::println("config_path: {}", config_path);
        tools::logger()->info("config_path: {}", config_path);
        image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 
            10,
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
                this->callback(msg);
            }
        );
        armor_img_pub = this->create_publisher<sensor_msgs::msg::Image>("debug/armor_img", 10);
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
    // @msg 接收图像信息
    // Orienta 消息没有时间戳，IMU 也使用接收时的 steady_clock，因此图像使用当前本机时间。
    void callback(sensor_msgs::msg::Image::ConstSharedPtr msg) {
        cv::Mat img{cv_bridge::toCvShare(msg, "bgr8")->image};
        const auto t = std::chrono::steady_clock::now();
        const auto imu_q = imu->try_imu_at(t, std::chrono::milliseconds(100));
        if (!imu_q) {
            std::println("[WARN] IMU data timeout, using the last valid orientation.");
        } else {
            last_imu = *imu_q;
        }
        const auto & q = last_imu;
        std::println(">>>>>{} : {},{},{},{}", t.time_since_epoch().count(), q.w(), q.x(), q.y(), q.z());

        const auto mode = cboard.mode;
        
        if (mode != last_mode) { 
            tools::logger()->info("Switch to {}", io::MODES[mode]);
            std::println("Switch to {}", io::MODES[mode]);
            last_mode = mode;
        }

        solver.set_R_gimbal2world(q);

        auto armors = detector.detect(img);
        for (const auto& armor : armors) {
            if (armor.points.empty()) {
                continue;
            }

            tools::draw_points(img, armor.points);

            const auto name_index = static_cast<std::size_t>(armor.name);
            if (name_index < auto_aim::ARMOR_NAMES.size()) {
                tools::draw_text(img, auto_aim::ARMOR_NAMES[name_index], armor.points[0],
                                 {255, 255, 255}, 2);
            }
        }

        // Debug output is optional and published once per input frame. A
        // missing debug viewer does not affect the main auto-aim pipeline.
        if (armor_img_pub) {
            const auto debug_image = cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg();
            armor_img_pub->publish(*debug_image);
        }

        auto targets = tracker.track(armors, t);

        auto command = aimer.aim(targets, t, cboard.bullet_speed);

        cboard.send(command);
    }

    void set_params() {
        this->declare_parameter<std::string>("config_path", config_path);
        config_path = this->get_parameter("config_path").as_string();

    }

    std::string config_path{default_config_path()};

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
    Eigen::Quaterniond last_imu{Eigen::Quaterniond::Identity()};

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr armor_img_pub;

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
