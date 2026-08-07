// 这是一个ROS2的主节点，承载AutoAim系统，是控制整个自瞄系统流程的子系统。

// c++ standard
#include <print>
#include <chrono>

// opencv
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// ros2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

// sp_vision
#include "io/cboard.hpp"
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


//自瞄节点
class AutoAim : public rclcpp::Node {
    public:
    AutoAim() : Node("auto_aim") {
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
    }

    private:
    // @msg 接收图像信息
    // $msg->header.stamp 单调时钟（steady_clock）的纳秒值，转换为chrono::steady_clock::time_point类型
    void callback(sensor_msgs::msg::Image::ConstSharedPtr msg) {
        cv::Mat img{cv_bridge::toCvShare(msg, "bgr8")->image};
        const auto stamp_ns =
            static_cast<int64_t>(msg->header.stamp.sec) * 1'000'000'000LL +
            static_cast<int64_t>(msg->header.stamp.nanosec);
        std::chrono::steady_clock::time_point t{std::chrono::nanoseconds(stamp_ns)};
        Eigen::Quaterniond q{cboard.imu_at(t)};

        auto mode = cboard.mode;
        
        if (mode != last_mode) { 
            tools::logger()->info("Switch to {}", io::MODES[mode]);
            std::println("Switch to {}", io::MODES[mode]);
            last_mode = mode;
        }

        solver.set_R_gimbal2world(q);

        auto armors = detector.detect(img);

        auto targets = tracker.track(armors, t);

        auto command = aimer.aim(targets, t, cboard.bullet_speed);

        cboard.send(command);
        
    }

    void set_params() {
        this->declare_parameter<std::string>("config_path", "configs/standard3.yaml");
        config_path = this->get_parameter("config_path").as_string();
    }

    std::string config_path{};

    tools::Exiter exiter;
    tools::Plotter plotter;
    tools::Recorder recorder;

    io::CBoard cboard{config_path};
    io::Camera camera{config_path};

    auto_aim::YOLO detector{config_path, false};
    auto_aim::Solver solver{config_path};
    auto_aim::Tracker tracker{config_path, solver};
    auto_aim::Aimer aimer{config_path};
    auto_aim::Shooter shooter{config_path};

    io::Mode last_mode{io::Mode::idle};

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoAim>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
