#pragma once

#include <math.h>

#include <Eigen/Geometry>
#include <atomic>
#include <array>
#include <optional>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <initializer_list>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "autoaim_msgs/msg/orienta.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io {
    class ROSIMU : public rclcpp::Node {
        public:
        ROSIMU();
        ~ROSIMU();

        std::optional<Eigen::Quaterniond> try_imu_at(
            std::chrono::steady_clock::time_point timestamp,
            std::chrono::milliseconds timeout);

        std::uint64_t imu_count() const;

    private:
        struct IMUData
        {
            Eigen::Quaterniond q;
            std::chrono::steady_clock::time_point timestamp;
        };

        void callback(autoaim_msgs::msg::Orienta::ConstSharedPtr msg);

        rclcpp::Subscription<autoaim_msgs::msg::Orienta>::SharedPtr orienta_sub_;

        tools::ThreadSafeQueue<IMUData> queue_;
        IMUData data_ahead_, data_behind_;
        bool has_initial_data_{false};
        std::atomic<std::uint64_t> imu_count_{0};
    };
}
