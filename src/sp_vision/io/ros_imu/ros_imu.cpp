#include "ros_imu.hpp"

#include "tools/logger.hpp"

namespace io {
    ROSIMU::ROSIMU() : Node("ros_imu"), queue_(5000) {
        orienta_sub_ = this->create_subscription<autoaim_msgs::msg::Orienta>(
            "imu/quaternion",
            10,
            [this](autoaim_msgs::msg::Orienta::ConstSharedPtr msg)
            {
                this->callback(msg);
            });
        tools::logger()->info("[ROSIMU] initialized");
    }

    ROSIMU::~ROSIMU() = default;

    void ROSIMU::callback(autoaim_msgs::msg::Orienta::ConstSharedPtr msg) {
        auto timestamp = std::chrono::steady_clock::now();
        Eigen::Quaterniond q{msg->w, msg->x, msg->y, msg->z};
        q.normalize();
        queue_.push({q, timestamp});
    }

    std::optional<Eigen::Quaterniond> ROSIMU::try_imu_at(
        std::chrono::steady_clock::time_point timestamp,
        std::chrono::milliseconds timeout) {
        if (!has_initial_data_) {
            if (!queue_.pop_for(data_ahead_, timeout) ||
                !queue_.pop_for(data_behind_, timeout)) {
                return std::nullopt;
            }
            has_initial_data_ = true;
        }

        if (data_behind_.timestamp < timestamp)
            data_ahead_ = data_behind_;

        while (true)
        {
            if (!queue_.pop_for(data_behind_, timeout)) {
                return std::nullopt;
            }
            if (data_behind_.timestamp > timestamp)
                break;
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
}
