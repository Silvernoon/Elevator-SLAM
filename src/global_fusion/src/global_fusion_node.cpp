// SPDX-License-Identifier: GPL-3.0-only
#include "global_optimization.h"

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <cstdint>
#include <functional>

class GlobalFusionNode final : public rclcpp::Node {
public:
    GlobalFusionNode() : Node("global_fusion") {
        global_frame_ = declare_parameter<std::string>("global_frame", "global_world");
        position_sigma_ = declare_parameter<double>("position_sigma", 0.1);
        if (!std::isfinite(position_sigma_) || position_sigma_ <= 0.0 || global_frame_.empty()) {
            throw std::runtime_error("invalid global position reference configuration");
        }

        odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/global_odometry", rclcpp::QoS(20).reliable());
        path_pub_ = create_publisher<nav_msgs::msg::Path>("~/global_path", rclcpp::QoS(2).reliable());
        odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/fusion/odometry", rclcpp::QoS(100).reliable(),
            std::bind(&GlobalFusionNode::OnOdometry, this, std::placeholders::_1));
        anchor_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/tag_Odometry", rclcpp::QoS(100).reliable(),
            std::bind(&GlobalFusionNode::OnAnchor, this, std::placeholders::_1));
    }

private:
    void OnOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr input) {
        const auto& p = input->pose.pose.position;
        const auto& q = input->pose.pose.orientation;
        if (!optimizer_.InputOdom(rclcpp::Time(input->header.stamp).seconds(), Eigen::Vector3d(p.x, p.y, p.z),
                                  Eigen::Quaterniond(q.w, q.x, q.y, q.z))) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Rejected invalid/non-monotonic fusion odometry");
            return;
        }
        Publish();
    }

    void OnAnchor(const nav_msgs::msg::Odometry::ConstSharedPtr input) {
        if (input->header.frame_id != global_frame_) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                                  "Position reference must be expressed in configured global_frame");
            return;
        }
        const auto& p = input->pose.pose.position;
        if (!optimizer_.InputAprilTag(rclcpp::Time(input->header.stamp).seconds(), p.x, p.y, p.z, position_sigma_)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Rejected invalid absolute position reference");
            return;
        }
        Publish();
    }

    void Publish() {
        const auto& poses = optimizer_.Trajectory();
        if (poses.empty()) return;
        path_.header.frame_id = global_frame_;
        path_.header.stamp = rclcpp::Time(static_cast<int64_t>(poses.rbegin()->first * 1e9));
        path_.poses.resize(poses.size());
        std::size_t index = 0;
        for (const auto& item : poses) {
            auto& pose = path_.poses[index++];
            pose.header.frame_id = global_frame_;
            pose.header.stamp = rclcpp::Time(static_cast<int64_t>(item.first * 1e9));
            pose.pose.position.x = item.second.p[0];
            pose.pose.position.y = item.second.p[1];
            pose.pose.position.z = item.second.p[2];
            pose.pose.orientation.w = item.second.q[0];
            pose.pose.orientation.x = item.second.q[1];
            pose.pose.orientation.y = item.second.q[2];
            pose.pose.orientation.z = item.second.q[3];
        }
        nav_msgs::msg::Odometry odometry;
        odometry.header = path_.header;
        odometry.child_frame_id = "global_imu";
        odometry.pose.pose = path_.poses.back().pose;
        odometry_pub_->publish(odometry);
        if (path_pub_->get_subscription_count()) path_pub_->publish(path_);
    }

    GlobalOptimization optimizer_;
    nav_msgs::msg::Path path_;
    std::string global_frame_;
    double position_sigma_ = 0.1;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr anchor_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<GlobalFusionNode>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("global_fusion"), "Global fusion failed: %s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
