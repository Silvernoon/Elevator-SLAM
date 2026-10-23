// SPDX-License-Identifier: GPL-3.0-or-later
#include "elevator_fusion/core.hpp"
#include <elevator_fusion/msg/fusion_status.hpp>
#include <lio/msg/lio_frame.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>


namespace elevator_fusion {
namespace {

template <typename T> class Ring {
public:
    explicit Ring(std::size_t capacity) : entries_(capacity) {}
    bool Empty() const { return size_ == 0; }
    bool Full() const { return size_ == entries_.size(); }
    std::size_t Size() const { return size_; }
    T& At(std::size_t index) { return entries_[(begin_ + index) % entries_.size()]; }
    const T& At(std::size_t index) const { return entries_[(begin_ + index) % entries_.size()]; }
    T& Front() { return At(0); }
    void Pop() {
        At(0) = T{};
        begin_ = (begin_ + 1) % entries_.size();
        --size_;
    }
    void Push(T value) {
        if (Full()) Pop();
        entries_[(begin_ + size_) % entries_.size()] = std::move(value);
        ++size_;
    }
    void Clear() { while (!Empty()) Pop(); }
private:
    std::vector<T> entries_;
    std::size_t begin_ = 0;
    std::size_t size_ = 0;
};

Pose FromMessage(const geometry_msgs::msg::Pose& message) {
    return {Eigen::Quaterniond(message.orientation.w, message.orientation.x,
                              message.orientation.y, message.orientation.z),
            Eigen::Vector3d(message.position.x, message.position.y, message.position.z)};
}

geometry_msgs::msg::Pose ToMessage(const Pose& pose) {
    geometry_msgs::msg::Pose message;
    message.position.x = pose.translation.x();
    message.position.y = pose.translation.y();
    message.position.z = pose.translation.z();
    message.orientation.w = pose.rotation.w();
    message.orientation.x = pose.rotation.x();
    message.orientation.y = pose.rotation.y();
    message.orientation.z = pose.rotation.z();
    return message;
}

Eigen::Matrix4f MakeTransformMatrix(const Pose& pose) {
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    result.topLeftCorner<3, 3>() = pose.rotation.toRotationMatrix().cast<float>();
    result.topRightCorner<3, 1>() = pose.translation.cast<float>();
    return result;
}

Pose ReadExtrinsic(rclcpp::Node& node, const std::string& name) {
    const auto values = node.declare_parameter<std::vector<double>>(name, std::vector<double>{});
    if (values.size() != 7) {
        throw std::runtime_error("set " + name + " to calibrated [x,y,z,qx,qy,qz,qw]");
    }
    Pose pose{Eigen::Quaterniond(values[6], values[3], values[4], values[5]),
              Eigen::Vector3d(values[0], values[1], values[2])};
    if (!pose.IsValid()) throw std::runtime_error("invalid rigid transform: " + name);
    return pose;
}

GeometryThresholds ReadThresholds(rclcpp::Node& node) {
    GeometryThresholds thresholds;
    const auto min_points = node.declare_parameter<std::int64_t>("geometry.min_points", 11);
    if (min_points <= 0 || min_points > std::numeric_limits<std::uint32_t>::max()) {
        throw std::runtime_error("geometry.min_points must be a positive uint32 value");
    }
    thresholds.min_points = static_cast<std::uint32_t>(min_points);
    thresholds.min_singular_value = node.declare_parameter<double>("geometry.min_singular_value", thresholds.min_singular_value);
    thresholds.min_mean_singular_value = node.declare_parameter<double>(
        "geometry.min_mean_singular_value", thresholds.min_mean_singular_value);
    return thresholds;
}

using Stamp = builtin_interfaces::msg::Time;

bool StampLess(const Stamp& lhs, const Stamp& rhs) {
    return lhs.sec < rhs.sec || (lhs.sec == rhs.sec && lhs.nanosec < rhs.nanosec);
}

bool StampEqual(const Stamp& lhs, const Stamp& rhs) {
    return lhs.sec == rhs.sec && lhs.nanosec == rhs.nanosec;
}

double StampSeconds(const Stamp& stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

struct PendingFrame {
    lio::msg::LioFrame::ConstSharedPtr message;
    std::chrono::steady_clock::time_point arrival;
};
struct PendingCloud {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message;
    std::chrono::steady_clock::time_point arrival;
};
struct CloudTransform {
    Stamp stamp;
    Pose world_to_fused;
    Pose world_to_body;
};

} // namespace

class FusionNode : public rclcpp::Node {
public:
    FusionNode() : Node("trajectory_fusion"), fusion_(ReadThresholds(*this)),
                   vio_history_(2048), frames_(128), clouds_(128), transforms_(2048) {
        enable_vio_ = declare_parameter<bool>("enable_vio", true);
        max_vio_gap_ = declare_parameter<double>("max_vio_gap", 0.15);
        max_wait_ = declare_parameter<double>("max_wait", 0.2);
        world_frame_ = declare_parameter<std::string>("world_frame", "fusion_world");
        body_frame_ = declare_parameter<std::string>("body_frame", "fusion_imu");
        if (!std::isfinite(max_vio_gap_) || max_vio_gap_ <= 0.0 || !std::isfinite(max_wait_) ||
            max_wait_ <= 0.0 || world_frame_.empty() || body_frame_.empty() || world_frame_ == body_frame_) {
            throw std::runtime_error("invalid fusion timing or frame configuration");
        }
        if (enable_vio_) vio_to_lio_ = ReadExtrinsic(*this, "imu_lio_T_imu_vio").Inverse();

        const auto reliable_qos = [](std::size_t depth) {
            return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable();
        };
        odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>("/fusion/odometry", reliable_qos(20));
        status_pub_ = create_publisher<elevator_fusion::msg::FusionStatus>("/fusion/status", reliable_qos(20));
        world_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/fusion/cloud_world", reliable_qos(4));
        body_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/fusion/cloud_body", reliable_qos(4));
        frame_sub_ = create_subscription<lio::msg::LioFrame>(
            "/LIO/frame", reliable_qos(128), std::bind(&FusionNode::OnFrame, this, std::placeholders::_1));
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/LIO/clouds_lidar", reliable_qos(16), std::bind(&FusionNode::OnCloud, this, std::placeholders::_1));
        if (enable_vio_) {
            vio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                "/vins_estimator/odometry", reliable_qos(512),
                std::bind(&FusionNode::OnVio, this, std::placeholders::_1));
            auto epoch_qos = reliable_qos(4).transient_local();
            epoch_sub_ = create_subscription<std_msgs::msg::UInt32>(
                "/vins_estimator/restart_epoch", epoch_qos,
                std::bind(&FusionNode::OnEpoch, this, std::placeholders::_1));
        }
        broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&FusionNode::OnTimer, this));
    }

private:
    void OnFrame(const lio::msg::LioFrame::ConstSharedPtr message) {
        if (have_frame_ && !StampLess(latest_frame_, message->header.stamp)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Dropping non-monotonic LIO frame; restart the pipeline after bag rewind");
            return;
        }
        if (message->header.frame_id != "world") {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "LIO scan frame must use world coordinates");
            return;
        }
        latest_frame_ = message->header.stamp;
        have_frame_ = true;
        if (frames_.Full()) RCLCPP_WARN(get_logger(), "Fusion scan queue overflow: dropping oldest scan");
        frames_.Push({message, std::chrono::steady_clock::now()});
        Drain();
    }

    void OnVio(const nav_msgs::msg::Odometry::ConstSharedPtr message) {
        const Pose input = FromMessage(message->pose.pose);
        if (!input.IsValid()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Dropping malformed VIO pose");
            return;
        }
        const Pose pose = input * vio_to_lio_;
        if (!vio_history_.Push({StampSeconds(message->header.stamp), pose})) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Dropping invalid or non-monotonic VIO pose");
            return;
        }
        Drain();
    }

    void OnEpoch(const std_msgs::msg::UInt32::ConstSharedPtr message) {
        if (!vio_epoch_ || *vio_epoch_ != message->data) {
            vio_epoch_ = message->data;
            vio_history_.Clear();
            fusion_.InvalidateVio();
        }
    }

    void OnCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        if (message->header.frame_id != "world") {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Input LIO cloud must be expressed in world");
            return;
        }
        if (clouds_.Full()) RCLCPP_WARN(get_logger(), "Fusion cloud queue overflow: dropping oldest cloud");
        clouds_.Push({message, std::chrono::steady_clock::now()});
        DrainClouds();
    }

    void OnTimer() { Drain(); }

    void Drain() {
        const auto now = std::chrono::steady_clock::now();
        while (!frames_.Empty()) {
            const auto& pending = frames_.Front();
            const auto& message = *pending.message;
            const double stamp = StampSeconds(message.header.stamp);
            const auto vio = enable_vio_ ? vio_history_.Interpolate(stamp, max_vio_gap_) : std::nullopt;
            if (enable_vio_ && !vio && now - pending.arrival < std::chrono::duration<double>(max_wait_)) break;
            LioFrame frame;
            frame.stamp = stamp;
            frame.imu_pose = FromMessage(message.imu_pose);
            frame.cloud_imu_pose = FromMessage(message.cloud_imu_pose);
            frame.in_elevator = message.in_elevator;
            frame.geometry.effective_points = message.effective_points;
            frame.geometry.lidar_updated = message.lidar_updated;
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    frame.geometry.normal_information(row, column) = message.normal_information[row * 3 + column];
                }
            }
            const auto result = fusion_.Update(frame, vio);
            if (result) {
                Publish(*result, message);
                transforms_.Push({message.header.stamp, result->cloud_world_to_fused_world,
                                  frame.cloud_imu_pose.Inverse()});
            } else RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected invalid LIO scan pose");
            frames_.Pop();
        }
        DrainClouds();
    }

    void Publish(const FusionResult& result, const lio::msg::LioFrame& input) {
        nav_msgs::msg::Odometry odometry;
        odometry.header.stamp = input.header.stamp;
        odometry.header.frame_id = world_frame_;
        odometry.child_frame_id = body_frame_;
        odometry.pose.pose = ToMessage(result.imu_pose);
        // This is a pose-selection output, not an independent covariance/velocity estimator.
        odometry_pub_->publish(odometry);
        geometry_msgs::msg::TransformStamped transform;
        transform.header = odometry.header;
        transform.child_frame_id = body_frame_;
        transform.transform.translation.x = result.imu_pose.translation.x();
        transform.transform.translation.y = result.imu_pose.translation.y();
        transform.transform.translation.z = result.imu_pose.translation.z();
        transform.transform.rotation = odometry.pose.pose.orientation;
        broadcaster_->sendTransform(transform);
        auto& status = status_message_;
        status.header = odometry.header;
        status.source = result.source == Source::kLio ? elevator_fusion::msg::FusionStatus::SOURCE_LIO
                                                      : elevator_fusion::msg::FusionStatus::SOURCE_VIO;
        status.reason = ReasonName(result.reason);
        status.degenerate = result.geometry.degenerate;
        for (int i = 0; i < 3; ++i) status.normal_singular_values[i] = result.geometry.singular_values(i);
        status.effective_points = input.effective_points;
        status.in_elevator = input.in_elevator;
        status_pub_->publish(status);
    }

    const CloudTransform* FindTransform(const Stamp& stamp) const {
        std::size_t low = 0;
        std::size_t high = transforms_.Size();
        while (low < high) {
            const auto middle = low + (high - low) / 2;
            if (StampLess(transforms_.At(middle).stamp, stamp)) low = middle + 1;
            else high = middle;
        }
        return low < transforms_.Size() && StampEqual(transforms_.At(low).stamp, stamp) ? &transforms_.At(low) : nullptr;
    }

    void DrainClouds() {
        const auto now = std::chrono::steady_clock::now();
        // Clouds may arrive before their scan frame, and may be decimated independently.
        const auto count = clouds_.Size();
        for (std::size_t index = 0; index < count; ++index) {
            auto pending = std::move(clouds_.Front());
            clouds_.Pop();
            const auto* transform = FindTransform(pending.message->header.stamp);
            if (transform) {
                auto& output = cloud_message_;
                if (world_cloud_pub_->get_subscription_count()) {
                    pcl_ros::transformPointCloud(MakeTransformMatrix(transform->world_to_fused), *pending.message, output);
                    output.header = pending.message->header;
                    output.header.frame_id = world_frame_;
                    world_cloud_pub_->publish(output);
                }
                if (body_cloud_pub_->get_subscription_count()) {
                    pcl_ros::transformPointCloud(MakeTransformMatrix(transform->world_to_body), *pending.message, output);
                    output.header = pending.message->header;
                    output.header.frame_id = body_frame_;
                    body_cloud_pub_->publish(output);
                }
            } else if (now - pending.arrival < std::chrono::duration<double>(max_wait_ * 2.0)) {
                clouds_.Push(std::move(pending));
            } else RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Dropping cloud without an exact scan-end pose; no nearest-pose substitution");
        }
    }

    TrajectoryFusion fusion_;
    PoseHistory vio_history_;
    Ring<PendingFrame> frames_;
    Ring<PendingCloud> clouds_;
    Ring<CloudTransform> transforms_;
    Pose vio_to_lio_;
    bool enable_vio_ = true;
    bool have_frame_ = false;
    Stamp latest_frame_;
    std::optional<std::uint32_t> vio_epoch_;
    double max_vio_gap_ = 0.15;
    double max_wait_ = 0.2;
    std::string world_frame_;
    std::string body_frame_;
    sensor_msgs::msg::PointCloud2 cloud_message_;
    elevator_fusion::msg::FusionStatus status_message_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Publisher<elevator_fusion::msg::FusionStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr world_cloud_pub_, body_cloud_pub_;
    rclcpp::Subscription<lio::msg::LioFrame>::SharedPtr frame_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr epoch_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

} // namespace elevator_fusion

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<elevator_fusion::FusionNode>();
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("trajectory_fusion"), "Fusion startup failed: %s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
