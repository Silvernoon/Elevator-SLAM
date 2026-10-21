// SPDX-License-Identifier: GPL-3.0-or-later
#include "elevator_fusion/core.hpp"
#include <elevator_fusion/FusionStatus.h>
#include <lio/LioFrame.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/UInt32.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
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

Pose FromMessage(const geometry_msgs::Pose& message) {
    return {Eigen::Quaterniond(message.orientation.w, message.orientation.x,
                              message.orientation.y, message.orientation.z),
            Eigen::Vector3d(message.position.x, message.position.y, message.position.z)};
}

geometry_msgs::Pose ToMessage(const Pose& pose) {
    geometry_msgs::Pose message;
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

Pose ReadExtrinsic(const ros::NodeHandle& node, const std::string& name) {
    std::vector<double> values;
    if (!node.getParam(name, values) || values.size() != 7) {
        throw std::runtime_error("set ~" + name + " to calibrated [x,y,z,qx,qy,qz,qw]");
    }
    Pose pose{Eigen::Quaterniond(values[6], values[3], values[4], values[5]),
              Eigen::Vector3d(values[0], values[1], values[2])};
    if (!pose.IsValid()) throw std::runtime_error("invalid rigid transform: " + name);
    return pose;
}

GeometryThresholds ReadThresholds(const ros::NodeHandle& node) {
    GeometryThresholds thresholds;
    int min_points = 11;
    node.param("geometry/min_points", min_points, min_points);
    if (min_points <= 0) throw std::runtime_error("geometry/min_points must be positive");
    thresholds.min_points = static_cast<std::uint32_t>(min_points);
    node.param("geometry/min_singular_value", thresholds.min_singular_value, thresholds.min_singular_value);
    node.param("geometry/min_mean_singular_value", thresholds.min_mean_singular_value,
               thresholds.min_mean_singular_value);
    return thresholds;
}

struct PendingFrame {
    lio::LioFrame::ConstPtr message;
    ros::WallTime arrival;
};
struct PendingCloud {
    sensor_msgs::PointCloud2::ConstPtr message;
    ros::WallTime arrival;
};
struct CloudTransform {
    ros::Time stamp;
    Pose world_to_fused;
    Pose world_to_body;
};

} // namespace

class FusionNode {
public:
    FusionNode() : private_node_("~"), fusion_(ReadThresholds(private_node_)),
                   vio_history_(2048), frames_(128), clouds_(128), transforms_(2048) {
        private_node_.param("enable_vio", enable_vio_, true);
        private_node_.param("max_vio_gap", max_vio_gap_, 0.15);
        private_node_.param("max_wait", max_wait_, 0.2);
        private_node_.param<std::string>("world_frame", world_frame_, "fusion_world");
        private_node_.param<std::string>("body_frame", body_frame_, "fusion_imu");
        if (!std::isfinite(max_vio_gap_) || max_vio_gap_ <= 0.0 || !std::isfinite(max_wait_) ||
            max_wait_ <= 0.0 || world_frame_.empty() || body_frame_.empty() || world_frame_ == body_frame_) {
            throw std::runtime_error("invalid fusion timing or frame configuration");
        }
        if (enable_vio_) vio_to_lio_ = ReadExtrinsic(private_node_, "imu_lio_T_imu_vio").Inverse();

        odometry_pub_ = node_.advertise<nav_msgs::Odometry>("/fusion/odometry", 20);
        status_pub_ = node_.advertise<elevator_fusion::FusionStatus>("/fusion/status", 20);
        world_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/fusion/cloud_world", 4);
        body_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/fusion/cloud_body", 4);
        frame_sub_ = node_.subscribe("/LIO/frame", 128, &FusionNode::OnFrame, this,
                                     ros::TransportHints().tcpNoDelay());
        cloud_sub_ = node_.subscribe("/LIO/clouds_lidar", 16, &FusionNode::OnCloud, this,
                                     ros::TransportHints().tcpNoDelay());
        if (enable_vio_) {
            vio_sub_ = node_.subscribe("/vins_estimator/odometry", 512, &FusionNode::OnVio, this,
                                       ros::TransportHints().tcpNoDelay());
            epoch_sub_ = node_.subscribe("/vins_estimator/restart_epoch", 4, &FusionNode::OnEpoch, this);
        }
        timer_ = node_.createWallTimer(ros::WallDuration(0.01), &FusionNode::OnTimer, this);
    }

private:
    void OnFrame(const lio::LioFrame::ConstPtr& message) {
        if (have_frame_ && message->header.stamp <= latest_frame_) {
            ROS_WARN_THROTTLE(2.0, "Dropping non-monotonic LIO frame; restart the pipeline after bag rewind");
            return;
        }
        if (message->header.frame_id != "world") {
            ROS_ERROR_THROTTLE(2.0, "LIO scan frame must use world coordinates");
            return;
        }
        latest_frame_ = message->header.stamp;
        have_frame_ = true;
        if (frames_.Full()) ROS_WARN("Fusion scan queue overflow: dropping oldest scan");
        frames_.Push({message, ros::WallTime::now()});
        Drain();
    }

    void OnVio(const nav_msgs::Odometry::ConstPtr& message) {
        const Pose input = FromMessage(message->pose.pose);
        if (!input.IsValid()) {
            ROS_WARN_THROTTLE(2.0, "Dropping malformed VIO pose");
            return;
        }
        const Pose pose = input * vio_to_lio_;
        if (!vio_history_.Push({message->header.stamp.toSec(), pose})) {
            ROS_WARN_THROTTLE(2.0, "Dropping invalid or non-monotonic VIO pose");
            return;
        }
        Drain();
    }

    void OnEpoch(const std_msgs::UInt32::ConstPtr& message) {
        if (!vio_epoch_ || *vio_epoch_ != message->data) {
            vio_epoch_ = message->data;
            vio_history_.Clear();
            fusion_.InvalidateVio();
        }
    }

    void OnCloud(const sensor_msgs::PointCloud2::ConstPtr& message) {
        if (message->header.frame_id != "world") {
            ROS_ERROR_THROTTLE(2.0, "Input LIO cloud must be expressed in world");
            return;
        }
        if (clouds_.Full()) ROS_WARN("Fusion cloud queue overflow: dropping oldest cloud");
        clouds_.Push({message, ros::WallTime::now()});
        DrainClouds();
    }

    void OnTimer(const ros::WallTimerEvent&) { Drain(); }

    void Drain() {
        const auto now = ros::WallTime::now();
        while (!frames_.Empty()) {
            const auto& pending = frames_.Front();
            const auto& message = *pending.message;
            const double stamp = message.header.stamp.toSec();
            const auto vio = enable_vio_ ? vio_history_.Interpolate(stamp, max_vio_gap_) : std::nullopt;
            if (enable_vio_ && !vio && (now - pending.arrival).toSec() < max_wait_) break;
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
            } else ROS_ERROR_THROTTLE(2.0, "Rejected invalid LIO scan pose");
            frames_.Pop();
        }
        DrainClouds();
    }

    void Publish(const FusionResult& result, const lio::LioFrame& input) {
        nav_msgs::Odometry odometry;
        odometry.header.stamp = input.header.stamp;
        odometry.header.frame_id = world_frame_;
        odometry.child_frame_id = body_frame_;
        odometry.pose.pose = ToMessage(result.imu_pose);
        // This is a pose-selection output, not an independent covariance/velocity estimator.
        odometry_pub_.publish(odometry);
        geometry_msgs::TransformStamped transform;
        transform.header = odometry.header;
        transform.child_frame_id = body_frame_;
        transform.transform.translation.x = result.imu_pose.translation.x();
        transform.transform.translation.y = result.imu_pose.translation.y();
        transform.transform.translation.z = result.imu_pose.translation.z();
        transform.transform.rotation = odometry.pose.pose.orientation;
        broadcaster_.sendTransform(transform);
        auto& status = status_message_;
        status.header = odometry.header;
        status.source = result.source == Source::kLio ? status.SOURCE_LIO : status.SOURCE_VIO;
        status.reason = ReasonName(result.reason);
        status.degenerate = result.geometry.degenerate;
        for (int i = 0; i < 3; ++i) status.normal_singular_values[i] = result.geometry.singular_values(i);
        status.effective_points = input.effective_points;
        status.in_elevator = input.in_elevator;
        status_pub_.publish(status);
    }

    const CloudTransform* FindTransform(const ros::Time& stamp) const {
        std::size_t low = 0;
        std::size_t high = transforms_.Size();
        while (low < high) {
            const auto middle = low + (high - low) / 2;
            if (transforms_.At(middle).stamp < stamp) low = middle + 1;
            else high = middle;
        }
        return low < transforms_.Size() && transforms_.At(low).stamp == stamp ? &transforms_.At(low) : nullptr;
    }

    void DrainClouds() {
        const auto now = ros::WallTime::now();
        // Clouds may arrive before their scan frame, and may be decimated independently.
        const auto count = clouds_.Size();
        for (std::size_t index = 0; index < count; ++index) {
            auto pending = std::move(clouds_.Front());
            clouds_.Pop();
            const auto* transform = FindTransform(pending.message->header.stamp);
            if (transform) {
                auto& output = cloud_message_;
                if (world_cloud_pub_.getNumSubscribers()) {
                    pcl_ros::transformPointCloud(MakeTransformMatrix(transform->world_to_fused), *pending.message, output);
                    output.header = pending.message->header;
                    output.header.frame_id = world_frame_;
                    world_cloud_pub_.publish(output);
                }
                if (body_cloud_pub_.getNumSubscribers()) {
                    pcl_ros::transformPointCloud(MakeTransformMatrix(transform->world_to_body), *pending.message, output);
                    output.header = pending.message->header;
                    output.header.frame_id = body_frame_;
                    body_cloud_pub_.publish(output);
                }
            } else if ((now - pending.arrival).toSec() < max_wait_ * 2.0) {
                clouds_.Push(std::move(pending));
            } else ROS_WARN_THROTTLE(2.0, "Dropping cloud without an exact scan-end pose; no nearest-pose substitution");
        }
    }

    ros::NodeHandle node_;
    ros::NodeHandle private_node_;
    TrajectoryFusion fusion_;
    PoseHistory vio_history_;
    Ring<PendingFrame> frames_;
    Ring<PendingCloud> clouds_;
    Ring<CloudTransform> transforms_;
    Pose vio_to_lio_;
    bool enable_vio_ = true;
    bool have_frame_ = false;
    ros::Time latest_frame_;
    std::optional<std::uint32_t> vio_epoch_;
    double max_vio_gap_ = 0.15;
    double max_wait_ = 0.2;
    std::string world_frame_;
    std::string body_frame_;
    sensor_msgs::PointCloud2 cloud_message_;
    elevator_fusion::FusionStatus status_message_;
    ros::Publisher odometry_pub_, status_pub_, world_cloud_pub_, body_cloud_pub_;
    ros::Subscriber frame_sub_, cloud_sub_, vio_sub_, epoch_sub_;
    ros::WallTimer timer_;
    tf2_ros::TransformBroadcaster broadcaster_;
};

} // namespace elevator_fusion

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_fusion");
    try {
        elevator_fusion::FusionNode node;
        ros::spin();
    } catch (const std::exception& error) {
        ROS_FATAL("Fusion startup failed: %s", error.what());
        return 1;
    }
    return 0;
}
