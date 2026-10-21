/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, HKUST
 * This file is part of VINS, licensed under GPLv3.
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/
#include "global_optimization.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <stdexcept>
#include <string>

class GlobalFusionNode {
public:
    GlobalFusionNode() : parameters_("~") {
        parameters_.param<std::string>("global_frame", global_frame_, "global_world");
        parameters_.param("position_sigma", position_sigma_, 0.1);
        if (!std::isfinite(position_sigma_) || position_sigma_ <= 0.0 || global_frame_.empty()) {
            throw std::runtime_error("invalid global position reference configuration");
        }
        odometry_pub_ = parameters_.advertise<nav_msgs::Odometry>("global_odometry", 20);
        path_pub_ = parameters_.advertise<nav_msgs::Path>("global_path", 2);
        odometry_sub_ = node_.subscribe("/fusion/odometry", 100, &GlobalFusionNode::OnOdometry, this);
        anchor_sub_ = node_.subscribe("/tag_Odometry", 100, &GlobalFusionNode::OnAnchor, this);
    }
private:
    void OnOdometry(const nav_msgs::Odometry::ConstPtr& input) {
        const auto& p = input->pose.pose.position;
        const auto& q = input->pose.pose.orientation;
        if (!optimizer_.InputOdom(input->header.stamp.toSec(), Eigen::Vector3d(p.x, p.y, p.z),
                                 Eigen::Quaterniond(q.w, q.x, q.y, q.z))) {
            ROS_WARN_THROTTLE(2.0, "Rejected invalid/non-monotonic fusion odometry");
            return;
        }
        Publish();
    }
    void OnAnchor(const nav_msgs::Odometry::ConstPtr& input) {
        if (input->header.frame_id != global_frame_) {
            ROS_ERROR_THROTTLE(2.0, "Position reference must be expressed in configured global_frame");
            return;
        }
        const auto& p = input->pose.pose.position;
        if (!optimizer_.InputAprilTag(input->header.stamp.toSec(), p.x, p.y, p.z, position_sigma_)) {
            ROS_WARN_THROTTLE(2.0, "Rejected invalid absolute position reference");
            return;
        }
        Publish();
    }
    void Publish() {
        const auto& poses = optimizer_.Trajectory();
        if (poses.empty()) return;
        path_.header.frame_id = global_frame_;
        path_.header.stamp.fromSec(poses.rbegin()->first);
        path_.poses.resize(poses.size());
        std::size_t index = 0;
        for (const auto& item : poses) {
            auto& pose = path_.poses[index++];
            pose.header.frame_id = global_frame_;
            pose.header.stamp.fromSec(item.first);
            pose.pose.position.x = item.second.p[0];
            pose.pose.position.y = item.second.p[1];
            pose.pose.position.z = item.second.p[2];
            pose.pose.orientation.w = item.second.q[0];
            pose.pose.orientation.x = item.second.q[1];
            pose.pose.orientation.y = item.second.q[2];
            pose.pose.orientation.z = item.second.q[3];
        }
        nav_msgs::Odometry odometry;
        odometry.header = path_.header;
        odometry.child_frame_id = "global_imu";
        odometry.pose.pose = path_.poses.back().pose;
        odometry_pub_.publish(odometry);
        if (path_pub_.getNumSubscribers()) path_pub_.publish(path_);
    }
    ros::NodeHandle node_, parameters_;
    GlobalOptimization optimizer_;
    nav_msgs::Path path_;
    std::string global_frame_;
    double position_sigma_ = 0.1;
    ros::Publisher odometry_pub_, path_pub_;
    ros::Subscriber odometry_sub_, anchor_sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "global_fusion");
    try {
        GlobalFusionNode node;
        ros::spin();
    } catch (const std::exception& error) {
        ROS_FATAL("Global fusion failed: %s", error.what());
        return 1;
    }
    return 0;
}
