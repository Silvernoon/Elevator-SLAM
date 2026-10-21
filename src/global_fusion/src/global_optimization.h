/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, HKUST
 * This file is part of VINS, licensed under GPLv3.
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/
#pragma once
#include <Eigen/Geometry>
#include <array>
#include <map>

// Optional absolute-position pose graph. Called from one transport executor;
// no detached worker, ROS messages, or shared mutable public path.
class GlobalOptimization {
public:
    struct Pose {
        std::array<double, 3> p;
        std::array<double, 4> q; // w, x, y, z
    };
    bool InputOdom(double stamp, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);
    bool InputAprilTag(double stamp, double x, double y, double z, double sigma = 0.1);
    const std::map<double, Pose>& Trajectory() const { return global_poses_; }

private:
    struct Anchor {
        Eigen::Vector3d position;
        double sigma;
    };
    void Optimize();
    std::map<double, Pose> local_poses_;
    std::map<double, Pose> global_poses_;
    std::map<double, Anchor> anchors_;
    Eigen::Isometry3d global_t_local_ = Eigen::Isometry3d::Identity();
    bool pending_anchors_ = false;
};
