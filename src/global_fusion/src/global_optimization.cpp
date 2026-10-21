/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, HKUST
 * This file is part of VINS, licensed under GPLv3.
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/
#include "global_optimization.h"
#include "Factors.h"
#include <cmath>
#include <iterator>

namespace {
Eigen::Isometry3d ToTransform(const GlobalOptimization::Pose& pose) {
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.linear() = Eigen::Quaterniond(pose.q[0], pose.q[1], pose.q[2], pose.q[3]).toRotationMatrix();
    result.translation() = Eigen::Vector3d(pose.p[0], pose.p[1], pose.p[2]);
    return result;
}
GlobalOptimization::Pose ToParameters(const Eigen::Isometry3d& pose) {
    const Eigen::Quaterniond q(pose.linear());
    const auto& p = pose.translation();
    return {{p.x(), p.y(), p.z()}, {q.w(), q.x(), q.y(), q.z()}};
}
struct InterpolatedPositionError {
    Eigen::Vector3d position;
    double alpha;
    double sigma;
    template <typename T> bool operator()(const T* first, const T* second, T* residual) const {
        for (int i = 0; i < 3; ++i) {
            residual[i] = (T(1.0 - alpha) * first[i] + T(alpha) * second[i] - T(position(i))) / T(sigma);
        }
        return true;
    }
};
} // namespace

bool GlobalOptimization::InputOdom(double stamp, const Eigen::Vector3d& position,
                                   const Eigen::Quaterniond& orientation) {
    if (!std::isfinite(stamp) || !position.allFinite() || !orientation.coeffs().allFinite() ||
        std::abs(orientation.squaredNorm() - 1.0) > 1e-6 ||
        (!local_poses_.empty() && stamp <= local_poses_.rbegin()->first)) return false;
    const Pose local{{position.x(), position.y(), position.z()},
                     {orientation.w(), orientation.x(), orientation.y(), orientation.z()}};
    local_poses_.emplace(stamp, local);
    global_poses_.emplace(stamp, ToParameters(global_t_local_ * ToTransform(local)));
    if (pending_anchors_) Optimize();
    return true;
}

bool GlobalOptimization::InputAprilTag(double stamp, double x, double y, double z, double sigma) {
    const Eigen::Vector3d position(x, y, z);
    if (!std::isfinite(stamp) || !position.allFinite() || !std::isfinite(sigma) || sigma <= 0.0) return false;
    anchors_[stamp] = {position, sigma};
    pending_anchors_ = true;
    Optimize();
    return true;
}

void GlobalOptimization::Optimize() {
    if (global_poses_.size() < 2) return;
    // Solve on a workspace: unusable solutions must not replace the published trajectory.
    auto workspace = global_poses_;
    ceres::Problem problem;
    auto* quaternion_parameterization = new ceres::QuaternionParameterization;
    for (auto& item : workspace) {
        problem.AddParameterBlock(item.second.q.data(), 4, quaternion_parameterization);
        problem.AddParameterBlock(item.second.p.data(), 3);
    }
    auto previous = local_poses_.begin();
    for (auto current = std::next(previous); current != local_poses_.end(); ++current) {
        const Eigen::Isometry3d relative = ToTransform(previous->second).inverse() * ToTransform(current->second);
        const auto delta = ToParameters(relative);
        auto& first = workspace.at(previous->first);
        auto& second = workspace.at(current->first);
        problem.AddResidualBlock(RelativeRTError::Create(delta.p[0], delta.p[1], delta.p[2],
                                 delta.q[0], delta.q[1], delta.q[2], delta.q[3], 0.1, 0.01),
                                 nullptr, first.q.data(), first.p.data(), second.q.data(), second.p.data());
        previous = current;
    }
    bool anchored = false;
    bool future_anchor = false;
    for (const auto& item : anchors_) {
        auto after = workspace.lower_bound(item.first);
        if (after == workspace.end()) {
            future_anchor = true;
            continue;
        }
        const auto& anchor = item.second;
        auto* loss = new ceres::HuberLoss(1.0);
        if (after->first == item.first) {
            problem.AddResidualBlock(TError::Create(anchor.position.x(), anchor.position.y(), anchor.position.z(),
                                     anchor.sigma), loss, after->second.p.data());
            anchored = true;
        } else if (after != workspace.begin()) {
            auto before = std::prev(after);
            const double alpha = (item.first - before->first) / (after->first - before->first);
            auto* cost = new ceres::AutoDiffCostFunction<InterpolatedPositionError, 3, 3, 3>(
                new InterpolatedPositionError{anchor.position, alpha, anchor.sigma});
            problem.AddResidualBlock(cost, loss, before->second.p.data(), after->second.p.data());
            anchored = true;
        } else delete loss;
    }
    if (!anchored) return;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 5;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (!summary.IsSolutionUsable()) return;
    global_poses_.swap(workspace);
    global_t_local_ = ToTransform(global_poses_.rbegin()->second) * ToTransform(local_poses_.rbegin()->second).inverse();
    pending_anchors_ = future_anchor;
}
