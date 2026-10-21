// SPDX-License-Identifier: GPL-3.0-or-later
#include "elevator_fusion/core.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace elevator_fusion {

Pose Pose::Inverse() const {
    const auto inverse_rotation = rotation.conjugate();
    return {inverse_rotation, inverse_rotation * -translation};
}

Pose Pose::operator*(const Pose& child) const {
    return {(rotation * child.rotation).normalized(), translation + rotation * child.translation};
}

Eigen::Vector3d Pose::operator*(const Eigen::Vector3d& point) const {
    return translation + rotation * point;
}

bool Pose::IsValid() const {
    return translation.allFinite() && rotation.coeffs().allFinite() &&
           std::abs(rotation.squaredNorm() - 1.0) < 1e-6;
}

PoseHistory::PoseHistory(std::size_t capacity) : samples_(capacity) {
    if (capacity < 2) throw std::invalid_argument("pose history capacity must be at least two");
}

const TimedPose& PoseHistory::At(std::size_t index) const {
    return samples_[(begin_ + index) % samples_.size()];
}

bool PoseHistory::Push(const TimedPose& sample) {
    if (!std::isfinite(sample.stamp) || !sample.pose.IsValid() ||
        (size_ && sample.stamp <= At(size_ - 1).stamp)) return false;
    if (size_ == samples_.size()) {
        samples_[begin_] = sample;
        begin_ = (begin_ + 1) % samples_.size();
    } else {
        samples_[(begin_ + size_) % samples_.size()] = sample;
        ++size_;
    }
    return true;
}

std::optional<Pose> PoseHistory::Interpolate(double stamp, double max_gap) const {
    if (!size_ || !std::isfinite(stamp) || !std::isfinite(max_gap) || max_gap <= 0.0 ||
        stamp < At(0).stamp || stamp > At(size_ - 1).stamp) return std::nullopt;
    std::size_t low = 0;
    std::size_t high = size_;
    while (low < high) {
        const std::size_t middle = low + (high - low) / 2;
        if (At(middle).stamp < stamp) low = middle + 1;
        else high = middle;
    }
    const auto& after = At(low);
    if (after.stamp == stamp) return after.pose;
    const auto& before = At(low - 1);
    const double gap = after.stamp - before.stamp;
    if (gap > max_gap) return std::nullopt;
    const double alpha = (stamp - before.stamp) / gap;
    return Pose{before.pose.rotation.slerp(alpha, after.pose.rotation).normalized(),
                (1.0 - alpha) * before.pose.translation + alpha * after.pose.translation};
}

void PoseHistory::Clear() {
    begin_ = 0;
    size_ = 0;
}

GeometryResult EvaluateGeometry(const Geometry& geometry, const GeometryThresholds& thresholds) {
    GeometryResult result;
    if (!geometry.lidar_updated || geometry.effective_points < thresholds.min_points ||
        !geometry.normal_information.allFinite()) return result;
    const auto& information = geometry.normal_information;
    if (!information.isApprox(information.transpose(), 1e-8)) return result;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(information, Eigen::EigenvaluesOnly);
    if (solver.info() != Eigen::Success || solver.eigenvalues().minCoeff() < -1e-8) return result;
    result.singular_values = solver.eigenvalues().cwiseMax(0.0).cwiseSqrt();
    result.degenerate = result.singular_values(0) < thresholds.min_singular_value ||
                        result.singular_values.mean() < thresholds.min_mean_singular_value;
    return result;
}

const char* SourceName(Source source) {
    return source == Source::kLio ? "lio" : "vio";
}

const char* ReasonName(Reason reason) {
    switch (reason) {
    case Reason::kHealthyLio: return "healthy_lio";
    case Reason::kElevatorLio: return "elevator_lio";
    case Reason::kGeometryFallback: return "geometry_fallback";
    case Reason::kVioUnavailable: return "vio_unavailable";
    case Reason::kVioUnprimed: return "vio_unprimed";
    }
    return "unknown";
}

TrajectoryFusion::TrajectoryFusion(GeometryThresholds thresholds) : thresholds_(thresholds) {
    if (!thresholds.min_points || !std::isfinite(thresholds.min_singular_value) ||
        !std::isfinite(thresholds.min_mean_singular_value) || thresholds.min_singular_value < 0.0 ||
        thresholds.min_mean_singular_value < 0.0) throw std::invalid_argument("invalid geometry thresholds");
}

std::optional<FusionResult> TrajectoryFusion::Update(const LioFrame& frame,
                                                    const std::optional<Pose>& vio_imu_pose) {
    if (!std::isfinite(frame.stamp) || !frame.imu_pose.IsValid() || !frame.cloud_imu_pose.IsValid() ||
        (vio_imu_pose && !vio_imu_pose->IsValid()) ||
        (previous_lio_ && frame.stamp <= previous_lio_->stamp)) return std::nullopt;

    FusionResult result;
    result.stamp = frame.stamp;
    result.geometry = EvaluateGeometry(frame.geometry, thresholds_);
    if (frame.in_elevator) result.reason = Reason::kElevatorLio;
    else if (result.geometry.degenerate) {
        if (!vio_imu_pose) result.reason = Reason::kVioUnavailable;
        else if (!previous_vio_) result.reason = Reason::kVioUnprimed;
        else {
            result.source = Source::kVio;
            result.reason = Reason::kGeometryFallback;
        }
    }

    if (!previous_lio_) fused_pose_ = frame.imu_pose;
    else if (result.source == Source::kVio) {
        fused_pose_ = fused_pose_ * previous_vio_->Inverse() * *vio_imu_pose;
    } else {
        fused_pose_ = fused_pose_ * previous_lio_->pose.Inverse() * frame.imu_pose;
    }
    result.imu_pose = fused_pose_;
    result.cloud_world_to_fused_world = fused_pose_ * frame.cloud_imu_pose.Inverse();
    previous_lio_ = TimedPose{frame.stamp, frame.imu_pose};
    previous_vio_ = vio_imu_pose;
    return result;
}

void TrajectoryFusion::InvalidateVio() {
    previous_vio_.reset();
}

void TrajectoryFusion::Reset() {
    previous_lio_.reset();
    previous_vio_.reset();
    fused_pose_ = Pose{};
}

} // namespace elevator_fusion
