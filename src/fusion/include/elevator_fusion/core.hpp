// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace elevator_fusion {

// T_parent_child: transforms coordinates from child to parent.
struct Pose {
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();

    Pose Inverse() const;
    Pose operator*(const Pose& child) const;
    Eigen::Vector3d operator*(const Eigen::Vector3d& point) const;
    bool IsValid() const;
};

struct TimedPose {
    double stamp = 0.0;
    Pose pose;
};

// Bounded, allocation-free after construction. No extrapolation or timestamp substitution.
class PoseHistory {
public:
    explicit PoseHistory(std::size_t capacity);
    bool Push(const TimedPose& sample);
    std::optional<Pose> Interpolate(double stamp, double max_gap) const;
    void Clear();

private:
    const TimedPose& At(std::size_t index) const;
    std::vector<TimedPose> samples_;
    std::size_t begin_ = 0;
    std::size_t size_ = 0;
};

struct Geometry {
    Eigen::Matrix3d normal_information = Eigen::Matrix3d::Zero();
    std::uint32_t effective_points = 0;
    bool lidar_updated = false;
};

struct GeometryThresholds {
    std::uint32_t min_points = 11;
    double min_singular_value = 7.0;
    double min_mean_singular_value = 10.0;
};

struct GeometryResult {
    bool degenerate = true;
    Eigen::Vector3d singular_values = Eigen::Vector3d::Zero(); // ascending
};

GeometryResult EvaluateGeometry(const Geometry& geometry, const GeometryThresholds& thresholds);

struct LioFrame {
    double stamp = 0.0;
    Pose imu_pose;       // Global displacement, including the elevator substate.
    Pose cloud_imu_pose; // Pose used to produce the input world cloud.
    Geometry geometry;
    bool in_elevator = false;
};

enum class Source { kLio, kVio };
enum class Reason { kHealthyLio, kElevatorLio, kGeometryFallback, kVioUnavailable, kVioUnprimed };
const char* SourceName(Source source);
const char* ReasonName(Reason reason);

struct FusionResult {
    double stamp = 0.0;
    Pose imu_pose;
    Pose cloud_world_to_fused_world;
    Source source = Source::kLio;
    Reason reason = Reason::kHealthyLio;
    GeometryResult geometry;
};

// Estimator-independent trajectory switching. Both source poses must describe the
// same rigid body at the scan timestamp; their world origins may be unrelated.
class TrajectoryFusion {
public:
    explicit TrajectoryFusion(GeometryThresholds thresholds = {});
    std::optional<FusionResult> Update(const LioFrame& frame, const std::optional<Pose>& vio_imu_pose);
    void Reset();
    void InvalidateVio();

private:
    GeometryThresholds thresholds_;
    std::optional<TimedPose> previous_lio_;
    std::optional<Pose> previous_vio_;
    Pose fused_pose_;
};

} // namespace elevator_fusion
