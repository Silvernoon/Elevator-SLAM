// SPDX-License-Identifier: GPL-3.0-or-later
#include "elevator_fusion/core.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace elevator_fusion;
namespace {
void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
void ExpectPoseNear(const Pose& actual, const Pose& expected, const char* message) {
    Require((actual.translation - expected.translation).norm() < 1e-9 &&
            actual.rotation.angularDistance(expected.rotation) < 1e-9, message);
}
Pose MakePose(double x, double y, double z, double yaw) {
    return {Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())),
            Eigen::Vector3d(x, y, z)};
}
LioFrame MakeFrame(double stamp, const Pose& p, bool healthy = true) {
    LioFrame result;
    result.stamp = stamp;
    result.imu_pose = p;
    result.cloud_imu_pose = p;
    result.geometry.effective_points = 300;
    result.geometry.lidar_updated = true;
    result.geometry.normal_information = Eigen::Matrix3d::Identity() * 100.0;
    if (!healthy) result.geometry.normal_information(0, 0) = 0.0;
    return result;
}
void TestGeometry() {
    auto input = MakeFrame(1.0, Pose{}).geometry;
    auto result = EvaluateGeometry(input, {});
    Require(!result.degenerate, "isotropic normals should constrain translation");
    input.normal_information.diagonal() << 0.0, 150.0, 150.0;
    Require(EvaluateGeometry(input, {}).degenerate, "same trace must not hide rank deficiency");
    input.normal_information = Eigen::Matrix3d::Identity() * 100.0;
    input.effective_points = 10;
    Require(EvaluateGeometry(input, {}).degenerate, "too few matched normals");
    input.effective_points = 300;
    input.lidar_updated = false;
    Require(EvaluateGeometry(input, {}).degenerate, "stale geometry is not a LiDAR update");
    input.lidar_updated = true;
    input.normal_information(0, 0) = -1.0;
    Require(EvaluateGeometry(input, {}).degenerate, "non-positive information must be rejected");
}
void TestInterpolation() {
    PoseHistory history(3);
    const auto a = MakePose(1, 2, 0, 0);
    const auto b = MakePose(3, 4, 0, 1.0);
    Require(history.Push({1.0, a}) && history.Push({2.0, b}), "valid history insert");
    ExpectPoseNear(*history.Interpolate(1.5, 1.1), MakePose(2, 3, 0, 0.5), "timestamp interpolation");
    Require(!history.Interpolate(0.9, 1.1) && !history.Interpolate(2.1, 1.1), "no extrapolation");
    Require(!history.Interpolate(1.5, 0.5), "do not bridge a stale VIO gap");
    Require(!history.Push({2.0, a}) && !history.Push({1.2, a}), "duplicate/out-of-order poses rejected");
    Require(history.Push({3.0, a}) && history.Push({4.0, b}), "history wrap");
    Require(!history.Interpolate(1.0, 1.1), "oldest sample evicted");
    ExpectPoseNear(*history.Interpolate(3.5, 1.1), MakePose(2, 3, 0, 0.5), "interpolation across ring wrap");
    history.Clear();
    Require(!history.Interpolate(3.5, 1.1), "epoch clear discards all old poses");
}
void TestSwitching() {
    TrajectoryFusion fusion;
    const Pose visual_world{Eigen::Quaterniond(Eigen::AngleAxisd(-0.7, Eigen::Vector3d::UnitY())),
                            Eigen::Vector3d(-12, 7, 1)};
    const Pose lio_drift{Eigen::Quaterniond(Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitX())),
                         Eigen::Vector3d(4, -8, 0.2)};
    const Pose p0 = MakePose(1, 2, 0, 0.5);
    const Pose p1 = MakePose(2, 3, 0, 0.8);
    const Pose p2 = MakePose(3, 3.2, 0, 1.0);
    const Pose p3 = MakePose(3.4, 4, 0, 1.2);
    ExpectPoseNear(fusion.Update(MakeFrame(0, p0), visual_world * p0)->imu_pose, p0, "initial world anchor");
    ExpectPoseNear(fusion.Update(MakeFrame(1, p1), visual_world * p1)->imu_pose, p1, "healthy LIO motion");
    auto switched = fusion.Update(MakeFrame(2, lio_drift * p2, false), visual_world * p2);
    Require(switched->source == Source::kVio, "degenerate LIO should select synchronized VIO");
    ExpectPoseNear(switched->imu_pose, p2, "switch must use a rigid-body increment, not mixed world offsets");
    auto recovered = fusion.Update(MakeFrame(3, lio_drift * p3), visual_world * p3);
    Require(recovered->source == Source::kLio, "recovery should select LIO");
    ExpectPoseNear(recovered->imu_pose, p3, "LIO recovery must retain fused-world continuity");
}
void TestElevator() {
    TrajectoryFusion fusion;
    auto first = MakeFrame(0, Pose{});
    fusion.Update(first, Pose{});
    auto inside = MakeFrame(1, MakePose(0, 0, 5, 0), false);
    inside.cloud_imu_pose = Pose{};
    inside.in_elevator = true;
    auto result = fusion.Update(inside, MakePose(0, 0, 0, 0));
    Require(result->source == Source::kLio && result->reason == Reason::kElevatorLio,
            "cab geometry must not override specialized Elevator-LIO");
    ExpectPoseNear(result->imu_pose, MakePose(0, 0, 5, 0), "elevator displacement retained");
    const Eigen::Vector3d point = result->cloud_world_to_fused_world * Eigen::Vector3d(1, 2, 1);
    Require((point - Eigen::Vector3d(1, 2, 6)).norm() < 1e-9, "cloud needs the elevator displacement exactly once");
    auto outside = MakeFrame(2, MakePose(0, 0, 5, 0));
    result = fusion.Update(outside, Pose{});
    ExpectPoseNear(result->imu_pose, outside.imu_pose, "committed elevator z must not be added twice");
    ExpectPoseNear(result->cloud_world_to_fused_world, Pose{}, "committed cloud already carries world displacement");
}
void TestOutageAndReset() {
    TrajectoryFusion fusion;
    fusion.Update(MakeFrame(0, Pose{}), Pose{});
    auto result = fusion.Update(MakeFrame(1, MakePose(1, 0, 0, 0), false), std::nullopt);
    Require(result->reason == Reason::kVioUnavailable && result->source == Source::kLio,
            "VIO outage must be explicit, not reuse a stale pose");
    ExpectPoseNear(result->imu_pose, MakePose(1, 0, 0, 0), "outage advances using available LIO increment");
    result = fusion.Update(MakeFrame(2, MakePose(2, 0, 0, 0), false), MakePose(100, 0, 0, 0));
    Require(result->reason == Reason::kVioUnprimed, "reacquired VIO needs a previous synchronized pose");
    result = fusion.Update(MakeFrame(3, MakePose(3, 0, 0, 0), false), MakePose(101, 0, 0, 0));
    ExpectPoseNear(result->imu_pose, MakePose(3, 0, 0, 0), "VIO arbitrary world origin must cancel");
    fusion.InvalidateVio();
    result = fusion.Update(MakeFrame(4, MakePose(4, 0, 0, 0), false), MakePose(-40, 0, 0, 0));
    Require(result->reason == Reason::kVioUnprimed, "reset epoch breaks the VIO delta chain");
    ExpectPoseNear(result->imu_pose, MakePose(4, 0, 0, 0), "VIO reset must not jump fused output");
    result = fusion.Update(MakeFrame(5, MakePose(5, 0, 0, 0), false), MakePose(-39, 0, 0, 0));
    ExpectPoseNear(result->imu_pose, MakePose(5, 0, 0, 0), "VIO resumes in new epoch");
    Require(!fusion.Update(MakeFrame(5, MakePose(900, 0, 0, 0)), Pose{}), "duplicate scan must not mutate fusion");
    result = fusion.Update(MakeFrame(6, MakePose(6, 0, 0, 0), false), MakePose(-38, 0, 0, 0));
    ExpectPoseNear(result->imu_pose, MakePose(6, 0, 0, 0), "rejected scan cannot poison the next increment");
}
} // namespace
int main() {
    try {
        TestGeometry();
        TestInterpolation();
        TestSwitching();
        TestElevator();
        TestOutageAndReset();
        std::cout << "geometry, interpolation, rigid switching, elevator clouds, VIO outage/reset: passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
