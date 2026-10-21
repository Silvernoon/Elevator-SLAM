// SPDX-License-Identifier: GPL-3.0-only
#include "global_optimization.h"
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
void ExpectNear(double actual, double expected, const char* message) {
    Require(std::abs(actual - expected) < 1e-4, message);
}
}
int main() {
    try {
        GlobalOptimization graph;
        const auto q = Eigen::Quaterniond::Identity();
        Require(graph.InputOdom(0, Eigen::Vector3d(0, 0, 0), q), "first pose");
        Require(graph.InputOdom(1, Eigen::Vector3d(1, 0, 0), q), "second pose");
        // The position source is asynchronous, not keyed to an exact odometry stamp.
        Require(graph.InputAprilTag(0.5, 10.5, 0, 0, 0.1), "intermediate position reference");
        const auto& poses = graph.Trajectory();
        ExpectNear(poses.at(0).p[0], 10, "asynchronous reference must translate the first pose");
        ExpectNear(poses.at(1).p[0], 11, "relative motion must be preserved under absolute alignment");
        Require(!graph.InputOdom(1, Eigen::Vector3d(500, 0, 0), q), "duplicate pose rejected");
        Require(!graph.InputAprilTag(0.75, 0, 0, 0, 0), "zero uncertainty rejected");
        Require(graph.InputAprilTag(1.5, 11.5, 0, 0, 0.1), "future anchor queued");
        Require(graph.InputOdom(2, Eigen::Vector3d(2, 0, 0), q), "future anchor obtains a bracket");
        ExpectNear(poses.at(2).p[0], 12, "new poses inherit the optimized global transform");
        ExpectNear(poses.at(2).p[0] - poses.at(1).p[0], 1, "position anchors cannot distort consistent motion");
        std::cout << "asynchronous anchors, relative motion, future observations, invalid inputs: passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
