#pragma once

#include <Eigen/Core>

#include <open3d/geometry/PointCloud.h>

#include <string>

namespace fricp::internal {

struct RobustOptions {
    int max_iteration = 50;
    double nu_begin_k = 3.0;
    double nu_end_k = 1.0 / 6.0;
    double nu_alpha = 0.5;
    double stop = 1e-5;
    bool use_anderson = true;
};

struct RobustResult {
    bool success = false;
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    double convergence_energy = 0.0;
    int iteration_count = 0;
    std::string message;
};

RobustResult RegisterRobustPointToPoint(
        const open3d::geometry::PointCloud& source,
        const open3d::geometry::PointCloud& target,
        const Eigen::Matrix4d& initial_transform,
        bool use_initial_transform,
        const RobustOptions& options);

}  // namespace fricp::internal
