#pragma once

#include <Eigen/Core>

#include <string>

namespace fricp {

enum class RegistrationMode {
    PointToPoint,
    PointToPlane,
    RobustPointToPoint
};

struct RegistrationOptions {
    RegistrationMode mode = RegistrationMode::RobustPointToPoint;
    double max_correspondence_distance = 0.05;
    int max_iteration = 50;
    double relative_fitness = 1e-6;
    double relative_rmse = 1e-6;
    bool use_initial_transform = false;
    Eigen::Matrix4d initial_transform = Eigen::Matrix4d::Identity();
    bool estimate_target_normals_if_missing = true;
    double normal_radius = 0.1;
    int normal_knn = 30;
    bool robust_use_anderson = true;
    double robust_nu_begin_k = 3.0;
    double robust_nu_end_k = 1.0 / 6.0;
    double robust_nu_alpha = 0.5;
    double robust_stop = 1e-5;
};

struct RegistrationResult {
    bool success = false;
    RegistrationMode mode = RegistrationMode::RobustPointToPoint;
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    double fitness = 0.0;
    double inlier_rmse = 0.0;
    double convergence_energy = 0.0;
    int iteration_count = 0;
    std::string message;
};

}  // namespace fricp
