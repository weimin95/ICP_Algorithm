#pragma once

#include <Eigen/Core>

#include <string>

namespace fricp {

enum class RegistrationMethod {
    ICP,
    AAICP,
    FastICP,
    RobustICP,
    PointToPlane,
    RobustPointToPlane,
    SparseICP,
    SparsePointToPlane
};

struct RegistrationOptions {
    RegistrationMethod method = RegistrationMethod::RobustICP;
    // Wrapper-only validation/result evaluation for upstream methods.
    double max_correspondence_distance = 0.05;

    bool use_initial_transform = false;
    Eigen::Matrix4d initial_transform = Eigen::Matrix4d::Identity();

    double stop = 1e-5;
    int max_icp = 100;
    int max_outer = 1;
    double p = 0.1;
    int anderson_m = 5;
    double beta = 1.0;
    double error_overflow_threshold = 0.05;
    double nu_begin_k = 3.0;
    double nu_end_k = 0.19245008972987526;
    double nu_alpha = 0.5;

    bool sicp_use_penalty = false;
    double sicp_mu = 10.0;
    double sicp_alpha = 1.2;
    double sicp_max_mu = 1e5;
    int sicp_max_icp = 100;
    int sicp_max_outer = 100;
    int sicp_max_inner = 1;
    double sicp_p = 0.4;

    bool estimate_target_normals_if_missing = true;
    double normal_radius = 0.1;
    int normal_knn = 30;
};

struct RegistrationResult {
    bool success = false;
    RegistrationMethod method = RegistrationMethod::RobustICP;
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    double fitness = 0.0;
    double inlier_rmse = 0.0;
    double convergence_energy = 0.0;
    int iteration_count = 0;
    std::string message;
};

}  // namespace fricp
