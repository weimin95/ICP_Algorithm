#include <fricp/FastRobustIcp.h>

#include <FRICP.h>
#include "internal/Open3DAdapters.h"
#include <internal/UpstreamParameterMapping.h>
#include <fricp/internal/TrainedData.h>

#include <open3d/geometry/KDTreeSearchParam.h>
#include <open3d/pipelines/registration/Registration.h>

#include <memory>

namespace fricp {

namespace {

bool NeedsTargetNormals(RegistrationMethod method) {
    return method == RegistrationMethod::PointToPlane ||
           method == RegistrationMethod::RobustPointToPlane ||
           method == RegistrationMethod::SparsePointToPlane;
}

bool HasMatchingNormals(const open3d::geometry::PointCloud& cloud) {
    return cloud.normals_.empty() ||
           cloud.normals_.size() == cloud.points_.size();
}

internal::Matrix3X MakeSourceNormals(const open3d::geometry::PointCloud& cloud,
                                     Eigen::Index point_count) {
    if (cloud.normals_.size() == cloud.points_.size() && !cloud.normals_.empty()) {
        return internal::NormalsToMatrix(cloud);
    }
    return internal::Matrix3X::Zero(3, point_count);
}

Eigen::Matrix4d RescaleTranslation(Eigen::Matrix4d transformation,
                                   double scale) {
    transformation.block<3, 1>(0, 3) *= scale;
    return transformation;
}

void PopulateResult(const RegistrationOptions& options,
                    RegistrationMethod method,
                    const open3d::geometry::PointCloud& source,
                    const open3d::geometry::PointCloud& target,
                    const Eigen::Matrix4d& transformation,
                    double convergence_energy,
                    int iteration_count,
                    RegistrationResult& result) {
    result.success = true;
    result.method = method;
    result.transformation = transformation;
    result.fitness = 0.0;
    result.inlier_rmse = 0.0;
    if (options.compute_registration_metrics) {
        const auto evaluation =
                open3d::pipelines::registration::EvaluateRegistration(
                        source, target, options.max_correspondence_distance,
                        transformation);
        result.fitness = evaluation.fitness_;
        result.inlier_rmse = evaluation.inlier_rmse_;
    }
    result.convergence_energy = convergence_energy;
    result.iteration_count = iteration_count;
    result.message = "ok";
}

bool IsSupportedMethod(RegistrationMethod method) {
    switch (method) {
        case RegistrationMethod::ICP:
        case RegistrationMethod::AAICP:
        case RegistrationMethod::FastICP:
        case RegistrationMethod::RobustICP:
        case RegistrationMethod::PointToPlane:
        case RegistrationMethod::RobustPointToPlane:
        case RegistrationMethod::SparseICP:
        case RegistrationMethod::SparsePointToPlane:
            return true;
    }
    return false;
}

::ICP::Parameters MakeIcpParameters(const internal::IcpParameterView& view) {
    ::ICP::Parameters parameters;
    parameters.p = view.p;
    parameters.max_icp = view.max_icp;
    parameters.max_outer = view.max_outer;
    parameters.stop = view.stop;
    parameters.anderson_m = view.anderson_m;
    parameters.beta_ = view.beta;
    parameters.error_overflow_threshold_ = view.error_overflow_threshold;
    parameters.nu_begin_k = view.nu_begin_k;
    parameters.nu_end_k = view.nu_end_k;
    parameters.nu_alpha = view.nu_alpha;
    parameters.use_init = view.use_init;
    parameters.init_trans = view.init_trans;
    parameters.f = view.f == internal::IcpRobustFunction::Welsch
                           ? ::ICP::WELSCH
                           : ::ICP::NONE;
    parameters.use_AA = view.use_AA;
    return parameters;
}

::SICP::Parameters MakeSicpParameters(const internal::SicpParameterView& view) {
    ::SICP::Parameters parameters;
    parameters.use_penalty = view.use_penalty;
    parameters.p = view.p;
    parameters.mu = view.mu;
    parameters.alpha = view.alpha;
    parameters.max_mu = view.max_mu;
    parameters.max_icp = view.max_icp;
    parameters.max_outer = view.max_outer;
    parameters.max_inner = view.max_inner;
    parameters.stop = view.stop;
    parameters.init_trans = view.init_trans;
    return parameters;
}

}  // namespace

FastRobustIcp::FastRobustIcp() = default;

FastRobustIcp::~FastRobustIcp() = default;

FastRobustIcp::FastRobustIcp(FastRobustIcp&&) noexcept = default;

FastRobustIcp& FastRobustIcp::operator=(FastRobustIcp&&) noexcept = default;

bool FastRobustIcp::Train(const open3d::geometry::PointCloud& target,
                          const RegistrationOptions& options) {
    if (target.points_.empty()) {
        last_error_ = "target point cloud must not be empty";
        ClearTraining();
        return false;
    }

    if (NeedsTargetNormals(options.method) && target.normals_.empty() &&
        !options.estimate_target_normals_if_missing) {
        last_error_ =
                "target point cloud normals are required for point-to-plane mode";
        ClearTraining();
        return false;
    }

    if (NeedsTargetNormals(options.method) && !HasMatchingNormals(target)) {
        last_error_ =
                "target point cloud normals must be empty or match the point count";
        ClearTraining();
        return false;
    }

    auto trained_data = std::make_unique<internal::TrainedData>();
    trained_data->is_trained = true;
    trained_data->method = options.method;
    trained_data->options = options;
    trained_data->target = target;
    trained_data->target_matrix = internal::PointCloudToMatrix(target);
    trained_data->target_bbox =
            internal::ComputeBoundingBoxStatistics(trained_data->target_matrix);

    if (NeedsTargetNormals(options.method)) {
        trained_data->cached_target_normals = target;
        if (trained_data->cached_target_normals.normals_.empty()) {
            trained_data->cached_target_normals.EstimateNormals(
                    open3d::geometry::KDTreeSearchParamHybrid(
                            options.normal_radius, options.normal_knn));
        }
        if (trained_data->cached_target_normals.normals_.empty()) {
            last_error_ =
                    "target point cloud normals could not be estimated";
            ClearTraining();
            return false;
        }
        trained_data->target_normals_matrix =
                internal::NormalsToMatrix(trained_data->cached_target_normals);
    }

    trained_data_ = std::move(trained_data);
    last_error_.clear();
    return true;
}

void FastRobustIcp::ClearTraining() {
    trained_data_.reset();
}

bool FastRobustIcp::IsTrained() const {
    return trained_data_ && trained_data_->is_trained;
}

bool FastRobustIcp::Register(const open3d::geometry::PointCloud& source,
                             const RegistrationOptions& options,
                             RegistrationResult& result) const {
    result = RegistrationResult {};

    if (!IsTrained()) {
        last_error_ = "call Train(...) before Register(...)";
        result.message = last_error_;
        return false;
    }

    const auto& target = trained_data_->target;

    if (source.points_.empty() || target.points_.empty()) {
        last_error_ = "source and target point clouds must not be empty";
        result.message = last_error_;
        return false;
    }

    if (options.max_correspondence_distance <= 0.0) {
        last_error_ = "max_correspondence_distance must be > 0";
        result.message = last_error_;
        return false;
    }

    if (trained_data_->method != options.method) {
        last_error_ = "trained mode does not match registration mode";
        result.message = last_error_;
        return false;
    }

    if (NeedsTargetNormals(options.method) &&
        trained_data_->cached_target_normals.normals_.empty()) {
        last_error_ =
                "target point cloud normals are required for point-to-plane mode";
        result.message = last_error_;
        return false;
    }

    if (NeedsTargetNormals(options.method) &&
        !HasMatchingNormals(trained_data_->cached_target_normals)) {
        last_error_ =
                "target point cloud normals must be empty or match the point count";
        result.message = last_error_;
        return false;
    }

    if (!IsSupportedMethod(options.method)) {
        last_error_ = "selected method is not implemented in the wrapper";
        result.message = last_error_;
        return false;
    }

    auto source_matrix = internal::PointCloudToMatrix(source);
    auto target_matrix = trained_data_->target_matrix;
    const auto normalization =
            internal::NormalizeSharedSourceTarget(source_matrix, target_matrix);
    const Eigen::Matrix4d adapted_initial_transform =
            options.use_initial_transform
                    ? internal::AdaptInitialTransform(options.initial_transform,
                                                      normalization)
                    : Eigen::Matrix4d::Identity();

    auto source_mean = normalization.source_mean;
    auto target_mean = normalization.target_mean;

    auto finish = [&](const Eigen::Matrix4d& normalized_transform,
                      double convergence_energy,
                      int iteration_count) {
        const auto transformed =
                RescaleTranslation(normalized_transform, normalization.scale);
        PopulateResult(options, options.method, source, target, transformed,
                       convergence_energy, iteration_count, result);
        last_error_.clear();
        return true;
    };

    switch (options.method) {
        case RegistrationMethod::ICP:
        case RegistrationMethod::FastICP:
        case RegistrationMethod::RobustICP: {
            auto parameters = MakeIcpParameters(
                    internal::DescribeIcpParameters(options));
            parameters.use_init = options.use_initial_transform;
            parameters.init_trans = adapted_initial_transform;
            parameters.f = options.method == RegistrationMethod::RobustICP
                                   ? ::ICP::WELSCH
                                   : ::ICP::NONE;
            parameters.use_AA =
                    options.method != RegistrationMethod::ICP;

            ::FRICP<3> fricp;
            fricp.point_to_point(source_matrix, target_matrix, source_mean,
                                 target_mean, parameters);
            return finish(parameters.res_trans, parameters.convergence_energy,
                          parameters.convergence_iter);
        }
        case RegistrationMethod::AAICP: {
            auto parameters = MakeIcpParameters(
                    internal::DescribeIcpParameters(options));
            parameters.use_init = options.use_initial_transform;
            parameters.init_trans = adapted_initial_transform;
            parameters.f = ::ICP::NONE;
            parameters.use_AA = false;

            ::AAICP::point_to_point_aaicp(source_matrix, target_matrix,
                                          source_mean, target_mean, parameters);
            return finish(parameters.res_trans, parameters.convergence_energy,
                          parameters.convergence_iter);
        }
        case RegistrationMethod::PointToPlane:
        case RegistrationMethod::RobustPointToPlane: {
            if (trained_data_->cached_target_normals.normals_.empty()) {
                last_error_ =
                        "target point cloud normals are required for point-to-plane mode";
                result.message = last_error_;
                return false;
            }

            auto parameters = MakeIcpParameters(
                    internal::DescribeIcpParameters(options));
            parameters.use_init = options.use_initial_transform;
            parameters.init_trans = adapted_initial_transform;
            parameters.f = options.method == RegistrationMethod::RobustPointToPlane
                                   ? ::ICP::WELSCH
                                   : ::ICP::NONE;
            parameters.use_AA =
                    options.method == RegistrationMethod::RobustPointToPlane;

            auto source_normals =
                    MakeSourceNormals(source, source_matrix.cols());
            auto target_normals = trained_data_->target_normals_matrix;

            ::FRICP<3> fricp;
            if (options.method == RegistrationMethod::PointToPlane) {
                fricp.point_to_plane(source_matrix, target_matrix, source_normals,
                                    target_normals, source_mean, target_mean,
                                    parameters);
            } else {
                fricp.point_to_plane_GN(source_matrix, target_matrix,
                                        source_normals, target_normals,
                                        source_mean, target_mean, parameters);
            }
            return finish(parameters.res_trans, parameters.convergence_energy,
                          parameters.convergence_iter);
        }
        case RegistrationMethod::SparseICP:
        case RegistrationMethod::SparsePointToPlane: {
            auto parameters = MakeSicpParameters(
                    internal::DescribeSicpParameters(options));
            parameters.init_trans = adapted_initial_transform;

            if (options.method == RegistrationMethod::SparseICP) {
                ::SICP::point_to_point(source_matrix, target_matrix, source_mean,
                                       target_mean, parameters);
            } else {
                if (trained_data_->cached_target_normals.normals_.empty()) {
                    last_error_ =
                            "target point cloud normals are required for point-to-plane mode";
                    result.message = last_error_;
                    return false;
                }
                auto target_normals = trained_data_->target_normals_matrix;
                ::SICP::point_to_plane(source_matrix, target_matrix, target_normals,
                                       source_mean, target_mean, parameters);
            }

            return finish(parameters.res_trans, parameters.convergence_mse,
                          parameters.convergence_iter);
        }
    }

    last_error_ = "selected method is not implemented in the wrapper";
    result.message = last_error_;
    return false;
}

const std::string& FastRobustIcp::GetLastError() const {
    return last_error_;
}

}  // namespace fricp
