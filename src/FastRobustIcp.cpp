#include <fricp/FastRobustIcp.h>

#include <fricp/internal/FastRobustCore.h>
#include <fricp/internal/TrainedData.h>

#include <open3d/geometry/KDTreeSearchParam.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/pipelines/registration/TransformationEstimation.h>

#include <memory>

namespace fricp {

namespace {

constexpr double kLegacyRelativeFitness = 1e-6;
constexpr double kLegacyRelativeRmse = 1e-6;

bool NeedsTargetNormals(RegistrationMethod method) {
    return method == RegistrationMethod::PointToPlane ||
           method == RegistrationMethod::RobustPointToPlane ||
           method == RegistrationMethod::SparsePointToPlane;
}

bool UsesCurrentPointToPointPath(RegistrationMethod method) {
    return method == RegistrationMethod::ICP;
}

bool UsesCurrentPointToPlanePath(RegistrationMethod method) {
    return method == RegistrationMethod::PointToPlane;
}

bool UsesCurrentRobustPath(RegistrationMethod method) {
    return method == RegistrationMethod::RobustICP;
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

    auto trained_data = std::make_unique<internal::TrainedData>();
    trained_data->is_trained = true;
    trained_data->method = options.method;
    trained_data->options = options;
    trained_data->target = target;

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

    if (UsesCurrentPointToPointPath(options.method)) {
        const Eigen::Matrix4d init = options.use_initial_transform
                                             ? options.initial_transform
                                             : Eigen::Matrix4d::Identity();
        const open3d::pipelines::registration::ICPConvergenceCriteria criteria(
                kLegacyRelativeFitness, kLegacyRelativeRmse, options.max_icp);
        const auto registration =
                open3d::pipelines::registration::RegistrationICP(
                        source, target, options.max_correspondence_distance, init,
                        open3d::pipelines::registration::
                                TransformationEstimationPointToPoint(false),
                        criteria);

        last_error_.clear();
        result.success = true;
        result.method = options.method;
        result.transformation = registration.transformation_;
        result.fitness = registration.fitness_;
        result.inlier_rmse = registration.inlier_rmse_;
        result.iteration_count = options.max_icp;
        result.message = "ok";
        return true;
    }

    if (UsesCurrentPointToPlanePath(options.method)) {
        const auto& target_with_normals = trained_data_->cached_target_normals;

        const Eigen::Matrix4d init = options.use_initial_transform
                                             ? options.initial_transform
                                             : Eigen::Matrix4d::Identity();
        const open3d::pipelines::registration::ICPConvergenceCriteria criteria(
                kLegacyRelativeFitness, kLegacyRelativeRmse, options.max_icp);
        const auto registration =
                open3d::pipelines::registration::RegistrationICP(
                        source, target_with_normals, options.max_correspondence_distance, init,
                        open3d::pipelines::registration::
                                TransformationEstimationPointToPlane(),
                        criteria);

        last_error_.clear();
        result.success = true;
        result.method = options.method;
        result.transformation = registration.transformation_;
        result.fitness = registration.fitness_;
        result.inlier_rmse = registration.inlier_rmse_;
        result.iteration_count = options.max_icp;
        result.message = "ok";
        return true;
    }

    if (UsesCurrentRobustPath(options.method)) {
        internal::RobustOptions robust_options;
        robust_options.max_iteration = options.max_icp;
        robust_options.nu_begin_k = options.nu_begin_k;
        robust_options.nu_end_k = options.nu_end_k;
        robust_options.nu_alpha = options.nu_alpha;
        robust_options.stop = options.stop;
        robust_options.use_anderson = true;

        const auto robust_target_cache =
                internal::BuildRobustTargetCache(trained_data_->target);
        const auto robust_result = internal::RegisterRobustPointToPoint(
                source, robust_target_cache, options.initial_transform,
                options.use_initial_transform, robust_options);
        if (!robust_result.success) {
            last_error_ = robust_result.message;
            result.message = last_error_;
            return false;
        }

        last_error_.clear();
        result.success = true;
        result.method = options.method;
        result.transformation = robust_result.transformation;
        result.convergence_energy = robust_result.convergence_energy;
        result.iteration_count = robust_result.iteration_count;
        result.message = "ok";
        return true;
    }

    last_error_ = "selected method is not implemented in the transitional wrapper yet";
    result.message = last_error_;
    return false;
}

const std::string& FastRobustIcp::GetLastError() const {
    return last_error_;
}

}  // namespace fricp
