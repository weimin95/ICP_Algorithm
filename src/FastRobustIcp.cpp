#include <fricp/FastRobustIcp.h>

#include <fricp/internal/FastRobustCore.h>

#include <open3d/geometry/KDTreeSearchParam.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/pipelines/registration/TransformationEstimation.h>

#include <memory>

namespace fricp {

FastRobustIcp::FastRobustIcp() = default;

FastRobustIcp::~FastRobustIcp() = default;

bool FastRobustIcp::Train(const open3d::geometry::PointCloud& target,
                          const RegistrationOptions& options) {
    if (target.points_.empty()) {
        last_error_ = "target point cloud must not be empty";
        ClearTraining();
        return false;
    }

    if (options.mode == RegistrationMode::PointToPlane &&
        target.normals_.empty() &&
        !options.estimate_target_normals_if_missing) {
        last_error_ =
                "target point cloud normals are required for point-to-plane mode";
        ClearTraining();
        return false;
    }

    auto trained_data = std::make_unique<internal::TrainedData>();
    trained_data->is_trained = true;
    trained_data->mode = options.mode;
    trained_data->options = options;
    trained_data->target = target;

    if (options.mode == RegistrationMode::RobustPointToPoint) {
        trained_data->robust_target_cache =
                internal::BuildRobustTargetCache(target);
    }

    if (options.mode == RegistrationMode::PointToPlane) {
        trained_data->target_with_normals = target;
        if (trained_data->target_with_normals.normals_.empty()) {
            trained_data->target_with_normals.EstimateNormals(
                    open3d::geometry::KDTreeSearchParamHybrid(
                            options.normal_radius, options.normal_knn));
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

    if (trained_data_->mode != options.mode) {
        last_error_ = "trained mode does not match registration mode";
        result.message = last_error_;
        return false;
    }

    if (options.mode == RegistrationMode::PointToPlane &&
        trained_data_->target_with_normals.normals_.empty()) {
        last_error_ =
                "target point cloud normals are required for point-to-plane mode";
        result.message = last_error_;
        return false;
    }

    if (options.mode == RegistrationMode::PointToPoint) {
        const Eigen::Matrix4d init = options.use_initial_transform
                                             ? options.initial_transform
                                             : Eigen::Matrix4d::Identity();
        const open3d::pipelines::registration::ICPConvergenceCriteria criteria(
                options.relative_fitness, options.relative_rmse, options.max_iteration);
        const auto registration =
                open3d::pipelines::registration::RegistrationICP(
                        source, target, options.max_correspondence_distance, init,
                        open3d::pipelines::registration::
                                TransformationEstimationPointToPoint(false),
                        criteria);

        last_error_.clear();
        result.success = true;
        result.mode = options.mode;
        result.transformation = registration.transformation_;
        result.fitness = registration.fitness_;
        result.inlier_rmse = registration.inlier_rmse_;
        result.iteration_count = options.max_iteration;
        result.message = "ok";
        return true;
    }

    if (options.mode == RegistrationMode::PointToPlane) {
        const auto& target_with_normals = trained_data_->target_with_normals;

        const Eigen::Matrix4d init = options.use_initial_transform
                                             ? options.initial_transform
                                             : Eigen::Matrix4d::Identity();
        const open3d::pipelines::registration::ICPConvergenceCriteria criteria(
                options.relative_fitness, options.relative_rmse, options.max_iteration);
        const auto registration =
                open3d::pipelines::registration::RegistrationICP(
                        source, target_with_normals, options.max_correspondence_distance, init,
                        open3d::pipelines::registration::
                                TransformationEstimationPointToPlane(),
                        criteria);

        last_error_.clear();
        result.success = true;
        result.mode = options.mode;
        result.transformation = registration.transformation_;
        result.fitness = registration.fitness_;
        result.inlier_rmse = registration.inlier_rmse_;
        result.iteration_count = options.max_iteration;
        result.message = "ok";
        return true;
    }

    if (options.mode == RegistrationMode::RobustPointToPoint) {
        internal::RobustOptions robust_options;
        robust_options.max_iteration = options.max_iteration;
        robust_options.nu_begin_k = options.robust_nu_begin_k;
        robust_options.nu_end_k = options.robust_nu_end_k;
        robust_options.nu_alpha = options.robust_nu_alpha;
        robust_options.stop = options.robust_stop;
        robust_options.use_anderson = options.robust_use_anderson;

        const auto robust_result = internal::RegisterRobustPointToPoint(
                source, trained_data_->robust_target_cache, options.initial_transform,
                options.use_initial_transform, robust_options);
        if (!robust_result.success) {
            last_error_ = robust_result.message;
            result.message = last_error_;
            return false;
        }

        last_error_.clear();
        result.success = true;
        result.mode = options.mode;
        result.transformation = robust_result.transformation;
        result.convergence_energy = robust_result.convergence_energy;
        result.iteration_count = robust_result.iteration_count;
        result.message = "ok";
        return true;
    }

    last_error_.clear();
    result.success = true;
    result.message = "ok";
    return true;
}

const std::string& FastRobustIcp::GetLastError() const {
    return last_error_;
}

}  // namespace fricp
