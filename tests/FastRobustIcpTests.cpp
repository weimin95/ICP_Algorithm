#include <fricp/FastRobustIcp.h>
#include <internal/Open3DAdapters.h>
#include <internal/UpstreamParameterMapping.h>

#include <ICP.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <type_traits>
#include <utility>
#include <vector>

static_assert(!std::is_copy_constructible_v<fricp::FastRobustIcp>);
static_assert(!std::is_copy_assignable_v<fricp::FastRobustIcp>);
static_assert(std::is_move_constructible_v<fricp::FastRobustIcp>);
static_assert(std::is_move_assignable_v<fricp::FastRobustIcp>);

namespace {

int TestAllMethodsAreAddressable() {
    using fricp::RegistrationMethod;
    const std::vector<RegistrationMethod> methods = {
            RegistrationMethod::ICP,
            RegistrationMethod::AAICP,
            RegistrationMethod::FastICP,
            RegistrationMethod::RobustICP,
            RegistrationMethod::PointToPlane,
            RegistrationMethod::RobustPointToPlane,
            RegistrationMethod::SparseICP,
            RegistrationMethod::SparsePointToPlane,
    };
    return methods.size() == 8 ? 0 : 1;
}

int TestOptionsExposeUpstreamAlignedFields() {
    fricp::RegistrationOptions options;
    options.method = fricp::RegistrationMethod::RobustICP;
    options.max_icp = 33;
    options.max_outer = 2;
    options.stop = 1e-6;
    options.p = 0.25;
    options.anderson_m = 7;
    options.beta = 0.8;
    options.error_overflow_threshold = 0.02;
    options.nu_begin_k = 4.0;
    options.nu_end_k = 0.2;
    options.nu_alpha = 0.4;
    options.sicp_use_penalty = true;
    options.sicp_mu = 12.0;
    options.sicp_alpha = 1.5;
    options.sicp_max_mu = 1e6;
    options.sicp_max_icp = 80;
    options.sicp_max_outer = 30;
    options.sicp_max_inner = 4;
    options.sicp_p = 0.3;

    return options.method == fricp::RegistrationMethod::RobustICP &&
                   options.max_icp == 33 &&
                   options.max_outer == 2 &&
                   options.stop == 1e-6 &&
                   options.p == 0.25 &&
                   options.anderson_m == 7 &&
                   options.beta == 0.8 &&
                   options.error_overflow_threshold == 0.02 &&
                   options.nu_begin_k == 4.0 &&
                   options.nu_end_k == 0.2 &&
                   options.nu_alpha == 0.4 &&
                   options.sicp_use_penalty &&
                   options.sicp_mu == 12.0 &&
                   options.sicp_alpha == 1.5 &&
                   options.sicp_max_mu == 1e6 &&
                   options.sicp_max_icp == 80 &&
                   options.sicp_max_outer == 30 &&
                   options.sicp_max_inner == 4 &&
                   options.sicp_p == 0.3
           ? 0
           : 1;
}

open3d::geometry::PointCloud MakeCubeCloud() {
    open3d::geometry::PointCloud cloud;
    cloud.points_ = {
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0},
        {1.0, 1.0, 0.0},
        {1.0, 0.0, 1.0},
        {0.0, 1.0, 1.0},
        {1.0, 1.0, 1.0},
    };
    return cloud;
}

Eigen::Matrix4d MakeKnownTransform() {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) =
            Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    transform.block<3, 1>(0, 3) = Eigen::Vector3d(0.2, -0.1, 0.05);
    return transform;
}

open3d::geometry::PointCloud MakeLShapeCloud() {
    open3d::geometry::PointCloud cloud;
    for (int x = -2; x <= 2; ++x) {
        for (int y = -2; y <= 2; ++y) {
            cloud.points_.push_back(Eigen::Vector3d(0.1 * x, 0.1 * y, 0.0));
        }
    }
    for (int z = -2; z <= 2; ++z) {
        for (int y = -2; y <= 2; ++y) {
            cloud.points_.push_back(Eigen::Vector3d(0.0, 0.1 * y, 0.1 * z));
        }
    }
    return cloud;
}

open3d::geometry::PointCloud MakeBoxGridCloud() {
    open3d::geometry::PointCloud cloud;
    for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
            for (int z = -1; z <= 1; ++z) {
                cloud.points_.push_back(
                        Eigen::Vector3d(0.12 * x, 0.10 * y, 0.08 * z));
            }
        }
    }
    return cloud;
}

open3d::geometry::PointCloud MakeAdapterSourceCloud() {
    open3d::geometry::PointCloud cloud;
    cloud.points_ = {
            {1.0, 2.0, 3.0},
            {5.0, 2.0, 3.0},
            {1.0, 6.0, 3.0},
    };
    cloud.normals_ = {
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0},
    };
    return cloud;
}

open3d::geometry::PointCloud MakeAdapterTargetCloud() {
    open3d::geometry::PointCloud cloud;
    cloud.points_ = {
            {2.0, -1.0, 0.0},
            {4.0, -1.0, 0.0},
            {2.0, 1.0, 0.0},
    };
    cloud.normals_ = {
            {0.0, 1.0, 0.0},
            {0.0, 1.0, 0.0},
            {0.0, 1.0, 0.0},
    };
    return cloud;
}

int TestPointCloudAndNormalAdapters() {
    const auto cloud = MakeAdapterSourceCloud();
    const auto points = fricp::internal::PointCloudToMatrix(cloud);
    const auto normals = fricp::internal::NormalsToMatrix(cloud);

    if (points.rows() != 3 || points.cols() != 3) {
        std::cerr << "expected 3x3 point matrix\n";
        return 1;
    }

    if (normals.rows() != 3 || normals.cols() != 3) {
        std::cerr << "expected 3x3 normal matrix\n";
        return 1;
    }

    if ((points.col(0) - Eigen::Vector3d(1.0, 2.0, 3.0)).norm() > 1e-12) {
        std::cerr << "expected first point to preserve ordering\n";
        return 1;
    }

    if ((normals.col(2) - Eigen::Vector3d(0.0, 0.0, 1.0)).norm() > 1e-12) {
        std::cerr << "expected third normal to preserve ordering\n";
        return 1;
    }

    return 0;
}

int TestBoundingBoxAndNormalizationAdapters() {
    auto source = fricp::internal::PointCloudToMatrix(MakeAdapterSourceCloud());
    auto target = fricp::internal::PointCloudToMatrix(MakeAdapterTargetCloud());

    const auto source_stats =
            fricp::internal::ComputeBoundingBoxStatistics(source);
    const auto target_stats =
            fricp::internal::ComputeBoundingBoxStatistics(target);

    if ((source_stats.min_bound - Eigen::Vector3d(1.0, 2.0, 3.0)).norm() > 1e-12) {
        std::cerr << "expected source min bound\n";
        return 1;
    }
    if ((source_stats.extent - Eigen::Vector3d(4.0, 4.0, 0.0)).norm() > 1e-12) {
        std::cerr << "expected source extent\n";
        return 1;
    }
    if ((target_stats.max_bound - Eigen::Vector3d(4.0, 1.0, 0.0)).norm() > 1e-12) {
        std::cerr << "expected target max bound\n";
        return 1;
    }

    const auto normalization =
            fricp::internal::NormalizeSharedSourceTarget(source, target);

    const double expected_scale = std::max(
            Eigen::Vector3d(4.0, 4.0, 0.0).norm(),
            Eigen::Vector3d(2.0, 2.0, 0.0).norm());
    if (std::abs(normalization.scale - expected_scale) > 1e-12) {
        std::cerr << "expected upstream shared scale\n";
        return 1;
    }

    if (source.rowwise().sum().norm() > 1e-12 ||
        target.rowwise().sum().norm() > 1e-12) {
        std::cerr << "expected centered point matrices\n";
        return 1;
    }

    const Eigen::Vector3d expected_source_mean =
            Eigen::Vector3d(7.0 / 3.0, 10.0 / 3.0, 3.0) / expected_scale;
    const Eigen::Vector3d expected_target_mean =
            Eigen::Vector3d(8.0 / 3.0, -1.0 / 3.0, 0.0) / expected_scale;
    if ((normalization.source_mean - expected_source_mean).norm() > 1e-12) {
        std::cerr << "expected scaled source mean\n";
        return 1;
    }
    if ((normalization.target_mean - expected_target_mean).norm() > 1e-12) {
        std::cerr << "expected scaled target mean\n";
        return 1;
    }

    return 0;
}

int TestInitialTransformAdaptationMatchesUpstreamRule() {
    auto source = fricp::internal::PointCloudToMatrix(MakeAdapterSourceCloud());
    auto target = fricp::internal::PointCloudToMatrix(MakeAdapterTargetCloud());
    const auto normalization =
            fricp::internal::NormalizeSharedSourceTarget(source, target);

    Eigen::Matrix4d initial = Eigen::Matrix4d::Identity();
    initial.block<3, 3>(0, 0) =
            Eigen::AngleAxisd(3.14159265358979323846 / 2.0,
                              Eigen::Vector3d::UnitZ())
                    .toRotationMatrix();
    initial.block<3, 1>(0, 3) = Eigen::Vector3d(3.0, 6.0, 9.0);

    const Eigen::Matrix4d adapted =
            fricp::internal::AdaptInitialTransform(initial, normalization);

    const Eigen::Vector3d expected_translation =
            initial.block<3, 1>(0, 3) / normalization.scale +
            initial.block<3, 3>(0, 0) * normalization.source_mean -
            normalization.target_mean;

    if ((adapted.block<3, 3>(0, 0) - initial.block<3, 3>(0, 0)).norm() > 1e-12) {
        std::cerr << "expected rotation to remain unchanged\n";
        return 1;
    }

    if ((adapted.block<3, 1>(0, 3) - expected_translation).norm() > 1e-12) {
        std::cerr << "expected upstream initial-transform adaptation\n";
        return 1;
    }

    return 0;
}

int TestIcpParameterMappingCapturesPublicOptions() {
    fricp::RegistrationOptions options;
    options.method = fricp::RegistrationMethod::RobustICP;
    options.use_initial_transform = true;
    options.initial_transform = Eigen::Matrix4d::Identity();
    options.initial_transform.block<3, 3>(0, 0) =
            Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitY()).toRotationMatrix();
    options.stop = 1e-6;
    options.max_icp = 33;
    options.max_outer = 2;
    options.anderson_m = 7;
    options.beta = 0.8;
    options.error_overflow_threshold = 0.02;
    options.nu_begin_k = 4.0;
    options.nu_end_k = 0.2;
    options.nu_alpha = 0.4;

    const auto mapped = fricp::internal::BuildIcpParameters(options);

    if (!mapped.use_init) {
        std::cerr << "expected init transform to be enabled\n";
        return 1;
    }
    if ((mapped.init_trans - options.initial_transform).cwiseAbs().maxCoeff() > 1e-12) {
        std::cerr << "expected init transform to be copied verbatim\n";
        return 1;
    }
    if (mapped.stop != options.stop || mapped.max_icp != options.max_icp ||
        mapped.max_outer != options.max_outer || mapped.anderson_m != options.anderson_m ||
        mapped.beta_ != options.beta ||
        mapped.error_overflow_threshold_ != options.error_overflow_threshold) {
        std::cerr << "expected scalar ICP options to map directly\n";
        return 1;
    }
    if (mapped.f != ::ICP::WELSCH || !mapped.use_AA) {
        std::cerr << "expected robust ICP defaults for method dispatch\n";
        return 1;
    }
    if (mapped.nu_begin_k != options.nu_begin_k ||
        mapped.nu_end_k != options.nu_end_k ||
        mapped.nu_alpha != options.nu_alpha) {
        std::cerr << "expected nu parameters to map directly for RobustICP\n";
        return 1;
    }

    return 0;
}

int TestIcpParameterMappingAppliesPointToPlaneNuEndDefault() {
    fricp::RegistrationOptions options;
    options.method = fricp::RegistrationMethod::RobustPointToPlane;

    const auto mapped = fricp::internal::BuildIcpParameters(options);
    const double expected_default = 1.0 / 6.0;

    if (std::abs(mapped.nu_end_k - expected_default) > 1e-12) {
        std::cerr << "expected point-to-plane robust nu_end_k default\n";
        return 1;
    }

    return 0;
}

int TestSicpParameterMappingCapturesPublicOptions() {
    fricp::RegistrationOptions options;
    options.method = fricp::RegistrationMethod::SparseICP;
    options.use_initial_transform = true;
    options.initial_transform = Eigen::Matrix4d::Identity();
    options.initial_transform.block<3, 3>(0, 0) =
            Eigen::AngleAxisd(-0.5, Eigen::Vector3d::UnitX()).toRotationMatrix();
    options.stop = 1e-7;
    options.sicp_use_penalty = true;
    options.sicp_mu = 12.0;
    options.sicp_alpha = 1.5;
    options.sicp_max_mu = 1e6;
    options.sicp_max_icp = 80;
    options.sicp_max_outer = 30;
    options.sicp_max_inner = 4;
    options.sicp_p = 0.3;

    const auto mapped = fricp::internal::BuildSicpParameters(options);

    if ((mapped.init_trans - options.initial_transform).cwiseAbs().maxCoeff() > 1e-12) {
        std::cerr << "expected SICP init transform to be copied verbatim\n";
        return 1;
    }
    if (mapped.stop != options.stop || mapped.use_penalty != options.sicp_use_penalty ||
        mapped.mu != options.sicp_mu || mapped.alpha != options.sicp_alpha ||
        mapped.max_mu != options.sicp_max_mu || mapped.max_icp != options.sicp_max_icp ||
        mapped.max_outer != options.sicp_max_outer ||
        mapped.max_inner != options.sicp_max_inner || mapped.p != options.sicp_p) {
        std::cerr << "expected sparse ICP options to map directly\n";
        return 1;
    }

    return 0;
}

void InjectOutliers(open3d::geometry::PointCloud& cloud) {
    const std::vector<Eigen::Vector3d> outliers = {
            {3.0, 3.0, 3.0},
            {-3.5, 2.0, -1.0},
            {2.8, -2.6, 1.5},
            {-2.9, -3.1, 2.7},
            {4.2, 0.5, -2.8},
            {-4.0, 1.3, 2.2},
            {3.7, -1.6, -3.3},
            {-3.8, -2.4, -2.1},
            {2.5, 4.0, -1.5},
            {-2.7, 3.8, 2.9},
            {4.5, -3.7, 0.8},
            {-4.3, -0.9, 3.4},
    };
    for (const auto& point : outliers) {
        cloud.points_.push_back(point);
    }
}

int TestConstructionStartsUntrained() {
    fricp::FastRobustIcp icp;

    if (icp.IsTrained()) {
        std::cerr << "expected a freshly constructed instance to be untrained\n";
        return 1;
    }

    return 0;
}

int TestFailedTrainLeavesObjectUntrained() {
    fricp::FastRobustIcp icp;
    const open3d::geometry::PointCloud valid_target = MakeCubeCloud();
    open3d::geometry::PointCloud empty_target;
    fricp::RegistrationOptions options;

    if (!icp.Train(valid_target, options)) {
        return 1;
    }

    if (!icp.IsTrained()) {
        return 1;
    }

    if (icp.Train(empty_target, options)) return 1;
    return icp.IsTrained() ? 1 : 0;
}

int TestRetrainOverwritesOldTraining() {
    fricp::FastRobustIcp icp;
    auto first_target = MakeCubeCloud();
    auto second_target = MakeLShapeCloud();
    auto source = second_target;
    fricp::RegistrationOptions options;
    fricp::RegistrationResult result;

    options.method = fricp::RegistrationMethod::ICP;
    options.max_correspondence_distance = 1.0;
    options.max_icp = 50;

    if (!icp.Train(first_target, options)) return 1;
    if (!icp.Train(second_target, options)) return 1;

    if (!icp.Register(source, options, result)) {
        return 1;
    }

    const Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
    if ((result.transformation - identity).cwiseAbs().maxCoeff() > 1e-2) {
        return 1;
    }

    return result.fitness > 0.95 ? 0 : 1;
}

int TestTrainRejectsEmptyTargetLeavesInstanceUntrained() {
    fricp::FastRobustIcp icp;
    const open3d::geometry::PointCloud target;
    fricp::RegistrationOptions options;

    const bool ok = icp.Train(target, options);
    if (ok) {
        std::cerr << "expected empty-point-cloud training failure\n";
        return 1;
    }

    if (icp.GetLastError().find("empty") == std::string::npos) {
        std::cerr << "expected an error message mentioning empty point clouds\n";
        return 1;
    }

    if (icp.IsTrained()) {
        std::cerr << "expected failed Train to leave the instance untrained\n";
        return 1;
    }

    return 0;
}

int TestRegisterFailsWhenNotTrained() {
    fricp::FastRobustIcp icp;
    const open3d::geometry::PointCloud source = MakeCubeCloud();
    fricp::RegistrationOptions options;
    fricp::RegistrationResult result;

    const bool ok = icp.Register(source, options, result);
    if (ok) {
        std::cerr << "expected register failure before training\n";
        return 1;
    }

    if (result.message.find("Train") == std::string::npos) {
        std::cerr << "expected an error message mentioning Train\n";
        return 1;
    }

    return 0;
}

int TestRegisterRejectsEmptySourceAfterTraining() {
    fricp::FastRobustIcp icp;
    const open3d::geometry::PointCloud target = MakeCubeCloud();
    open3d::geometry::PointCloud source;
    fricp::RegistrationOptions options;
    fricp::RegistrationResult result;

    if (!icp.Train(target, options)) {
        std::cerr << "expected training to succeed before empty-source registration\n";
        return 1;
    }

    if (!icp.IsTrained()) {
        std::cerr << "expected trained state after successful Train\n";
        return 1;
    }

    const bool ok = icp.Register(source, options, result);
    if (ok) {
        std::cerr << "expected empty-source register failure after training\n";
        return 1;
    }

    if (result.message.find("empty") == std::string::npos &&
        result.message.find("source") == std::string::npos) {
        std::cerr << "expected an error message mentioning empty source data\n";
        return 1;
    }

    return 0;
}

int TestRegisterRejectsInvalidDistanceAfterTraining() {
    fricp::FastRobustIcp icp;
    open3d::geometry::PointCloud source;
    open3d::geometry::PointCloud target;
    fricp::RegistrationOptions options;
    fricp::RegistrationResult result;

    source.points_.push_back({0.0, 0.0, 0.0});
    target.points_.push_back({0.0, 0.0, 0.0});

    // Train validates target-dependent inputs only.
    // Register validates call-time values such as max_correspondence_distance.
    options.max_correspondence_distance = 0.0;

    if (!icp.Train(target, options)) {
        std::cerr << "expected training to succeed with register-time distance value\n";
        return 1;
    }

    if (!icp.IsTrained()) {
        std::cerr << "expected trained state after successful Train\n";
        return 1;
    }

    if (icp.Register(source, options, result)) {
        std::cerr << "expected invalid max_correspondence_distance failure\n";
        return 1;
    }

    if (result.message.find("max_correspondence_distance") == std::string::npos) {
        std::cerr << "expected max_correspondence_distance error message\n";
        return 1;
    }

    return 0;
}

int TestRejectsPointToPlaneWithoutNormalsWhenDisabled() {
    fricp::FastRobustIcp icp;
    open3d::geometry::PointCloud target;

    target.points_.push_back({0.0, 0.0, 0.0});
    fricp::RegistrationOptions options;
    options.method = fricp::RegistrationMethod::PointToPlane;
    options.estimate_target_normals_if_missing = false;

    if (icp.IsTrained()) {
        std::cerr << "expected a fresh instance to start untrained\n";
        return 1;
    }

    if (icp.Train(target, options)) {
        std::cerr << "expected point-to-plane training failure without normals\n";
        return 1;
    }

    if (icp.GetLastError().find("normals") == std::string::npos) {
        std::cerr << "expected normals error message\n";
        return 1;
    }

    if (icp.IsTrained()) {
        std::cerr << "expected failed Train to leave the instance untrained\n";
        return 1;
    }

    return 0;
}

int TestPointToPointRecoversKnownTransform() {
    open3d::geometry::PointCloud target = MakeCubeCloud();
    open3d::geometry::PointCloud source = target;
    source.Transform(MakeKnownTransform());

    fricp::FastRobustIcp icp;
    fricp::RegistrationOptions options;
    fricp::RegistrationResult result;

    options.method = fricp::RegistrationMethod::ICP;
    options.max_correspondence_distance = 2.0;
    options.max_icp = 100;

    if (!icp.Train(target, options)) {
        std::cerr << "expected training to succeed for point-to-point test\n";
        return 1;
    }

    if (!icp.Register(source, options, result)) {
        std::cerr << "expected point-to-point registration success\n";
        return 1;
    }

    const Eigen::Matrix4d expected = MakeKnownTransform().inverse();
    if ((result.transformation - expected).cwiseAbs().maxCoeff() > 1e-2) {
        std::cerr << "expected point-to-point transform recovery\n";
        return 1;
    }

    return 0;
}

int TestPointToPlaneEstimatesTargetNormals() {
    open3d::geometry::PointCloud target = MakeLShapeCloud();
    open3d::geometry::PointCloud source = target;

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) =
            (Eigen::AngleAxisd(0.12, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(-0.08, Eigen::Vector3d::UnitX()))
                    .toRotationMatrix();
    transform.block<3, 1>(0, 3) = Eigen::Vector3d(0.04, -0.03, 0.06);
    source.Transform(transform);

    fricp::FastRobustIcp icp;
    fricp::RegistrationOptions options;
    fricp::RegistrationResult result;

    options.method = fricp::RegistrationMethod::PointToPlane;
    options.max_correspondence_distance = 0.3;
    options.max_icp = 100;
    options.estimate_target_normals_if_missing = true;
    options.normal_radius = 0.25;
    options.normal_knn = 20;

    if (!icp.Train(target, options)) {
        std::cerr << "expected training to succeed for point-to-plane test\n";
        return 1;
    }

    if (!icp.Register(source, options, result)) {
        std::cerr << "expected point-to-plane registration success\n";
        return 1;
    }

    const Eigen::Matrix4d expected = transform.inverse();
    if ((result.transformation - expected).cwiseAbs().maxCoeff() > 2e-2) {
        std::cerr << "expected point-to-plane transform recovery\n";
        return 1;
    }

    return 0;
}

int TestMoveSemanticsPreserveTrainingState() {
    fricp::FastRobustIcp icp;
    const open3d::geometry::PointCloud target = MakeCubeCloud();
    open3d::geometry::PointCloud source = target;
    source.Transform(MakeKnownTransform());
    fricp::RegistrationOptions options;
    fricp::RegistrationResult result;

    options.method = fricp::RegistrationMethod::ICP;
    options.max_correspondence_distance = 2.0;
    options.max_icp = 100;

    if (!icp.Train(target, options)) {
        std::cerr << "expected training to succeed before move construction\n";
        return 1;
    }

    fricp::FastRobustIcp moved = std::move(icp);
    if (!moved.IsTrained()) {
        std::cerr << "expected move construction to preserve training state\n";
        return 1;
    }

    if (!moved.Register(source, options, result)) {
        std::cerr << "expected move-constructed instance to register successfully\n";
        return 1;
    }

    fricp::FastRobustIcp assigned;
    assigned = std::move(moved);
    if (!assigned.IsTrained()) {
        std::cerr << "expected move assignment to preserve training state\n";
        return 1;
    }

    if (!assigned.Register(source, options, result)) {
        std::cerr << "expected move-assigned instance to register successfully\n";
        return 1;
    }

    return 0;
}

int TestPointToPlaneTrainOnceRegisterTwice() {
    auto target = MakeLShapeCloud();
    auto source_a = target;
    auto source_b = target;

    Eigen::Matrix4d ta = Eigen::Matrix4d::Identity();
    ta.block<3, 1>(0, 3) = Eigen::Vector3d(0.03, -0.02, 0.04);
    source_a.Transform(ta);

    Eigen::Matrix4d tb = Eigen::Matrix4d::Identity();
    tb.block<3, 1>(0, 3) = Eigen::Vector3d(-0.02, 0.01, 0.05);
    source_b.Transform(tb);

    fricp::FastRobustIcp icp;
    fricp::RegistrationOptions train_options;
    train_options.method = fricp::RegistrationMethod::PointToPlane;
    train_options.max_correspondence_distance = 0.3;
    train_options.estimate_target_normals_if_missing = true;
    train_options.normal_radius = 0.25;
    train_options.normal_knn = 20;

    fricp::RegistrationOptions register_options;
    register_options.method = fricp::RegistrationMethod::PointToPlane;
    register_options.max_correspondence_distance = 0.3;
    register_options.estimate_target_normals_if_missing = false;

    fricp::RegistrationResult result_a;
    fricp::RegistrationResult result_b;

    if (!icp.Train(target, train_options)) {
        std::cerr << "expected point-to-plane training to succeed\n";
        return 1;
    }
    if (!icp.Register(source_a, register_options, result_a)) {
        std::cerr << "expected first point-to-plane registration to succeed: "
                  << icp.GetLastError() << '\n';
        return 1;
    }
    if (!icp.Register(source_b, register_options, result_b)) {
        std::cerr << "expected second point-to-plane registration to succeed: "
                  << icp.GetLastError() << '\n';
        return 1;
    }
    return (!result_a.success || !result_b.success) ? 1 : 0;
}

int TestTrainCachesAutoEstimatedNormalsForRegister() {
    fricp::FastRobustIcp icp;
    auto target = MakeLShapeCloud();
    auto source = target;

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 1>(0, 3) = Eigen::Vector3d(0.02, -0.01, 0.03);
    source.Transform(transform);

    fricp::RegistrationOptions train_options;
    train_options.method = fricp::RegistrationMethod::PointToPlane;
    train_options.max_correspondence_distance = 0.3;
    train_options.estimate_target_normals_if_missing = true;
    train_options.normal_radius = 0.25;
    train_options.normal_knn = 20;

    fricp::RegistrationOptions register_options = train_options;
    register_options.estimate_target_normals_if_missing = false;

    fricp::RegistrationResult result;

    if (!icp.Train(target, train_options)) {
        std::cerr << "expected training to auto-estimate target normals\n";
        return 1;
    }

    if (!icp.IsTrained()) {
        std::cerr << "expected trained state after auto-estimated Train\n";
        return 1;
    }

    if (!icp.Register(source, register_options, result)) {
        std::cerr << "expected cached normals to support later register: "
                  << icp.GetLastError() << '\n';
        return 1;
    }

    return result.success ? 0 : 1;
}

int TestPointToPlaneRegisterFailsAfterPointToPointTraining() {
    fricp::FastRobustIcp icp;
    auto target = MakeCubeCloud();
    auto source = target;
    fricp::RegistrationOptions train_options;
    train_options.method = fricp::RegistrationMethod::ICP;
    fricp::RegistrationOptions register_options;
    register_options.method = fricp::RegistrationMethod::PointToPlane;
    register_options.max_correspondence_distance = 1.0;
    fricp::RegistrationResult result;

    if (!icp.Train(target, train_options)) return 1;
    if (icp.Register(source, register_options, result)) return 1;
    return result.message.find("mode") == std::string::npos;
}

int TestRobustModeRecoversKnownTransform() {
    open3d::geometry::PointCloud target = MakeCubeCloud();
    open3d::geometry::PointCloud source = target;
    source.Transform(MakeKnownTransform());

    fricp::FastRobustIcp icp;
    fricp::RegistrationOptions options;
    fricp::RegistrationResult result;

    options.method = fricp::RegistrationMethod::RobustICP;
    options.max_correspondence_distance = 2.0;
    options.max_icp = 100;

    if (!icp.Train(target, options)) {
        std::cerr << "expected training to succeed for robust test\n";
        return 1;
    }

    if (!icp.Register(source, options, result)) {
        std::cerr << "expected robust registration success\n";
        return 1;
    }

    const Eigen::Matrix4d expected = MakeKnownTransform().inverse();
    if ((result.transformation - expected).cwiseAbs().maxCoeff() > 1e-2) {
        std::cerr << "expected robust transform recovery\n";
        return 1;
    }

    return 0;
}

int TestRobustModeBeatsPointToPointWithOutliers() {
    open3d::geometry::PointCloud target = MakeBoxGridCloud();
    open3d::geometry::PointCloud source = target;
    source.Transform(MakeKnownTransform());
    InjectOutliers(source);

    fricp::FastRobustIcp icp;
    fricp::RegistrationResult point_to_point_result;
    fricp::RegistrationResult robust_result;

    fricp::RegistrationOptions point_to_point_options;
    point_to_point_options.method = fricp::RegistrationMethod::ICP;
    point_to_point_options.max_correspondence_distance = 6.0;
    point_to_point_options.max_icp = 100;

    fricp::RegistrationOptions robust_options = point_to_point_options;
    robust_options.method = fricp::RegistrationMethod::RobustICP;

    if (!icp.Train(target, point_to_point_options)) {
        std::cerr << "expected training to succeed for outlier point-to-point test\n";
        return 1;
    }

    if (!icp.Register(source, point_to_point_options, point_to_point_result)) {
        std::cerr << "expected point-to-point success on outlier case\n";
        return 1;
    }
    if (!icp.Train(target, robust_options)) {
        std::cerr << "expected training to succeed for outlier robust test\n";
        return 1;
    }
    if (!icp.Register(source, robust_options, robust_result)) {
        std::cerr << "expected robust success on outlier case\n";
        return 1;
    }

    const Eigen::Matrix4d expected = MakeKnownTransform().inverse();
    const double point_to_point_error =
            (point_to_point_result.transformation - expected).norm();
    const double robust_error = (robust_result.transformation - expected).norm();
    if (!(robust_error + 1e-3 < point_to_point_error)) {
        std::cerr << "expected robust mode to outperform point-to-point on outliers\n";
        return 1;
    }

    return 0;
}

int TestRobustModeTrainOnceRegisterTwice() {
    open3d::geometry::PointCloud target = MakeCubeCloud();
    open3d::geometry::PointCloud source_a = target;
    open3d::geometry::PointCloud source_b = target;

    const Eigen::Matrix4d transform = MakeKnownTransform();
    source_a.Transform(transform);
    source_b.Transform(transform);

    fricp::FastRobustIcp icp;
    fricp::RegistrationOptions options;
    fricp::RegistrationResult result_a;
    fricp::RegistrationResult result_b;

    options.method = fricp::RegistrationMethod::RobustICP;
    options.max_correspondence_distance = 6.0;
    options.max_icp = 100;

    if (!icp.Train(target, options)) {
        std::cerr << "expected robust training to succeed once\n";
        return 1;
    }

    if (!icp.Register(source_a, options, result_a)) {
        std::cerr << "expected first robust registration to succeed\n";
        return 1;
    }
    if (!icp.Register(source_b, options, result_b)) {
        std::cerr << "expected second robust registration to succeed\n";
        return 1;
    }

    const Eigen::Matrix4d expected_a = transform.inverse();
    const Eigen::Matrix4d expected_b = transform.inverse();
    if ((result_a.transformation - expected_a).cwiseAbs().maxCoeff() > 1e-2) {
        std::cerr << "expected first robust transform recovery\n";
        return 1;
    }
    if ((result_b.transformation - expected_b).cwiseAbs().maxCoeff() > 1e-2) {
        std::cerr << "expected second robust transform recovery\n";
        return 1;
    }

    return 0;
}

int TestTrainSetsTrainedState() {
    fricp::FastRobustIcp icp;
    const open3d::geometry::PointCloud target = MakeCubeCloud();
    fricp::RegistrationOptions options;

    if (!icp.Train(target, options)) {
        std::cerr << "expected training to succeed for state test\n";
        return 1;
    }

    if (!icp.IsTrained()) {
        std::cerr << "expected trained state after Train\n";
        return 1;
    }

    icp.ClearTraining();
    if (icp.IsTrained()) {
        std::cerr << "expected untrained state after ClearTraining\n";
        return 1;
    }

    return 0;
}

int TestPointToPointRegisterRejectsModeMismatch() {
    open3d::geometry::PointCloud target = MakeCubeCloud();
    open3d::geometry::PointCloud source = target;
    source.Transform(MakeKnownTransform());

    fricp::FastRobustIcp icp;
    fricp::RegistrationOptions train_options;
    fricp::RegistrationOptions register_options;
    fricp::RegistrationResult result;

    train_options.method = fricp::RegistrationMethod::ICP;
    train_options.max_correspondence_distance = 2.0;
    register_options.method = fricp::RegistrationMethod::RobustICP;
    register_options.max_correspondence_distance = 2.0;
    register_options.max_icp = 100;

    if (!icp.Train(target, train_options)) {
        std::cerr << "expected training to succeed for mode mismatch test\n";
        return 1;
    }

    if (icp.Register(source, register_options, result)) {
        std::cerr << "expected register to reject a mismatched mode\n";
        return 1;
    }

    if (result.message.find("mode") == std::string::npos) {
        std::cerr << "expected mode mismatch error message\n";
        return 1;
    }

    return 0;
}

}  // namespace

int main() {
    if (TestAllMethodsAreAddressable() != 0) {
        return 1;
    }
    if (TestOptionsExposeUpstreamAlignedFields() != 0) {
        return 1;
    }
    if (TestPointCloudAndNormalAdapters() != 0) {
        return 1;
    }
    if (TestBoundingBoxAndNormalizationAdapters() != 0) {
        return 1;
    }
    if (TestInitialTransformAdaptationMatchesUpstreamRule() != 0) {
        return 1;
    }
    if (TestIcpParameterMappingCapturesPublicOptions() != 0) {
        return 1;
    }
    if (TestIcpParameterMappingAppliesPointToPlaneNuEndDefault() != 0) {
        return 1;
    }
    if (TestSicpParameterMappingCapturesPublicOptions() != 0) {
        return 1;
    }
    if (TestConstructionStartsUntrained() != 0) {
        return 1;
    }
    if (TestFailedTrainLeavesObjectUntrained() != 0) {
        return 1;
    }
    if (TestRetrainOverwritesOldTraining() != 0) {
        return 1;
    }
    if (TestTrainRejectsEmptyTargetLeavesInstanceUntrained() != 0) {
        return 1;
    }
    if (TestRegisterFailsWhenNotTrained() != 0) {
        return 1;
    }
    if (TestRegisterRejectsEmptySourceAfterTraining() != 0) {
        return 1;
    }
    if (TestRegisterRejectsInvalidDistanceAfterTraining() != 0) {
        return 1;
    }
    if (TestTrainSetsTrainedState() != 0) {
        return 1;
    }
    if (TestRejectsPointToPlaneWithoutNormalsWhenDisabled() != 0) {
        return 1;
    }
    if (TestPointToPointRecoversKnownTransform() != 0) {
        return 1;
    }
    if (TestMoveSemanticsPreserveTrainingState() != 0) {
        return 1;
    }
    if (TestPointToPointRegisterRejectsModeMismatch() != 0) {
        return 1;
    }
    if (TestPointToPlaneTrainOnceRegisterTwice() != 0) {
        return 1;
    }
    if (TestTrainCachesAutoEstimatedNormalsForRegister() != 0) {
        return 1;
    }
    if (TestPointToPlaneRegisterFailsAfterPointToPointTraining() != 0) {
        return 1;
    }
    if (TestPointToPlaneEstimatesTargetNormals() != 0) {
        return 1;
    }
    if (TestRobustModeRecoversKnownTransform() != 0) {
        return 1;
    }
    if (TestRobustModeTrainOnceRegisterTwice() != 0) {
        return 1;
    }
    if (TestRobustModeBeatsPointToPointWithOutliers() != 0) {
        return 1;
    }

    return 0;
}
