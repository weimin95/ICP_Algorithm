#include <fricp/FastRobustIcp.h>

#include <Eigen/Geometry>

#include <iostream>

namespace {

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
    open3d::geometry::PointCloud empty_target;
    fricp::RegistrationOptions options;
    if (icp.Train(empty_target, options)) return 1;
    return icp.IsTrained() ? 1 : 0;
}

int TestRetrainOverwritesOldTraining() {
    fricp::FastRobustIcp icp;
    auto first_target = MakeCubeCloud();
    auto second_target = MakeLShapeCloud();
    fricp::RegistrationOptions options;
    if (!icp.Train(first_target, options)) return 1;
    if (!icp.Train(second_target, options)) return 1;
    return icp.IsTrained() ? 0 : 1;
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
    options.mode = fricp::RegistrationMode::PointToPlane;
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

    options.mode = fricp::RegistrationMode::PointToPoint;
    options.max_correspondence_distance = 2.0;
    options.max_iteration = 100;

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

    options.mode = fricp::RegistrationMode::PointToPlane;
    options.max_correspondence_distance = 0.3;
    options.max_iteration = 100;
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

int TestRobustModeRecoversKnownTransform() {
    open3d::geometry::PointCloud target = MakeCubeCloud();
    open3d::geometry::PointCloud source = target;
    source.Transform(MakeKnownTransform());

    fricp::FastRobustIcp icp;
    fricp::RegistrationOptions options;
    fricp::RegistrationResult result;

    options.mode = fricp::RegistrationMode::RobustPointToPoint;
    options.max_correspondence_distance = 2.0;
    options.max_iteration = 100;

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
    point_to_point_options.mode = fricp::RegistrationMode::PointToPoint;
    point_to_point_options.max_correspondence_distance = 6.0;
    point_to_point_options.max_iteration = 100;

    fricp::RegistrationOptions robust_options = point_to_point_options;
    robust_options.mode = fricp::RegistrationMode::RobustPointToPoint;

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

}  // namespace

int main() {
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
    if (TestPointToPlaneEstimatesTargetNormals() != 0) {
        return 1;
    }
    if (TestRobustModeRecoversKnownTransform() != 0) {
        return 1;
    }
    if (TestRobustModeBeatsPointToPointWithOutliers() != 0) {
        return 1;
    }

    return 0;
}
