#include <fricp/FastRobustIcp.h>

#include <iostream>

int main() {
    open3d::geometry::PointCloud target;
    target.points_ = {
            {0.0, 0.0, 0.0},
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0},
    };

    open3d::geometry::PointCloud source = target;
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 1>(0, 3) = Eigen::Vector3d(0.1, -0.05, 0.02);
    source.Transform(transform);

    fricp::FastRobustIcp icp;
    fricp::RegistrationOptions options;
    options.mode = fricp::RegistrationMode::RobustPointToPoint;
    options.max_correspondence_distance = 1.0;
    options.max_iteration = 50;

    fricp::RegistrationResult result;
    if (!icp.Register(source, target, options, result)) {
        std::cerr << "registration failed: " << result.message << '\n';
        return 1;
    }

    std::cout << "Estimated transform:\n" << result.transformation << '\n';
    return 0;
}
