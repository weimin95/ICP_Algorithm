#include <fricp/FastRobustIcp.h>

#include <iostream>
#include <open3d/Open3D.h>

int main() {
    open3d::geometry::PointCloud target;

    bool success = open3d::io::ReadPointCloud("E:\\work\\project\\HansVision\\7\\dianqu3D\\MV-DLS250U-05(00DA6285188)\\OP20\\test\\target.ply", target);
    auto sampleCloud = target.VoxelDownSample(0.1);
    target = *sampleCloud;

    open3d::geometry::PointCloud source;
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

    success = open3d::io::ReadPointCloud("E:\\work\\project\\HansVision\\7\\dianqu3D\\MV-DLS250U-05(00DA6285188)\\OP20\\test\\source.ply", source);
    auto source_sampleCloud = source.VoxelDownSample(0.1);

    fricp::FastRobustIcp icp;
    fricp::RegistrationOptions options;
    options.method = fricp::RegistrationMethod::RobustICP;
    options.max_correspondence_distance = 0.5;
    options.max_icp = 50;
    options.max_outer = 2;
    options.nu_begin_k = 3.0;
    options.nu_end_k = 1.0 / 6.0;
    options.nu_alpha = 0.5;

    fricp::RegistrationResult result;
    if (!icp.Train(target, options)) {
        std::cerr << "training failed: " << icp.GetLastError() << '\n';
        return 1;
    }

    if (!icp.Register(*source_sampleCloud, options, result)) {
        std::cerr << "registration failed: " << result.message << '\n';
        return 1;
    }

    std::cout << "Estimated transform:\n" << result.transformation << '\n';
    source.Transform(result.transformation);
    success = open3d::io::WritePointCloud("E:\\work\\project\\HansVision\\7\\dianqu3D\\MV-DLS250U-05(00DA6285188)\\OP20\\test\\source_aligned.ply", source);
    return 0;
}
