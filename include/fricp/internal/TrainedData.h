#pragma once

#include <fricp/Types.h>
#include <fricp/internal/FastRobustCore.h>

#include <open3d/geometry/PointCloud.h>

namespace fricp::internal {

struct TrainedData {
    bool is_trained = false;
    RegistrationMethod method = RegistrationMethod::ICP;
    RegistrationOptions options;
    open3d::geometry::PointCloud target;
    // Cached target cloud for point-to-plane registration.
    open3d::geometry::PointCloud target_with_normals;
    RobustTargetCache robust_target_cache;
};

}  // namespace fricp::internal
