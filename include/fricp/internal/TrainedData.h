#pragma once

#include <fricp/Types.h>

#include <open3d/geometry/PointCloud.h>

namespace fricp::internal {

struct TrainedData {
    bool is_trained = false;
    RegistrationMode mode = RegistrationMode::PointToPoint;
    RegistrationOptions options;
    open3d::geometry::PointCloud target;
    open3d::geometry::PointCloud target_with_normals;
};

}  // namespace fricp::internal
