#pragma once

#include <fricp/Types.h>

#include <open3d/geometry/PointCloud.h>

namespace fricp::internal {

struct TrainedData {
    bool is_trained = false;
    RegistrationMethod method = RegistrationMethod::ICP;
    RegistrationOptions options;
    open3d::geometry::PointCloud target;
    open3d::geometry::PointCloud cached_target_normals;
};

}  // namespace fricp::internal
