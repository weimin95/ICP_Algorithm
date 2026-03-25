#pragma once

#include <fricp/Types.h>

#include <internal/Open3DAdapters.h>

#include <open3d/geometry/PointCloud.h>

namespace fricp::internal {

struct TrainedData {
    bool is_trained = false;
    RegistrationMethod method = RegistrationMethod::ICP;
    RegistrationOptions options;
    open3d::geometry::PointCloud target;
    open3d::geometry::PointCloud cached_target_normals;
    Matrix3X target_matrix;
    Matrix3X target_normals_matrix;
    BoundingBoxStatistics target_bbox;
};

}  // namespace fricp::internal
