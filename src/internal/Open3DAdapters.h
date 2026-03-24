#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <open3d/geometry/PointCloud.h>

namespace fricp::internal {

using Matrix3X = Eigen::Matrix<double, 3, Eigen::Dynamic>;

struct BoundingBoxStatistics {
    Eigen::Vector3d min_bound = Eigen::Vector3d::Zero();
    Eigen::Vector3d max_bound = Eigen::Vector3d::Zero();
    Eigen::Vector3d extent = Eigen::Vector3d::Zero();
    double diagonal_norm = 0.0;
};

struct SharedNormalization {
    double scale = 1.0;
    BoundingBoxStatistics source_bbox;
    BoundingBoxStatistics target_bbox;
    Eigen::Vector3d source_mean = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_mean = Eigen::Vector3d::Zero();
};

Matrix3X PointCloudToMatrix(const open3d::geometry::PointCloud& cloud);
Matrix3X NormalsToMatrix(const open3d::geometry::PointCloud& cloud);

BoundingBoxStatistics ComputeBoundingBoxStatistics(const Matrix3X& points);
BoundingBoxStatistics ComputeBoundingBoxStatistics(
        const open3d::geometry::PointCloud& cloud);

SharedNormalization NormalizeSharedSourceTarget(Matrix3X& source,
                                                Matrix3X& target);
SharedNormalization NormalizeSharedSourceTarget(
        const open3d::geometry::PointCloud& source,
        const open3d::geometry::PointCloud& target,
        Matrix3X& normalized_source,
        Matrix3X& normalized_target);

Eigen::Matrix4d AdaptInitialTransform(
        const Eigen::Matrix4d& initial_transform,
        const SharedNormalization& normalization);

}  // namespace fricp::internal
