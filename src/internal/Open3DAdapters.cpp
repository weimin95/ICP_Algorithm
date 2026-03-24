#include "internal/Open3DAdapters.h"

#include <algorithm>

namespace fricp::internal {
namespace {

BoundingBoxStatistics ComputeBoundingBoxStatisticsFromPoints(
        const Matrix3X& points) {
    BoundingBoxStatistics stats;
    if (points.cols() == 0) {
        return stats;
    }

    stats.min_bound = points.rowwise().minCoeff();
    stats.max_bound = points.rowwise().maxCoeff();
    stats.extent = stats.max_bound - stats.min_bound;
    stats.diagonal_norm = stats.extent.norm();
    return stats;
}

}  // namespace

Matrix3X PointCloudToMatrix(const open3d::geometry::PointCloud& cloud) {
    Matrix3X matrix(3, static_cast<Eigen::Index>(cloud.points_.size()));
    for (Eigen::Index i = 0; i < matrix.cols(); ++i) {
        matrix.col(i) = cloud.points_[static_cast<size_t>(i)];
    }
    return matrix;
}

Matrix3X NormalsToMatrix(const open3d::geometry::PointCloud& cloud) {
    Matrix3X matrix(3, static_cast<Eigen::Index>(cloud.normals_.size()));
    for (Eigen::Index i = 0; i < matrix.cols(); ++i) {
        matrix.col(i) = cloud.normals_[static_cast<size_t>(i)];
    }
    return matrix;
}

BoundingBoxStatistics ComputeBoundingBoxStatistics(const Matrix3X& points) {
    return ComputeBoundingBoxStatisticsFromPoints(points);
}

BoundingBoxStatistics ComputeBoundingBoxStatistics(
        const open3d::geometry::PointCloud& cloud) {
    return ComputeBoundingBoxStatistics(PointCloudToMatrix(cloud));
}

SharedNormalization NormalizeSharedSourceTarget(Matrix3X& source,
                                                Matrix3X& target) {
    SharedNormalization normalization;
    normalization.source_bbox = ComputeBoundingBoxStatistics(source);
    normalization.target_bbox = ComputeBoundingBoxStatistics(target);
    normalization.scale = std::max(normalization.source_bbox.diagonal_norm,
                                   normalization.target_bbox.diagonal_norm);
    if (normalization.scale <= 0.0) {
        normalization.scale = 1.0;
    }

    if (source.cols() == 0 || target.cols() == 0) {
        return normalization;
    }

    source /= normalization.scale;
    target /= normalization.scale;

    normalization.source_mean = source.rowwise().sum() /
                                static_cast<double>(source.cols());
    normalization.target_mean = target.rowwise().sum() /
                                static_cast<double>(target.cols());

    source.colwise() -= normalization.source_mean;
    target.colwise() -= normalization.target_mean;
    return normalization;
}

SharedNormalization NormalizeSharedSourceTarget(
        const open3d::geometry::PointCloud& source,
        const open3d::geometry::PointCloud& target,
        Matrix3X& normalized_source,
        Matrix3X& normalized_target) {
    normalized_source = PointCloudToMatrix(source);
    normalized_target = PointCloudToMatrix(target);
    return NormalizeSharedSourceTarget(normalized_source, normalized_target);
}

Eigen::Matrix4d AdaptInitialTransform(
        const Eigen::Matrix4d& initial_transform,
        const SharedNormalization& normalization) {
    Eigen::Matrix4d adapted = initial_transform;
    adapted.block<3, 1>(0, 3) /= normalization.scale;
    adapted.block<3, 1>(0, 3) +=
            adapted.block<3, 3>(0, 0) * normalization.source_mean -
            normalization.target_mean;
    return adapted;
}

}  // namespace fricp::internal
