#include <fricp/internal/AndersonAcceleration.h>
#include <fricp/internal/FastRobustCore.h>

#include <open3d/geometry/KDTreeFlann.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace fricp::internal {
namespace {

using Matrix3X = Eigen::Matrix<double, 3, Eigen::Dynamic>;
using VectorX = Eigen::VectorXd;

Matrix3X PointCloudToMatrix(const open3d::geometry::PointCloud& cloud) {
    Matrix3X matrix(3, static_cast<Eigen::Index>(cloud.points_.size()));
    for (Eigen::Index i = 0; i < matrix.cols(); ++i) {
        matrix.col(i) = cloud.points_[static_cast<size_t>(i)];
    }
    return matrix;
}

Matrix3X TransformPoints(const Matrix3X& points, const Eigen::Matrix4d& transform) {
    return (transform.block<3, 3>(0, 0) * points).colwise() +
           transform.block<3, 1>(0, 3);
}

Eigen::Matrix3d Skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m = Eigen::Matrix3d::Zero();
    m(0, 1) = -v.z();
    m(0, 2) = v.y();
    m(1, 0) = v.z();
    m(1, 2) = -v.x();
    m(2, 0) = -v.y();
    m(2, 1) = v.x();
    return m;
}

Eigen::Matrix4d SE3Exp(const Eigen::Matrix<double, 6, 1>& xi) {
    const Eigen::Vector3d omega = xi.head<3>();
    const Eigen::Vector3d upsilon = xi.tail<3>();
    const double theta = omega.norm();
    const Eigen::Matrix3d omega_hat = Skew(omega);
    const Eigen::Matrix3d omega_hat_sq = omega_hat * omega_hat;

    double a = 1.0;
    double b = 0.5;
    double c = 1.0 / 6.0;
    if (theta > 1e-12) {
        a = std::sin(theta) / theta;
        b = (1.0 - std::cos(theta)) / (theta * theta);
        c = (theta - std::sin(theta)) / (theta * theta * theta);
    }

    const Eigen::Matrix3d rotation =
            Eigen::Matrix3d::Identity() + a * omega_hat + b * omega_hat_sq;
    const Eigen::Matrix3d v_matrix =
            Eigen::Matrix3d::Identity() + b * omega_hat + c * omega_hat_sq;

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = rotation;
    transform.block<3, 1>(0, 3) = v_matrix * upsilon;
    return transform;
}

Eigen::Matrix<double, 6, 1> SE3Log(const Eigen::Matrix4d& transform) {
    Eigen::Matrix<double, 6, 1> xi = Eigen::Matrix<double, 6, 1>::Zero();
    const Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
    const Eigen::Vector3d translation = transform.block<3, 1>(0, 3);

    Eigen::AngleAxisd angle_axis(rotation);
    const double theta = angle_axis.angle();
    Eigen::Vector3d omega = Eigen::Vector3d::Zero();
    if (theta > 1e-12) {
        omega = angle_axis.axis() * theta;
    }

    const Eigen::Matrix3d omega_hat = Skew(omega);
    const Eigen::Matrix3d omega_hat_sq = omega_hat * omega_hat;
    Eigen::Matrix3d v_inverse = Eigen::Matrix3d::Identity() - 0.5 * omega_hat;

    if (theta > 1e-12) {
        const double half_theta = 0.5 * theta;
        const double cot_half_theta = std::cos(half_theta) / std::sin(half_theta);
        v_inverse +=
                (1.0 - (theta * cot_half_theta) / 2.0) / (theta * theta) * omega_hat_sq;
    } else {
        v_inverse += (1.0 / 12.0) * omega_hat_sq;
    }

    xi.head<3>() = omega;
    xi.tail<3>() = v_inverse * translation;
    return xi;
}

Eigen::Matrix4d ComputeWeightedTransform(const Matrix3X& source,
                                         const Matrix3X& target,
                                         const VectorX& weights) {
    const double weight_sum = weights.sum();
    if (weight_sum <= 0.0) {
        return Eigen::Matrix4d::Identity();
    }

    const VectorX normalized = weights / weight_sum;
    const Eigen::Vector3d source_mean = source * normalized;
    const Eigen::Vector3d target_mean = target * normalized;

    Matrix3X centered_source = source.colwise() - source_mean;
    Matrix3X centered_target = target.colwise() - target_mean;
    const Eigen::Matrix3d sigma =
            centered_source * normalized.asDiagonal() * centered_target.transpose();
    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
            sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix3d rotation = svd.matrixV() * svd.matrixU().transpose();
    if (rotation.determinant() < 0.0) {
        Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
        correction(2, 2) = -1.0;
        rotation = svd.matrixV() * correction * svd.matrixU().transpose();
    }

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = rotation;
    transform.block<3, 1>(0, 3) = target_mean - rotation * source_mean;
    return transform;
}

double ComputeEnergy(const Matrix3X& source, const Matrix3X& target) {
    return (source - target).colwise().squaredNorm().mean();
}

double Median(std::vector<double> values) {
    if (values.empty()) {
        return 0.0;
    }
    const size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    double median = values[middle];
    if (values.size() % 2 == 0) {
        std::nth_element(values.begin(), values.begin() + middle - 1, values.end());
        median = 0.5 * (median + values[middle - 1]);
    }
    return median;
}

double Median(const VectorX& values) {
    std::vector<double> data(static_cast<size_t>(values.size()));
    for (Eigen::Index i = 0; i < values.size(); ++i) {
        data[static_cast<size_t>(i)] = values[i];
    }
    return Median(std::move(data));
}

double FindKNearestMedian(const open3d::geometry::PointCloud& cloud, int nk) {
    open3d::geometry::KDTreeFlann tree(cloud);
    std::vector<double> medians;
    medians.reserve(cloud.points_.size());
    std::vector<int> indices(static_cast<size_t>(nk));
    std::vector<double> distances(static_cast<size_t>(nk));

    for (const auto& point : cloud.points_) {
        const int found = tree.SearchKNN(point, nk, indices, distances);
        if (found <= 1) {
            medians.push_back(0.0);
            continue;
        }

        std::vector<double> local;
        local.reserve(static_cast<size_t>(found - 1));
        for (int i = 1; i < found; ++i) {
            local.push_back(std::sqrt(distances[static_cast<size_t>(i)]));
        }
        medians.push_back(Median(std::move(local)));
    }

    return Median(std::move(medians));
}

double WelschEnergy(const VectorX& residuals, double nu) {
    if (nu <= 0.0) {
        return residuals.squaredNorm();
    }
    double energy = 0.0;
    for (Eigen::Index i = 0; i < residuals.size(); ++i) {
        const double ratio = residuals[i] / nu;
        energy += 1.0 - std::exp(-(ratio * ratio) / 2.0);
    }
    return energy;
}

VectorX WelschWeights(const VectorX& residuals, double nu) {
    VectorX weights(residuals.size());
    if (nu <= 0.0) {
        weights.setOnes();
        return weights;
    }
    for (Eigen::Index i = 0; i < residuals.size(); ++i) {
        const double ratio = residuals[i] / nu;
        weights[i] = std::exp(-(ratio * ratio) / 2.0);
    }
    return weights;
}

}  // namespace

RobustResult RegisterRobustPointToPoint(
        const open3d::geometry::PointCloud& source,
        const open3d::geometry::PointCloud& target,
        const Eigen::Matrix4d& initial_transform,
        bool use_initial_transform,
        const RobustOptions& options) {
    RobustResult result;

    if (source.points_.empty() || target.points_.empty()) {
        result.message = "source and target point clouds must not be empty";
        return result;
    }

    Matrix3X source_matrix = PointCloudToMatrix(source);
    const Matrix3X target_matrix = PointCloudToMatrix(target);
    const Matrix3X original_source = source_matrix;

    open3d::geometry::KDTreeFlann target_tree(target);
    Eigen::Matrix4d total_transform =
            use_initial_transform ? initial_transform : Eigen::Matrix4d::Identity();
    source_matrix = TransformPoints(source_matrix, total_transform);

    Matrix3X correspondences(3, source_matrix.cols());
    VectorX weights = VectorX::Ones(source_matrix.cols());
    VectorX residuals(source_matrix.cols());
    std::vector<int> indices(1);
    std::vector<double> distances(1);
    AndersonAcceleration accelerator;
    double previous_energy = std::numeric_limits<double>::max();

    auto update_correspondences = [&]() {
        for (Eigen::Index i = 0; i < source_matrix.cols(); ++i) {
            const Eigen::Vector3d query = source_matrix.col(i);
            target_tree.SearchKNN(query, 1, indices, distances);
            correspondences.col(i) = target.points_[static_cast<size_t>(indices[0])];
            residuals[i] = std::sqrt(distances[0]);
        }
    };

    update_correspondences();

    const double nu_target = options.nu_end_k * FindKNearestMedian(target, 7);
    double nu = std::max(options.nu_begin_k * Median(residuals), nu_target);
    if (nu <= 0.0) {
        nu = 1.0;
    }
    if (options.use_anderson) {
        const Eigen::Matrix<double, 6, 1> log_transform = SE3Log(total_transform);
        accelerator.Init(5, 6, log_transform.data());
    }

    bool done = false;
    while (!done) {
        for (int iter = 0; iter < options.max_iteration; ++iter) {
            const double energy = WelschEnergy(residuals, nu);
            weights = WelschWeights(residuals, nu);

            const Eigen::Matrix4d delta =
                    ComputeWeightedTransform(source_matrix, correspondences, weights);
            const Eigen::Matrix4d svd_transform = delta * total_transform;

            if (options.use_anderson) {
                const Eigen::Matrix<double, 6, 1> log_svd_transform =
                        SE3Log(svd_transform);
                if (energy < previous_energy) {
                    previous_energy = energy;
                    const Eigen::VectorXd accelerated_log =
                            accelerator.Compute(log_svd_transform.data());
                    total_transform = SE3Exp(
                            Eigen::Map<const Eigen::Matrix<double, 6, 1>>(
                                    accelerated_log.data()));
                } else {
                    accelerator.Replace(log_svd_transform.data());
                    total_transform = svd_transform;
                    previous_energy = energy;
                }
            } else {
                total_transform = svd_transform;
            }

            source_matrix = TransformPoints(original_source, total_transform);

            update_correspondences();
            result.iteration_count += 1;

            if ((delta - Eigen::Matrix4d::Identity()).norm() < options.stop) {
                break;
            }
        }

        if (std::abs(nu - nu_target) < options.stop) {
            done = true;
        } else {
            nu = std::max(nu * options.nu_alpha, nu_target);
            if (options.use_anderson) {
                const Eigen::Matrix<double, 6, 1> log_transform = SE3Log(total_transform);
                accelerator.Reset(log_transform.data());
                previous_energy = std::numeric_limits<double>::max();
            }
        }
    }

    result.success = true;
    result.transformation = total_transform;
    result.convergence_energy = WelschEnergy(residuals, nu);
    result.message = "ok";
    return result;
}

}  // namespace fricp::internal
