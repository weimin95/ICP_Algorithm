#pragma once

#include <Eigen/Core>
#include <Eigen/QR>

#include <algorithm>
#include <cassert>

namespace fricp::internal {

class AndersonAcceleration {
public:
    AndersonAcceleration() = default;

    void Init(int history, int dimension, const double* initial_values) {
        assert(history > 0);
        history_ = history;
        dimension_ = dimension;
        current_u_ = Eigen::Map<const Eigen::VectorXd>(initial_values, dimension_);
        current_f_.resize(dimension_);
        previous_delta_g_.resize(dimension_, history_);
        previous_delta_f_.resize(dimension_, history_);
        normal_matrix_.resize(history_, history_);
        theta_.resize(history_);
        scales_.resize(history_);
        iteration_ = 0;
        column_index_ = 0;
    }

    void Reset(const double* values) {
        iteration_ = 0;
        column_index_ = 0;
        current_u_ = Eigen::Map<const Eigen::VectorXd>(values, dimension_);
    }

    void Replace(const double* values) {
        current_u_ = Eigen::Map<const Eigen::VectorXd>(values, dimension_);
    }

    const Eigen::VectorXd& Compute(const double* values) {
        assert(iteration_ >= 0);

        constexpr double kEpsilon = 1e-14;
        const Eigen::Map<const Eigen::VectorXd> g(values, dimension_);
        current_f_ = g - current_u_;

        if (iteration_ == 0) {
            previous_delta_f_.col(0) = -current_f_;
            previous_delta_g_.col(0) = -g;
            current_u_ = g;
        } else {
            previous_delta_f_.col(column_index_) += current_f_;
            previous_delta_g_.col(column_index_) += g;

            const double scale =
                    std::max(kEpsilon, previous_delta_f_.col(column_index_).norm());
            scales_(column_index_) = scale;
            previous_delta_f_.col(column_index_) /= scale;

            const int m_k = std::min(history_, iteration_);
            if (m_k == 1) {
                theta_(0) = 0.0;
                const double squared_norm =
                        previous_delta_f_.col(column_index_).squaredNorm();
                normal_matrix_(0, 0) = squared_norm;
                const double norm = std::sqrt(squared_norm);
                if (norm > kEpsilon) {
                    theta_(0) = (previous_delta_f_.col(column_index_) / norm)
                                        .dot(current_f_ / norm);
                }
            } else {
                const Eigen::VectorXd inner_products =
                        (previous_delta_f_.col(column_index_).transpose() *
                         previous_delta_f_.leftCols(m_k))
                                .transpose();
                normal_matrix_.block(column_index_, 0, 1, m_k) =
                        inner_products.transpose();
                normal_matrix_.block(0, column_index_, m_k, 1) = inner_products;
                decomposition_.compute(normal_matrix_.topLeftCorner(m_k, m_k));
                theta_.head(m_k) = decomposition_.solve(
                        previous_delta_f_.leftCols(m_k).transpose() * current_f_);
            }

            current_u_ = g - previous_delta_g_.leftCols(std::min(history_, iteration_)) *
                                     ((theta_.head(std::min(history_, iteration_)).array() /
                                       scales_.head(std::min(history_, iteration_)).array())
                                              .matrix());

            column_index_ = (column_index_ + 1) % history_;
            previous_delta_f_.col(column_index_) = -current_f_;
            previous_delta_g_.col(column_index_) = -g;
        }

        iteration_ += 1;
        return current_u_;
    }

private:
    int history_ = -1;
    int dimension_ = -1;
    int iteration_ = -1;
    int column_index_ = -1;

    Eigen::VectorXd current_u_;
    Eigen::VectorXd current_f_;
    Eigen::MatrixXd previous_delta_g_;
    Eigen::MatrixXd previous_delta_f_;
    Eigen::MatrixXd normal_matrix_;
    Eigen::VectorXd theta_;
    Eigen::VectorXd scales_;
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> decomposition_;
};

}  // namespace fricp::internal
