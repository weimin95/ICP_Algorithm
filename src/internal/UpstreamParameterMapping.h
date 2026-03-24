#pragma once

#include <fricp/Types.h>

namespace fricp::internal {

enum class IcpRobustFunction {
    None,
    Welsch,
};

struct IcpParameterView {
    double p = 0.0;
    int max_icp = 0;
    int max_outer = 0;
    double stop = 0.0;
    int anderson_m = 0;
    double beta = 0.0;
    double error_overflow_threshold = 0.0;
    double nu_begin_k = 0.0;
    double nu_end_k = 0.0;
    double nu_alpha = 0.0;
    bool use_init = false;
    Eigen::Matrix4d init_trans = Eigen::Matrix4d::Identity();
    IcpRobustFunction f = IcpRobustFunction::None;
    bool use_AA = false;
};

struct SicpParameterView {
    bool use_penalty = false;
    double p = 0.0;
    double mu = 0.0;
    double alpha = 0.0;
    double max_mu = 0.0;
    int max_icp = 0;
    int max_outer = 0;
    int max_inner = 0;
    double stop = 0.0;
    Eigen::Matrix4d init_trans = Eigen::Matrix4d::Identity();
};

IcpParameterView DescribeIcpParameters(const RegistrationOptions& options);
SicpParameterView DescribeSicpParameters(const RegistrationOptions& options);

}  // namespace fricp::internal
