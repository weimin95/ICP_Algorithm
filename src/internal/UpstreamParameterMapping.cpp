#include "internal/UpstreamParameterMapping.h"

#include <fricp/internal/FastRobustCore.h>

#include <ICP.h>

namespace fricp::internal {
namespace {

constexpr double kUnifiedDefaultNuEndK = 0.19245008972987526;
constexpr double kUpstreamPointToPlaneNuEndK = 1.0 / 6.0;

}  // namespace

ICP::Parameters BuildIcpParameters(const RegistrationOptions& options) {
    ICP::Parameters parameters;
    parameters.p = options.p;
    parameters.max_icp = options.max_icp;
    parameters.max_outer = options.max_outer;
    parameters.stop = options.stop;
    parameters.anderson_m = options.anderson_m;
    parameters.beta_ = options.beta;
    parameters.error_overflow_threshold_ = options.error_overflow_threshold;
    parameters.nu_begin_k = options.nu_begin_k;
    parameters.nu_end_k = options.nu_end_k;
    parameters.nu_alpha = options.nu_alpha;
    parameters.use_init = options.use_initial_transform;
    parameters.init_trans = options.initial_transform;

    switch (options.method) {
        case RegistrationMethod::ICP:
        case RegistrationMethod::PointToPlane:
        case RegistrationMethod::SparseICP:
        case RegistrationMethod::SparsePointToPlane:
            parameters.f = ICP::NONE;
            parameters.use_AA = false;
            break;
        case RegistrationMethod::AAICP:
            parameters.f = ICP::NONE;
            parameters.use_AA = false;
            break;
        case RegistrationMethod::FastICP:
            parameters.f = ICP::NONE;
            parameters.use_AA = true;
            break;
        case RegistrationMethod::RobustICP:
            parameters.f = ICP::WELSCH;
            parameters.use_AA = true;
            break;
        case RegistrationMethod::RobustPointToPlane:
            parameters.f = ICP::WELSCH;
            parameters.use_AA = true;
            if (parameters.nu_end_k == kUnifiedDefaultNuEndK) {
                parameters.nu_end_k = kUpstreamPointToPlaneNuEndK;
            }
            break;
    }

    return parameters;
}

SICP::Parameters BuildSicpParameters(const RegistrationOptions& options) {
    SICP::Parameters parameters;
    parameters.use_penalty = options.sicp_use_penalty;
    parameters.p = options.sicp_p;
    parameters.mu = options.sicp_mu;
    parameters.alpha = options.sicp_alpha;
    parameters.max_mu = options.sicp_max_mu;
    parameters.max_icp = options.sicp_max_icp;
    parameters.max_outer = options.sicp_max_outer;
    parameters.max_inner = options.sicp_max_inner;
    parameters.stop = options.stop;
    parameters.init_trans = options.initial_transform;
    return parameters;
}

RobustOptions BuildLegacyRobustOptions(const RegistrationOptions& options) {
    const auto icp_parameters = BuildIcpParameters(options);
    RobustOptions robust_options;
    robust_options.max_iteration = icp_parameters.max_icp;
    robust_options.nu_begin_k = icp_parameters.nu_begin_k;
    robust_options.nu_end_k = icp_parameters.nu_end_k;
    robust_options.nu_alpha = icp_parameters.nu_alpha;
    robust_options.stop = icp_parameters.stop;
    robust_options.use_anderson = icp_parameters.use_AA;
    return robust_options;
}

}  // namespace fricp::internal
