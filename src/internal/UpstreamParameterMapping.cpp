#include "internal/UpstreamParameterMapping.h"

namespace fricp::internal {
namespace {

constexpr double kUnifiedDefaultNuEndK = 0.19245008972987526;
constexpr double kUpstreamPointToPlaneNuEndK = 1.0 / 6.0;

IcpParameterView DescribeIcpParametersImpl(const RegistrationOptions& options) {
    IcpParameterView view;
    view.p = options.p;
    view.max_icp = options.max_icp;
    view.max_outer = options.max_outer;
    view.stop = options.stop;
    view.anderson_m = options.anderson_m;
    view.beta = options.beta;
    view.error_overflow_threshold = options.error_overflow_threshold;
    view.nu_begin_k = options.nu_begin_k;
    view.nu_end_k = options.nu_end_k;
    view.nu_alpha = options.nu_alpha;
    view.use_init = options.use_initial_transform;
    view.init_trans = options.initial_transform;

    switch (options.method) {
        case RegistrationMethod::ICP:
        case RegistrationMethod::PointToPlane:
        case RegistrationMethod::SparseICP:
        case RegistrationMethod::SparsePointToPlane:
            view.f = IcpRobustFunction::None;
            view.use_AA = false;
            break;
        case RegistrationMethod::AAICP:
            view.f = IcpRobustFunction::None;
            view.use_AA = false;
            break;
        case RegistrationMethod::FastICP:
            view.f = IcpRobustFunction::None;
            view.use_AA = true;
            break;
        case RegistrationMethod::RobustICP:
            view.f = IcpRobustFunction::Welsch;
            view.use_AA = true;
            break;
        case RegistrationMethod::RobustPointToPlane:
            view.f = IcpRobustFunction::Welsch;
            view.use_AA = true;
            if (view.nu_end_k == kUnifiedDefaultNuEndK) {
                view.nu_end_k = kUpstreamPointToPlaneNuEndK;
            }
            break;
    }

    return view;
}

}  // namespace

IcpParameterView DescribeIcpParameters(const RegistrationOptions& options) {
    return DescribeIcpParametersImpl(options);
}

SicpParameterView DescribeSicpParameters(const RegistrationOptions& options) {
    SicpParameterView view;
    view.use_penalty = options.sicp_use_penalty;
    view.p = options.sicp_p;
    view.mu = options.sicp_mu;
    view.alpha = options.sicp_alpha;
    view.max_mu = options.sicp_max_mu;
    view.max_icp = options.sicp_max_icp;
    view.max_outer = options.sicp_max_outer;
    view.max_inner = options.sicp_max_inner;
    view.stop = options.stop;
    if (options.use_initial_transform) {
        view.init_trans = options.initial_transform;
    }
    return view;
}

}  // namespace fricp::internal
