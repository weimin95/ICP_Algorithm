#pragma once

#include <fricp/Export.h>
#include <fricp/Types.h>

namespace ICP {
class Parameters;
}  // namespace ICP

namespace SICP {
struct Parameters;
}  // namespace SICP

namespace fricp::internal {

struct RobustOptions;

FRICP_API ICP::Parameters BuildIcpParameters(const RegistrationOptions& options);
FRICP_API SICP::Parameters BuildSicpParameters(const RegistrationOptions& options);
FRICP_API RobustOptions BuildLegacyRobustOptions(const RegistrationOptions& options);

}  // namespace fricp::internal
