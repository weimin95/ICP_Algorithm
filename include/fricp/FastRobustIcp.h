#pragma once

#include <fricp/Export.h>
#include <fricp/Types.h>
#include <open3d/geometry/PointCloud.h>

namespace fricp {

class FRICP_API FastRobustIcp {
public:
    FastRobustIcp();
    ~FastRobustIcp();

    bool Register(const open3d::geometry::PointCloud& source,
                  const open3d::geometry::PointCloud& target,
                  const RegistrationOptions& options,
                  RegistrationResult& result) const;

    const std::string& GetLastError() const;

private:
    mutable std::string last_error_;
};

}  // namespace fricp
