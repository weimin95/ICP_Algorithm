#pragma once

#include <fricp/Export.h>
#include <fricp/Types.h>
#include <open3d/geometry/PointCloud.h>

namespace fricp {

class FRICP_API FastRobustIcp {
public:
    FastRobustIcp();
    ~FastRobustIcp();

    bool Train(const open3d::geometry::PointCloud& target,
               const RegistrationOptions& options);
    void ClearTraining();
    bool IsTrained() const;

    bool Register(const open3d::geometry::PointCloud& source,
                  const RegistrationOptions& options,
                  RegistrationResult& result) const;

    const std::string& GetLastError() const;

private:
    mutable std::string last_error_;
    open3d::geometry::PointCloud trained_target_;
    RegistrationOptions trained_options_;
    bool is_trained_ = false;
};

}  // namespace fricp
