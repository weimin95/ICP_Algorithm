#pragma once

#include <fricp/Export.h>
#include <fricp/Types.h>
#include <fricp/internal/TrainedData.h>
#include <open3d/geometry/PointCloud.h>

#include <memory>

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
    std::unique_ptr<internal::TrainedData> trained_data_;
};

}  // namespace fricp
