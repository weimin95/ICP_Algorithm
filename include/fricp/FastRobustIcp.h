#pragma once

#include <fricp/Export.h>
#include <fricp/Types.h>
#include <open3d/geometry/PointCloud.h>

#include <memory>

namespace fricp {

namespace internal {
struct TrainedData;
}

class FRICP_API FastRobustIcp {
public:
    FastRobustIcp();
    ~FastRobustIcp();

    FastRobustIcp(const FastRobustIcp&) = delete;
    FastRobustIcp& operator=(const FastRobustIcp&) = delete;
    FastRobustIcp(FastRobustIcp&&) noexcept;
    FastRobustIcp& operator=(FastRobustIcp&&) noexcept;

    // Cache a target cloud for repeated registrations with one configured method.
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
