# FRICP DLL Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Build a Windows C++ DLL named `fricp` that exposes a C++ class API for ICP registration over `open3d::geometry::PointCloud`, with `PointToPoint`, `PointToPlane`, and `RobustPointToPoint` modes.

**Architecture:** The DLL exports a thin public class in `include/fricp/FastRobustIcp.h`. Standard ICP modes delegate to Open3D's registration API. Robust point-to-point mode adapts the upstream Fast-Robust-ICP point-to-point solver into an internal in-memory Eigen-based core. Tests use synthetic point clouds and run through CTest.

**Tech Stack:** C++17, CMake, Open3D C++ API, Eigen, Windows DLL exports, CTest

---

### Task 1: Bootstrap The DLL Build And Test Harness

**Files:**
- Create: `d:\proj\git\fricp\CMakeLists.txt`
- Create: `d:\proj\git\fricp\include\fricp\Export.h`
- Create: `d:\proj\git\fricp\include\fricp\Types.h`
- Create: `d:\proj\git\fricp\include\fricp\FastRobustIcp.h`
- Create: `d:\proj\git\fricp\src\FastRobustIcp.cpp`
- Create: `d:\proj\git\fricp\tests\FastRobustIcpTests.cpp`

**Step 1: Write the failing test**

Create a smoke test that expects the public API to be constructible and to reject empty inputs.

```cpp
#include <fricp/FastRobustIcp.h>
#include <iostream>

int main() {
    fricp::FastRobustIcp icp;
    open3d::geometry::PointCloud source;
    open3d::geometry::PointCloud target;
    fricp::RegistrationOptions options;
    fricp::RegistrationResult result;

    const bool ok = icp.Register(source, target, options, result);
    if (ok) {
        std::cerr << "expected empty-point-cloud validation failure\n";
        return 1;
    }
    if (result.message.empty()) {
        std::cerr << "expected a descriptive error message\n";
        return 1;
    }
    return 0;
}
```

**Step 2: Run test to verify it fails**

Run:

```powershell
cmake -S . -B build
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

Expected:

- configure may fail because `CMakeLists.txt` does not exist yet, or
- build fails because `fricp/FastRobustIcp.h` and implementation are missing

**Step 3: Write minimal implementation**

Create a minimal DLL project and stub API.

`CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.20)
project(fricp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Open3D REQUIRED)

add_library(fricp SHARED
    src/FastRobustIcp.cpp
)

target_include_directories(fricp
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(fricp
    PUBLIC
        Open3D::Open3D
)

target_compile_definitions(fricp
    PRIVATE FRICP_BUILD_DLL
)

add_executable(fricp_tests
    tests/FastRobustIcpTests.cpp
)

target_link_libraries(fricp_tests
    PRIVATE
        fricp
)

enable_testing()
add_test(NAME fricp_tests COMMAND fricp_tests)
```

`include/fricp/Export.h`

```cpp
#pragma once

#if defined(_WIN32)
#  if defined(FRICP_BUILD_DLL)
#    define FRICP_API __declspec(dllexport)
#  else
#    define FRICP_API __declspec(dllimport)
#  endif
#else
#  define FRICP_API
#endif
```

`include/fricp/Types.h`

```cpp
#pragma once

#include <Eigen/Core>
#include <string>

namespace fricp {

enum class RegistrationMode {
    PointToPoint,
    PointToPlane,
    RobustPointToPoint
};

struct RegistrationOptions {
    RegistrationMode mode = RegistrationMode::RobustPointToPoint;
    double max_correspondence_distance = 0.05;
    int max_iteration = 50;
    double relative_fitness = 1e-6;
    double relative_rmse = 1e-6;
    bool use_initial_transform = false;
    Eigen::Matrix4d initial_transform = Eigen::Matrix4d::Identity();
    bool estimate_target_normals_if_missing = true;
    double normal_radius = 0.1;
    int normal_knn = 30;
    bool robust_use_anderson = true;
    double robust_nu_begin_k = 3.0;
    double robust_nu_end_k = 1.0 / 6.0;
    double robust_nu_alpha = 0.5;
    double robust_stop = 1e-5;
};

struct RegistrationResult {
    bool success = false;
    RegistrationMode mode = RegistrationMode::RobustPointToPoint;
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    double fitness = 0.0;
    double inlier_rmse = 0.0;
    double convergence_energy = 0.0;
    int iteration_count = 0;
    std::string message;
};

}  // namespace fricp
```

`include/fricp/FastRobustIcp.h`

```cpp
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
```

`src/FastRobustIcp.cpp`

```cpp
#include <fricp/FastRobustIcp.h>

namespace fricp {

FastRobustIcp::FastRobustIcp() = default;
FastRobustIcp::~FastRobustIcp() = default;

bool FastRobustIcp::Register(const open3d::geometry::PointCloud& source,
                             const open3d::geometry::PointCloud& target,
                             const RegistrationOptions& options,
                             RegistrationResult& result) const {
    (void)options;
    result = RegistrationResult{};
    if (source.points_.empty() || target.points_.empty()) {
        last_error_ = "source and target point clouds must not be empty";
        result.message = last_error_;
        return false;
    }
    last_error_.clear();
    result.success = true;
    return true;
}

const std::string& FastRobustIcp::GetLastError() const { return last_error_; }

}  // namespace fricp
```

**Step 4: Run test to verify it passes**

Run:

```powershell
cmake -S . -B build
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure -R fricp_tests
```

Expected:

- configure succeeds
- `fricp` DLL and `fricp_tests` build
- smoke test passes

**Step 5: Commit**

If this workspace is initialized as git:

```powershell
git add CMakeLists.txt include src tests
git commit -m "build: bootstrap fricp dll project"
```

### Task 2: Add Input Validation Behavior

**Files:**
- Modify: `d:\proj\git\fricp\tests\FastRobustIcpTests.cpp`
- Modify: `d:\proj\git\fricp\src\FastRobustIcp.cpp`

**Step 1: Write the failing test**

Extend the test executable with explicit invalid-option cases.

```cpp
static int TestRejectsInvalidDistance() {
    fricp::FastRobustIcp icp;
    open3d::geometry::PointCloud source;
    open3d::geometry::PointCloud target;
    source.points_.push_back({0.0, 0.0, 0.0});
    target.points_.push_back({0.0, 0.0, 0.0});

    fricp::RegistrationOptions options;
    options.max_correspondence_distance = 0.0;
    fricp::RegistrationResult result;

    if (icp.Register(source, target, options, result)) return 1;
    return result.message.find("max_correspondence_distance") == std::string::npos;
}

static int TestRejectsPointToPlaneWithoutNormalsWhenDisabled() {
    fricp::FastRobustIcp icp;
    open3d::geometry::PointCloud source;
    open3d::geometry::PointCloud target;
    source.points_.push_back({0.0, 0.0, 0.0});
    target.points_.push_back({0.0, 0.0, 0.0});

    fricp::RegistrationOptions options;
    options.mode = fricp::RegistrationMode::PointToPlane;
    options.estimate_target_normals_if_missing = false;
    fricp::RegistrationResult result;

    if (icp.Register(source, target, options, result)) return 1;
    return result.message.find("normals") == std::string::npos;
}
```

Update `main()` to run all test functions and return non-zero on the first failure.

**Step 2: Run test to verify it fails**

Run:

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure -R fricp_tests
```

Expected:

- tests fail because `Register` only checks empty inputs

**Step 3: Write minimal implementation**

Add validation before dispatching any registration logic.

```cpp
if (options.max_correspondence_distance <= 0.0) {
    last_error_ = "max_correspondence_distance must be > 0";
    result.message = last_error_;
    return false;
}

if (options.mode == RegistrationMode::PointToPlane &&
    target.normals_.empty() &&
    !options.estimate_target_normals_if_missing) {
    last_error_ = "target point cloud normals are required for point-to-plane mode";
    result.message = last_error_;
    return false;
}
```

**Step 4: Run test to verify it passes**

Run the same `ctest` command and expect all validation tests to pass.

**Step 5: Commit**

```powershell
git add src/FastRobustIcp.cpp tests/FastRobustIcpTests.cpp
git commit -m "test: add input validation for registration api"
```

### Task 3: Implement Open3D Point-To-Point Mode

**Files:**
- Modify: `d:\proj\git\fricp\tests\FastRobustIcpTests.cpp`
- Modify: `d:\proj\git\fricp\src\FastRobustIcp.cpp`

**Step 1: Write the failing test**

Add a deterministic point-to-point registration test with a known transform.

```cpp
static open3d::geometry::PointCloud MakeCubeCloud() {
    open3d::geometry::PointCloud cloud;
    cloud.points_ = {
        {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0},
        {1.0, 1.0, 0.0}, {1.0, 0.0, 1.0}, {0.0, 1.0, 1.0}, {1.0, 1.0, 1.0}
    };
    return cloud;
}

static Eigen::Matrix4d MakeKnownTransform() {
    Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
    t.block<3,3>(0,0) =
        Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    t.block<3,1>(0,3) = Eigen::Vector3d(0.2, -0.1, 0.05);
    return t;
}

static int TestPointToPointRecoversKnownTransform() {
    auto target = MakeCubeCloud();
    auto source = target;
    source.Transform(MakeKnownTransform());

    fricp::FastRobustIcp icp;
    fricp::RegistrationOptions options;
    options.mode = fricp::RegistrationMode::PointToPoint;
    options.max_correspondence_distance = 2.0;
    options.max_iteration = 100;
    fricp::RegistrationResult result;

    if (!icp.Register(source, target, options, result)) return 1;
    const Eigen::Matrix4d expected = MakeKnownTransform().inverse();
    return ((result.transformation - expected).cwiseAbs().maxCoeff() > 1e-2) ? 1 : 0;
}
```

**Step 2: Run test to verify it fails**

Run the test suite and expect the new test to fail because point-to-point still
returns the default identity transform.

**Step 3: Write minimal implementation**

Use Open3D's ICP API for point-to-point mode.

```cpp
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/pipelines/registration/TransformationEstimation.h>

using namespace open3d::pipelines::registration;

ICPConvergenceCriteria criteria(
    options.relative_fitness,
    options.relative_rmse,
    options.max_iteration);

const Eigen::Matrix4d init =
    options.use_initial_transform ? options.initial_transform
                                  : Eigen::Matrix4d::Identity();

RegistrationResult o3d_result = RegistrationICP(
    source,
    target,
    options.max_correspondence_distance,
    init,
    TransformationEstimationPointToPoint(false),
    criteria);
```

Copy the result into the DLL-facing structure.

```cpp
result.success = true;
result.mode = options.mode;
result.transformation = o3d_result.transformation_;
result.fitness = o3d_result.fitness_;
result.inlier_rmse = o3d_result.inlier_rmse_;
result.iteration_count = options.max_iteration;
result.message = "ok";
```

**Step 4: Run test to verify it passes**

Run `ctest` again and expect point-to-point to pass.

**Step 5: Commit**

```powershell
git add src/FastRobustIcp.cpp tests/FastRobustIcpTests.cpp
git commit -m "feat: implement point-to-point registration"
```

### Task 4: Implement Open3D Point-To-Plane Mode

**Files:**
- Modify: `d:\proj\git\fricp\tests\FastRobustIcpTests.cpp`
- Modify: `d:\proj\git\fricp\src\FastRobustIcp.cpp`

**Step 1: Write the failing test**

Add a planar cloud test that requires target-normal estimation.

```cpp
static open3d::geometry::PointCloud MakePlaneCloud() {
    open3d::geometry::PointCloud cloud;
    for (int x = -2; x <= 2; ++x) {
        for (int y = -2; y <= 2; ++y) {
            cloud.points_.push_back(Eigen::Vector3d(0.1 * x, 0.1 * y, 0.0));
        }
    }
    return cloud;
}

static int TestPointToPlaneEstimatesTargetNormals() {
    auto target = MakePlaneCloud();
    auto source = target;

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3,3>(0,0) =
        Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitY()).toRotationMatrix();
    transform.block<3,1>(0,3) = Eigen::Vector3d(0.01, -0.02, 0.03);
    source.Transform(transform);

    fricp::FastRobustIcp icp;
    fricp::RegistrationOptions options;
    options.mode = fricp::RegistrationMode::PointToPlane;
    options.max_correspondence_distance = 0.5;
    options.max_iteration = 100;
    options.estimate_target_normals_if_missing = true;
    options.normal_radius = 0.3;
    options.normal_knn = 10;
    fricp::RegistrationResult result;

    if (!icp.Register(source, target, options, result)) return 1;
    return (result.fitness <= 0.5) ? 1 : 0;
}
```

**Step 2: Run test to verify it fails**

Run `ctest` and expect failure because point-to-plane dispatch is not yet
implemented.

**Step 3: Write minimal implementation**

Estimate target normals when required, then call Open3D point-to-plane ICP.

```cpp
open3d::geometry::PointCloud target_copy = target;
if (target_copy.normals_.empty() && options.estimate_target_normals_if_missing) {
    target_copy.EstimateNormals(
        open3d::geometry::KDTreeSearchParamHybrid(
            options.normal_radius, options.normal_knn));
}

RegistrationResult o3d_result = RegistrationICP(
    source,
    target_copy,
    options.max_correspondence_distance,
    init,
    TransformationEstimationPointToPlane(),
    criteria);
```

Write the same result fields as in Task 3.

**Step 4: Run test to verify it passes**

Run the test suite and expect the new point-to-plane case to pass.

**Step 5: Commit**

```powershell
git add src/FastRobustIcp.cpp tests/FastRobustIcpTests.cpp
git commit -m "feat: implement point-to-plane registration"
```

### Task 5: Introduce Internal Robust Solver Types And Point Conversion

**Files:**
- Create: `d:\proj\git\fricp\src\internal\AndersonAcceleration.h`
- Create: `d:\proj\git\fricp\src\internal\NanoFlannAdaptor.h`
- Create: `d:\proj\git\fricp\src\internal\RobustKernels.h`
- Create: `d:\proj\git\fricp\src\internal\FastRobustCore.h`
- Create: `d:\proj\git\fricp\src\internal\FastRobustCore.cpp`
- Modify: `d:\proj\git\fricp\CMakeLists.txt`
- Modify: `d:\proj\git\fricp\tests\FastRobustIcpTests.cpp`

**Step 1: Write the failing test**

Add a robust-mode smoke test on a clean cloud. It can initially assert success
and non-identity output for a translated source.

```cpp
static int TestRobustModeReturnsTransformOnCleanData() {
    auto target = MakeCubeCloud();
    auto source = target;
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3,1>(0,3) = Eigen::Vector3d(0.3, 0.0, 0.0);
    source.Transform(transform);

    fricp::FastRobustIcp icp;
    fricp::RegistrationOptions options;
    options.mode = fricp::RegistrationMode::RobustPointToPoint;
    options.max_correspondence_distance = 2.0;
    fricp::RegistrationResult result;

    if (!icp.Register(source, target, options, result)) return 1;
    return result.transformation.isIdentity(1e-4) ? 1 : 0;
}
```

**Step 2: Run test to verify it fails**

Run `ctest` and expect failure because robust mode still has no implementation.

**Step 3: Write minimal implementation**

Create internal robust solver types.

`FastRobustCore.h`

```cpp
#pragma once

#include <Eigen/Core>
#include <open3d/geometry/PointCloud.h>

namespace fricp::internal {

struct RobustOptions {
    int max_iteration = 50;
    double nu_begin_k = 3.0;
    double nu_end_k = 1.0 / 6.0;
    double nu_alpha = 0.5;
    double stop = 1e-5;
    bool use_anderson = true;
};

struct RobustResult {
    bool success = false;
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    double convergence_energy = 0.0;
    int iteration_count = 0;
    std::string message;
};

RobustResult RegisterRobustPointToPoint(
    const open3d::geometry::PointCloud& source,
    const open3d::geometry::PointCloud& target,
    const Eigen::Matrix4d& initial_transform,
    bool use_initial_transform,
    const RobustOptions& options);

}  // namespace fricp::internal
```

In `FastRobustCore.cpp`, first implement only:

- point cloud to Eigen `3 x N` conversion
- scale and center preprocessing
- a placeholder return path that reports `not implemented`

This step is intentionally minimal; do not port the full solver yet.

**Step 4: Run test to verify it passes**

This step should still fail. Do not mark Task 5 complete until the robust smoke
test actually passes with at least a basic solver path. The minimal acceptable
green state is a simple weighted SVD ICP loop without robustness yet, housed in
`FastRobustCore.cpp`.

Use this minimal loop:

```cpp
for (int iter = 0; iter < options.max_iteration; ++iter) {
    // nearest-neighbor correspondence
    // uniform weights
    // weighted SVD rigid transform update
    // stop when transform delta < options.stop
}
```

Return the recovered transform after restoring source mean, target mean, and
 scale.

**Step 5: Commit**

```powershell
git add CMakeLists.txt src/internal tests/FastRobustIcpTests.cpp
git commit -m "feat: add internal robust solver scaffold"
```

### Task 6: Port Welsch Reweighting And Anderson Acceleration

**Files:**
- Modify: `d:\proj\git\fricp\src\internal\RobustKernels.h`
- Modify: `d:\proj\git\fricp\src\internal\FastRobustCore.h`
- Modify: `d:\proj\git\fricp\src\internal\FastRobustCore.cpp`
- Modify: `d:\proj\git\fricp\tests\FastRobustIcpTests.cpp`

**Step 1: Write the failing test**

Add an outlier-heavy test that should favor robust mode over standard point-to-
point.

```cpp
static void InjectOutliers(open3d::geometry::PointCloud& cloud) {
    cloud.points_.push_back(Eigen::Vector3d(5.0, 5.0, 5.0));
    cloud.points_.push_back(Eigen::Vector3d(-5.0, 4.0, -3.0));
    cloud.points_.push_back(Eigen::Vector3d(6.0, -4.0, 2.0));
}

static int TestRobustModeBeatsPointToPointWithOutliers() {
    auto target = MakeCubeCloud();
    auto source = target;
    source.Transform(MakeKnownTransform());
    InjectOutliers(source);

    fricp::FastRobustIcp icp;
    fricp::RegistrationResult p2p_result;
    fricp::RegistrationResult robust_result;

    fricp::RegistrationOptions p2p_options;
    p2p_options.mode = fricp::RegistrationMode::PointToPoint;
    p2p_options.max_correspondence_distance = 2.0;
    p2p_options.max_iteration = 100;

    fricp::RegistrationOptions robust_options = p2p_options;
    robust_options.mode = fricp::RegistrationMode::RobustPointToPoint;

    if (!icp.Register(source, target, p2p_options, p2p_result)) return 1;
    if (!icp.Register(source, target, robust_options, robust_result)) return 1;

    const Eigen::Matrix4d expected = MakeKnownTransform().inverse();
    const double p2p_err =
        (p2p_result.transformation - expected).norm();
    const double robust_err =
        (robust_result.transformation - expected).norm();
    return !(robust_err < p2p_err);
}
```

**Step 2: Run test to verify it fails**

Run `ctest` and expect robust mode not to outperform the standard ICP path yet.

**Step 3: Write minimal implementation**

Port only the upstream pieces needed by robust point-to-point:

- Welsch energy
- Welsch weights
- k-nearest-neighbor median scale estimate
- Anderson acceleration state update
- weighted rigid transform step

Do not port:

- file output
- ground truth metrics
- point-to-plane paths
- Sparse ICP
- command-line parsing

The update loop should mirror the upstream robust point-to-point path:

```cpp
// initialize correspondences and residuals
// compute nu1 and nu2
// iterate:
//   compute energy
//   apply robust weights
//   compute weighted rigid transform
//   optionally apply Anderson acceleration in log-transform space
//   update correspondences
//   stop on transform delta
//   decrease nu1 toward nu2
```

Set `RobustResult.convergence_energy` and `RobustResult.iteration_count`.

**Step 4: Run test to verify it passes**

Run `ctest` and expect the outlier test to pass.

**Step 5: Commit**

```powershell
git add src/internal tests/FastRobustIcpTests.cpp
git commit -m "feat: port robust point-to-point solver"
```

### Task 7: Wire Robust Mode Into The Exported Class

**Files:**
- Create: `d:\proj\git\fricp\src\FastRobustIcpImpl.h`
- Create: `d:\proj\git\fricp\src\FastRobustIcpImpl.cpp`
- Modify: `d:\proj\git\fricp\src\FastRobustIcp.cpp`
- Modify: `d:\proj\git\fricp\CMakeLists.txt`

**Step 1: Write the failing test**

No new behavior test is needed if Task 6 already exercises the public class.
Instead, add one assertion that `result.convergence_energy > 0.0` in robust
mode.

```cpp
if (robust_result.convergence_energy <= 0.0) return 1;
```

**Step 2: Run test to verify it fails**

Run `ctest` and expect failure because the public API does not yet propagate the
robust solver's convergence energy.

**Step 3: Write minimal implementation**

Introduce an internal implementation bridge to keep `FastRobustIcp.cpp` thin.

`FastRobustIcpImpl.h`

```cpp
#pragma once

#include <fricp/Types.h>
#include <open3d/geometry/PointCloud.h>

namespace fricp::internal {

bool RunRegistration(const open3d::geometry::PointCloud& source,
                     const open3d::geometry::PointCloud& target,
                     const RegistrationOptions& options,
                     RegistrationResult& result,
                     std::string& last_error);

}  // namespace fricp::internal
```

Move mode dispatch into `RunRegistration(...)`.

For robust mode:

```cpp
RobustOptions robust_options;
robust_options.max_iteration = options.max_iteration;
robust_options.nu_begin_k = options.robust_nu_begin_k;
robust_options.nu_end_k = options.robust_nu_end_k;
robust_options.nu_alpha = options.robust_nu_alpha;
robust_options.stop = options.robust_stop;
robust_options.use_anderson = options.robust_use_anderson;

RobustResult robust = RegisterRobustPointToPoint(
    source,
    target,
    options.initial_transform,
    options.use_initial_transform,
    robust_options);
```

Copy the robust fields into `RegistrationResult`.

**Step 4: Run test to verify it passes**

Run `ctest` and expect robust metadata propagation to pass.

**Step 5: Commit**

```powershell
git add src/FastRobustIcp.cpp src/FastRobustIcpImpl.* CMakeLists.txt
git commit -m "refactor: isolate registration dispatch from public api"
```

### Task 8: Add Example Consumer Program

**Files:**
- Create: `d:\proj\git\fricp\examples\register_example.cpp`
- Modify: `d:\proj\git\fricp\CMakeLists.txt`

**Step 1: Write the failing test**

Treat example compilation as the test. Add the example target to CMake before
creating the source file so the build fails.

```cmake
add_executable(fricp_example examples/register_example.cpp)
target_link_libraries(fricp_example PRIVATE fricp Open3D::Open3D)
```

**Step 2: Run test to verify it fails**

Run:

```powershell
cmake --build build --config Debug
```

Expected:

- build fails because `examples/register_example.cpp` does not exist yet

**Step 3: Write minimal implementation**

Create a minimal example that builds two synthetic clouds and runs the API.

```cpp
#include <fricp/FastRobustIcp.h>
#include <iostream>

int main() {
    open3d::geometry::PointCloud target;
    target.points_ = {
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };

    open3d::geometry::PointCloud source = target;
    Eigen::Matrix4d init = Eigen::Matrix4d::Identity();
    init.block<3,1>(0,3) = Eigen::Vector3d(0.1, 0.0, 0.0);
    source.Transform(init);

    fricp::FastRobustIcp icp;
    fricp::RegistrationOptions options;
    options.mode = fricp::RegistrationMode::RobustPointToPoint;
    options.max_correspondence_distance = 1.0;

    fricp::RegistrationResult result;
    if (!icp.Register(source, target, options, result)) {
        std::cerr << result.message << '\n';
        return 1;
    }

    std::cout << result.transformation << '\n';
    return 0;
}
```

**Step 4: Run test to verify it passes**

Run:

```powershell
cmake --build build --config Debug --target fricp_example
```

Expected:

- example builds successfully

**Step 5: Commit**

```powershell
git add examples/register_example.cpp CMakeLists.txt
git commit -m "docs: add dll consumer example"
```

### Task 9: Full Verification And Packaging Check

**Files:**
- Modify as needed based on test fixes

**Step 1: Write the failing test**

Add one final aggregate assertion in `tests/FastRobustIcpTests.cpp` that runs
all scenarios in a single executable and prints the failing test name. If this
helper already exists, skip this step.

**Step 2: Run test to verify it fails**

Run the full build and test pipeline.

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Expected:

- any remaining integration issues appear here

**Step 3: Write minimal implementation**

Fix only the failures revealed by the full Release verification:

- missing exports
- missing include directories
- incorrect Open3D linkage
- numerical thresholds that are too tight for Release

Do not add new features in this task.

**Step 4: Run test to verify it passes**

Re-run:

```powershell
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Expected:

- all tests pass
- `fricp.dll`, `fricp.lib`, `fricp_tests.exe`, and `fricp_example.exe` are
  generated

**Step 5: Commit**

```powershell
git add .
git commit -m "chore: finalize fricp dll build"
```

### Task 10: Record Consumer Notes

**Files:**
- Create: `d:\proj\git\fricp\README.md`

**Step 1: Write the failing test**

This is a docs-only task; use build reproducibility as the validation target.
There is no separate code test.

**Step 2: Run test to verify it fails**

Skip. This task documents already-verified behavior.

**Step 3: Write minimal implementation**

Write a concise consumer README covering:

- required Open3D dependency
- required compiler/runtime compatibility
- how to configure with CMake
- where to find `fricp.dll` and `fricp.lib`
- a short sample using `FastRobustIcp`

Suggested outline:

```md
# fricp

## Requirements
## Build
## Public API
## Example
## ABI Notes
```

**Step 4: Run test to verify it passes**

Re-run the Release build and tests once after the README is added to ensure no
paths or targets were accidentally changed.

**Step 5: Commit**

```powershell
git add README.md
git commit -m "docs: add consumer build instructions"
```

---

**Execution Notes**

- The workspace is currently not a Git repository, so commit steps are written
  for the intended workflow but cannot be executed until the project is placed
  under git.
- If Open3D is not discoverable through `find_package(Open3D REQUIRED)`, stop
  and fix dependency configuration before continuing.
- Keep all public APIs ASCII-only and avoid exposing internal robust solver
  headers in the install surface.
