# FRICP DLL Design

**Date:** 2026-03-23

**Status:** Approved by user

**Goal**

Build a Windows C++ DLL around Open3D and Fast-Robust-ICP so third-party C++
projects can call ICP registration directly with
`open3d::geometry::PointCloud`.

**Constraints**

- Deliverable is a DLL project that builds `fricp.dll` and the matching import
  library.
- The public API exports a C++ class, not free functions.
- The public API accepts `open3d::geometry::PointCloud`.
- The caller also links against the same Open3D C++ installation.
- Required modes:
  `PointToPoint`, `PointToPlane`, and `RobustPointToPoint`.
- `PointToPoint` and `PointToPlane` should reuse Open3D's native registration
  implementation where practical.
- `RobustPointToPoint` should use the upstream Fast-Robust-ICP algorithmic
  core rather than a loose approximation.

**Recommended Architecture**

Use a mixed implementation strategy:

- `PointToPoint`: wrap Open3D
  `open3d::pipelines::registration::RegistrationICP(...)` with
  `TransformationEstimationPointToPoint`.
- `PointToPlane`: wrap Open3D
  `open3d::pipelines::registration::RegistrationICP(...)` with
  `TransformationEstimationPointToPlane`.
- `RobustPointToPoint`: extract and adapt the point-to-point robust core from
  `yaoyx689/Fast-Robust-ICP` into an internal library module that works
  directly on in-memory point clouds.

This gives the smallest maintenance surface while still preserving the main
value of the upstream project.

**Public API Shape**

Export a single class from the DLL:

```cpp
class FRICP_API FastRobustIcp {
public:
    FastRobustIcp();
    ~FastRobustIcp();

    bool Register(const open3d::geometry::PointCloud& source,
                  const open3d::geometry::PointCloud& target,
                  const RegistrationOptions& options,
                  RegistrationResult& result) const;

    const std::string& GetLastError() const;
};
```

Expose a compact set of public types:

- `RegistrationMode`
- `RegistrationOptions`
- `RegistrationResult`
- DLL export macro

Keep the public header thin and place implementation details behind an internal
bridge layer.

**Configuration Model**

The options structure should cover:

- mode selection
- ICP convergence settings
- max correspondence distance
- optional initial transform
- optional target-normal estimation for point-to-plane
- robust-mode tuning for Welsch reweighting and Anderson acceleration

The result structure should cover:

- success flag
- selected mode
- estimated `Eigen::Matrix4d`
- fitness
- inlier RMSE
- robust convergence energy
- iteration count
- human-readable message

**Algorithm Mapping**

`PointToPoint`

- Use Open3D point-to-point estimation.
- Evaluate final registration with Open3D so metrics match the library's
  standard interpretation.

`PointToPlane`

- Use Open3D point-to-plane estimation.
- If target normals are missing and the option allows it, estimate them before
  registration.

`RobustPointToPoint`

- Convert `open3d::geometry::PointCloud` points into Eigen `3 x N` matrices.
- Reproduce the upstream preprocessing pattern: scaling, de-meaning, optional
  initial transform handling, and final transform restoration.
- Keep Welsch weighting and Anderson acceleration from the upstream core.
- Remove command-line, file IO, output logging, and ground-truth code paths.
- Return only in-memory results.

The initial implementation explicitly excludes robust point-to-plane mode.

**Data Flow**

1. Validate input point clouds and options.
2. Copy input point clouds so the caller's data is never modified.
3. Route by registration mode.
4. For Open3D-backed modes, call the corresponding Open3D registration API.
5. For robust mode, convert to Eigen, run the adapted upstream solver, and
   recover the final transform in original scale.
6. Evaluate the output transform with Open3D and write normalized metrics into
   `RegistrationResult`.

**Error Handling**

Use a simple synchronous error model:

- `Register(...)` returns `true` on success and `false` on failure.
- `RegistrationResult.message` contains the per-call status.
- `GetLastError()` mirrors the last failure recorded by the class.

Expected failure cases:

- empty source or target point cloud
- invalid numeric options such as non-positive correspondence distance
- point-to-plane without normals when auto-estimation is disabled
- Open3D exceptions
- numerical failure inside the robust solver

**Project Layout**

```text
fricp/
  CMakeLists.txt
  include/
    fricp/
      Export.h
      FastRobustIcp.h
      Types.h
  src/
    FastRobustIcp.cpp
    FastRobustIcpImpl.h
    FastRobustIcpImpl.cpp
    internal/
      AndersonAcceleration.h
      FastRobustCore.h
      FastRobustCore.cpp
      NanoFlannAdaptor.h
      RobustKernels.h
  tests/
    FastRobustIcpTests.cpp
  examples/
    register_example.cpp
  docs/
    plans/
```

**Build System**

Use modern CMake:

- `add_library(fricp SHARED ...)`
- export macro based on `__declspec(dllexport/dllimport)` on Windows
- require C++17
- link against `Open3D::Open3D`
- produce a minimal example executable that links to the DLL

**Testing Strategy**

Use synthetic point clouds generated in memory:

1. point-to-point recovers a known rigid transform
2. point-to-plane recovers a known rigid transform on a cloud with normals
3. robust point-to-point outperforms standard point-to-point with injected
   outliers
4. validation failures return `false` and a useful error message

Tests should avoid external data files and focus on deterministic geometric
fixtures.

**Known Environment Caveats**

- Because the API exposes Open3D C++ types directly, the caller and the DLL
  must use a compatible toolchain, runtime, and Open3D build.
- The current workspace is not a Git repository, so this design doc cannot be
  committed here even though the brainstorming workflow would normally require a
  commit.

**Primary References**

- Fast-Robust-ICP upstream repository:
  https://github.com/yaoyx689/Fast-Robust-ICP
- Open3D `PointCloud` C++ API:
  https://www.open3d.org/docs/release/cpp_api/classopen3d_1_1geometry_1_1_point_cloud.html
- Open3D registration C++ API:
  https://www.open3d.org/docs/0.19.0/cpp_api/pipelines_2registration_2_registration_8h_source.html
