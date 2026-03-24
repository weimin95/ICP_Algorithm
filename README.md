# fricp

`fricp` is a Windows C++ DLL wrapper around Open3D ICP and the upstream
Fast-Robust-ICP registration methods.

## Features

- Exported C++ class API: `fricp::FastRobustIcp`
- Input type: `open3d::geometry::PointCloud`
- Methods:
  - `ICP`
  - `AAICP`
  - `FastICP`
  - `RobustICP`
  - `PointToPlane`
  - `RobustPointToPlane`
  - `SparseICP`
  - `SparsePointToPlane`

## Requirements

- CMake 3.20+
- MSVC / Visual Studio 2022 recommended
- Open3D C++ development package

This project auto-detects a local SDK in:

`./.deps/open3d/open3d-devel-windows-amd64-0.19.0/CMake`

If you use another Open3D installation, configure with:

```powershell
cmake -S . -B build -DOpen3D_DIR="path\\to\\Open3D\\CMake"
```

## Build

Release build:

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Artifacts are generated under `build/Release/`, including:

- `fricp.dll`
- `fricp.lib`
- `fricp_tests.exe`
- `fricp_example.exe`

## Debug vs Release

On Windows, Open3D debug and release SDKs are not ABI-compatible.

- Use the Open3D release SDK with `Release` builds.
- Use the Open3D debug SDK with `Debug` builds.

If you build a `Debug` target against a release Open3D package, calls using
`open3d::geometry::PointCloud` can crash because of MSVC STL/CRT ABI
mismatch.

## Public API

```cpp
fricp::FastRobustIcp icp;
fricp::RegistrationOptions options;
fricp::RegistrationResult result;

options.method = fricp::RegistrationMethod::RobustICP;

if (!icp.Train(target_cloud, options)) {
    // handle training failure
}

const bool ok = icp.Register(source_cloud, options, result);
```

The API returns:

- success / failure
- estimated `Eigen::Matrix4d`
- fitness and inlier RMSE for standard ICP modes
- convergence energy for robust modes
- descriptive error message on failure

Lifecycle helpers:

- `Train(...)` caches a target cloud and its derived data for later registrations.
- `Register(...)` uses the cached target to align each source cloud.
- `IsTrained()` reports whether a target is currently cached.
- `ClearTraining()` discards the cached target and resets the trained state.

`RegistrationOptions` is a single unified options struct. It covers the
shared ICP controls, the sparse ICP controls, and the target-normal
preprocessing controls for point-to-plane modes.

For `PointToPlane`, `RobustPointToPlane`, and `SparsePointToPlane`, the
wrapper automatically estimates target normals during `Train(...)` when they
are missing and `estimate_target_normals_if_missing` is enabled.
`max_correspondence_distance` is used by the wrapper for input validation and
for Open3D-side result evaluation; it is not a native cutoff parameter in the
vendored upstream solvers.

## Example

See:

- `examples/register_example.cpp`

The example trains once on a target cloud, then registers a source cloud
against that trained target. It keeps the file-based load, downsample, and
write flow in place while using the final wrapper API.

Build target:

```powershell
cmake --build build --config Release --target fricp_example
```
