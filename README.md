# fricp

`fricp` is a Windows C++ DLL wrapper around Open3D ICP and the
Fast-Robust-ICP point-to-point robust registration strategy.

## Features

- Exported C++ class API: `fricp::FastRobustIcp`
- Input type: `open3d::geometry::PointCloud`
- Modes:
  - `PointToPoint`
  - `PointToPlane`
  - `RobustPointToPoint`

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

if (!icp.Train(target_cloud, options)) {
    // handle training failure
}

const bool ok = icp.Register(source_cloud, options, result);
```

The API returns:

- success / failure
- estimated `Eigen::Matrix4d`
- fitness and inlier RMSE for standard ICP modes
- convergence energy for robust mode
- descriptive error message on failure

Lifecycle helpers:

- `Train(...)` caches a target cloud and its derived data for later registrations.
- `Register(...)` uses the cached target to align each source cloud.
- `IsTrained()` reports whether a target is currently cached.
- `ClearTraining()` discards the cached target and resets the trained state.

## Example

See:

- `examples/register_example.cpp`

The example trains once on a target cloud, then registers a source cloud
against that trained target. This is the intended pattern for repeated
registrations against one target without retraining.

Build target:

```powershell
cmake --build build --config Release --target fricp_example
```
