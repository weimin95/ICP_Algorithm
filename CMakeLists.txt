cmake_minimum_required(VERSION 3.20)
project(fricp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT Open3D_DIR)
    set(_fricp_local_open3d_dir
        "${CMAKE_CURRENT_SOURCE_DIR}/.deps/open3d/open3d-devel-windows-amd64-0.19.0/CMake")
    if(EXISTS "${_fricp_local_open3d_dir}/Open3DConfig.cmake")
        set(Open3D_DIR "${_fricp_local_open3d_dir}" CACHE PATH
            "Path to the Open3D CMake package" FORCE)
    endif()
endif()

find_package(Open3D CONFIG REQUIRED)

get_filename_component(_fricp_open3d_root "${Open3D_DIR}" DIRECTORY)
file(GLOB _fricp_open3d_runtime_dlls
    "${_fricp_open3d_root}/bin/*.dll")

if(MSVC AND NOT EXISTS "${Open3D_DIR}/Open3DTargets-debug.cmake")
    message(WARNING
        "The configured Open3D package does not provide Debug import targets. "
        "Use Release builds with this SDK, or point Open3D_DIR to a debug SDK "
        "when building Debug on Windows.")
endif()

set(FRICP_SOURCES
    src/FastRobustIcp.cpp
    src/internal/Open3DAdapters.cpp
    src/internal/UpstreamParameterMapping.cpp
    src/internal/FastRobustCore.cpp
    src/vendor_include_probe.cpp
)

set(FRICP_HEADERS
    include/fricp/Export.h
    include/fricp/FastRobustIcp.h
    include/fricp/Types.h
    include/fricp/internal/AndersonAcceleration.h
    include/fricp/internal/FastRobustCore.h
    include/fricp/internal/TrainedData.h
    src/internal/Open3DAdapters.h
    src/internal/UpstreamParameterMapping.h
)

add_library(fricp SHARED
    ${FRICP_SOURCES}
    ${FRICP_HEADERS}
)

target_include_directories(fricp
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/vendor/fast_robust_icp
)

target_link_libraries(fricp
    PUBLIC
        Open3D::Open3D
)

target_compile_definitions(fricp
    PRIVATE
        FRICP_BUILD_DLL
)

function(fricp_copy_runtime_dlls target_name)
    foreach(runtime_dll IN LISTS _fricp_open3d_runtime_dlls)
        add_custom_command(TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${runtime_dll}"
                "$<TARGET_FILE_DIR:${target_name}>"
            VERBATIM
        )
    endforeach()
endfunction()

fricp_copy_runtime_dlls(fricp)

add_executable(fricp_tests
    tests/FastRobustIcpTests.cpp
    src/internal/Open3DAdapters.cpp
    src/internal/UpstreamParameterMapping.cpp
)

target_link_libraries(fricp_tests
    PRIVATE
        fricp
)

target_include_directories(fricp_tests
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/vendor/fast_robust_icp
)

fricp_copy_runtime_dlls(fricp_tests)

add_executable(fricp_example
    examples/register_example.cpp
)

target_link_libraries(fricp_example
    PRIVATE
        fricp
)

fricp_copy_runtime_dlls(fricp_example)

include(CTest)
if(BUILD_TESTING)
    add_test(NAME fricp_tests COMMAND fricp_tests)
endif()
