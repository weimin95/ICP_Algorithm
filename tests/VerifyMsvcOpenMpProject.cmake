if(NOT DEFINED PROJECT_FILE)
    message(FATAL_ERROR "PROJECT_FILE is required")
endif()

if(NOT EXISTS "${PROJECT_FILE}")
    message(FATAL_ERROR "Project file not found: ${PROJECT_FILE}")
endif()

file(READ "${PROJECT_FILE}" project_contents)

if(project_contents MATCHES "(/openmp|OpenMPSupport)")
    message(STATUS "Verified OpenMP compile flag in ${PROJECT_FILE}")
else()
    message(FATAL_ERROR "Expected OpenMP compile support in ${PROJECT_FILE}")
endif()
