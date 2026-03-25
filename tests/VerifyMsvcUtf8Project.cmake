if(NOT DEFINED PROJECT_FILE)
    message(FATAL_ERROR "PROJECT_FILE is required")
endif()

if(NOT EXISTS "${PROJECT_FILE}")
    message(FATAL_ERROR "Project file not found: ${PROJECT_FILE}")
endif()

file(READ "${PROJECT_FILE}" project_contents)

if(project_contents MATCHES "/utf-8")
    message(STATUS "Verified UTF-8 compile flag in ${PROJECT_FILE}")
else()
    message(FATAL_ERROR "Expected /utf-8 in ${PROJECT_FILE}")
endif()
