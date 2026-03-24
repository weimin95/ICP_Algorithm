#pragma once

#if defined(_WIN32)
#    if defined(FRICP_BUILD_DLL)
#        define FRICP_API __declspec(dllexport)
#    else
#        define FRICP_API __declspec(dllimport)
#    endif
#else
#    define FRICP_API
#endif
