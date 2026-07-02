cmake_minimum_required(VERSION 3.20)
project(gnss VERSION 0.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(GNSS_BUILD_TESTS "Build unit tests" ON)
option(GNSS_ENABLE_UHD "Build UHD/B200 capture support when UHD is available" ON)
set(GNSS_UHD_ROOT "C:/Users/Miguel/radioconda/Library" CACHE PATH "UHD installation root")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_library(gnss_core STATIC
    src/acquisition.cpp
    src/ca_code.cpp
    src/math.cpp
    src/pvt.cpp
    src/tracking.cpp
)
target_include_directories(gnss_core PUBLIC include)
target_compile_definitions(gnss_core PUBLIC NOMINMAX WIN32_LEAN_AND_MEAN)
if(MSVC)
    target_compile_options(gnss_core PRIVATE /W4 /permissive- /utf-8)
else()
    target_compile_options(gnss_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(gnss src/main.cpp)
target_link_libraries(gnss PRIVATE gnss_core)
if(MSVC)
    target_compile_options(gnss PRIVATE /W4 /permissive- /utf-8)
else()
    target_compile_options(gnss PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(GNSS_ENABLE_UHD)
    find_package(UHD QUIET)
    if(UHD_FOUND)
        target_link_libraries(gnss PRIVATE UHD::uhd)
        target_compile_definitions(gnss PRIVATE GNSS_HAS_UHD=1)
        message(STATUS "UHD found: B200 capture enabled")
    else()
        message(WARNING "UHD not found: building gnss with offline IQ and PVT modes only")
    endif()
endif()

if(GNSS_BUILD_TESTS)
    enable_testing()
    add_executable(gnss-tests tests/core_tests.cpp)
    target_link_libraries(gnss-tests PRIVATE gnss_core)
    add_test(NAME gnss-tests COMMAND gnss-tests)
endif()

install(TARGETS gnss gnss_core
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
)
