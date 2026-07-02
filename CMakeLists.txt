cmake_minimum_required(VERSION 3.20)
project(softgps VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(SOFTGPS_BUILD_TESTS "Build unit tests" ON)
set(SOFTGPS_UHD_ROOT "C:/Users/Miguel/radioconda/Library" CACHE PATH "UHD installation root")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(UHD REQUIRED)

add_library(softgps_core STATIC
    src/acquisition.cpp
    src/ca_code.cpp
)
target_include_directories(softgps_core PUBLIC include)
target_compile_definitions(softgps_core PUBLIC NOMINMAX WIN32_LEAN_AND_MEAN)
if(MSVC)
    target_compile_options(softgps_core PRIVATE /W4 /permissive- /utf-8)
else()
    target_compile_options(softgps_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(softgps src/main.cpp)
target_link_libraries(softgps PRIVATE softgps_core UHD::uhd)
if(MSVC)
    target_compile_options(softgps PRIVATE /W4 /permissive- /utf-8)
endif()

if(SOFTGPS_BUILD_TESTS)
    enable_testing()
    add_executable(softgps-tests tests/core_tests.cpp)
    target_link_libraries(softgps-tests PRIVATE softgps_core)
    add_test(NAME softgps-tests COMMAND softgps-tests)
endif()

install(TARGETS softgps softgps_core
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
)
