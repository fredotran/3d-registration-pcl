cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(pcl_registration VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build options
option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_EXAMPLES "Build example executables" ON)
option(BUILD_TOOLS "Build utility tools" ON)
option(BUILD_BENCHMARKS "Build benchmark executables" OFF)
option(WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

# Compiler warnings
if(MSVC)
    add_compile_options(/W4)
    if(WARNINGS_AS_ERRORS)
        add_compile_options(/WX)
    endif()
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
    if(WARNINGS_AS_ERRORS)
        add_compile_options(-Werror)
    endif()
endif()

# Find PCL
find_package(PCL 1.7 REQUIRED COMPONENTS common io filters features registration visualization)

# Include directories
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/include)

# Add PCL definitions
add_definitions(${PCL_DEFINITIONS})

# Create library target for common code
add_library(pcl_registration_lib
    src/parameters.cpp
)

target_link_libraries(pcl_registration_lib ${PCL_LIBRARIES})
target_include_directories(pcl_registration_lib PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
)

# Main registration executable
if(BUILD_EXAMPLES)
    add_executable(main_registration src/main_registration.cpp)
    target_link_libraries(main_registration pcl_registration_lib ${PCL_LIBRARIES})

    add_executable(experiment2 src/experiment2.cpp)
    target_link_libraries(experiment2 pcl_registration_lib ${PCL_LIBRARIES})

    add_executable(experiment2_ds src/experiment2_ds.cpp)
    target_link_libraries(experiment2_ds pcl_registration_lib ${PCL_LIBRARIES})

    add_executable(verification_test1 src/verification_test1.cpp)
    target_link_libraries(verification_test1 pcl_registration_lib ${PCL_LIBRARIES})

    add_executable(verification_test2 src/verification_test2.cpp)
    target_link_libraries(verification_test2 pcl_registration_lib ${PCL_LIBRARIES})

    add_executable(fred_exp_sacia_param_grid_search src/fred_exp_sacia_param_grid_search.cpp)
    target_link_libraries(fred_exp_sacia_param_grid_search pcl_registration_lib ${PCL_LIBRARIES})

    add_executable(exp_sacia_param_grid_search src/exp_sacia_param_grid_search.cpp)
    target_link_libraries(exp_sacia_param_grid_search pcl_registration_lib ${PCL_LIBRARIES})
endif()

# Utility tools
if(BUILD_TOOLS)
    add_executable(visualization src/visualization.cpp)
    target_link_libraries(visualization pcl_registration_lib ${PCL_LIBRARIES})

    add_executable(cpp_version src/cpp_version.cpp)
    target_link_libraries(cpp_version pcl_registration_lib ${PCL_LIBRARIES})

    add_executable(point_cloud_converter src/point_cloud_converter.cpp)
    target_link_libraries(point_cloud_converter pcl_registration_lib ${PCL_LIBRARIES})
endif()

# Testing
if(BUILD_TESTS)
    enable_testing()
    find_package(GTest QUIET)
    if(GTest_FOUND)
        add_subdirectory(tests)
    else()
        message(WARNING "GTest not found, tests will not be built")
    endif()
endif()

# Benchmarks
if(BUILD_BENCHMARKS)
    add_executable(benchmark_registration benchmarks/benchmark_registration.cpp)
    target_link_libraries(benchmark_registration pcl_registration_lib ${PCL_LIBRARIES})
    target_include_directories(benchmark_registration PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/benchmarks
    )
endif()

# Installation
install(TARGETS pcl_registration_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY data/ DESTINATION share/pcl_registration/data)
install(DIRECTORY results/ DESTINATION share/pcl_registration/results OPTIONAL)

# Print configuration summary
message(STATUS "PCL Registration Configuration:")
message(STATUS "  Version: ${PROJECT_VERSION}")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  PCL Version: ${PCL_VERSION}")
message(STATUS "  Build Tests: ${BUILD_TESTS}")
message(STATUS "  Build Examples: ${BUILD_EXAMPLES}")
message(STATUS "  Build Tools: ${BUILD_TOOLS}")
message(STATUS "  Build Benchmarks: ${BUILD_BENCHMARKS}")
message(STATUS "  Warnings as Errors: ${WARNINGS_AS_ERRORS}")