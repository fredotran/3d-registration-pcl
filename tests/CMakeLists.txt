# Tests CMakeLists.txt
find_package(GTest REQUIRED)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${GTEST_INCLUDE_DIRS})

# Create test executable
add_executable(pcl_registration_tests
    test_main.cpp
    test_settings.cpp
    test_file_io.cpp
    test_transformation.cpp
    test_error_metrics.cpp
    test_point_cloud_utils.cpp
)

target_link_libraries(pcl_registration_tests
    pcl_registration_lib
    ${PCL_LIBRARIES}
    GTest::GTest
    GTest::Main
)

# Register tests with CTest
include(GoogleTest)
gtest_discover_tests(pcl_registration_tests)
