#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "benchmark.hpp"
#include "tools.hpp"
#include "parameters.hpp"

/**
 * Benchmark example for registration pipeline performance
 * This demonstrates how to use the benchmark framework
 */

void benchmark_point_cloud_operations() {
    std::cout << "\n=== Point Cloud Operations Benchmark ===" << std::endl;

    // Benchmark point cloud creation
    Benchmark::run("Create point cloud (1000 points)", []() {
        auto cloud = PointCloudPtr(new PointCloud);
        for (int i = 0; i < 1000; ++i) {
            cloud->push_back(pcl::PointXYZ(
                static_cast<float>(i),
                static_cast<float>(i),
                static_cast<float>(i)
            ));
        }
    }, 5);

    // Benchmark point cloud transformation
    auto cloud = PointCloudPtr(new PointCloud);
    for (int i = 0; i < 1000; ++i) {
        cloud->push_back(pcl::PointXYZ(
            static_cast<float>(i),
            static_cast<float>(i),
            static_cast<float>(i)
        ));
    }

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(0.5, Eigen::Vector3f::UnitZ()));

    Benchmark::run("Transform point cloud (1000 points)", [&]() {
        auto transformed = PointCloudPtr(new PointCloud);
        pcl::transformPointCloud(*cloud, *transformed, transform);
    }, 5);
}

void benchmark_error_metrics() {
    std::cout << "\n=== Error Metrics Benchmark ===" << std::endl;

    // Create test clouds
    auto cloud1 = PointCloudPtr(new PointCloud);
    auto cloud2 = PointCloudPtr(new PointCloud);

    for (int i = 0; i < 1000; ++i) {
        cloud1->push_back(pcl::PointXYZ(
            static_cast<float>(i),
            static_cast<float>(i),
            static_cast<float>(i)
        ));
        cloud2->push_back(pcl::PointXYZ(
            static_cast<float>(i) + 0.1f,
            static_cast<float>(i) + 0.1f,
            static_cast<float>(i) + 0.1f
        ));
    }

    Benchmark::run("Mean Target Registration Error (1000 points)", [&]() {
        double mtre = meanTargetRegistrationError(cloud1, cloud2);
        (void)mtre; // Suppress unused variable warning
    }, 10);

    Benchmark::run("Registration Error Bias (1000 points)", [&]() {
        auto [bias_x, bias_y, bias_z] = registrationErrorBias(cloud1, cloud2);
        (void)bias_x; (void)bias_y; (void)bias_z; // Suppress unused variable warnings
    }, 10);
}

void benchmark_file_io() {
    std::cout << "\n=== File I/O Benchmark ===" << std::endl;

    // Create a test cloud
    auto cloud = PointCloudPtr(new PointCloud);
    for (int i = 0; i < 1000; ++i) {
        cloud->push_back(pcl::PointXYZ(
            static_cast<float>(i),
            static_cast<float>(i),
            static_cast<float>(i)
        ));
    }

    Benchmark::run("Save PCD file (1000 points)", [&]() {
        pcl::io::savePCDFileASCII("benchmark_test.pcd", *cloud);
    }, 5);

    Benchmark::run("Load PCD file (1000 points)", [&]() {
        auto loaded = loadingCloud("benchmark_test.pcd");
    }, 5);

    // Clean up
    std::remove("benchmark_test.pcd");
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "   3D Registration PCL Benchmark Suite   " << std::endl;
    std::cout << "========================================" << std::endl;

    try {
        benchmark_point_cloud_operations();
        benchmark_error_metrics();
        benchmark_file_io();

        std::cout << "\n========================================" << std::endl;
        std::cout << "   All benchmarks completed successfully   " << std::endl;
        std::cout << "========================================" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Benchmark failed with exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
