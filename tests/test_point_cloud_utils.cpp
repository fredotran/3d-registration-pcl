#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "tools.hpp"
#include "parameters.hpp"

TEST(PointCloudUtilsTest, CropPointCloud) {
    // Test point cloud cropping
    auto inputCloud = PointCloudPtr(new PointCloud);
    auto outputCloud = PointCloudPtr(new PointCloud);

    // Create a cloud with points in different regions
    for (int i = 0; i < 10; ++i) {
        inputCloud->push_back(pcl::PointXYZ(static_cast<float>(i), static_cast<float>(i), 0.0));
    }

    // Crop to keep only points with x in [3, 7] and y in [3, 7]
    cropPointCloud(inputCloud, outputCloud, 3.0, 7.0, 3.0, 7.0);

    // Should have points at (3,3), (4,4), (5,5), (6,6), (7,7)
    EXPECT_EQ(outputCloud->size(), 5);

    // Verify the points are within the crop bounds
    for (const auto& point : outputCloud->points) {
        EXPECT_GE(point.x, 3.0);
        EXPECT_LE(point.x, 7.0);
        EXPECT_GE(point.y, 3.0);
        EXPECT_LE(point.y, 7.0);
    }
}

TEST(PointCloudUtilsTest, CropPointCloudNoPoints) {
    // Test cropping with no points in region
    auto inputCloud = PointCloudPtr(new PointCloud);
    auto outputCloud = PointCloudPtr(new PointCloud);

    inputCloud->push_back(pcl::PointXYZ(1.0, 1.0, 0.0));
    inputCloud->push_back(pcl::PointXYZ(2.0, 2.0, 0.0));

    // Crop to region with no points
    cropPointCloud(inputCloud, outputCloud, 10.0, 20.0, 10.0, 20.0);

    EXPECT_EQ(outputCloud->size(), 0);
}

TEST(PointCloudUtilsTest, LoadingCloud) {
    // Test loading a point cloud from file
    // First create a test PCD file
    auto testCloud = PointCloudPtr(new PointCloud);
    testCloud->push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
    testCloud->push_back(pcl::PointXYZ(4.0, 5.0, 6.0));
    testCloud->push_back(pcl::PointXYZ(7.0, 8.0, 9.0));

    pcl::io::savePCDFileASCII("test_cloud.pcd", *testCloud);

    // Now load it back
    auto loadedCloud = loadingCloud("test_cloud.pcd");

    EXPECT_EQ(loadedCloud->size(), 3);
    EXPECT_NEAR(loadedCloud->points[0].x, 1.0, 1e-5);
    EXPECT_NEAR(loadedCloud->points[0].y, 2.0, 1e-5);
    EXPECT_NEAR(loadedCloud->points[0].z, 3.0, 1e-5);

    // Clean up
    std::remove("test_cloud.pcd");
}

TEST(PointCloudUtilsTest, LoadingCloudNonExistent) {
    // Test loading a non-existent file
    EXPECT_THROW(loadingCloud("nonexistent_file.pcd"), std::runtime_error);
}

TEST(PointCloudUtilsTest, LoadingCloudWithNaN) {
    // Test loading a cloud with NaN values
    auto testCloud = PointCloudPtr(new PointCloud);
    testCloud->push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
    testCloud->push_back(pcl::PointXYZ(std::numeric_limits<float>::quiet_NaN(), 5.0, 6.0));
    testCloud->push_back(pcl::PointXYZ(7.0, 8.0, 9.0));

    pcl::io::savePCDFileASCII("test_cloud_nan.pcd", *testCloud);

    // Load should remove NaN values
    auto loadedCloud = loadingCloud("test_cloud_nan.pcd");

    EXPECT_EQ(loadedCloud->size(), 2);  // NaN point should be removed

    // Clean up
    std::remove("test_cloud_nan.pcd");
}

TEST(PointCloudUtilsTest, ComputeReferenceCloud) {
    // Test reference cloud computation
    auto surfaceModel = PointCloudPtr(new PointCloud);

    // Create a simple surface model
    for (int i = 0; i < 20; ++i) {
        for (int j = 0; j < 20; ++j) {
            surfaceModel->push_back(pcl::PointXYZ(static_cast<float>(i), static_cast<float>(j), 0.0));
        }
    }

    double seedRef = 12345;
    double sourceWidth = 5.0;
    double sourceHeight = 5.0;
    double x_uncertainty = 1.0;
    double y_uncertainty = 1.0;

    auto referenceCloud = computeReferenceCloud(surfaceModel, &seedRef, sourceWidth, sourceHeight, x_uncertainty, y_uncertainty);

    // Reference cloud should be smaller than surface model
    EXPECT_GT(referenceCloud->size(), 0);
    EXPECT_LT(referenceCloud->size(), surfaceModel->size());
}

TEST(PointCloudUtilsTest, ComputeSourceCloud) {
    // Test source cloud computation
    auto targetCloud = PointCloudPtr(new PointCloud);

    // Create a target cloud
    for (int i = 0; i < 20; ++i) {
        for (int j = 0; j < 20; ++j) {
            targetCloud->push_back(pcl::PointXYZ(static_cast<float>(i), static_cast<float>(j), 0.0));
        }
    }

    double seed = 12345;
    double sourceWidth = 5.0;
    double sourceHeight = 5.0;
    double x_uncertainty = 1.0;
    double y_uncertainty = 1.0;

    auto sourceCloud = computeSourceCloud(targetCloud, &seed, sourceWidth, sourceHeight, x_uncertainty, y_uncertainty);

    // Source cloud should be smaller than target cloud
    EXPECT_GT(sourceCloud->size(), 0);
    EXPECT_LT(sourceCloud->size(), targetCloud->size());
}
