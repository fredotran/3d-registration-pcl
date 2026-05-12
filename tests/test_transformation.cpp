#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "tools.hpp"
#include "parameters.hpp"

TEST(TransformationTest, CustomRotationIdentity) {
    // Test identity transformation (no rotation)
    auto cloudPtr = PointCloudPtr(new PointCloud);
    cloudPtr->push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
    cloudPtr->push_back(pcl::PointXYZ(4.0, 5.0, 6.0));

    Eigen::Vector4f centroid(0, 0, 0, 0);
    Eigen::Vector4f origin_centroid(0, 0, 0, 0);
    Eigen::Vector4f new_centroid(0, 0, 0, 0);

    auto transform = customRotation(cloudPtr, centroid, origin_centroid, new_centroid, 0.0, 0.0, 0.0);

    // Identity transformation should be close to identity matrix
    EXPECT_NEAR(transform.matrix()(0, 0), 1.0, 1e-5);
    EXPECT_NEAR(transform.matrix()(1, 1), 1.0, 1e-5);
    EXPECT_NEAR(transform.matrix()(2, 2), 1.0, 1e-5);
    EXPECT_NEAR(transform.matrix()(3, 3), 1.0, 1e-5);
}

TEST(TransformationTest, CustomRotationXAxis) {
    // Test rotation around X-axis
    auto cloudPtr = PointCloudPtr(new PointCloud);
    cloudPtr->push_back(pcl::PointXYZ(1.0, 0.0, 0.0));

    Eigen::Vector4f centroid(0, 0, 0, 0);
    Eigen::Vector4f origin_centroid(0, 0, 0, 0);
    Eigen::Vector4f new_centroid(0, 0, 0, 0);

    auto transform = customRotation(cloudPtr, centroid, origin_centroid, new_centroid, 90.0, 0.0, 0.0);

    // Point on X-axis should remain unchanged after X-axis rotation
    auto transformedCloud = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*cloudPtr, *transformedCloud, transform);

    EXPECT_NEAR(transformedCloud->points[0].x, 1.0, 1e-5);
    EXPECT_NEAR(transformedCloud->points[0].y, 0.0, 1e-5);
    EXPECT_NEAR(transformedCloud->points[0].z, 0.0, 1e-5);
}

TEST(TransformationTest, CustomRotationYAxis) {
    // Test rotation around Y-axis
    auto cloudPtr = PointCloudPtr(new PointCloud);
    cloudPtr->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));

    Eigen::Vector4f centroid(0, 0, 0, 0);
    Eigen::Vector4f origin_centroid(0, 0, 0, 0);
    Eigen::Vector4f new_centroid(0, 0, 0, 0);

    auto transform = customRotation(cloudPtr, centroid, origin_centroid, new_centroid, 0.0, 90.0, 0.0);

    // Point on Y-axis should remain unchanged after Y-axis rotation
    auto transformedCloud = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*cloudPtr, *transformedCloud, transform);

    EXPECT_NEAR(transformedCloud->points[0].x, 0.0, 1e-5);
    EXPECT_NEAR(transformedCloud->points[0].y, 1.0, 1e-5);
    EXPECT_NEAR(transformedCloud->points[0].z, 0.0, 1e-5);
}

TEST(TransformationTest, CustomRotationZAxis) {
    // Test rotation around Z-axis
    auto cloudPtr = PointCloudPtr(new PointCloud);
    cloudPtr->push_back(pcl::PointXYZ(0.0, 0.0, 1.0));

    Eigen::Vector4f centroid(0, 0, 0, 0);
    Eigen::Vector4f origin_centroid(0, 0, 0, 0);
    Eigen::Vector4f new_centroid(0, 0, 0, 0);

    auto transform = customRotation(cloudPtr, centroid, origin_centroid, new_centroid, 0.0, 0.0, 90.0);

    // Point on Z-axis should remain unchanged after Z-axis rotation
    auto transformedCloud = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*cloudPtr, *transformedCloud, transform);

    EXPECT_NEAR(transformedCloud->points[0].x, 0.0, 1e-5);
    EXPECT_NEAR(transformedCloud->points[0].y, 0.0, 1e-5);
    EXPECT_NEAR(transformedCloud->points[0].z, 1.0, 1e-5);
}

TEST(TransformationTest, CustomTranslation) {
    // Test translation
    auto cloudPtr = PointCloudPtr(new PointCloud);
    cloudPtr->push_back(pcl::PointXYZ(1.0, 2.0, 3.0));

    Eigen::Affine3f inputTransform = Eigen::Affine3f::Identity();

    auto resultTransform = customTranslation(cloudPtr, inputTransform, 5.0, 10.0, 15.0);

    auto transformedCloud = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*cloudPtr, *transformedCloud, resultTransform);

    EXPECT_NEAR(transformedCloud->points[0].x, 6.0, 1e-5);
    EXPECT_NEAR(transformedCloud->points[0].y, 12.0, 1e-5);
    EXPECT_NEAR(transformedCloud->points[0].z, 18.0, 1e-5);
}

TEST(TransformationTest, CorrectedPointCloud) {
    // Test corrected point cloud transformation
    auto cloudPtr = PointCloudPtr(new PointCloud);
    cloudPtr->push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
    cloudPtr->push_back(pcl::PointXYZ(4.0, 5.0, 6.0));

    Eigen::Vector4f centroid(0, 0, 0, 0);
    Eigen::Vector4f origin_centroid(0, 0, 0, 0);
    Eigen::Vector4f new_centroid(0, 0, 0, 0);

    auto correctedCloud = correctedPointCloud(cloudPtr, centroid, origin_centroid, new_centroid, 0.0, 0.0, 0.0);

    // With no rotation and zero centroids, cloud should remain similar
    EXPECT_EQ(correctedCloud->size(), cloudPtr->size());
}

TEST(TransformationTest, Distance) {
    // Test distance calculation
    pcl::PointXYZ p1(0.0, 0.0, 0.0);
    pcl::PointXYZ p2(3.0, 4.0, 0.0);

    double dist = distance(p1, p2);
    EXPECT_NEAR(dist, 5.0, 1e-5);
}

TEST(TransformationTest, DistanceSamePoint) {
    // Test distance between same points
    pcl::PointXYZ p1(1.0, 2.0, 3.0);
    pcl::PointXYZ p2(1.0, 2.0, 3.0);

    double dist = distance(p1, p2);
    EXPECT_NEAR(dist, 0.0, 1e-5);
}
