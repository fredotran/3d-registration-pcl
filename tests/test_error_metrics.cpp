#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "parameters.hpp"
#include "tools.hpp"

TEST(ErrorMetricsTest, MeanTargetRegistrationErrorIdentity) {
    // Test MTRE with identical clouds (should be zero)
    auto cloud1 = PointCloudPtr(new PointCloud);
    auto cloud2 = PointCloudPtr(new PointCloud);

    cloud1->push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
    cloud1->push_back(pcl::PointXYZ(4.0, 5.0, 6.0));

    cloud2->push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
    cloud2->push_back(pcl::PointXYZ(4.0, 5.0, 6.0));

    double mtre = meanTargetRegistrationError(cloud1, cloud2);
    EXPECT_NEAR(mtre, 0.0, 1e-5);
}

TEST(ErrorMetricsTest, MeanTargetRegistrationErrorSimple) {
    // Test MTRE with known displacement
    auto cloud1 = PointCloudPtr(new PointCloud);
    auto cloud2 = PointCloudPtr(new PointCloud);

    cloud1->push_back(pcl::PointXYZ(0.0, 0.0, 0.0));
    cloud1->push_back(pcl::PointXYZ(1.0, 0.0, 0.0));

    cloud2->push_back(pcl::PointXYZ(1.0, 0.0, 0.0));  // Displaced by 1.0
    cloud2->push_back(pcl::PointXYZ(2.0, 0.0, 0.0));  // Displaced by 1.0

    double mtre = meanTargetRegistrationError(cloud1, cloud2);
    EXPECT_NEAR(mtre, 1.0, 1e-5);
}

TEST(ErrorMetricsTest, MeanTargetRegistrationError3D) {
    // Test MTRE with 3D displacement
    auto cloud1 = PointCloudPtr(new PointCloud);
    auto cloud2 = PointCloudPtr(new PointCloud);

    cloud1->push_back(pcl::PointXYZ(0.0, 0.0, 0.0));

    cloud2->push_back(pcl::PointXYZ(3.0, 4.0, 0.0));  // Distance = 5.0

    double mtre = meanTargetRegistrationError(cloud1, cloud2);
    EXPECT_NEAR(mtre, 5.0, 1e-5);
}

TEST(ErrorMetricsTest, RegistrationErrorBiasIdentity) {
    // Test registration error bias with identical clouds (should be zero)
    auto cloud1 = PointCloudPtr(new PointCloud);
    auto cloud2 = PointCloudPtr(new PointCloud);

    cloud1->push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
    cloud1->push_back(pcl::PointXYZ(4.0, 5.0, 6.0));

    cloud2->push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
    cloud2->push_back(pcl::PointXYZ(4.0, 5.0, 6.0));

    auto [bias_x, bias_y, bias_z] = registrationErrorBias(cloud1, cloud2);
    EXPECT_NEAR(bias_x, 0.0, 1e-5);
    EXPECT_NEAR(bias_y, 0.0, 1e-5);
    EXPECT_NEAR(bias_z, 0.0, 1e-5);
}

TEST(ErrorMetricsTest, RegistrationErrorBiasTranslation) {
    // Test registration error bias with uniform translation
    auto cloud1 = PointCloudPtr(new PointCloud);
    auto cloud2 = PointCloudPtr(new PointCloud);

    cloud1->push_back(pcl::PointXYZ(0.0, 0.0, 0.0));
    cloud1->push_back(pcl::PointXYZ(1.0, 1.0, 1.0));

    cloud2->push_back(pcl::PointXYZ(2.0, 3.0, 4.0));  // Translated by (2, 3, 4)
    cloud2->push_back(pcl::PointXYZ(3.0, 4.0, 5.0));  // Translated by (2, 3, 4)

    auto [bias_x, bias_y, bias_z] = registrationErrorBias(cloud1, cloud2);
    EXPECT_NEAR(bias_x, -2.0, 1e-5);
    EXPECT_NEAR(bias_y, -3.0, 1e-5);
    EXPECT_NEAR(bias_z, -4.0, 1e-5);
}

TEST(ErrorMetricsTest, GetCoordinates) {
    // Test coordinate extraction
    auto cloud = PointCloudPtr(new PointCloud);

    cloud->push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
    cloud->push_back(pcl::PointXYZ(4.0, 5.0, 6.0));
    cloud->push_back(pcl::PointXYZ(7.0, 8.0, 9.0));

    auto [x_coords, y_coords, z_coords] = getCoordinates(cloud);

    EXPECT_EQ(x_coords.size(), 3);
    EXPECT_EQ(y_coords.size(), 3);
    EXPECT_EQ(z_coords.size(), 3);

    EXPECT_NEAR(x_coords[0], 1.0, 1e-5);
    EXPECT_NEAR(y_coords[0], 2.0, 1e-5);
    EXPECT_NEAR(z_coords[0], 3.0, 1e-5);

    EXPECT_NEAR(x_coords[1], 4.0, 1e-5);
    EXPECT_NEAR(y_coords[1], 5.0, 1e-5);
    EXPECT_NEAR(z_coords[1], 6.0, 1e-5);

    EXPECT_NEAR(x_coords[2], 7.0, 1e-5);
    EXPECT_NEAR(y_coords[2], 8.0, 1e-5);
    EXPECT_NEAR(z_coords[2], 9.0, 1e-5);
}

TEST(ErrorMetricsTest, Get3DCoordinatesXYZ) {
    // Test 3D coordinate extraction
    auto cloud = PointCloudPtr(new PointCloud);

    cloud->push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
    cloud->push_back(pcl::PointXYZ(4.0, 5.0, 6.0));

    auto coords = Get3DCoordinatesXYZ(cloud);

    EXPECT_EQ(coords.size(), 2);

    EXPECT_NEAR(coords[0].x, 1.0, 1e-5);
    EXPECT_NEAR(coords[0].y, 2.0, 1e-5);
    EXPECT_NEAR(coords[0].z, 3.0, 1e-5);

    EXPECT_NEAR(coords[1].x, 4.0, 1e-5);
    EXPECT_NEAR(coords[1].y, 5.0, 1e-5);
    EXPECT_NEAR(coords[1].z, 6.0, 1e-5);
}

TEST(ErrorMetricsTest, AppendTimestamp) {
    // Test timestamp appending
    std::string filename = "test_results";
    std::string result = appendTimestamp(filename);

    // Result should contain original filename and timestamp
    EXPECT_TRUE(result.find("test_results") != std::string::npos);
    EXPECT_TRUE(result.find("_") != std::string::npos);
    // Should not have extension
    EXPECT_TRUE(result.find(".csv") == std::string::npos);
}
