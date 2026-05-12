#include "../include/tools.hpp"
#include <ctime>

// RUN FROM THE BUILD FOLDER
// RUN A REGISTRATION BETWEEN 2 POINT CLOUDS SAMPLES (source_verification2.pcd & transformed source_verification2.pcd)
// WITH A CUSTOM TRANSFORMATION
// Results expected = enough registration to be able to perform ICP

int main(int argc, char **argv)
{
    time_t timetoday;
    time(&timetoday);
    std::cout << "verification_test2 initiated on : " << asctime(localtime(&timetoday)) << std::endl;

    Settings pipelineSettings = getPipelineDefaultSettings();
    pipelineSettings.setValue(SACIA_MIN_SAMPLE_DIST, 0.01f);
    pipelineSettings.setValue(SACIA_MAX_CORRESPONDENCE_DIST, 1.0f);
    pipelineSettings.setValue(SACIA_NUM_ITERATIONS, 400.0f);
    pipelineSettings.setValue(SACIA_NUM_SAMPLES, 1500.0f);
    pipelineSettings.setValue(NORMALS_SEARCH_RADIUS, 0.1);
    pipelineSettings.setValue(FPFH_SEARCH_RADIUS, 0.1);

    std::string sourceFilename = "../data/source_verification2.pcd";
    std::cout << "Reading file : " << sourceFilename << std::endl;
    auto sourceCloudPtr = loadingCloud(sourceFilename);
PointCloudPtr targetCloudPtr(new PointCloud);
copyPointCloud(*sourceCloudPtr, *targetCloudPtr);

    // Custom transformation: 70 degree rotation around Z
    Eigen::Affine3f transform = customRotation(
        sourceCloudPtr,
        Eigen::Vector4f::Zero(),
        Eigen::Vector4f::Zero(),
        Eigen::Vector4f::Zero(),
        0.0f, 0.0f, 70.0f);

    std::cout << "\nTransformation : \n" << transform.matrix() << std::endl;

    PointCloudPtr sourceTransformedCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceCloudPtr, *sourceTransformedCloudPtr, transform);

    // Shows the point clouds before alignment
    visualizeMatchingResults(sourceTransformedCloudPtr, targetCloudPtr);

    auto pipelineOutputPtr = pipelineAllPoints(sourceTransformedCloudPtr, targetCloudPtr, pipelineSettings);
    auto& transformedCloudPtr = std::get<2>(pipelineOutputPtr);
    auto& final_transformation = std::get<3>(pipelineOutputPtr);

    PointCloudPtr finalTransformedCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceTransformedCloudPtr, *finalTransformedCloudPtr, final_transformation);

    // Shows the point clouds after alignment
    visualizeMatchingResults(finalTransformedCloudPtr, targetCloudPtr);

    return 0;
}