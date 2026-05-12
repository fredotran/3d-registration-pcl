#include "../include/tools.hpp"
#include <ctime>

// RUN FROM THE BUILD FOLDER
// RUN A REGISTRATION BETWEEN 2 POINT CLOUDS SAMPLES (source_verification1.pcd & target_verification1.pcd)
// WITH A PRE-HANDED CUSTOM TRANSFORMATION ON target_verification1.pcd
// Results expected = enough registration to be able to perform ICP

int main(int argc, char **argv)
{
    time_t timetoday;
    time(&timetoday);
    std::cout << "verification_test1 initiated on : " << asctime(localtime(&timetoday)) << std::endl;

    Settings pipelineSettings = getPipelineDefaultSettings();
    pipelineSettings.setValue(SACIA_MIN_SAMPLE_DIST, 0.01f);
    pipelineSettings.setValue(SACIA_MAX_CORRESPONDENCE_DIST, 1.0f);
    pipelineSettings.setValue(SACIA_NUM_ITERATIONS, 400.0f);
    pipelineSettings.setValue(SACIA_NUM_SAMPLES, 1500.0f);
    pipelineSettings.setValue(NORMALS_SEARCH_RADIUS, 0.1);
    pipelineSettings.setValue(FPFH_SEARCH_RADIUS, 0.1);

    std::string sourceFilename = "../data/source_verification1.pcd";
    std::string targetFilename = "../data/target_verification1.pcd";
    std::cout << "Reading source file : " << sourceFilename << std::endl;
    std::cout << "Reading target file : " << targetFilename << std::endl;

    auto sourceCloudPtr = loadingCloud(sourceFilename);
    auto targetCloudPtr = loadingCloud(targetFilename);

    // Shows the point clouds before alignment
    visualizeMatchingResults(sourceCloudPtr, targetCloudPtr);

    auto pipelineOutputPtr = pipelineAllPoints(sourceCloudPtr, targetCloudPtr, pipelineSettings);
    auto& transformedCloudPtr = std::get<2>(pipelineOutputPtr);
    auto& final_transformation = std::get<3>(pipelineOutputPtr);

    PointCloudPtr finalTransformedCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceCloudPtr, *finalTransformedCloudPtr, final_transformation);

    // Shows the point clouds after alignment
    visualizeMatchingResults(finalTransformedCloudPtr, targetCloudPtr);

    return 0;
}