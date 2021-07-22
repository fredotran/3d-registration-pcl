#include "./include/tools.hpp"
#include <ctime>

// RUN FROM THE BUILD FOLDER

int main(int argc, char **argv)
{
    time_t timetoday;
    time(&timetoday);
    std::cout << "verification_test1 initiated on : " << asctime(localtime(&timetoday)) << std::endl;
    
    PointCloudPtr sourceCloudPtr(new PointCloud);
    PointCloudPtr targetCloudPtr(new PointCloud);
    PointCloudPtr outPointCloudPtr(new PointCloud);
    PointCloudPtr croppedPointCloudPtr(new PointCloud);

    // define maps that can be filled with algorithm-specific settings (e.g. for SAC-IA)
    Settings pipelineSettings = getPipelineDefaultSettings();
    // set up all required settings
    pipelineSettings.setValue(SACIA_MIN_SAMPLE_DIST, 0.01f);
    pipelineSettings.setValue(SACIA_MAX_CORRESPONDENCE_DIST, 1.0f);
    pipelineSettings.setValue(SACIA_NUM_ITERATIONS, 400.0f);
    pipelineSettings.setValue(SACIA_NUM_SAMPLES, 1500.0f);

    pipelineSettings.setValue(NORMALS_SEARCH_RADIUS, 0.1);  
    pipelineSettings.setValue(FPFH_SEARCH_RADIUS, 0.1); 

    // Reading pt cloud
    std::string sourceFilename = "../data/source_verification1.pcd";
    std::string targetFilename = "../data/target_verification1.pcd";
    std::cout << "Reading source file : " << sourceFilename << std::endl;
    std::cout << "Reading target file : " << targetFilename << std::endl;
    sourceCloudPtr = loadingCloud(sourceFilename);
    targetCloudPtr = loadingCloud(targetFilename);

    visualizeMatchingResults(sourceCloudPtr, targetCloudPtr);

    PointCloudPtr newPtCloudPtr(new PointCloud);
    PointCloud &sourceTransformedCloud = *sourceCloudPtr;
    PointCloud &targetCloud = *targetCloudPtr;

    copyPointCloud(*sourceCloudPtr, *newPtCloudPtr);
    PointCloud &newPtCloud = *newPtCloudPtr;

    Eigen::Matrix4f rotation_mat;
    PointCloudPtr finalCloudPtr(new PointCloud);
    // get the transformation matrix
    rotation_mat = pipelineAllPoints(newPtCloudPtr, targetCloudPtr, pipelineSettings);

    PointCloudPtr finalTransformedCloudPtr(new PointCloud);
    PointCloud &finalTransformedCloud = *finalTransformedCloudPtr;
    pcl::transformPointCloud(newPtCloud, finalTransformedCloud, rotation_mat);

    visualizeMatchingResults(finalTransformedCloudPtr, targetCloudPtr);


}
