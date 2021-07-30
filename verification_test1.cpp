#include "./include/tools.hpp"
#include <ctime>

// RUN FROM THE BUILD FOLDER

// RUN A REGISTRATION BETWEEN 2 POINT CLOUDS SAMPLES (source_verification1.pcd & target_verification1.pcd) WITH A PRE-HANDED CUSTOM TRANSFORMATION ON target_verification1.pcd
// Results expected = enough registration to be able to perform ICP i.e. the blue points & red points are well aligned in the viewer & the MTRE is very low ~ 1e-4

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
    // set up all required settings to register the point clouds data 
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
    // shows the point clouds before alignment
    visualizeMatchingResults(sourceCloudPtr, targetCloudPtr);

    PointCloudPtr newPtCloudPtr(new PointCloud);
    PointCloud &sourceTransformedCloud = *sourceCloudPtr;
    PointCloud &targetCloud = *targetCloudPtr;

    copyPointCloud(*sourceCloudPtr, *newPtCloudPtr);
    PointCloud &newPtCloud = *newPtCloudPtr;

    // pipeline with all points
    pipelineAllPointsOutputPtr pipelineOutputPtr;
    Eigen::Matrix4f final_transformation;
  
    PointCloudPtr finalCloudPtr(new PointCloud);
    PointCloudPtr transformedCloudPtr(new PointCloud);

    pipelineOutputPtr = pipelineAllPoints(sourceCloudPtr, targetCloudPtr, pipelineSettings);
    std::tie(sourceCloudPtr, targetCloudPtr, transformedCloudPtr, final_transformation) = pipelineOutputPtr;

    PointCloudPtr finalTransformedCloudPtr(new PointCloud);
    PointCloud &finalTransformedCloud = *finalTransformedCloudPtr;
    pcl::transformPointCloud(*sourceCloudPtr, finalTransformedCloud, final_transformation);
    copyPointCloud(*finalTransformedCloudPtr, *finalCloudPtr);

    // shows the point clouds after alignment
    visualizeMatchingResults(finalTransformedCloudPtr, targetCloudPtr);


}
