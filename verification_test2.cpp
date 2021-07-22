#include "./include/tools.hpp"
#include <ctime>
// RUN FROM THE BUILD FOLDER

int main(int argc, char **argv)
{
    time_t timetoday;
    time(&timetoday);
    std::cout << "experiment2 initiated on : " << asctime(localtime(&timetoday)) << std::endl;

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
    std::string sourceFilename = "../data/source_verification2.pcd";
    std::cout << "Reading file : " << sourceFilename << std::endl;
    sourceCloudPtr = loadingCloud(sourceFilename);
    copyPointCloud(*sourceCloudPtr, *targetCloudPtr);

    Eigen::Vector4f centroid(Eigen::Vector4f::Zero());
    Eigen::Vector4f origin_centroid(Eigen::Vector4f::Zero());
    Eigen::Vector4f new_centroid(Eigen::Vector4f::Zero());
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    PointCloudPtr newPtCloudPtr(new PointCloud);
    PointCloudPtr sourceTransformedCloudPtr(new PointCloud);

    copyPointCloud(*sourceCloudPtr, *newPtCloudPtr);

    PointCloud &sourceTransformedCloud = *sourceTransformedCloudPtr;
    PointCloud &newPtCloud = *newPtCloudPtr;
    PointCloud &targetCloud = *targetCloudPtr;

    double randomAngleOnX(0), randomAngleOnY(0), randomAngleOnZ(70);

    transform = customRotation(newPtCloudPtr,
                               centroid,
                               origin_centroid,
                               new_centroid,
                               randomAngleOnX, randomAngleOnY, randomAngleOnZ);

    std::cout << "\nTransformation : \n"
              << transform.matrix() << std::endl;

    pcl::transformPointCloud(*sourceCloudPtr, sourceTransformedCloud, transform);
    visualizeMatchingResults(sourceTransformedCloudPtr, targetCloudPtr);

    Eigen::Matrix4f rotation_mat;
    PointCloudPtr finalCloudPtr(new PointCloud);
    // get the transformation matrix
    rotation_mat = pipelineAllPoints(sourceTransformedCloudPtr, targetCloudPtr, pipelineSettings);

    PointCloudPtr finalTransformedCloudPtr(new PointCloud);
    PointCloud &finalTransformedCloud = *finalTransformedCloudPtr;
    pcl::transformPointCloud(*sourceTransformedCloudPtr, finalTransformedCloud, rotation_mat);

    visualizeMatchingResults(finalTransformedCloudPtr, targetCloudPtr);

    return 0;
}