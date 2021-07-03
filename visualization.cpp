#include "./include/tools.hpp"

int main(int argc, char **argv)
{
    /*******************************************************/
    /****************** READ POINT CLOUDS ******************/
    /*******************************************************/
    PointCloudPtr sourceCloudPtr(new PointCloud);
    PointCloudPtr targetCloudPtr(new PointCloud);

    if (argc < 2)
    {
        throw std::runtime_error("Required arguments: source_pcl_filename.pcd target_pcl_filename.pcd");
    }
    std::string sourceFileName = argv[1];
    std::string targetFileName = argv[2];
    std::cout << "Reading " << sourceFileName << std::endl;
    std::cout << "Reading " << targetFileName << std::endl;

    sourceCloudPtr = loadingCloud(sourceFileName);
    targetCloudPtr = loadingCloud(targetFileName);

    PointCloud &sourceCloud = *sourceCloudPtr;
    PointCloud &targetCloud = *targetCloudPtr;

    visualizePtClouds(sourceCloudPtr, targetCloudPtr);

    return 0;
}