#include <pcl/features/shot.h>

#include "./include/tools.hpp"

// RUN FROM THE BUILD FOLDER
// Test file for SHOT features - mostly inactive, kept for reference

int main(int argc, char** argv) {
    Descriptor shot;
    PointCloudPtr sourceCloudPtr(new PointCloud);
    PointCloudPtr targetCloudPtr(new PointCloud);
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormalsPtr(new pcl::PointCloud<pcl::Normal>);
    LocalDescriptorSHOTPtr shotFeaturesSrc(new pcl::PointCloud<SHOTFeature>);

    std::string sourcePath = "../data/y675_58_5025_17.pcd";
    std::string targetPath = "../data/reference_LM.pcd";

    sourceCloudPtr = loadingCloud(sourcePath);
    targetCloudPtr = loadingCloud(targetPath);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormals(new pcl::search::KdTree<pcl::PointXYZ>);
    sourceNormalsPtr = computeNormals(sourceCloudPtr, treeNormals, 2.0);

    // Setup the SHOT features
    pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, SHOTFeature> shotEstimation;
    shotEstimation.setInputCloud(sourceCloudPtr);
    shotEstimation.setInputNormals(sourceNormalsPtr);
    shotEstimation.setSearchMethod(treeNormals);
    shotEstimation.setRadiusSearch(2);
    shotEstimation.compute(*shotFeaturesSrc);

    std::cout << "SHOT output points.size (): " << shotFeaturesSrc->size() << std::endl;

    return 0;
}