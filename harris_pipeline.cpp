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

    //visualizePtClouds(sourceCloudPtr, targetCloudPtr);

    /*******************************************************/
    /****************** COMPUTING NORMALS ******************/
    /*******************************************************/

    pcl::PointCloud<pcl::PointNormal>::Ptr sourceNormalsPtr;
    pcl::PointCloud<pcl::PointNormal>::Ptr targetNormalsPtr;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormals(new pcl::search::KdTree<pcl::PointXYZ>);

    //sourceNormalsPtr = computePointNormals(sourceCloudPtr, treeNormals, srcSearchRadiusSift);
    sourceNormalsPtr = computePointNormals(sourceCloudPtr, treeNormals, SearchRadiusSift);
    targetNormalsPtr = computePointNormals(targetCloudPtr, treeNormals, SearchRadiusSift);

    /////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////// HARRIS PIPELINE ///////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////

    Detector Harris;
    PtCloudPointWithIntensityPtr srcIntensityHarrisKeypointsPtr;
    PtCloudPointWithIntensityPtr trgIntensityHarrisKeypointsPtr;

    srcIntensityHarrisKeypointsPtr = Harris.computeHarris3DKeypoints(sourceNormalsPtr, SearchRadiusHarris, threshold_harris);
    trgIntensityHarrisKeypointsPtr = Harris.computeHarris3DKeypoints(targetNormalsPtr, SearchRadiusHarris, threshold_harris);

    visualizePtClouds(sourceCloudPtr,
                      targetCloudPtr);
    //visualizeSinglePointCloudForHarris(targetCloudPtr, trgIntensityHarrisKeypointsPtr);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceFPFHHarris(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetFPFHHarris(new pcl::PointCloud<pcl::FPFHSignature33>);

    Descriptor FPFHHarris;

    sourceFPFHHarris = FPFHHarris.computeFPFH(sourceCloudPtr,
                                              sourceNormalsPtr,
                                              srcIntensityHarrisKeypointsPtr,
                                              treeNormals,
                                              searchRadiusFPFH);

    targetFPFHHarris = FPFHHarris.computeFPFH(targetCloudPtr,
                                              targetNormalsPtr,
                                              trgIntensityHarrisKeypointsPtr,
                                              treeNormals,
                                              searchRadiusFPFH);

    SearchingMethods SampleConsensusIAHarris;
    Eigen::Matrix4f final_transformation_harris = Eigen::Matrix4f::Identity();

    final_transformation_harris = SampleConsensusIAHarris.computeSACInitialAlignment(srcIntensityHarrisKeypointsPtr,
                                                                                     trgIntensityHarrisKeypointsPtr,
                                                                                     sourceFPFHHarris,
                                                                                     targetFPFHHarris,
                                                                                     min_sample_dist,
                                                                                     max_correspondence_dist,
                                                                                     nr_iters,
                                                                                     nr_samples);

    PointCloudPtr transformedCloudPtr2(new PointCloud);
    PointCloud &transformedCloud2 = *transformedCloudPtr2;
    pcl::transformPointCloud(sourceCloud, transformedCloud2, final_transformation_harris);

    visualizeMatchingResults(targetCloudPtr, transformedCloudPtr2);

    //visualizeSinglePointCloudForBRISK(targetCloudPtr, targetSiftKeypointsPtr);
    /*visualizeMultipleMatchingResults(sourceCloudPtr,
                                     sourceCloudPtr,
                                     transformedCloudPtr1,
                                     transformedCloudPtr2);*/
    /*multipleViewPointCloudsSift(sourceCloudPtr,
                                targetCloudPtr,
                                sourceSiftKeypointsPtr,
                                targetSiftKeypointsPtr,
                                transformedCloudPtr);*/

    //visualizeSinglePointCloudForHarris(transformedCloudPtr2, trgIntensityHarrisKeypointsPtr);

    return 0;
}