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
    ///////////////////////////////////// SIFT PIPELINE /////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////

    Detector SIFT;
    PtCloudPointWithScalePtr sourceSiftKeypointsPtr;
    PtCloudPointWithScalePtr targetSiftKeypointsPtr;
    //PointCloudPtr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);

    sourceSiftKeypointsPtr = SIFT.computeSiftKeypoints(sourceNormalsPtr,
                                                       min_scale_source,
                                                       n_octaves_source,
                                                       n_scales_per_octave_source,
                                                       min_contrast_source);

    targetSiftKeypointsPtr = SIFT.computeSiftKeypoints(targetNormalsPtr,
                                                       min_scale_target,
                                                       n_octaves_target,
                                                       n_scales_per_octave_target,
                                                       min_contrast_target);

    //copyPointCloud(*targetSiftKeypointsPtr, *cloud_temp);
    //pcl::io::savePCDFileASCII("sift_points_ref_6-8.pcd", *cloud_temp);

    //visualizePtClouds(sourceCloudPtr, targetCloudPtr);
    //visualizeSinglePointCloudForBRISK(sourceCloudPtr, sourceSiftKeypointsPtr);

    Descriptor FPFHSift;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceFPFHSift(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetFPFHSift(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeFPFH(new pcl::search::KdTree<pcl::PointXYZ>);

    sourceFPFHSift = FPFHSift.computeFPFH(sourceCloudPtr,
                                          sourceNormalsPtr,
                                          sourceSiftKeypointsPtr,
                                          treeFPFH,
                                          searchRadiusFPFH);

    targetFPFHSift = FPFHSift.computeFPFH(targetCloudPtr,
                                          targetNormalsPtr,
                                          targetSiftKeypointsPtr,
                                          treeFPFH,
                                          searchRadiusFPFH);

    SearchingMethods SampleConsensusIASift;
    Eigen::Matrix4f final_transformation_sift = Eigen::Matrix4f::Identity();
    final_transformation_sift = SampleConsensusIASift.computeSACInitialAlignment(sourceSiftKeypointsPtr,
                                                                                 targetSiftKeypointsPtr,
                                                                                 sourceFPFHSift,
                                                                                 targetFPFHSift,
                                                                                 min_sample_dist,
                                                                                 max_correspondence_dist,
                                                                                 nr_iters,
                                                                                 nr_samples);

    PointCloudPtr transformedCloudPtr1(new PointCloud);
    PointCloud &transformedCloud1 = *transformedCloudPtr1;

    pcl::transformPointCloud(sourceCloud, transformedCloud1, final_transformation_sift);

    visualizeMatchingResults(targetCloudPtr, transformedCloudPtr1);

    /*multipleViewPointCloudsSift(sourceCloudPtr,
                                targetCloudPtr,
                                sourceSiftKeypointsPtr,
                                targetSiftKeypointsPtr,
                                transformedCloudPtr);*/

    //visualizeSinglePointCloudForBRISK(targetCloudPtr, targetSiftKeypointsPtr);

    //visualizeSinglePointCloudForHarris(transformedCloudPtr2, trgIntensityHarrisKeypointsPtr);

    /*visualizeMultipleMatchingResults(targetCloudPtr,
                                     targetCloudPtr,
                                     transformedCloudPtr1,
                                     transformedCloudPtr2);*/

    return 0;
}