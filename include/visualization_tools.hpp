//#include "registration.hpp"
#include "param_parsing.hpp"

////////////////////////////////
////////// PROTOTYPES //////////
////////////////////////////////

// single viewers tools
void visualizeSinglePointCloud(PointCloudPtr inputCloudPtr);
void visualizeMatchingResults(PointCloudPtr inputCloudPtr,
                              PointCloudPtr transformedCloudPtr);

void visualizeSinglePointCloudForHarris(PointCloudPtr inputCloudPtr,
                                        PtCloudPointWithIntensityPtr inputKeypointsCloudPtr);

void visualizeSinglePointCloudForBRISK(PointCloudPtr inputCloudPtr,
                                       PtCloudPointWithScalePtr inputKeypointsCloudPtr);
void visualizePtClouds(PointCloudPtr inputSrcCloudPtr,
                       PointCloudPtr inputTrgCloudPtr);

void visualizeResults(PointCloudPtr referenceCloudPtr,
                      PointCloudPtr sourceCloudPtr,
                      PointCloudPtr customTransformedCloudPtr,
                      PointCloudPtr transformedCloudPtr);

// multiple viewers tools
void visualizeMultipleResultsPointCloudHarris(PointCloudPtr sourceCloudPtr,
                                              PointCloudPtr targetCloudPtr,
                                              PtCloudPointWithIntensityPtr srcKeypointsCloudPtr,
                                              PtCloudPointWithIntensityPtr trgKeypointsCloudPtr,
                                              PointCloudPtr transformedCloudPtr);

void visualizeMultipleResultsPointCloudSift(PointCloudPtr inputSrcCloudPtr,
                                            PointCloudPtr inputTrgCloudPtr,
                                            PtCloudPointWithScalePtr sourceSiftKeypointsPtr,
                                            PtCloudPointWithScalePtr targetSiftKeypointsPtr,
                                            PointCloudPtr transformedCloudPtr);

void visualizeMultipleMatchingResults(PointCloudPtr inputTrgCloudPtr1,
                                      PointCloudPtr inputTrgCloudPtr2,
                                      PointCloudPtr transformedCloudPtr1,
                                      PointCloudPtr transformedCloudPtr2);

void visualizeMultipleResults(PointCloudPtr referenceCloudPtr1,
                              PointCloudPtr sourceCloudPtr1,
                              PointCloudPtr customTransformedCloudPtr1,
                              PointCloudPtr transformedCloudPtr1,
                              PointCloudPtr referenceCloudPtr2,
                              PointCloudPtr sourceCloudPtr2,
                              PointCloudPtr customTransformedCloudPtr2,
                              PointCloudPtr transformedCloudPtr2);

void visualizationToolSift(PointCloudPtr sourceCloudPtr,
                           PointCloudPtr targetCloudPtr,
                           PointCloudPtr transformedCloudPtr,
                           PtCloudPointWithScalePtr srcSiftKeypointsCloudPtr,
                           PtCloudPointWithScalePtr trgSiftKeypointsCloudPtr,
                           Settings pipelineSettings,
                           std::string pipelineType);

void visualizationToolHarris(PointCloudPtr sourceCloudPtr,
                             PointCloudPtr targetCloudPtr,
                             PointCloudPtr transformedCloudPtr,
                             PtCloudPointWithIntensityPtr srcHarrisKeypointsCloudPtr,
                             PtCloudPointWithIntensityPtr trgHarrisKeypointsCloudPtr,
                             Settings pipelineSettings,
                             std::string pipelineType);

/*******************************************************************************************************************************************************/
/*******************************************************************************************************************************************************/
/*******************************************************************************************************************************************************/

////////////////////////////////
//////////// METHODS ///////////
////////////////////////////////

/* Method to visualize single point cloud */
void visualizeSinglePointCloud(PointCloudPtr inputCloudPtr)
{
    // Visualization of keypoints along with the original cloud
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(inputCloudPtr, 255, 255, 0);

    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addPointCloud(inputCloudPtr, "cloud");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}

/* Method to visualize cropped results (3 point clouds) */
void visualizeCroppedResults(PointCloudPtr inputCloudPtr,
                             PointCloudPtr croppedReferenceCloudPtr,
                             PointCloudPtr croppedSourceCloudPtr)
{

    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addPointCloud(inputCloudPtr, "reference cloud");
    viewer.addPointCloud(croppedReferenceCloudPtr, "cropped reference cloud");
    viewer.addPointCloud(croppedSourceCloudPtr, "cropped source cloud");
    // Add transformed point cloud to viewer
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tf_cloud_color_handler1(inputCloudPtr, 255, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ>(inputCloudPtr, tf_cloud_color_handler1, "initial aligned cloud1");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tf_cloud_color_handler2(croppedReferenceCloudPtr, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(croppedReferenceCloudPtr, tf_cloud_color_handler2, "initial aligned cloud2");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tf_cloud_color_handler3(croppedSourceCloudPtr, 0, 0, 255);
    viewer.addPointCloud<pcl::PointXYZ>(croppedSourceCloudPtr, tf_cloud_color_handler3, "initial aligned cloud3");

    pcl::PointXYZ o;
    o.x = 0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere(o, 10, "sphere", 0);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
        pcl_sleep(0.05);
    }
}

/* Method to visualize 2 point clouds */
void visualizePtClouds(PointCloudPtr inputSrcCloudPtr,
                       PointCloudPtr inputTrgCloudPtr)
{
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addPointCloud(inputSrcCloudPtr, "source cloud");
    viewer.addPointCloud(inputTrgCloudPtr, "reference cloud");
    viewer.addCoordinateSystem(0.0, "source cloud", 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tf_cloud_color_handler1(inputSrcCloudPtr, 255, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ>(inputSrcCloudPtr, tf_cloud_color_handler1, "initial aligned cloud1");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tf_cloud_color_handler2(inputTrgCloudPtr, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(inputTrgCloudPtr, tf_cloud_color_handler2, "initial aligned cloud2");

    //viewer.initCameraParameters();
    //viewer.setCameraPosition(0, 0, 0, 0, 0, 0);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
        pcl_sleep(0.01);
    }
}

/* Method to visualize the Harris Keypoints */
void visualizeSinglePointCloudForHarris(PointCloudPtr inputCloudPtr,
                                        PtCloudPointWithIntensityPtr inputKeypointsCloudPtr)
{
    // Visualization of keypoints along with the original cloud
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> keypoints_color_handler(inputKeypointsCloudPtr, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(inputCloudPtr, 255, 255, 0);

    viewer.initCameraParameters();
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addPointCloud(inputCloudPtr, "cloud");
    viewer.addPointCloud(inputKeypointsCloudPtr, keypoints_color_handler, "keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}

/* Method to visualize multiple clouds with Harris Keypoints */
void visualizeMultipleResultsPointCloudHarris(PointCloudPtr sourceCloudPtr,
                                              PointCloudPtr targetCloudPtr,
                                              PtCloudPointWithIntensityPtr srcKeypointsCloudPtr,
                                              PtCloudPointWithIntensityPtr trgKeypointsCloudPtr,
                                              PointCloudPtr transformedCloudPtr)
{
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    int v1(0);
    int v2(1);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer.initCameraParameters();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_cloud_color_handler(sourceCloudPtr, 255, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tar_cloud_color_handler(targetCloudPtr, 0, 255, 255);
    // visualizer 1
    viewer.addPointCloud(sourceCloudPtr, src_cloud_color_handler, "source cloud v1", v1);
    viewer.addPointCloud(targetCloudPtr, tar_cloud_color_handler, "target cloud v1", v1);
    // visualizer 2
    viewer.addPointCloud(targetCloudPtr, tar_cloud_color_handler, "target cloud v2", v2);
    viewer.addPointCloud(transformedCloudPtr, tar_cloud_color_handler, "transformed cloud v2", v2);
    // Show keypoints in 3D viewer
    pcl::visualization::PointCloudColorHandlerCustom<PointWithIntensity> src_keypoints_color_handler(srcKeypointsCloudPtr, 255, 0, 0);
    viewer.addPointCloud<PointWithIntensity>(srcKeypointsCloudPtr, src_keypoints_color_handler, "source keypoints", v1);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "source keypoints");
    pcl::visualization::PointCloudColorHandlerCustom<PointWithIntensity> tar_keypoints_color_handler(trgKeypointsCloudPtr, 0, 0, 255);
    viewer.addPointCloud<PointWithIntensity>(trgKeypointsCloudPtr, tar_keypoints_color_handler, "target keypoints", v1);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "target keypoints");
    // Add transformed point cloud to viewer
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tf_cloud_color_handler(transformedCloudPtr, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(transformedCloudPtr, tf_cloud_color_handler, "initial aligned cloudv2", v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color_handler(targetCloudPtr, 0, 0, 255);
    viewer.addPointCloud<pcl::PointXYZ>(targetCloudPtr, target_cloud_color_handler, "aligned cloudv2", v2);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
        pcl_sleep(0.01);
    }
}

/* Method to visualize the matching results & the keypoints */
void visualizeMultipleResultsPointCloudSift(PointCloudPtr inputSrcCloudPtr,
                                            PointCloudPtr inputTrgCloudPtr,
                                            PtCloudPointWithScalePtr sourceSiftKeypointsPtr,
                                            PtCloudPointWithScalePtr targetSiftKeypointsPtr,
                                            PointCloudPtr transformedCloudPtr)
{
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    int v1(0);
    int v2(1);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer.initCameraParameters();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_cloud_color_handler(inputSrcCloudPtr, 255, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tar_cloud_color_handler(inputTrgCloudPtr, 0, 255, 255);
    // visualizer 1
    viewer.addPointCloud(inputSrcCloudPtr, src_cloud_color_handler, "source cloud v1", v1);
    viewer.addPointCloud(inputTrgCloudPtr, tar_cloud_color_handler, "target cloud v1", v1);
    // visualizer 2
    viewer.addPointCloud(inputTrgCloudPtr, tar_cloud_color_handler, "target cloud v2", v2);
    viewer.addPointCloud(transformedCloudPtr, tar_cloud_color_handler, "transformed cloud v2", v2);
    // Show keypoints in 3D viewer
    pcl::visualization::PointCloudColorHandlerCustom<PointWithScale> src_keypoints_color_handler(sourceSiftKeypointsPtr, 255, 0, 0);
    viewer.addPointCloud<PointWithScale>(sourceSiftKeypointsPtr, src_keypoints_color_handler, "source keypoints", v1);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "source keypoints");
    pcl::visualization::PointCloudColorHandlerCustom<PointWithScale> tar_keypoints_color_handler(targetSiftKeypointsPtr, 0, 0, 255);
    viewer.addPointCloud<PointWithScale>(targetSiftKeypointsPtr, tar_keypoints_color_handler, "target keypoints", v1);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "target keypoints");
    // Add transformed point cloud to viewer
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tf_cloud_color_handler(transformedCloudPtr, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(transformedCloudPtr, tf_cloud_color_handler, "initial aligned cloud", v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color_handler(inputTrgCloudPtr, 0, 0, 255);
    viewer.addPointCloud<pcl::PointXYZ>(inputTrgCloudPtr, target_cloud_color_handler, "aligned cloudv2", v2);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
        pcl_sleep(0.01);
    }
}

/* Method to visualize the BRISK Keypoints */
void visualizeSinglePointCloudForBRISK(PointCloudPtr inputCloudPtr,
                                       PtCloudPointWithScalePtr inputKeypointsCloudPtr)
{
    // Visualization of keypoints along with the original cloud
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    pcl::visualization::PointCloudColorHandlerCustom<PointWithScale> keypoints_color_handler(inputKeypointsCloudPtr, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(inputCloudPtr, 255, 255, 0);

    viewer.initCameraParameters();
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addPointCloud(inputCloudPtr, "cloud");
    viewer.addPointCloud(inputKeypointsCloudPtr, keypoints_color_handler, "keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}

/* Method to visualize the matching results */
void visualizeMatchingResults(PointCloudPtr inputCloudPtr,
                              PointCloudPtr transformedCloudPtr)
{

    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.initCameraParameters();
    viewer.setCameraPosition(200, 200, 200, 200, 200, 200);
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addPointCloud(inputCloudPtr, "reference cloud");
    viewer.addPointCloud(transformedCloudPtr, "transformed cloud");
    // Add transformed point cloud to viewer
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tf_cloud_color_handler1(inputCloudPtr, 255, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ>(inputCloudPtr, tf_cloud_color_handler1, "initial aligned cloud1");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tf_cloud_color_handler2(transformedCloudPtr, 0, 0, 255);
    viewer.addPointCloud<pcl::PointXYZ>(transformedCloudPtr, tf_cloud_color_handler2, "initial aligned cloud2");

    pcl::PointXYZ o;
    o.x = 0;
    o.y = 0;
    o.z = 0;

    viewer.addSphere(o, 1, "sphere", 0);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
        pcl_sleep(0.01);
    }
}

/* Method to visualize the comparison between matching results (2 point clouds) */
void visualizeMultipleMatchingResults(PointCloudPtr inputTrgCloudPtr1,
                                      PointCloudPtr inputTrgCloudPtr2,
                                      PointCloudPtr transformedCloudPtr1,
                                      PointCloudPtr transformedCloudPtr2)
{
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    int v1(0);
    int v2(1);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer.initCameraParameters();
    //viewer.setCameraPosition(0, 0, 0, 0, 0, 0);
    // Add transformed point cloud to viewer
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_cloud_color_handler(inputTrgCloudPtr1, 255, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tar_cloud_color_handler(inputTrgCloudPtr2, 0, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tf_cloud_color_handler1(transformedCloudPtr1, 0, 0, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tf_cloud_color_handler2(transformedCloudPtr2, 255, 0, 0);
    viewer.addPointCloud(inputTrgCloudPtr1, src_cloud_color_handler, "target cloud v1", v1);
    viewer.addPointCloud<pcl::PointXYZ>(transformedCloudPtr1, tf_cloud_color_handler1, "matching cloud1", v1);
    viewer.addPointCloud(inputTrgCloudPtr2, tar_cloud_color_handler, "target cloud v2", v2);
    viewer.addPointCloud<pcl::PointXYZ>(transformedCloudPtr2, tf_cloud_color_handler2, "matching cloud2", v2);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
        pcl_sleep(0.01);
    }
}

/* Method to visualize the comparison between matching results*/
void visualizeResults(PointCloudPtr referenceCloudPtr,
                      PointCloudPtr sourceCloudPtr,
                      PointCloudPtr customTransformedCloudPtr,
                      PointCloudPtr transformedCloudPtr)
{
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);

    // Add transformed point cloud to viewer
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ref_cloud_color_handler1(referenceCloudPtr, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_cloud_color_handler1(sourceCloudPtr, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tf_cloud_color_handler1(transformedCloudPtr, 0, 0, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cst_cloud_color_handler1(customTransformedCloudPtr, 135, 206, 235);

    viewer.addPointCloud(referenceCloudPtr, ref_cloud_color_handler1, "ref cloud v1");
    viewer.addPointCloud(sourceCloudPtr, src_cloud_color_handler1, "source cloud v1");
    viewer.addPointCloud(customTransformedCloudPtr, cst_cloud_color_handler1, "custom transformed cloud v1");
    viewer.addPointCloud<pcl::PointXYZ>(transformedCloudPtr, tf_cloud_color_handler1, "matched cloud1");

    pcl::PointXYZ o;
    o.x = 0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere(o, 10, "sphere", 0);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
        pcl_sleep(0.01);
    }
}

/* Method to visualize the comparison between matching results (3 point clouds))*/
void visualizeMultipleResults(PointCloudPtr referenceCloudPtr1,
                              PointCloudPtr sourceCloudPtr1,
                              PointCloudPtr customTransformedCloudPtr1,
                              PointCloudPtr transformedCloudPtr1,
                              PointCloudPtr referenceCloudPtr2,
                              PointCloudPtr sourceCloudPtr2,
                              PointCloudPtr customTransformedCloudPtr2,
                              PointCloudPtr transformedCloudPtr2)
{
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    int v1(0);
    int v2(1);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    // Add transformed point cloud to viewer
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ref_cloud_color_handler1(referenceCloudPtr1, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_cloud_color_handler1(sourceCloudPtr1, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tf_cloud_color_handler1(transformedCloudPtr1, 0, 0, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cst_cloud_color_handler1(customTransformedCloudPtr1, 135, 206, 235);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ref_cloud_color_handler2(referenceCloudPtr2, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_cloud_color_handler2(sourceCloudPtr2, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tf_cloud_color_handler2(transformedCloudPtr2, 0, 0, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cst_cloud_color_handler2(customTransformedCloudPtr2, 135, 206, 235);

    viewer.addPointCloud(referenceCloudPtr1, ref_cloud_color_handler1, "ref cloud v1", v1);
    viewer.addPointCloud(sourceCloudPtr1, src_cloud_color_handler1, "source cloud v1", v1);
    viewer.addPointCloud(customTransformedCloudPtr1, cst_cloud_color_handler1, "custom transformed cloud v1", v1);
    viewer.addPointCloud<pcl::PointXYZ>(transformedCloudPtr1, tf_cloud_color_handler1, "matched cloud1", v1);

    viewer.addPointCloud(referenceCloudPtr2, ref_cloud_color_handler2, "ref cloud v2", v2);
    viewer.addPointCloud(sourceCloudPtr2, src_cloud_color_handler2, "source cloud v2", v2);
    viewer.addPointCloud(customTransformedCloudPtr2, cst_cloud_color_handler2, "custom transformed cloud v2", v2);
    viewer.addPointCloud<pcl::PointXYZ>(transformedCloudPtr2, tf_cloud_color_handler2, "matched cloud2", v2);

    pcl::PointXYZ o;
    o.x = 0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere(o, 10, "sphere", 0);
    viewer.addSphere(o, 10, "sphere2", 1);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
        pcl_sleep(0.01);
    }
}

////////////////////////////////////////////

void visualizationToolSift(PointCloudPtr sourceCloudPtr,
                           PointCloudPtr targetCloudPtr,
                           PointCloudPtr transformedCloudPtr,
                           PtCloudPointWithScalePtr srcSiftKeypointsCloudPtr,
                           PtCloudPointWithScalePtr trgSiftKeypointsCloudPtr,
                           Settings pipelineSettings,
                           std::string pipelineType)
{
    //double visualizerParameter = pipelineSettings.getValue(VISUALIZER_PARAMETER);

    if (pipelineSettings.getValue(VISUALIZER_PARAMETER) == 0.0f)
    {
        std::cout << "\nNot displaying any results" << std::endl;
    }

    if (pipelineSettings.getValue(VISUALIZER_PARAMETER) == 1.0f)
    {
        visualizeMatchingResults(transformedCloudPtr, targetCloudPtr);
        std::cout << "\nDisplaying registration results of the point clouds" << std::endl;
    }

    if (pipelineSettings.getValue(VISUALIZER_PARAMETER) == 2.0f)
    {

        if (pipelineType == "sift")
        {
            std::cout << "\nDisplaying registration results of the point clouds & the sift associated keypoints" << std::endl;
            visualizeMultipleResultsPointCloudSift(sourceCloudPtr,
                                                   targetCloudPtr,
                                                   srcSiftKeypointsCloudPtr,
                                                   trgSiftKeypointsCloudPtr,
                                                   transformedCloudPtr);
        }
    }
}

void visualizationToolHarris(PointCloudPtr sourceCloudPtr,
                             PointCloudPtr targetCloudPtr,
                             PointCloudPtr transformedCloudPtr,
                             PtCloudPointWithIntensityPtr srcHarrisKeypointsCloudPtr,
                             PtCloudPointWithIntensityPtr trgHarrisKeypointsCloudPtr,
                             Settings pipelineSettings,
                             std::string pipelineType)
{
    //double visualizerParameter = pipelineSettings.getValue(VISUALIZER_PARAMETER);

    if (pipelineSettings.getValue(VISUALIZER_PARAMETER) == 0.0f)
    {
        std::cout << "\nNot displaying any results" << std::endl;
    }

    if (pipelineSettings.getValue(VISUALIZER_PARAMETER) == 1.0f)
    {
        visualizeMatchingResults(transformedCloudPtr, targetCloudPtr);
        std::cout << "\nDisplaying registration results of the point clouds" << std::endl;
    }

    if (pipelineSettings.getValue(VISUALIZER_PARAMETER) == 2.0f)
    {

        if (pipelineType == "harris")
        {
            std::cout << "\nDisplaying registration results of the point clouds & the harris associated keypoints" << std::endl;
            visualizeMultipleResultsPointCloudHarris(sourceCloudPtr,
                                                     targetCloudPtr,
                                                     srcHarrisKeypointsCloudPtr,
                                                     trgHarrisKeypointsCloudPtr,
                                                     transformedCloudPtr);
        }
    }
}