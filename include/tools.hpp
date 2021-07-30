//#include "visualization_tools.hpp"
#include "registration.hpp"

////////////////////////////////
////////// PROTOTYPES //////////
////////////////////////////////

// reading pointCloud
PointCloudPtr loadingCloud(std::string fileName);

// Transformation tools
PointCloudPtr correctedPointCloud(PointCloudPtr sourcePointCloudPtr,
                                  Eigen::Vector4f centroid,
                                  Eigen::Vector4f origin_centroid,
                                  Eigen::Vector4f new_centroid,
                                  float theta_x, float theta_y, float theta_z);
Eigen::Affine3f customRotation(PointCloudPtr sourcePointCloudPtr,
                               Eigen::Vector4f centroid,
                               Eigen::Vector4f origin_centroid,
                               Eigen::Vector4f new_centroid,
                               float theta_x, float theta_y, float theta_z);
Eigen::Affine3f customTranslation(PointCloudPtr sourcePointCloudPtr,
                                  Eigen::Affine3f inputTransformationMatrix,
                                  float trans_x, float trans_y, float trans_z);

// Compute point clouds and processing
PointCloudPtr cropPointCloud(PointCloudPtr cloudPtr,
                             PointCloudPtr cloudOutPtr,
                             double xMin, double xMax,
                             double yMin, double yMax);
PointCloudPtr computeReferenceCloud(PointCloudPtr surfaceModelCloudPtr,
                                    double sourceWidth, double sourceHeight,
                                    double x_uncertainty, double y_uncertainty);

/*PointCloudPtr computeSourceCloud(PointCloudPtr targetCloudPtr,
                                 double sourceWidth, double sourceHeight,
                                 double x_uncertainty, double y_uncertainty); */

// full pipelines
void fullRegistration(std::string &fullParametersFilename,
                      Settings pipelineSettings,
                      std::string &savedFilename,
                      double *seedRef,
                      double *seedSource,
                      double *seedCustomRotation);
tuplePointCloudPtr fullPipelineSift(tupleParameters parametersList, double *seedRef, double *seedSource,
                                    double *seedCustomRotation, Settings pipelineSettings);
tuplePointCloudPtr fullPipelineHarris(tupleParameters parametersList, double *seedRef, double *seedSource,
                                      double *seedCustomRotation, Settings pipelineSettings);
tuplePointCloudPtr fullPipeline(tupleParameters parametersList, double *seedRef, double *seedSource, double *seedCustomRotation,
                                Settings pipelineSettings);

/*******************************************************************************************************************************************************/
/*******************************************************************************************************************************************************/
/*******************************************************************************************************************************************************/

////////////////////////////////
//////////// METHODS ///////////
////////////////////////////////

/* Method to read and load the point cloud using its filename */
PointCloudPtr loadingCloud(std::string fileName)
{
    PointCloudPtr CloudPtr(new PointCloud);
    PointCloud &Cloud = *CloudPtr;

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName, Cloud) == -1) // load the file
    {
        PCL_ERROR("Couldn't read file\n");
        exit(0);
    }
    std::cout << "Loaded source file : " << CloudPtr->size() << " points." << std::endl;
    // Remove NaN values from point clouds
    std::vector<int> nanValues;
    pcl::removeNaNFromPointCloud(Cloud, Cloud, nanValues);

    return CloudPtr;
}

/* Method to perform a custom transformation on an input point cloud and returns a resulted point cloud */
PointCloudPtr correctedPointCloud(PointCloudPtr sourcePointCloudPtr,
                                  Eigen::Vector4f centroid,
                                  Eigen::Vector4f origin_centroid,
                                  Eigen::Vector4f new_centroid,
                                  float theta_x, float theta_y, float theta_z)
{

    PointCloudPtr localPointCloudPtr(new PointCloud);
    copyPointCloud(*sourcePointCloudPtr, *localPointCloudPtr);

    Eigen::Affine3f transform_translation = Eigen::Affine3f::Identity();
    pcl::compute3DCentroid(*localPointCloudPtr, centroid);
    transform_translation.translation() = new_centroid.head<3>() - centroid.head<3>();

    PointCloudPtr transformed_cloud(new PointCloud);
    // You can either apply transform_1 or transform_2; they are the same
    std::cout << "Transformation used for the origin correction : \n"
              << transform_translation.matrix() << std::endl;
    pcl::transformPointCloud(*localPointCloudPtr, *transformed_cloud, transform_translation);
    std::cout << "\nSource point cloud (white) " << std::endl;
    std::cout << "Corrected coordinates point cloud (green) " << std::endl;

    pcl::compute3DCentroid(*transformed_cloud, new_centroid);
    std::cout << "\nNew centroid coordinates (after correction) : \n"
              << new_centroid << std::endl;

    PointCloudPtr rotation_cloud(new PointCloud);
    Eigen::Affine3f transform_rotation = Eigen::Affine3f::Identity();
    Eigen::Matrix3f rotationX(Eigen::AngleAxisf((theta_x * M_PI) / 180, Eigen::Vector3f::UnitX()));
    Eigen::Matrix3f rotationY(Eigen::AngleAxisf((theta_y * M_PI) / 180, Eigen::Vector3f::UnitY()));
    Eigen::Matrix3f rotationZ(Eigen::AngleAxisf((theta_z * M_PI) / 180, Eigen::Vector3f::UnitZ()));
    // The same rotation matrix as before; theta radians around X axis
    transform_rotation.rotate(rotationX);
    // The same rotation matrix as before; theta radians around Y axis
    transform_rotation.rotate(rotationY);
    // The same rotation matrix as before; theta radians around Z axis
    transform_rotation.rotate(rotationZ);
    std::cout << "\nTransformation used for the rotation : \n"
              << transform_rotation.matrix() << std::endl;
    pcl::transformPointCloud(*transformed_cloud, *rotation_cloud, transform_rotation);

    // Transform back the pcl to the default , using the inverse
    PointCloudPtr new_cloud(new PointCloud);
    Eigen::Affine3f inverse_transform_translation = Eigen::Affine3f::Identity();
    inverse_transform_translation = transform_translation.inverse();
    pcl::transformPointCloud(*rotation_cloud, *new_cloud, inverse_transform_translation);

    return new_cloud;
}

/* Method to do a custom rotation of the point cloud */
Eigen::Affine3f customRotation(PointCloudPtr sourcePointCloudPtr,
                               Eigen::Vector4f centroid,
                               Eigen::Vector4f origin_centroid,
                               Eigen::Vector4f new_centroid,
                               float theta_x, float theta_y, float theta_z)
{

    PointCloudPtr localPointCloudPtr(new PointCloud);
    copyPointCloud(*sourcePointCloudPtr, *localPointCloudPtr);
    Eigen::Affine3f transform_translation = Eigen::Affine3f::Identity();
    pcl::compute3DCentroid(*localPointCloudPtr, centroid);
    transform_translation.translation() = new_centroid.head<3>() - centroid.head<3>();

    PointCloudPtr transformed_cloud(new PointCloud);
    std::cout << "\nSource point cloud (red) " << std::endl;
    std::cout << "Corrected coordinates point cloud (green) " << std::endl;

    pcl::compute3DCentroid(*transformed_cloud, new_centroid);
    std::cout << "\nNew centroid coordinates (after correction) : \n"
              << new_centroid << std::endl;

    PointCloudPtr rotation_cloud(new PointCloud);
    Eigen::Affine3f transform_rotation = Eigen::Affine3f::Identity();
    Eigen::Matrix3f rotationX(Eigen::AngleAxisf((theta_x * M_PI) / 180, Eigen::Vector3f::UnitX()));
    Eigen::Matrix3f rotationY(Eigen::AngleAxisf((theta_y * M_PI) / 180, Eigen::Vector3f::UnitY()));
    Eigen::Matrix3f rotationZ(Eigen::AngleAxisf((theta_z * M_PI) / 180, Eigen::Vector3f::UnitZ()));
    // The same rotation matrix as before; theta radians around X axis
    transform_rotation.rotate(rotationX);
    // The same rotation matrix as before; theta radians around Y axis
    transform_rotation.rotate(rotationY);
    // The same rotation matrix as before; theta radians around Z axis
    transform_rotation.rotate(rotationZ);

    // Transform back the pcl to the default , using the inverse
    PointCloudPtr new_cloud(new PointCloud);
    Eigen::Affine3f inverse_transform_translation = Eigen::Affine3f::Identity();
    inverse_transform_translation = transform_translation.inverse();
    Eigen::Affine3f final_matrix = (inverse_transform_translation * transform_rotation) * transform_translation;
    Eigen::Affine3f inverse_final_matrix = final_matrix.inverse();

    return final_matrix;
}

/* Methods to do an translation offset of the point cloud */
Eigen::Affine3f customTranslation(PointCloudPtr sourcePointCloudPtr,
                                  Eigen::Affine3f inputTransformationMatrix,
                                  float trans_x, float trans_y, float trans_z)
{

    PointCloudPtr localPointCloudPtr(new PointCloud);
    copyPointCloud(*sourcePointCloudPtr, *localPointCloudPtr);
    // Doing the offset translation
    inputTransformationMatrix.translation() << trans_x, trans_y, trans_z;
    // Define translations.
    PointCloudPtr final_cloud(new PointCloud);
    pcl::transformPointCloud(*localPointCloudPtr, *final_cloud, inputTransformationMatrix);

    return inputTransformationMatrix;
}

/* Returns the cropped point cloud based on the dimensions of the input cloud and specification of x/y cutout limits */
PointCloudPtr cropPointCloud(PointCloudPtr cloudPtr,
                             PointCloudPtr cloudOutPtr,
                             double xMin, double xMax,
                             double yMin, double yMax)
{
    // GetMinMax of the input point cloud
    pcl::PointXYZ ref_min_pt, ref_max_pt;
    pcl::getMinMax3D(*cloudPtr, ref_min_pt, ref_max_pt);
    double zBoundariesRef[2] = {ref_min_pt.z, ref_max_pt.z};

    // Min (left)
    Eigen::Vector4f minPoints;
    minPoints[0] = xMin;              // x axis
    minPoints[1] = yMin;              // y axis
    minPoints[2] = zBoundariesRef[0]; // z axis
    minPoints[3] = 1.0;               // z axis

    // Max (right)
    Eigen::Vector4f maxPoints;
    maxPoints[0] = xMax;              // x axis
    maxPoints[1] = yMax;              // y axis
    maxPoints[2] = zBoundariesRef[1]; // z axis
    maxPoints[3] = 1.0;               // z axis

    // crop point cloud
    pcl::CropBox<PointXYZ> cropFilter;
    cropFilter.setInputCloud(cloudPtr);
    cropFilter.setMin(minPoints);
    cropFilter.setMax(maxPoints);
    cropFilter.filter(*cloudOutPtr);

    return cloudOutPtr;
}

/* Compute reference point cloud */
PointCloudPtr computeReferenceCloud(PointCloudPtr surfaceModelCloudPtr, double *seedRef,
                                    double sourceWidth, double sourceHeight,
                                    double x_uncertainty, double y_uncertainty)
{

    PointCloudPtr targetCloudPtr(new PointCloud);
    PointCloudPtr croppedPointCloudPtr(new PointCloud);
    PointCloudPtr outPointCloudPtr(new PointCloud);
    pcl::PointXYZ ref_min_pt, ref_max_pt;
    pcl::getMinMax3D(*surfaceModelCloudPtr, ref_min_pt, ref_max_pt);
    double xBoundariesRef[2] = {ref_min_pt.x, ref_max_pt.x};
    double yBoundariesRef[2] = {ref_min_pt.y, ref_max_pt.y};
    double zBoundariesRef[2] = {ref_min_pt.z, ref_max_pt.z};

    // Compute reference
    std::array<double, 4> coordRandomizedRef;
    coordRandomizedRef = randomizeReferenceCutout(seedRef, xBoundariesRef, yBoundariesRef,
                                                  sourceWidth, sourceHeight,
                                                  x_uncertainty, y_uncertainty);

    double refRandomizedX_min = coordRandomizedRef[0];
    double refRandomizedX_max = coordRandomizedRef[1];
    double refRandomizedY_min = coordRandomizedRef[2];
    double refRandomizedY_max = coordRandomizedRef[3];
    // cropping
    croppedPointCloudPtr = cropPointCloud(surfaceModelCloudPtr,
                                          outPointCloudPtr,
                                          refRandomizedX_min, refRandomizedX_max,
                                          refRandomizedY_min, refRandomizedY_max);

    copyPointCloud(*croppedPointCloudPtr, *targetCloudPtr);

    return targetCloudPtr;
}

/* Compute source point cloud */
PointCloudPtr computeSourceCloud(PointCloudPtr targetCloudPtr, double *seed,
                                 double sourceWidth, double sourceHeight,
                                 double x_uncertainty, double y_uncertainty)
{
    PointCloudPtr sourceCloudPtr(new PointCloud);
    PointCloudPtr outPointCloudPtr(new PointCloud);
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*targetCloudPtr, min_pt, max_pt);
    double xBoundaries[2] = {min_pt.x, max_pt.x};
    double yBoundaries[2] = {min_pt.y, max_pt.y};
    double zBoundaries[2] = {min_pt.z, max_pt.z};

    // Compute source data
    std::array<double, 4> coordSourceRandomized;
    coordSourceRandomized = randomizeSourceCutout(seed, xBoundaries, yBoundaries,
                                                  sourceWidth, sourceHeight);

    double RandomizedX_min_source = coordSourceRandomized[0];
    double RandomizedX_max_source = coordSourceRandomized[1];
    double RandomizedY_min_source = coordSourceRandomized[2];
    double RandomizedY_max_source = coordSourceRandomized[3];
    // random cropping
    sourceCloudPtr = cropPointCloud(targetCloudPtr,
                                    outPointCloudPtr,
                                    RandomizedX_min_source, RandomizedX_max_source,
                                    RandomizedY_min_source, RandomizedY_max_source);

    return sourceCloudPtr;
}

//Full pipeline to run (SIFT)
tuplePointCloudPtr fullPipelineSift(tupleParameters parametersList, double *seedRef, double *seedSource, double *seedCustomRotation,
                                    Settings pipelineSettings)
{
    // Extract values from tuple
    std::string pipelineType, typeTransformation, referenceDataFilename;
    double x_uncertainty, y_uncertainty, sourceWidth, sourceHeight;
    double x_angle_min, x_angle_max, y_angle_min, y_angle_max, z_angle_min, z_angle_max;

    // Method to unzip multiple returns value from parametersArray
    std::tie(pipelineType, typeTransformation, referenceDataFilename,
             x_uncertainty, y_uncertainty, sourceWidth, sourceHeight,
             x_angle_min, x_angle_max, y_angle_min, y_angle_max, z_angle_min, z_angle_max) = parametersList;

    PointCloudPtr sourceCloudPtr(new PointCloud);
    PointCloudPtr targetCloudPtr(new PointCloud);
    PointCloudPtr surfaceModelCloudPtr(new PointCloud);
    PointCloudPtr outPointCloudPtr(new PointCloud);
    PointCloudPtr croppedPointCloudPtr(new PointCloud);

    // Reading parameters txt file
    std::string modelSurfaceFileName = referenceDataFilename;
    std::cout << "Reading " << modelSurfaceFileName << std::endl;
    surfaceModelCloudPtr = loadingCloud(referenceDataFilename);

    // Computing reference and source cloud
    targetCloudPtr = computeReferenceCloud(surfaceModelCloudPtr, seedRef,
                                           sourceWidth, sourceHeight,
                                           x_uncertainty, y_uncertainty);
    sourceCloudPtr = computeSourceCloud(targetCloudPtr, seedSource,
                                        sourceWidth, sourceHeight,
                                        x_uncertainty, y_uncertainty);

    // Randomization of angles
    double randomAngleOnX, randomAngleOnY, randomAngleOnZ;
    randomAngleOnX = randomizeDoubleUniform(seedCustomRotation, x_angle_min, x_angle_max);
    randomAngleOnY = randomizeDoubleUniform(seedCustomRotation, y_angle_min, y_angle_max);
    randomAngleOnZ = randomizeDoubleUniform(seedCustomRotation, z_angle_min, z_angle_max);
    std::cout << "\nangle on x : " << randomAngleOnX << std::endl;
    std::cout << "angle on y : " << randomAngleOnY << std::endl;
    std::cout << "angle on z : " << randomAngleOnZ << std::endl;

    Eigen::Vector4f centroid(Eigen::Vector4f::Zero());
    Eigen::Vector4f origin_centroid(Eigen::Vector4f::Zero());
    Eigen::Vector4f new_centroid(Eigen::Vector4f::Zero());
    Eigen::Affine3f first_transform = Eigen::Affine3f::Identity();
    Eigen::Affine3f second_transform = Eigen::Affine3f::Identity();
    PointCloudPtr newPtCloudPtr(new PointCloud);
    PointCloudPtr correctPtCloudPtr(new PointCloud);
    PointCloudPtr sourceTransformedCloudPtr(new PointCloud);

    copyPointCloud(*sourceCloudPtr, *newPtCloudPtr);
    float trans_x = 0, trans_y = 0, trans_z = 0;
    first_transform = customRotation(newPtCloudPtr,
                                     centroid,
                                     origin_centroid,
                                     new_centroid,
                                     randomAngleOnX, randomAngleOnY, randomAngleOnZ);

    std::cout << "\nFirst transformation : \n"
              << first_transform.matrix() << std::endl;
    pcl::transformPointCloud(*newPtCloudPtr, *correctPtCloudPtr, first_transform);
    second_transform = customTranslation(correctPtCloudPtr,
                                         second_transform,
                                         trans_x, trans_y, trans_z);
    pcl::transformPointCloud(*correctPtCloudPtr, *sourceTransformedCloudPtr, second_transform);

    std::cout << "\nSecond transformation : \n"
              << second_transform.matrix() << std::endl;
    Eigen::Affine3f final_transform = first_transform * second_transform;

    std::cout << "\nFinal transformation : \n"
              << final_transform.matrix() << std::endl;

    PointCloud &sourceTransformedCloud = *sourceTransformedCloudPtr;
    PointCloud &targetCloud = *targetCloudPtr;

    // sift
    Eigen::Matrix4f final_transformation;
    pipelineSiftOutputPtr pipelineOutputPtr;
    PtCloudPointWithScalePtr srcKeypointsPtr;
    PtCloudPointWithScalePtr trgKeypointsPtr;
    PointCloudPtr finalCloudPtr(new PointCloud);
    PointCloudPtr transformedCloudPtr(new PointCloud);
    PointCloudPtr finalTransformedCloudPtr(new PointCloud);
    PointCloud &finalTransformedCloud = *finalTransformedCloudPtr;

    pipelineOutputPtr = siftPipeline(sourceTransformedCloudPtr, targetCloudPtr, pipelineSettings);
    std::tie(sourceTransformedCloudPtr, targetCloudPtr, transformedCloudPtr, srcKeypointsPtr, trgKeypointsPtr, final_transformation) = pipelineOutputPtr;

    pcl::transformPointCloud(sourceTransformedCloud, finalTransformedCloud, final_transformation);
    copyPointCloud(*finalTransformedCloudPtr, *finalCloudPtr);

    // display the points
    //double visualizerParameter = pipelineSettings.getValue(VISUALIZER_PARAMETER);

    visualizationToolSift(sourceCloudPtr,
                          targetCloudPtr,
                          transformedCloudPtr,
                          srcKeypointsPtr,
                          trgKeypointsPtr,
                          pipelineSettings,
                          pipelineType);

    return {sourceCloudPtr, targetCloudPtr, sourceTransformedCloudPtr, finalCloudPtr, randomAngleOnX, randomAngleOnY, randomAngleOnZ};
}

//Full pipeline to run (harris)
tuplePointCloudPtr fullPipelineHarris(tupleParameters parametersList, double *seedRef, double *seedSource, double *seedCustomRotation,
                                      Settings pipelineSettings)
{
    // Extract values from tuple
    std::string pipelineType, typeTransformation, referenceDataFilename;
    double x_uncertainty, y_uncertainty, sourceWidth, sourceHeight;
    double x_angle_min, x_angle_max, y_angle_min, y_angle_max, z_angle_min, z_angle_max;

    // Method to unzip multiple returns value from parametersArray
    std::tie(pipelineType, typeTransformation, referenceDataFilename,
             x_uncertainty, y_uncertainty, sourceWidth, sourceHeight,
             x_angle_min, x_angle_max, y_angle_min, y_angle_max, z_angle_min, z_angle_max) = parametersList;

    PointCloudPtr sourceCloudPtr(new PointCloud);
    PointCloudPtr targetCloudPtr(new PointCloud);
    PointCloudPtr surfaceModelCloudPtr(new PointCloud);
    PointCloudPtr outPointCloudPtr(new PointCloud);
    PointCloudPtr croppedPointCloudPtr(new PointCloud);

    // Reading parameters txt file
    std::string modelSurfaceFileName = referenceDataFilename;
    std::cout << "Reading " << modelSurfaceFileName << std::endl;
    surfaceModelCloudPtr = loadingCloud(referenceDataFilename);

    // Computing reference and source cloud
    targetCloudPtr = computeReferenceCloud(surfaceModelCloudPtr, seedRef,
                                           sourceWidth, sourceHeight,
                                           x_uncertainty, y_uncertainty);
    sourceCloudPtr = computeSourceCloud(targetCloudPtr, seedSource,
                                        sourceWidth, sourceHeight,
                                        x_uncertainty, y_uncertainty);

    // Randomization of angles
    double randomAngleOnX, randomAngleOnY, randomAngleOnZ;
    randomAngleOnX = randomizeDoubleUniform(seedCustomRotation, x_angle_min, x_angle_max);
    randomAngleOnY = randomizeDoubleUniform(seedCustomRotation, y_angle_min, y_angle_max);
    randomAngleOnZ = randomizeDoubleUniform(seedCustomRotation, z_angle_min, z_angle_max);
    std::cout << "\nangle on x : " << randomAngleOnX << std::endl;
    std::cout << "angle on y : " << randomAngleOnY << std::endl;
    std::cout << "angle on z : " << randomAngleOnZ << std::endl;

    Eigen::Vector4f centroid(Eigen::Vector4f::Zero());
    Eigen::Vector4f origin_centroid(Eigen::Vector4f::Zero());
    Eigen::Vector4f new_centroid(Eigen::Vector4f::Zero());
    Eigen::Affine3f first_transform = Eigen::Affine3f::Identity();
    Eigen::Affine3f second_transform = Eigen::Affine3f::Identity();
    PointCloudPtr newPtCloudPtr(new PointCloud);
    PointCloudPtr correctPtCloudPtr(new PointCloud);
    PointCloudPtr sourceTransformedCloudPtr(new PointCloud);

    copyPointCloud(*sourceCloudPtr, *newPtCloudPtr);
    float trans_x = 0, trans_y = 0, trans_z = 0;
    first_transform = customRotation(newPtCloudPtr,
                                     centroid,
                                     origin_centroid,
                                     new_centroid,
                                     randomAngleOnX, randomAngleOnY, randomAngleOnZ);

    std::cout << "\nFirst transformation : \n"
              << first_transform.matrix() << std::endl;
    pcl::transformPointCloud(*newPtCloudPtr, *correctPtCloudPtr, first_transform);
    second_transform = customTranslation(correctPtCloudPtr,
                                         second_transform,
                                         trans_x, trans_y, trans_z);
    pcl::transformPointCloud(*correctPtCloudPtr, *sourceTransformedCloudPtr, second_transform);

    std::cout << "\nSecond transformation : \n"
              << second_transform.matrix() << std::endl;
    Eigen::Affine3f final_transform = first_transform * second_transform;

    std::cout << "\nFinal transformation : \n"
              << final_transform.matrix() << std::endl;

    PointCloud &sourceTransformedCloud = *sourceTransformedCloudPtr;
    PointCloud &targetCloud = *targetCloudPtr;

    // harris

    Eigen::Matrix4f final_transformation;
    pipelineHarrisOutputPtr pipelineOutputPtr;
    PtCloudPointWithIntensityPtr srcKeypointsPtr;
    PtCloudPointWithIntensityPtr trgKeypointsPtr;
    PointCloudPtr finalCloudPtr(new PointCloud);
    PointCloudPtr finalTransformedCloudPtr(new PointCloud);
    PointCloudPtr transformedCloudPtr(new PointCloud);
    PointCloud &finalTransformedCloud = *finalTransformedCloudPtr;

    pipelineOutputPtr = harrisPipeline(sourceTransformedCloudPtr, targetCloudPtr, pipelineSettings);
    std::tie(sourceTransformedCloudPtr, targetCloudPtr, transformedCloudPtr, srcKeypointsPtr, trgKeypointsPtr, final_transformation) = pipelineOutputPtr;

    pcl::transformPointCloud(sourceTransformedCloud, finalTransformedCloud, final_transformation);
    copyPointCloud(*finalTransformedCloudPtr, *finalCloudPtr);

    // display the points
    //double visualizerParameter = pipelineSettings.getValue(VISUALIZER_PARAMETER);

    visualizationToolHarris(sourceCloudPtr,
                            targetCloudPtr,
                            transformedCloudPtr,
                            srcKeypointsPtr,
                            trgKeypointsPtr,
                            pipelineSettings,
                            pipelineType);

    return {sourceCloudPtr, targetCloudPtr, sourceTransformedCloudPtr, finalCloudPtr, randomAngleOnX, randomAngleOnY, randomAngleOnZ};
}

//Full pipeline without detection step
tuplePointCloudPtr fullPipeline(tupleParameters parametersList, double *seedRef, double *seedSource, double *seedCustomRotation,
                                Settings pipelineSettings)
{
    // Extract values from tuple
    std::string pipelineType, typeTransformation, referenceDataFilename;
    double x_uncertainty, y_uncertainty, sourceWidth, sourceHeight;
    double x_angle_min, x_angle_max, y_angle_min, y_angle_max, z_angle_min, z_angle_max;

    // Method to unzip multiple returns value from parametersArray
    std::tie(pipelineType, typeTransformation, referenceDataFilename,
             x_uncertainty, y_uncertainty, sourceWidth, sourceHeight,
             x_angle_min, x_angle_max, y_angle_min, y_angle_max, z_angle_min, z_angle_max) = parametersList;

    PointCloudPtr sourceCloudPtr(new PointCloud);
    PointCloudPtr targetCloudPtr(new PointCloud);
    PointCloudPtr surfaceModelCloudPtr(new PointCloud);
    PointCloudPtr outPointCloudPtr(new PointCloud);
    PointCloudPtr croppedPointCloudPtr(new PointCloud);

    // Reading parameters txt file
    std::string modelSurfaceFileName = referenceDataFilename;
    std::cout << "Reading " << modelSurfaceFileName << std::endl;
    surfaceModelCloudPtr = loadingCloud(referenceDataFilename);

    // Computing reference and source cloud
    targetCloudPtr = computeReferenceCloud(surfaceModelCloudPtr, seedRef,
                                           sourceWidth, sourceHeight,
                                           x_uncertainty, y_uncertainty);
    sourceCloudPtr = computeSourceCloud(targetCloudPtr, seedSource,
                                        sourceWidth, sourceHeight,
                                        x_uncertainty, y_uncertainty);

    // Randomization of angles
    double randomAngleOnX, randomAngleOnY, randomAngleOnZ;
    randomAngleOnX = randomizeDoubleUniform(seedCustomRotation, x_angle_min, x_angle_max);
    randomAngleOnY = randomizeDoubleUniform(seedCustomRotation, y_angle_min, y_angle_max);
    randomAngleOnZ = randomizeDoubleUniform(seedCustomRotation, z_angle_min, z_angle_max);
    std::cout << "\nangle on x : " << randomAngleOnX << std::endl;
    std::cout << "angle on y : " << randomAngleOnY << std::endl;
    std::cout << "angle on z : " << randomAngleOnZ << std::endl;

    Eigen::Vector4f centroid(Eigen::Vector4f::Zero());
    Eigen::Vector4f origin_centroid(Eigen::Vector4f::Zero());
    Eigen::Vector4f new_centroid(Eigen::Vector4f::Zero());
    Eigen::Affine3f first_transform = Eigen::Affine3f::Identity();
    Eigen::Affine3f second_transform = Eigen::Affine3f::Identity();
    PointCloudPtr newPtCloudPtr(new PointCloud);
    PointCloudPtr correctPtCloudPtr(new PointCloud);
    PointCloudPtr sourceTransformedCloudPtr(new PointCloud);

    copyPointCloud(*sourceCloudPtr, *newPtCloudPtr);
    float trans_x = 0, trans_y = 0, trans_z = 0;
    first_transform = customRotation(newPtCloudPtr,
                                     centroid,
                                     origin_centroid,
                                     new_centroid,
                                     randomAngleOnX, randomAngleOnY, randomAngleOnZ);

    std::cout << "\nFirst transformation : \n"
              << first_transform.matrix() << std::endl;
    pcl::transformPointCloud(*newPtCloudPtr, *correctPtCloudPtr, first_transform);
    second_transform = customTranslation(correctPtCloudPtr,
                                         second_transform,
                                         trans_x, trans_y, trans_z);
    pcl::transformPointCloud(*correctPtCloudPtr, *sourceTransformedCloudPtr, second_transform);

    std::cout << "\nSecond transformation : \n"
              << second_transform.matrix() << std::endl;
    Eigen::Affine3f final_transform = first_transform * second_transform;

    std::cout << "\nFinal transformation : \n"
              << final_transform.matrix() << std::endl;

    PointCloud &sourceTransformedCloud = *sourceTransformedCloudPtr;
    PointCloud &targetCloud = *targetCloudPtr;

    // pipeline with all points
    pipelineAllPointsOutputPtr pipelineOutputPtr;
    Eigen::Matrix4f final_transformation;
    PointCloudPtr finalCloudPtr(new PointCloud);
    PointCloudPtr transformedCloudPtr(new PointCloud);

    pipelineOutputPtr = pipelineAllPoints(sourceTransformedCloudPtr, targetCloudPtr, pipelineSettings);
    std::tie(sourceTransformedCloudPtr, targetCloudPtr, transformedCloudPtr, final_transformation) = pipelineOutputPtr;

    PointCloudPtr finalTransformedCloudPtr(new PointCloud);
    PointCloud &finalTransformedCloud = *finalTransformedCloudPtr;
    pcl::transformPointCloud(*sourceTransformedCloudPtr, finalTransformedCloud, final_transformation);
    copyPointCloud(*finalTransformedCloudPtr, *finalCloudPtr);

    return {sourceCloudPtr, targetCloudPtr, sourceTransformedCloudPtr, finalCloudPtr, randomAngleOnX, randomAngleOnY, randomAngleOnZ};
}

/* get the distance between points */
double distance(const PointXYZ &p1, const PointXYZ &p2)
{
    Eigen::Vector3f diff = p1.getVector3fMap() - p2.getVector3fMap();
    return (diff.norm());
}

/* Get separated axis coordinates of the points in the point cloud */
tupleOfVectorDouble getCoordinates(PointCloudPtr srcPointCloudPtr)
{
    PointCloud &srcPointCloud = *srcPointCloudPtr;
    std::vector<double> mtre_x_src, mtre_y_src, mtre_z_src;

    // Sizes of the inputs point cloud are the same
    for (auto &srcPoint : srcPointCloud)
    {
        double value_mtre_x_src = (srcPoint.x);
        double value_mtre_y_src = (srcPoint.y);
        double value_mtre_z_src = (srcPoint.z);
        mtre_x_src.push_back(value_mtre_x_src);
        mtre_y_src.push_back(value_mtre_y_src);
        mtre_z_src.push_back(value_mtre_z_src);
    }

    return {mtre_x_src, mtre_y_src, mtre_z_src};
}

/* Get the points coordinates in 3D */
vectorPointXYZ Get3DCoordinatesXYZ(PointCloudPtr srcPointCloudPtr)
{
    PointCloud &srcPointCloud = *srcPointCloudPtr;
    vectorPointXYZ coords;
    // Sizes of the inputs point cloud are the same
    for (auto &Point : srcPointCloud)
    {
        PointXYZ points = Point;
        coords.push_back(points);
    }
    return coords;
}

/* Registation error bias */
tupleOfDouble registrationErrorBias(PointCloudPtr srcPointCloudPtr, PointCloudPtr transformedSrcPointCloudPtr)
{
    tupleOfVectorDouble vectorCoordinatesSource;
    tupleOfVectorDouble vectorCoordinatesTransformedSrc;
    std::vector<double> mtre_x_src, mtre_y_src, mtre_z_src;
    std::vector<double> mtre_x_transformed_src, mtre_y_transformed_src, mtre_z_transformed_src;
    std::vector<double> mtre_x, mtre_y, mtre_z;

    PointCloud &sourceCloud = *srcPointCloudPtr;
    PointCloud &transformedSrcPointCloud = *transformedSrcPointCloudPtr;

    vectorCoordinatesSource = getCoordinates(srcPointCloudPtr);
    vectorCoordinatesTransformedSrc = getCoordinates(transformedSrcPointCloudPtr);
    std::tie(mtre_x_src, mtre_y_src, mtre_z_src) = vectorCoordinatesSource;
    std::tie(mtre_x_transformed_src, mtre_y_transformed_src, mtre_z_transformed_src) = vectorCoordinatesTransformedSrc;

    for (int i = 0; i < sourceCloud.size(); i++)
    {
        double value_mtre_x = mtre_x_src.at(i) - mtre_x_transformed_src.at(i);
        double value_mtre_y = mtre_y_src.at(i) - mtre_y_transformed_src.at(i);
        double value_mtre_z = mtre_z_src.at(i) - mtre_z_transformed_src.at(i);
        mtre_x.push_back(value_mtre_x);
        mtre_y.push_back(value_mtre_y);
        mtre_z.push_back(value_mtre_z);
    }
    double average_mtre_x_src = std::accumulate(mtre_x.begin(), mtre_x.end(), 0.0) / mtre_x.size();
    double average_mtre_y_src = std::accumulate(mtre_y.begin(), mtre_y.end(), 0.0) / mtre_y.size();
    double average_mtre_z_src = std::accumulate(mtre_z.begin(), mtre_z.end(), 0.0) / mtre_z.size();

    return {average_mtre_x_src, average_mtre_y_src, average_mtre_z_src};
}

/* Mean target registration error */
double meanTargetRegistrationError(PointCloudPtr srcPointCloudPtr, PointCloudPtr transformedSrcPointCloudPtr)
{
    vectorPointXYZ coordinatesSrc;
    vectorPointXYZ coordinatesSrcTransformed;
    std::vector<double> norms;
    PointCloud &sourceCloud = *srcPointCloudPtr;
    PointCloud &transformedCloud = *transformedSrcPointCloudPtr;

    coordinatesSrc = Get3DCoordinatesXYZ(srcPointCloudPtr);
    coordinatesSrcTransformed = Get3DCoordinatesXYZ(transformedSrcPointCloudPtr);
    for (int i = 0; i < sourceCloud.size(); i++)
    {
        // Euclidean norm
        double norm = distance(coordinatesSrc.at(i), coordinatesSrcTransformed.at(i));
        norms.push_back(norm);
    }
    double average_norm = std::accumulate(norms.begin(), norms.end(), 0.0) / norms.size();

    return average_norm;
}

/* Append current time stamp to provided filename (after stripping any extension) */
std::string appendTimestamp(std::string filename)
{
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
    auto curTimeStr = oss.str();
    return removeExtension(filename) + "_" + curTimeStr;
}

/* Full registration pipeline including the errors calculation */
void fullRegistration(std::string &fullParametersFilename,
                      Settings pipelineSettings,
                      std::string &savedFilename,
                      double *seedRef,
                      double *seedSource,
                      double *seedCustomRotation)
{

    double initialSeedRef = *seedRef;
    double initialSeedSource = *seedSource;
    double initialSeedRot = *seedCustomRotation;

    tupleParameters parametersList;
    std::vector<std::string> parameters;

    // Read the parameters from filename
    parameters = readParameters(fullParametersFilename);
    // Create an array containing all the elements from the text file
    parametersList = parametersArray(fullParametersFilename, parameters);

    // Extract values from tuple
    std::string pipelineType, typeTransformation, referenceDataFilename;
    double x_uncertainty, y_uncertainty, sourceWidth, sourceHeight;
    double x_angle_min, x_angle_max, y_angle_min, y_angle_max, z_angle_min, z_angle_max;

    // Method to unzip multiple returns value from parametersArray
    std::tie(pipelineType, typeTransformation, referenceDataFilename,
             x_uncertainty, y_uncertainty, sourceWidth, sourceHeight,
             x_angle_min, x_angle_max, y_angle_min, y_angle_max, z_angle_min, z_angle_max) = parametersList;

    {

        tuplePointCloudPtr PointCloudsPtrOutput;

        if (pipelineType == "all")
        {
            // run witout detection step, i.e. use all points as keypoints
            std::cout << "\nVisualizer parameter : " << pipelineSettings.getValue(VISUALIZER_PARAMETER) << std::endl;
            PointCloudsPtrOutput = fullPipeline(parametersList, seedRef, seedSource,
                                                seedCustomRotation, pipelineSettings);
        }
        else if (pipelineType == "sift")
        {
            // SIFT detection pipeline
            std::cout << "\nVisualizer parameter : " << pipelineSettings.getValue(VISUALIZER_PARAMETER) << std::endl;
            PointCloudsPtrOutput = fullPipelineSift(parametersList, seedRef, seedSource,
                                                    seedCustomRotation, pipelineSettings);
        }
        else if (pipelineType == "harris")
        {
            // Harris detection pipeline
            std::cout << "\nVisualizer parameter : " << pipelineSettings.getValue(VISUALIZER_PARAMETER) << std::endl;
            PointCloudsPtrOutput = fullPipelineHarris(parametersList, seedRef, seedSource,
                                                      seedCustomRotation, pipelineSettings);
        }
        else
        {
            std::cout << "===ERROR=== aborting: pipelineType not recognized: " << pipelineType << std::endl;
            return;
        }

        PointCloudPtr sourceCloudPtr(new PointCloud);
        PointCloudPtr targetCloudPtr(new PointCloud);
        PointCloudPtr sourceTransformedCloudPtr(new PointCloud);
        PointCloudPtr transformedCloudPtr(new PointCloud);
        double angleX, angleY, angleZ;
        std::tie(sourceCloudPtr, targetCloudPtr, sourceTransformedCloudPtr, transformedCloudPtr, angleX, angleY, angleZ) = PointCloudsPtrOutput;
        PointCloud &sourceCloud = *sourceCloudPtr;

        // Calculate the mean target registration error
        std::cout << "\n-----------------------------------------------------------------------------------------------" << std::endl;
        std::cout << "--------------------------------- Mean Target Registration Error --------------------------------" << std::endl;
        std::cout << "-------------------------------------------------------------------------------------------------" << std::endl;
        double mtre = meanTargetRegistrationError(sourceCloudPtr, transformedCloudPtr);
        std::cout << "[" << pipelineType << "] Mean Target Registration Error between original source and transformed cloud: " << mtre << std::endl;

        // Calculate the Registration error bias
        tupleOfDouble bias;
        double bias_x, bias_y, bias_z;
        bias = registrationErrorBias(sourceCloudPtr, transformedCloudPtr);
        std::tie(bias_x, bias_y, bias_z) = bias;

        std::cout << "\n-----------------------------------------------------------------------------------------------" << std::endl;
        std::cout << "------------------------------------ Registration error bias ------------------------------------" << std::endl;
        std::cout << "-------------------------------------------------------------------------------------------------" << std::endl;
        std::cout << "\n[" << pipelineType << "] Registration error bias on x between original source and transformed cloud: " << bias_x << std::endl;
        std::cout << "[" << pipelineType << "] Registration error bias on y between original source and transformed cloud: " << bias_y << std::endl;
        std::cout << "[" << pipelineType << "] Registration error bias on z between original source and transformed cloud: " << bias_z << std::endl;

        // populate settings map for writing to results file
        StringMap settingsMap;
        settingsMap["pipelineType"] = pipelineType;
        settingsMap["err_MTRE"] = std::to_string(mtre);
        settingsMap["err_bias_x"] = std::to_string(bias_x);
        settingsMap["err_bias_y"] = std::to_string(bias_y);
        settingsMap["err_bias_z"] = std::to_string(bias_z);
        settingsMap["angle_x"] = std::to_string(angleX);
        settingsMap["angle_y"] = std::to_string(angleY);
        settingsMap["angle_z"] = std::to_string(angleZ);
        settingsMap["x_uncertainty"] = std::to_string(x_uncertainty);
        settingsMap["y_uncertainty"] = std::to_string(y_uncertainty);
        settingsMap["sourceWidth"] = std::to_string(sourceWidth);
        settingsMap["sourceHeight"] = std::to_string(sourceHeight);
        settingsMap["dataset"] = referenceDataFilename;
        settingsMap["seedRef"] = std::to_string(initialSeedRef);
        settingsMap["seedSource"] = std::to_string(initialSeedSource);
        settingsMap["seedRot"] = std::to_string(initialSeedRot);

        // transfer pipelinesettings to settingsmap
        DoubleMap pipelineSettingsMap = pipelineSettings.getAllSettings();
        for (auto item : pipelineSettingsMap)
        {
            settingsMap[item.first] = std::to_string(item.second);
        }
        // save results to csv formatted file
        saveResults(savedFilename, settingsMap);

        // TODO add argument to steer if clouds should be visualized and fix visualisation (pipeline got stuck)
        /*if (true) {
            std::cout << "\nGreen : Reference point cloud " << std::endl;
            std::cout << "Red : Source point cloud " << std::endl;
            std::cout << "Light blue : Custom transformed point cloud " << std::endl;
            std::cout << "Blue : Point cloud obtained through alignment proccess " << std::endl;
            
            visualizeResults(targetCloudPtr,
                            sourceCloudPtr,
                            sourceTransformedCloudPtr,
                            transformedCloudPtr);
        }*/
    }
}
