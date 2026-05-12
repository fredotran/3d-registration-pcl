#pragma once

#include "registration.hpp"

///////////////////////////////
//////// PROTOTYPES ///////////
///////////////////////////////

// reading pointCloud
PointCloudPtr loadingCloud(const std::string& fileName);

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
                                     double *seedRef,
                                     double sourceWidth, double sourceHeight,
                                     double x_uncertainty, double y_uncertainty);
PointCloudPtr computeSourceCloud(PointCloudPtr targetCloudPtr, double *seed,
                                  double sourceWidth, double sourceHeight,
                                  double x_uncertainty, double y_uncertainty);

// full pipelines
void fullRegistration(const std::string &fullParametersFilename,
                      Settings pipelineSettings,
                      const std::string &savedFilename,
                      double *seedRef,
                      double *seedSource,
                      double *seedCustomRotation);
TuplePointCloudPtr fullPipelineSift(TupleParameters parametersList, double *seedRef, double *seedSource,
                                     double *seedCustomRotation, Settings pipelineSettings);
TuplePointCloudPtr fullPipelineHarris(TupleParameters parametersList, double *seedRef, double *seedSource,
                                       double *seedCustomRotation, Settings pipelineSettings);
TuplePointCloudPtr fullPipeline(TupleParameters parametersList, double *seedRef, double *seedSource, double *seedCustomRotation,
                                 Settings pipelineSettings);

///////////////////////////////
////////// METHODS ///////////
///////////////////////////////

/* Method to read and load the point cloud using its filename */
PointCloudPtr loadingCloud(const std::string& fileName)
{
    auto cloudPtr = PointCloudPtr(new PointCloud);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName, *cloudPtr) == -1) {
        PCL_ERROR("Couldn't read file: %s\n", fileName.c_str());
        throw std::runtime_error("Failed to read file: " + fileName);
    }
    std::cout << "Loaded source file : " << cloudPtr->size() << " points." << std::endl;

    // Remove NaN values from point clouds
    std::vector<int> nanValues;
    pcl::removeNaNFromPointCloud(*cloudPtr, *cloudPtr, nanValues);

    return cloudPtr;
}

/* Apply a rigid-body transformation: translate to origin, rotate, translate back */
PointCloudPtr correctedPointCloud(PointCloudPtr sourcePointCloudPtr,
                                   Eigen::Vector4f centroid,
                                   Eigen::Vector4f origin_centroid,
                                   Eigen::Vector4f new_centroid,
                                   float theta_x, float theta_y, float theta_z)
{
    auto localPointCloudPtr = PointCloudPtr(new PointCloud);
    copyPointCloud(*sourcePointCloudPtr, *localPointCloudPtr);

    // Translation to new centroid
    Eigen::Affine3f transform_translation = Eigen::Affine3f::Identity();
    pcl::compute3DCentroid(*localPointCloudPtr, centroid);
    transform_translation.translation() = new_centroid.head<3>() - centroid.head<3>();

    auto transformed_cloud = PointCloudPtr(new PointCloud);
    std::cout << "Transformation used for the origin correction : \n"
              << transform_translation.matrix() << std::endl;
    pcl::transformPointCloud(*localPointCloudPtr, *transformed_cloud, transform_translation);
    std::cout << "\nSource point cloud (white) " << std::endl;
    std::cout << "Corrected coordinates point cloud (green) " << std::endl;

    pcl::compute3DCentroid(*transformed_cloud, new_centroid);
    std::cout << "\nNew centroid coordinates (after correction) : \n"
              << new_centroid << std::endl;

    // Rotation
    auto rotation_cloud = PointCloudPtr(new PointCloud);
    Eigen::Affine3f transform_rotation = Eigen::Affine3f::Identity();
    transform_rotation.rotate(Eigen::AngleAxisf((theta_x * M_PI) / 180, Eigen::Vector3f::UnitX()));
    transform_rotation.rotate(Eigen::AngleAxisf((theta_y * M_PI) / 180, Eigen::Vector3f::UnitY()));
    transform_rotation.rotate(Eigen::AngleAxisf((theta_z * M_PI) / 180, Eigen::Vector3f::UnitZ()));
    std::cout << "\nTransformation used for the rotation : \n"
              << transform_rotation.matrix() << std::endl;
    pcl::transformPointCloud(*transformed_cloud, *rotation_cloud, transform_rotation);

    // Translate back using inverse
    auto new_cloud = PointCloudPtr(new PointCloud);
    Eigen::Affine3f inverse_transform = transform_translation.inverse();
    pcl::transformPointCloud(*rotation_cloud, *new_cloud, inverse_transform);

    return new_cloud;
}

/* Method to do a custom rotation of the point cloud, returning the combined transformation matrix */
Eigen::Affine3f customRotation(PointCloudPtr sourcePointCloudPtr,
                                Eigen::Vector4f centroid,
                                Eigen::Vector4f origin_centroid,
                                Eigen::Vector4f new_centroid,
                                float theta_x, float theta_y, float theta_z)
{
    auto localPointCloudPtr = PointCloudPtr(new PointCloud);
    copyPointCloud(*sourcePointCloudPtr, *localPointCloudPtr);

    Eigen::Affine3f transform_translation = Eigen::Affine3f::Identity();
    pcl::compute3DCentroid(*localPointCloudPtr, centroid);
    transform_translation.translation() = new_centroid.head<3>() - centroid.head<3>();

    // Apply translation
    auto transformed_cloud = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*localPointCloudPtr, *transformed_cloud, transform_translation);

    pcl::compute3DCentroid(*transformed_cloud, new_centroid);
    std::cout << "\nNew centroid coordinates (after correction) : \n"
              << new_centroid << std::endl;

    // Rotation
    Eigen::Affine3f transform_rotation = Eigen::Affine3f::Identity();
    transform_rotation.rotate(Eigen::AngleAxisf((theta_x * M_PI) / 180, Eigen::Vector3f::UnitX()));
    transform_rotation.rotate(Eigen::AngleAxisf((theta_y * M_PI) / 180, Eigen::Vector3f::UnitY()));
    transform_rotation.rotate(Eigen::AngleAxisf((theta_z * M_PI) / 180, Eigen::Vector3f::UnitZ()));

    // Combine: inverse_translation * rotation * translation
    Eigen::Affine3f final_matrix = transform_translation.inverse() * transform_rotation * transform_translation;

    return final_matrix;
}

/* Methods to do a translation offset of the point cloud */
Eigen::Affine3f customTranslation(PointCloudPtr sourcePointCloudPtr,
                                   Eigen::Affine3f inputTransformationMatrix,
                                   float trans_x, float trans_y, float trans_z)
{
    auto localPointCloudPtr = PointCloudPtr(new PointCloud);
    copyPointCloud(*sourcePointCloudPtr, *localPointCloudPtr);

    inputTransformationMatrix.translation() << trans_x, trans_y, trans_z;
    auto final_cloud = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*localPointCloudPtr, *final_cloud, inputTransformationMatrix);

    return inputTransformationMatrix;
}

/* Returns the cropped point cloud based on specified x/y cutout limits */
PointCloudPtr cropPointCloud(PointCloudPtr cloudPtr,
                              PointCloudPtr cloudOutPtr,
                              double xMin, double xMax,
                              double yMin, double yMax)
{
    pcl::PointXYZ ref_min_pt, ref_max_pt;
    pcl::getMinMax3D(*cloudPtr, ref_min_pt, ref_max_pt);

    Eigen::Vector4f minPoints(xMin, yMin, ref_min_pt.z, 1.0);
    Eigen::Vector4f maxPoints(xMax, yMax, ref_max_pt.z, 1.0);

    pcl::CropBox<PointXYZ> cropFilter;
    cropFilter.setInputCloud(cloudPtr);
    cropFilter.setMin(minPoints);
    cropFilter.setMax(maxPoints);
    cropFilter.filter(*cloudOutPtr);

    return cloudOutPtr;
}

/* Compute reference point cloud by cropping the surface model */
PointCloudPtr computeReferenceCloud(PointCloudPtr surfaceModelCloudPtr, double *seedRef,
                                     double sourceWidth, double sourceHeight,
                                     double x_uncertainty, double y_uncertainty)
{
    auto targetCloudPtr = PointCloudPtr(new PointCloud);
    auto outPointCloudPtr = PointCloudPtr(new PointCloud);

    pcl::PointXYZ ref_min_pt, ref_max_pt;
    pcl::getMinMax3D(*surfaceModelCloudPtr, ref_min_pt, ref_max_pt);
    double xBoundariesRef[2] = {ref_min_pt.x, ref_max_pt.x};
    double yBoundariesRef[2] = {ref_min_pt.y, ref_max_pt.y};

    auto coordRandomizedRef = randomizeReferenceCutout(
        seedRef, xBoundariesRef, yBoundariesRef,
        sourceWidth, sourceHeight, x_uncertainty, y_uncertainty);

    cropPointCloud(surfaceModelCloudPtr, outPointCloudPtr,
                   coordRandomizedRef[0], coordRandomizedRef[1],
                   coordRandomizedRef[2], coordRandomizedRef[3]);

    copyPointCloud(*outPointCloudPtr, *targetCloudPtr);
    return targetCloudPtr;
}

/* Compute source point cloud by cropping the target/reference cloud */
PointCloudPtr computeSourceCloud(PointCloudPtr targetCloudPtr, double *seed,
                                  double sourceWidth, double sourceHeight,
                                  double x_uncertainty, double y_uncertainty)
{
    auto sourceCloudPtr = PointCloudPtr(new PointCloud);
    auto outPointCloudPtr = PointCloudPtr(new PointCloud);

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*targetCloudPtr, min_pt, max_pt);
    double xBoundaries[2] = {min_pt.x, max_pt.x};
    double yBoundaries[2] = {min_pt.y, max_pt.y};

    auto coordSourceRandomized = randomizeSourceCutout(
        seed, xBoundaries, yBoundaries, sourceWidth, sourceHeight);

    cropPointCloud(targetCloudPtr, outPointCloudPtr,
                   coordSourceRandomized[0], coordSourceRandomized[1],
                   coordSourceRandomized[2], coordSourceRandomized[3]);

    return sourceCloudPtr;
}

/* Common setup logic shared by all three pipelines.
   Returns: <sourceCloud, targetCloud, transformedSource, customRotationMatrix, angleX, angleY, angleZ> */
static std::tuple<PointCloudPtr, PointCloudPtr, PointCloudPtr, Eigen::Affine3f, double, double, double>
setupPipeline(TupleParameters& parametersList, double *seedRef, double *seedSource, double *seedCustomRotation)
{
    std::string pipelineType, typeTransformation, referenceDataFilename;
    double x_uncertainty, y_uncertainty, sourceWidth, sourceHeight;
    double x_angle_min, x_angle_max, y_angle_min, y_angle_max, z_angle_min, z_angle_max;

    std::tie(pipelineType, typeTransformation, referenceDataFilename,
             x_uncertainty, y_uncertainty, sourceWidth, sourceHeight,
             x_angle_min, x_angle_max, y_angle_min, y_angle_max, z_angle_min, z_angle_max) = parametersList;

    auto surfaceModelCloudPtr = loadingCloud(referenceDataFilename);

    auto targetCloudPtr = computeReferenceCloud(surfaceModelCloudPtr, seedRef,
                                                 sourceWidth, sourceHeight,
                                                 x_uncertainty, y_uncertainty);
    auto sourceCloudPtr = computeSourceCloud(targetCloudPtr, seedSource,
                                              sourceWidth, sourceHeight,
                                              x_uncertainty, y_uncertainty);

    // Randomize rotation angles
    double randomAngleOnX = randomizeDoubleUniform(seedCustomRotation, x_angle_min, x_angle_max);
    double randomAngleOnY = randomizeDoubleUniform(seedCustomRotation, y_angle_min, y_angle_max);
    double randomAngleOnZ = randomizeDoubleUniform(seedCustomRotation, z_angle_min, z_angle_max);
    std::cout << "\nangle on x : " << randomAngleOnX << "\nangle on y : " << randomAngleOnY
              << "\nangle on z : " << randomAngleOnZ << std::endl;

    // Apply custom rotation
    Eigen::Affine3f first_transform = customRotation(sourceCloudPtr,
                                                      Eigen::Vector4f::Zero(),
                                                      Eigen::Vector4f::Zero(),
                                                      Eigen::Vector4f::Zero(),
                                                      randomAngleOnX, randomAngleOnY, randomAngleOnZ);

    std::cout << "\nFirst transformation : \n" << first_transform.matrix() << std::endl;

    // Apply translation (zero offset, so second_transform is identity)
    Eigen::Affine3f second_transform = Eigen::Affine3f::Identity();
    second_transform.translation() << 0.0f, 0.0f, 0.0f;

    Eigen::Affine3f final_transform = first_transform * second_transform;
    std::cout << "\nFinal transformation : \n" << final_transform.matrix() << std::endl;

    auto sourceTransformedCloudPtr = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceCloudPtr, *sourceTransformedCloudPtr, final_transform);

    return {sourceCloudPtr, targetCloudPtr, sourceTransformedCloudPtr, final_transform,
            randomAngleOnX, randomAngleOnY, randomAngleOnZ};
}

// Template for full pipeline that handles all three pipeline types
template <typename PipelineOutput, int TransformIndex>
TuplePointCloudPtr fullPipelineTemplate(
    TupleParameters parametersList,
    double *seedRef,
    double *seedSource,
    double *seedCustomRotation,
    Settings pipelineSettings,
    std::function<PipelineOutput(PointCloudPtr, PointCloudPtr, Settings)> pipelineFunc)
{
    auto [sourceCloudPtr, targetCloudPtr, sourceTransformedCloudPtr, final_transform,
          angleX, angleY, angleZ] = setupPipeline(parametersList, seedRef, seedSource, seedCustomRotation);

    auto pipelineOutput = pipelineFunc(sourceTransformedCloudPtr, targetCloudPtr, pipelineSettings);

    auto finalCloudPtr = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceTransformedCloudPtr, *finalCloudPtr, std::get<TransformIndex>(pipelineOutput));

    std::cout << "\nNot displaying any results" << std::endl;

    return {sourceCloudPtr, targetCloudPtr, sourceTransformedCloudPtr, finalCloudPtr,
            angleX, angleY, angleZ};
}

// Full pipeline with SIFT detection
TuplePointCloudPtr fullPipelineSift(TupleParameters parametersList, double *seedRef, double *seedSource,
                                     double *seedCustomRotation, Settings pipelineSettings)
{
    return fullPipelineTemplate<PipelineSiftOutput, 5>(
        parametersList, seedRef, seedSource, seedCustomRotation, pipelineSettings, siftPipeline);
}

// Full pipeline with Harris detection
TuplePointCloudPtr fullPipelineHarris(TupleParameters parametersList, double *seedRef, double *seedSource,
                                       double *seedCustomRotation, Settings pipelineSettings)
{
    return fullPipelineTemplate<PipelineHarrisOutput, 5>(
        parametersList, seedRef, seedSource, seedCustomRotation, pipelineSettings, harrisPipeline);
}

// Full pipeline without detection (all points)
TuplePointCloudPtr fullPipeline(TupleParameters parametersList, double *seedRef, double *seedSource, double *seedCustomRotation,
                                 Settings pipelineSettings)
{
    return fullPipelineTemplate<PipelineAllPointsOutput, 3>(
        parametersList, seedRef, seedSource, seedCustomRotation, pipelineSettings, pipelineAllPoints);
}

/* get the distance between points */
double distance(const PointXYZ &p1, const PointXYZ &p2)
{
    Eigen::Vector3f diff = p1.getVector3fMap() - p2.getVector3fMap();
    return diff.norm();
}

/* Get separated axis coordinates of the points in the point cloud */
TupleOfVectorDouble getCoordinates(PointCloudPtr srcPointCloudPtr)
{
    auto& srcPointCloud = *srcPointCloudPtr;
    std::vector<double> mtre_x_src, mtre_y_src, mtre_z_src;
    mtre_x_src.reserve(srcPointCloud.size());
    mtre_y_src.reserve(srcPointCloud.size());
    mtre_z_src.reserve(srcPointCloud.size());

    for (auto& srcPoint : srcPointCloud) {
        mtre_x_src.push_back(srcPoint.x);
        mtre_y_src.push_back(srcPoint.y);
        mtre_z_src.push_back(srcPoint.z);
    }

    return {mtre_x_src, mtre_y_src, mtre_z_src};
}

/* Get the points coordinates in 3D */
VectorPointXYZ Get3DCoordinatesXYZ(PointCloudPtr srcPointCloudPtr)
{
    auto& srcPointCloud = *srcPointCloudPtr;
    VectorPointXYZ coords;
    coords.reserve(srcPointCloud.size());

    for (auto& point : srcPointCloud) {
        coords.push_back(point);
    }
    return coords;
}

/* Registration error bias */
TupleOfDouble registrationErrorBias(PointCloudPtr srcPointCloudPtr, PointCloudPtr transformedSrcPointCloudPtr)
{
    auto [mtre_x_src, mtre_y_src, mtre_z_src] = getCoordinates(srcPointCloudPtr);
    auto [mtre_x_transformed_src, mtre_y_transformed_src, mtre_z_transformed_src] = getCoordinates(transformedSrcPointCloudPtr);

    std::vector<double> mtre_x, mtre_y, mtre_z;
    mtre_x.reserve(mtre_x_src.size());
    mtre_y.reserve(mtre_y_src.size());
    mtre_z.reserve(mtre_z_src.size());

    for (size_t i = 0; i < mtre_x_src.size(); ++i) {
        mtre_x.push_back(mtre_x_src[i] - mtre_x_transformed_src[i]);
        mtre_y.push_back(mtre_y_src[i] - mtre_y_transformed_src[i]);
        mtre_z.push_back(mtre_z_src[i] - mtre_z_transformed_src[i]);
    }

    double avg_x = std::accumulate(mtre_x.begin(), mtre_x.end(), 0.0) / mtre_x.size();
    double avg_y = std::accumulate(mtre_y.begin(), mtre_y.end(), 0.0) / mtre_y.size();
    double avg_z = std::accumulate(mtre_z.begin(), mtre_z.end(), 0.0) / mtre_z.size();

    return {avg_x, avg_y, avg_z};
}

/* Mean target registration error */
double meanTargetRegistrationError(PointCloudPtr srcPointCloudPtr, PointCloudPtr transformedSrcPointCloudPtr)
{
    auto coordinatesSrc = Get3DCoordinatesXYZ(srcPointCloudPtr);
    auto coordinatesSrcTransformed = Get3DCoordinatesXYZ(transformedSrcPointCloudPtr);

    std::vector<double> norms;
    norms.reserve(coordinatesSrc.size());

    for (size_t i = 0; i < coordinatesSrc.size(); ++i) {
        norms.push_back(distance(coordinatesSrc[i], coordinatesSrcTransformed[i]));
    }

    return std::accumulate(norms.begin(), norms.end(), 0.0) / norms.size();
}

/* Append current time stamp to provided filename (after stripping any extension) */
std::string appendTimestamp(const std::string& filename)
{
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
    return removeExtension(filename) + "_" + oss.str();
}

/* Full registration pipeline including the errors calculation */
void fullRegistration(const std::string &fullParametersFilename,
                      Settings pipelineSettings,
                      const std::string &savedFilename,
                      double *seedRef,
                      double *seedSource,
                      double *seedCustomRotation)
{
    double initialSeedRef = *seedRef;
    double initialSeedSource = *seedSource;
    double initialSeedRot = *seedCustomRotation;

    auto parameters = readParameters(fullParametersFilename);
    auto parametersList = parametersArray(fullParametersFilename, parameters);

    std::string pipelineType, typeTransformation, referenceDataFilename;
    double x_uncertainty, y_uncertainty, sourceWidth, sourceHeight;
    double x_angle_min, x_angle_max, y_angle_min, y_angle_max, z_angle_min, z_angle_max;

    std::tie(pipelineType, typeTransformation, referenceDataFilename,
             x_uncertainty, y_uncertainty, sourceWidth, sourceHeight,
             x_angle_min, x_angle_max, y_angle_min, y_angle_max, z_angle_min, z_angle_max) = parametersList;

    TuplePointCloudPtr PointCloudsPtrOutput;

    if (pipelineType == "all") {
        std::cout << "\nVisualizer parameter : " << pipelineSettings.getValue(VISUALIZER_PARAMETER) << std::endl;
        PointCloudsPtrOutput = fullPipeline(parametersList, seedRef, seedSource,
                                             seedCustomRotation, pipelineSettings);
    } else if (pipelineType == "sift") {
        std::cout << "\nVisualizer parameter : " << pipelineSettings.getValue(VISUALIZER_PARAMETER) << std::endl;
        PointCloudsPtrOutput = fullPipelineSift(parametersList, seedRef, seedSource,
                                                 seedCustomRotation, pipelineSettings);
    } else if (pipelineType == "harris") {
        std::cout << "\nVisualizer parameter : " << pipelineSettings.getValue(VISUALIZER_PARAMETER) << std::endl;
        PointCloudsPtrOutput = fullPipelineHarris(parametersList, seedRef, seedSource,
                                                   seedCustomRotation, pipelineSettings);
    } else {
        std::cout << "===ERROR=== aborting: pipelineType not recognized: " << pipelineType << std::endl;
        return;
    }

    PointCloudPtr sourceCloudPtr, targetCloudPtr, sourceTransformedCloudPtr, transformedCloudPtr;
    double angleX, angleY, angleZ;
    std::tie(sourceCloudPtr, targetCloudPtr, sourceTransformedCloudPtr, transformedCloudPtr,
             angleX, angleY, angleZ) = PointCloudsPtrOutput;

    // Calculate the mean target registration error
    std::cout << "\n-----------------------------------------------------------------------------------------------\n"
              << "--------------------------------- Mean Target Registration Error --------------------------------\n"
              << "-------------------------------------------------------------------------------------------------\n";
    double mtre = meanTargetRegistrationError(sourceCloudPtr, transformedCloudPtr);
    std::cout << "[" << pipelineType << "] Mean Target Registration Error between original source and transformed cloud: "
              << mtre << std::endl;

    // Calculate the Registration error bias
    auto bias = registrationErrorBias(sourceCloudPtr, transformedCloudPtr);
    double bias_x, bias_y, bias_z;
    std::tie(bias_x, bias_y, bias_z) = bias;

    std::cout << "\n-----------------------------------------------------------------------------------------------\n"
              << "------------------------------------ Registration error bias ------------------------------------\n"
              << "-------------------------------------------------------------------------------------------------\n"
              << "\n[" << pipelineType << "] Registration error bias on x: " << bias_x << "\n"
              << "[" << pipelineType << "] Registration error bias on y: " << bias_y << "\n"
              << "[" << pipelineType << "] Registration error bias on z: " << bias_z << std::endl;

    // Populate settings map for writing to results file
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

    auto pipelineSettingsMap = pipelineSettings.getAllSettings();
    for (const auto& item : pipelineSettingsMap) {
        settingsMap[item.first] = std::to_string(item.second);
    }

    saveResults(savedFilename, settingsMap);
}