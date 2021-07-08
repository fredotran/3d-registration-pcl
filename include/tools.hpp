#include "visualization_tools.hpp"

////////////////////////////////
////////// PROTOTYPES //////////
////////////////////////////////

// saving tools
void savingResults(std::string &parametersFilename, double siftMRTE, double harrisMRTE,
                   double seedRef, double seedSource, double seedCustomRotation);

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

// Parameters input
tupleParameters parametersArray(std::string parametersFilename,
                                std::vector<std::string> parameters);
std::vector<std::string> readParameters(std::string &inputFilename);

// Randomization
float randomAngles(float angleMin, float angleMax);
double randomizeDoubleUniform(double *seed, double min, double max);
std::array<double, 2> randomizeCoordinate(double *seed, double boundariesX[2], double boundariesY[2], double bufferX, double bufferY);
std::array<double, 4> randomizeSourceCutout(double *seed, double refModelBoundariesX[2], double refModelBoundariesY[2],
                                            double sourceWidth, double sourceHeight, double xUncertainty, double yUncertainty);
PointCloudPtr cropPointCloud(PointCloudPtr cloudPtr,
                             PointCloudPtr cloudOutPtr,
                             double randomized_x_min, double randomized_x_max,
                             double randomized_y_min, double randomized_y_max);

// Compute point clouds
PointCloudPtr computeReferenceCloud(PointCloudPtr surfaceModelCloudPtr,
                                    double sourceWidth, double sourceHeight,
                                    double x_uncertainty, double y_uncertainty);

PointCloudPtr computeSourceCloud(PointCloudPtr targetCloudPtr,
                                 double sourceWidth, double sourceHeight,
                                 double x_uncertainty, double y_uncertainty);

/*******************************************************************************************************************************************************/
/*******************************************************************************************************************************************************/
/*******************************************************************************************************************************************************/

////////////////////////////////
//////////// METHODS ///////////
////////////////////////////////

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

/* Reading the parameters text file */
std::vector<std::string> readParameters(std::string &inputFilename)
{
    ifstream inFile;
    inFile.open(inputFilename);
    std::vector<std::string> tokens;
    std::string titles = "", parameters = "";

    if (inFile.is_open())
        std::cout << "File has been opened" << std::endl;
    else
        std::cout << "NO FILE HAS BEEN OPENED" << std::endl;

    while (!inFile.eof())
    {
        getline(inFile, titles, '=');      //grtting string upto =
        getline(inFile, parameters, '\n'); //getting string after =
        tokens.push_back(parameters);
    }
    inFile.close();

    return tokens;
}

/* Transforming the data from the text file to the corresponding types*/
tupleParameters parametersArray(std::string parametersFilename,
                                std::vector<std::string> parameters)
{
    std::string pipelineType = parameters[0];
    double numIterPipeline = std::stod(parameters[1]);
    std::string referenceDataFilename = parameters[2];
    double x_uncertainty = std::stod(parameters[3]);
    double y_uncertainty = std::stod(parameters[4]);
    double sourceWidth = std::stod(parameters[5]);
    double sourceHeight = std::stod(parameters[6]);
    double x_angle_min = std::stod(parameters[7]);
    double x_angle_max = std::stod(parameters[8]);
    double y_angle_min = std::stod(parameters[9]);
    double y_angle_max = std::stod(parameters[10]);
    double z_angle_min = std::stod(parameters[11]);
    double z_angle_max = std::stod(parameters[12]);

    return {pipelineType, numIterPipeline, referenceDataFilename,
            x_uncertainty, y_uncertainty, sourceWidth, sourceHeight,
            x_angle_min, x_angle_max, y_angle_min, y_angle_max, z_angle_min, z_angle_max};
}

int randomAngle(int angleMin, int angleMax)
{
    srand((unsigned)time(0));
    int randomizedAngle;
    randomizedAngle = (rand() % angleMax) + angleMin;
    return randomizedAngle;
}

/* Generates a random number between min and max (uniform distribution). The seed will be incremented so that it 
can be directly used in subsequent calls. */
double randomizeDoubleUniform(double *seed, double min, double max)
{
    *seed += 1;
    std::mt19937 generator(*seed);                           //Standard mersenne_twister_engine
    std::uniform_real_distribution<> distribution(min, max); // define distribution
    return distribution(generator);                          // generate random double
}

/* Randomize a 2D coordinate within the provided boundaries under the condition that the distance between the coordinate and 
the boundaries may not be less than the specified buffers. */
std::array<double, 2> randomizeCoordinate(double *seed, double boundariesX[2], double boundariesY[2], double bufferX, double bufferY)
{
    double xMin = boundariesX[0] + bufferX;
    double xMax = boundariesX[1] - bufferX;
    double yMin = boundariesY[0] + bufferY;
    double yMax = boundariesY[1] - bufferY;
    std::array<double, 2> coord;
    coord[0] = randomizeDoubleUniform(seed, xMin, xMax);
    coord[1] = randomizeDoubleUniform(seed, yMin, yMax);
    return coord;
}

/* Returns randomized rectangular (xMin, xMax, yMin, yMax) cut-out boundaries to be used for reference point cloud extraction 
from surface model point cloud(input: surface model data boundaries, source_width, 
source_height, x_uncertainty, y_uncertainty; output: reference PC centroid x & y, reference_width, reference_height) */
std::array<double, 4> randomizeReferenceCutout(double *seed, double surfaceModelBoundariesX[2], double surfaceModelBoundariesY[2],
                                               double sourceWidth, double sourceHeight, double xUncertainty, double yUncertainty)
{
    // compute width and height of reference point cloud
    double refWidth = sourceWidth + 2 * xUncertainty;
    double refHeight = sourceHeight + 2 * yUncertainty;
    double refSize = std::max(refWidth, refHeight);

    // randomize reference point cloud centroid
    std::array<double, 2> centroid = randomizeCoordinate(seed, surfaceModelBoundariesX, surfaceModelBoundariesY, refSize * 0.5, refSize * 0.5);

    // compute reference point cloud boundaries
    double xMin = centroid[0] - refSize * 0.5;
    double xMax = centroid[0] + refSize * 0.5;
    double yMin = centroid[1] - refSize * 0.5;
    double yMax = centroid[1] + refSize * 0.5;
    std::array<double, 4> boundaries = {xMin, xMax, yMin, yMax};

    return boundaries;
}

/* Returns randomized rectangular (xMin, xMax, yMin, yMax) cut-out boundaries to be used for source point cloud extraction
from reference point cloud */
std::array<double, 4> randomizeSourceCutout(double *seed, double refModelBoundariesX[2], double refModelBoundariesY[2],
                                            double sourceWidth, double sourceHeight, double xUncertainty, double yUncertainty)
{
    // randomize source centroid
    double xBuffer = xUncertainty + (0.5 * sourceWidth);
    double yBuffer = yUncertainty + (0.5 * sourceHeight);
    double buffer = std::max(xBuffer, yBuffer);
    std::array<double, 2> centroid = randomizeCoordinate(seed, refModelBoundariesX, refModelBoundariesY, buffer, buffer);

    // compute reference point cloud boundaries
    double xMin = centroid[0] - sourceWidth * 0.5;
    double xMax = centroid[0] + sourceWidth * 0.5;
    double yMin = centroid[1] - sourceHeight * 0.5;
    double yMax = centroid[1] + sourceHeight * 0.5;
    std::array<double, 4> boundaries = {xMin, xMax, yMin, yMax};

    return boundaries;
}

/* Returns the cropped point cloud based on the dimensions of the input cloud*/
PointCloudPtr cropPointCloud(PointCloudPtr cloudPtr,
                             PointCloudPtr cloudOutPtr,
                             double randomized_x_min, double randomized_x_max,
                             double randomized_y_min, double randomized_y_max)
{
    // GetMinMax of the input point cloud
    pcl::PointXYZ ref_min_pt, ref_max_pt;
    pcl::getMinMax3D(*cloudPtr, ref_min_pt, ref_max_pt);
    double xBoundariesRef[2] = {ref_min_pt.x, ref_max_pt.x};
    double yBoundariesRef[2] = {ref_min_pt.y, ref_max_pt.y};
    double zBoundariesRef[2] = {ref_min_pt.z, ref_max_pt.z};
    // Get the local boundaries
    double local_boundaries_x = abs(xBoundariesRef[1] - xBoundariesRef[0]);
    double local_boundaries_y = abs(yBoundariesRef[1] - yBoundariesRef[0]);
    double local_boundaries_z = abs(zBoundariesRef[1] - zBoundariesRef[0]);
    std::cout << "Global minimum dimensions : " << xBoundariesRef[0] << " " << yBoundariesRef[0] << " " << zBoundariesRef[0] << "\n";
    std::cout << "Global maximum dimensions : " << xBoundariesRef[1] << " " << yBoundariesRef[1] << " " << zBoundariesRef[1] << "\n";
    std::cout << "Local box dimensions (before cropping): x = " << local_boundaries_x << ", y = " << local_boundaries_y << ", z = " << local_boundaries_z << endl;

    // Min (left)
    double croppingCubeMin_x = randomized_x_min; //+ x_boundaries_min;
    double croppingCubeMin_y = randomized_y_min; //+ y_boundaries_min;
    Eigen::Vector4f croppingCubeMinPoints;
    croppingCubeMinPoints[0] = croppingCubeMin_x; // x axis
    croppingCubeMinPoints[1] = croppingCubeMin_y; // y axis
    croppingCubeMinPoints[2] = zBoundariesRef[0]; // z axis
    croppingCubeMinPoints[3] = 1.0;               // z axis

    // Max (right)
    double croppingCubeMax_x = randomized_x_max;
    double croppingCubeMax_y = randomized_y_max;
    Eigen::Vector4f croppingCubeMaxPoints;
    croppingCubeMaxPoints[0] = croppingCubeMax_x; // x axis
    croppingCubeMaxPoints[1] = croppingCubeMax_y; // y axis
    croppingCubeMaxPoints[2] = zBoundariesRef[1]; // z axis
    croppingCubeMaxPoints[3] = 1.0;               // z axis

    // Display dimensions after cropping
    double newLocal_x = abs(randomized_x_max - randomized_x_min);
    double newLocal_y = abs(randomized_y_max - randomized_y_min);
    double newLocal_z = local_boundaries_z;
    std::cout << "Local box dimensions (after cropping): x = " << newLocal_x << ", y = " << newLocal_y << ", z = " << newLocal_z << endl;

    // crop point cloud
    pcl::CropBox<PointXYZ> cropFilter;
    cropFilter.setInputCloud(cloudPtr);
    cropFilter.setMin(croppingCubeMinPoints);
    cropFilter.setMax(croppingCubeMaxPoints);
    cropFilter.filter(*cloudOutPtr);

    return cloudOutPtr;
}

/* Compute reference point cloud */
PointCloudPtr computeReferenceCloud(PointCloudPtr surfaceModelCloudPtr, double seedRef,
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
    coordRandomizedRef = randomizeReferenceCutout(&seedRef, xBoundariesRef, yBoundariesRef,
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
PointCloudPtr computeSourceCloud(PointCloudPtr targetCloudPtr, double seed,
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
    coordSourceRandomized = randomizeSourceCutout(&seed, xBoundaries, yBoundaries,
                                                  sourceWidth, sourceHeight,
                                                  x_uncertainty, y_uncertainty);

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

/* Full pipeline to run (SIFT + HARRIS) and compare */
tuplePointCloudPtr fullPipeline(tupleParameters parametersList, double seedRef, double seedSource, double seedCustomRotation)
{
    // Extract values from tuple
    std::string pipelineType, referenceDataFilename;
    double numIterPipeline, x_uncertainty, y_uncertainty, sourceWidth, sourceHeight;
    double x_angle_min, x_angle_max, y_angle_min, y_angle_max, z_angle_min, z_angle_max;

    // Method to unzip multiple returns value from parametersArray
    std::tie(pipelineType, numIterPipeline, referenceDataFilename,
             x_uncertainty, y_uncertainty, sourceWidth, sourceHeight,
             x_angle_min, x_angle_max, y_angle_min, y_angle_max, z_angle_min, z_angle_max) = parametersList;

    /*******************************************************/
    /***************** COMPUTE POINT CLOUDS ****************/
    /*******************************************************/

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
    randomAngleOnX = randomizeDoubleUniform(&seedCustomRotation, x_angle_min, x_angle_max);
    randomAngleOnY = randomizeDoubleUniform(&seedCustomRotation, y_angle_min, y_angle_max);
    randomAngleOnZ = randomizeDoubleUniform(&seedCustomRotation, z_angle_min, z_angle_max);

    std::cout << "angle on x : " << randomAngleOnX << std::endl;
    std::cout << "angle on y : " << randomAngleOnY << std::endl;
    std::cout << "angle on z : " << randomAngleOnZ << std::endl;

    /*******************************************************/
    /******************* CUSTOM ROTATION *******************/
    /*******************************************************/

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

    /*******************************************************/
    /******************** SIFT PIPELINE ********************/
    /*******************************************************/

    Eigen::Matrix4f rotation_mat_sift;
    rotation_mat_sift = rotationSiftPipeline(sourceTransformedCloudPtr, targetCloudPtr);
    PointCloudPtr finalTransformedCloudPtr(new PointCloud);
    PointCloud &finalTransformedCloud = *finalTransformedCloudPtr;
    pcl::transformPointCloud(sourceTransformedCloud, finalTransformedCloud, rotation_mat_sift);

    /*******************************************************/
    /******************* HARRIS PIPELINE *******************/
    /*******************************************************/

    Eigen::Matrix4f rotation_mat_harris;
    rotation_mat_harris = rotationHarrisPipeline(sourceTransformedCloudPtr, targetCloudPtr);
    PointCloudPtr finalTransformedCloudPtr2(new PointCloud);
    PointCloud &finalTransformedCloud2 = *finalTransformedCloudPtr2;
    pcl::transformPointCloud(sourceTransformedCloud, finalTransformedCloud2, rotation_mat_harris);

    return {sourceCloudPtr, targetCloudPtr, sourceTransformedCloudPtr, finalTransformedCloudPtr, finalTransformedCloudPtr2};
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

/* FUll registration pipeline including the errors calculation */
void fullRegistration(std::string &fullParametersFilename,
                      std::string &parametersFileName,
                      double &seedRef,
                      double &seedSource,
                      double &seedCustomRotation)
{
    tupleParameters parametersList;
    std::vector<std::string> parameters;

    // Read the parameters from filename
    parameters = readParameters(fullParametersFilename);
    // Create an array containing all the elements from the text file
    parametersList = parametersArray(fullParametersFilename, parameters);
    // Extract values from tuple
    std::string pipelineType, referenceDataFilename;
    double numIterPipeline, x_uncertainty, y_uncertainty, sourceWidth, sourceHeight;
    double x_angle_min, x_angle_max, y_angle_min, y_angle_max, z_angle_min, z_angle_max;

    // Method to unzip multiple returns value from parametersArray
    std::tie(pipelineType, numIterPipeline, referenceDataFilename,
             x_uncertainty, y_uncertainty, sourceWidth, sourceHeight,
             x_angle_min, x_angle_max, y_angle_min, y_angle_max, z_angle_min, z_angle_max) = parametersList;

    // Run the full pipeline with different seeds
    tuplePointCloudPtr PointCloudsPtrOutput;
    PointCloudPtr sourceCloudPtr(new PointCloud);
    PointCloudPtr targetCloudPtr(new PointCloud);
    PointCloudPtr sourceTransformedCloudPtr(new PointCloud);
    PointCloudPtr transformedCloudPtr1(new PointCloud);
    PointCloudPtr transformedCloudPtr2(new PointCloud);

    // Registration pipeline with SIFT & Harris
    //double seedRef(1.0), seedSource(1.0), seedCustomRotation(1.0);
    PointCloudsPtrOutput = fullPipeline(parametersList, seedRef, seedSource, seedCustomRotation);
    std::tie(sourceCloudPtr, targetCloudPtr, sourceTransformedCloudPtr, transformedCloudPtr1, transformedCloudPtr2) = PointCloudsPtrOutput;

    // Calculate the mean target registration error
    std::cout << "\n-----------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "--------------------------------- Mean Target Registration Error --------------------------------" << std::endl;
    std::cout << "-------------------------------------------------------------------------------------------------" << std::endl;
    double euclidean_distance_sift = meanTargetRegistrationError(sourceCloudPtr, transformedCloudPtr1);
    double euclidean_distance_harris = meanTargetRegistrationError(sourceCloudPtr, transformedCloudPtr2);
    std::cout << "\n[SIFT] Mean Target Registration Error between original source and transformed cloud: " << euclidean_distance_sift << std::endl;
    std::cout << "[HARRIS] Mean Target Registration Error between original source and transformed cloud: " << euclidean_distance_harris << std::endl;

    // Calculate the Registration error bias
    PointCloud &sourceCloud = *sourceCloudPtr;
    tupleOfDouble vectorMTRESift;
    tupleOfDouble vectorMTREHarris;
    double mtre_x_sift, mtre_y_sift, mtre_z_sift;
    double mtre_x_harris, mtre_y_harris, mtre_z_harris;

    vectorMTRESift = registrationErrorBias(sourceCloudPtr, transformedCloudPtr1);
    vectorMTREHarris = registrationErrorBias(sourceCloudPtr, transformedCloudPtr2);
    std::tie(mtre_x_sift, mtre_y_sift, mtre_z_sift) = vectorMTRESift;
    std::tie(mtre_x_harris, mtre_y_harris, mtre_z_harris) = vectorMTREHarris;

    std::cout << "\n-----------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "------------------------------------ Registration error bias ------------------------------------" << std::endl;
    std::cout << "-------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "\n[SIFT] Registration error bias on x between original source and transformed cloud: " << mtre_x_sift << std::endl;
    std::cout << "[SIFT] Registration error bias on y between original source and transformed cloud: " << mtre_y_sift << std::endl;
    std::cout << "[SIFT] Registration error bias on z between original source and transformed cloud: " << mtre_z_sift << std::endl;
    std::cout << "\n[HARRIS] Registration error bias on x between original source and transformed cloud: " << mtre_x_harris << std::endl;
    std::cout << "[HARRIS] Registration error bias on y between original source and transformed cloud: " << mtre_y_harris << std::endl;
    std::cout << "[HARRIS] Registration error bias on z between original source and transformed cloud: " << mtre_z_harris << std::endl;

    savingResults(parametersFileName, euclidean_distance_sift, euclidean_distance_harris,
                  seedRef, seedSource, seedCustomRotation);

    std::cout << "\nGreen : Reference point cloud " << std::endl;
    std::cout << "Red : Source point cloud " << std::endl;
    std::cout << "Light blue : Custom transformed point cloud " << std::endl;
    std::cout << "Blue : Point cloud obtained through alignment proccess " << std::endl;
    // Visualization
    //visualizeCroppedResults(targetCloudPtr, sourceCloudPtr, transformedCloudPtr1);
    visualizeMultipleResults(targetCloudPtr,
                             sourceCloudPtr,
                             sourceTransformedCloudPtr,
                             transformedCloudPtr1,
                             targetCloudPtr,
                             sourceCloudPtr,
                             sourceTransformedCloudPtr,
                             transformedCloudPtr2);
}

void savingResults(std::string &parametersFilename, double siftMRTE, double harrisMRTE,
                   double seedRef, double seedSource, double seedCustomRotation)
{
    // Saving the results
    std::ofstream outfile;
    std::string savedFilename = "../results/" + parametersFilename + "_results";

    outfile.open(savedFilename, std::ios_base::app);
    outfile << "[SIFT] MRTE : "
            << siftMRTE << "\n";
    outfile << "[HARRIS] MRTE : "
            << harrisMRTE << "\n";
    outfile << "[SEED] Reference cutout : "
            << seedRef << "\n";
    outfile << "[SEED] Source cutout : "
            << seedSource << "\n";
    outfile << "[SEED] Custom rotation : "
            << seedCustomRotation << "\n";
    outfile.close();
    std::cout << "\nResults saved." << std::endl;
}