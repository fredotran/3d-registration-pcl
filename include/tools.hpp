#include "visualization_tools.hpp"

////////////////////////////////
////////// PROTOTYPES //////////
////////////////////////////////

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
PointCloudPtr randomCropPointCloud(PointCloudPtr cloudPtr,
                                   PointCloudPtr cloudOutPtr,
                                   double x_boundaries_min, double x_boundaries_max,
                                   double y_boundaries_min, double y_boundaries_max,
                                   double z_boundaries_min, double z_boundaries_max,
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
    //std::cout << centroid << std::endl;
    transform_translation.translation() = new_centroid.head<3>() - centroid.head<3>();

    PointCloudPtr transformed_cloud(new PointCloud);
    // You can either apply transform_1 or transform_2; they are the same
    std::cout << "Transformation used for the origin correction : \n"
              << transform_translation.matrix() << std::endl;
    pcl::transformPointCloud(*localPointCloudPtr, *transformed_cloud, transform_translation);
    std::cout << "\nSource point cloud (white) " << std::endl;
    std::cout << "Corrected coordinates point cloud (green) " << std::endl;
    //visualizeMatchingResults(localPointCloudPtr, transformed_cloud);

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
    //pcl::transformPointCloud(*rotation_cloud, *new_cloud, inverse_transform_translation);
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
    double xBuffer = xUncertainty;
    double yBuffer = yUncertainty;
    std::array<double, 2> centroid = randomizeCoordinate(seed, refModelBoundariesX, refModelBoundariesY, xBuffer, yBuffer);

    // compute reference point cloud boundaries
    double xMin = centroid[0] - sourceWidth * 0.5;
    double xMax = centroid[0] + sourceWidth * 0.5;
    double yMin = centroid[1] - sourceHeight * 0.5;
    double yMax = centroid[1] + sourceHeight * 0.5;
    std::array<double, 4> boundaries = {xMin, xMax, yMin, yMax};

    return boundaries;
}

/* Returns the cropped point cloud based on the dimensions of the input cloud*/
PointCloudPtr randomCropPointCloud(PointCloudPtr cloudPtr,
                                   PointCloudPtr cloudOutPtr,
                                   double x_boundaries_min, double x_boundaries_max,
                                   double y_boundaries_min, double y_boundaries_max,
                                   double z_boundaries_min, double z_boundaries_max,
                                   double randomized_x_min, double randomized_x_max,
                                   double randomized_y_min, double randomized_y_max)
{
    double ratioWidthAndHeight = abs((x_boundaries_max - x_boundaries_min) / (y_boundaries_max - y_boundaries_min));
    std::cout << "\nRatio between width and height : " << ratioWidthAndHeight << std::endl;
    // Get the local boundaries
    double local_boundaries_x = abs(x_boundaries_max - x_boundaries_min);
    double local_boundaries_y = abs(y_boundaries_max - y_boundaries_min);
    double local_boundaries_z = abs(z_boundaries_max - z_boundaries_min);
    double crop_x_min = abs(randomized_x_min - x_boundaries_min);
    double crop_y_min = abs(randomized_y_min - y_boundaries_min);
    std::cout << "Global minimum dimensions : " << x_boundaries_min << " " << y_boundaries_min << " " << z_boundaries_min << "\n";
    std::cout << "Global maximum dimensions : " << x_boundaries_max << " " << y_boundaries_max << " " << z_boundaries_max << "\n";
    std::cout << "Local box dimensions (before cropping): x = " << local_boundaries_x << ", y = " << local_boundaries_y << ", z = " << local_boundaries_z << endl;

    double croppingCubeMin_x = (crop_x_min) + x_boundaries_min;
    double croppingCubeMin_y = (crop_y_min) + y_boundaries_min;

    Eigen::Vector4f croppingCubeMinPoints;
    croppingCubeMinPoints[0] = croppingCubeMin_x; // x axis
    croppingCubeMinPoints[1] = croppingCubeMin_y; // y axis
    croppingCubeMinPoints[2] = z_boundaries_min;  // z axis
    croppingCubeMinPoints[3] = 1.0;               // z axis

    double crop_x_max = abs(randomized_x_max - x_boundaries_max);
    double crop_y_max = abs(randomized_y_max - y_boundaries_max);
    double croppingCubeMax_x = x_boundaries_max - (crop_x_max);
    double croppingCubeMax_y = y_boundaries_max - (crop_y_max);

    Eigen::Vector4f croppingCubeMaxPoints;
    croppingCubeMaxPoints[0] = croppingCubeMax_x; // x axis
    croppingCubeMaxPoints[1] = croppingCubeMax_y; // y axis
    croppingCubeMaxPoints[2] = z_boundaries_max;  // z axis
    croppingCubeMaxPoints[3] = 1.0;               // z axis

    double newLocal_x = abs(x_boundaries_max - (crop_x_min + crop_x_max) - x_boundaries_min);
    double newLocal_y = abs(y_boundaries_max - (crop_y_min + crop_y_max) - y_boundaries_min);
    double newLocal_z = abs(z_boundaries_max - z_boundaries_min);
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
PointCloudPtr computeReferenceCloud(PointCloudPtr surfaceModelCloudPtr,
                                    double sourceWidth, double sourceHeight,
                                    double x_uncertainty, double y_uncertainty)
{

    PointCloudPtr targetCloudPtr(new PointCloud);
    PointCloudPtr croppedPointCloudPtr(new PointCloud);
    PointCloudPtr outPointCloudPtr(new PointCloud);
    pcl::PointXYZ ref_min_pt, ref_max_pt;

    // GetMinMax of the input point cloud
    pcl::getMinMax3D(*surfaceModelCloudPtr, ref_min_pt, ref_max_pt);
    double xBoundariesRef[2] = {ref_min_pt.x, ref_max_pt.x};
    double yBoundariesRef[2] = {ref_min_pt.y, ref_max_pt.y};
    double zBoundariesRef[2] = {ref_min_pt.z, ref_max_pt.z};

    std::array<double, 4> coordRandomizedRef;
    double seedRef = 1;
    coordRandomizedRef = randomizeReferenceCutout(&seedRef, xBoundariesRef, yBoundariesRef,
                                                  sourceWidth, sourceHeight,
                                                  x_uncertainty, y_uncertainty);

    double refRandomizedX_min = coordRandomizedRef[0];
    double refRandomizedX_max = coordRandomizedRef[1];
    double refRandomizedY_min = coordRandomizedRef[2];
    double refRandomizedY_max = coordRandomizedRef[3];
    // random cropping
    croppedPointCloudPtr = randomCropPointCloud(surfaceModelCloudPtr,
                                                outPointCloudPtr,
                                                xBoundariesRef[0], xBoundariesRef[1],
                                                yBoundariesRef[0], yBoundariesRef[1],
                                                zBoundariesRef[0], zBoundariesRef[1],
                                                refRandomizedX_min, refRandomizedX_max,
                                                refRandomizedY_min, refRandomizedY_max);

    copyPointCloud(*croppedPointCloudPtr, *targetCloudPtr);

    return targetCloudPtr;
}

/* Compute source point cloud */
PointCloudPtr computeSourceCloud(PointCloudPtr targetCloudPtr,
                                 double sourceWidth, double sourceHeight,
                                 double x_uncertainty, double y_uncertainty)
{
    PointCloudPtr sourceCloudPtr(new PointCloud);
    PointCloudPtr outPointCloudPtr(new PointCloud);

    // Compute source data
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*targetCloudPtr, min_pt, max_pt);
    double xBoundaries[2] = {min_pt.x, max_pt.x};
    double yBoundaries[2] = {min_pt.y, max_pt.y};
    double zBoundaries[2] = {min_pt.z, max_pt.z};

    std::array<double, 4> coordSourceRandomized;
    double seed = 2;
    coordSourceRandomized = randomizeSourceCutout(&seed, xBoundaries, yBoundaries,
                                                  sourceWidth, sourceHeight,
                                                  x_uncertainty, y_uncertainty);

    double RandomizedX_min_source = coordSourceRandomized[0];
    double RandomizedX_max_source = coordSourceRandomized[1];
    double RandomizedY_min_source = coordSourceRandomized[2];
    double RandomizedY_max_source = coordSourceRandomized[3];
    // random cropping
    sourceCloudPtr = randomCropPointCloud(targetCloudPtr,
                                          outPointCloudPtr,
                                          xBoundaries[0], xBoundaries[1],
                                          yBoundaries[0], yBoundaries[1],
                                          zBoundaries[0], zBoundaries[1],
                                          RandomizedX_min_source, RandomizedX_max_source,
                                          RandomizedY_min_source, RandomizedY_max_source);

    return sourceCloudPtr;
}