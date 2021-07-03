#include "./include/tools.hpp"

int main(int argc, char **argv)
{
    std::string parametersFilename = "../data/parameters.txt";
    std::vector<std::string> parameters;
    tupleParameters parametersList;

    // Read the parameters from filename
    parameters = readParameters(parametersFilename);
    // Create an array containing all the elements from the text file
    parametersList = parametersArray(parametersFilename, parameters);
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

    // Compute reference data
    std::string modelSurfaceFileName = referenceDataFilename;
    std::cout << "Reading " << modelSurfaceFileName << std::endl;
    surfaceModelCloudPtr = loadingCloud(referenceDataFilename);

    targetCloudPtr = computeReferenceCloud(surfaceModelCloudPtr,
                                           sourceWidth, sourceHeight,
                                           x_uncertainty, y_uncertainty);

    sourceCloudPtr = computeSourceCloud(targetCloudPtr,
                                        sourceWidth, sourceHeight,
                                        x_uncertainty, y_uncertainty);

    // visualization
    visualizeCroppedResults(surfaceModelCloudPtr, targetCloudPtr, sourceCloudPtr);

    // Randomization of angles
    double randomAngleOnX, randomAngleOnY, randomAngleOnZ;
    double seed(1.0);

    randomAngleOnX = randomizeDoubleUniform(&seed, x_angle_min, x_angle_max);
    randomAngleOnY = randomizeDoubleUniform(&seed, y_angle_min, y_angle_max);
    randomAngleOnZ = randomizeDoubleUniform(&seed, z_angle_min, z_angle_max);

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
    float trans_x = 0, trans_y = 0, trans_z = 100;

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

    visualizeCroppedResults(surfaceModelCloudPtr, targetCloudPtr, sourceTransformedCloudPtr);

    PointCloud &sourceTransformedCloud = *sourceTransformedCloudPtr;
    PointCloud &targetCloud = *targetCloudPtr;

    /*******************************************************/
    /******************** SIFT PIPELINE ********************/
    /*******************************************************/
    /*
    Eigen::Matrix4f translation_mat_sift;
    translation_mat_sift = translationSiftPipeline(sourceTransformedCloudPtr, targetCloudPtr);
    PointCloudPtr transformedCloudPtr1(new PointCloud);
    PointCloud &transformedCloud1 = *transformedCloudPtr1;
    pcl::transformPointCloud(sourceTransformedCloud, transformedCloud1, translation_mat_sift);
    visualizeCroppedResults(targetCloudPtr, sourceCloudPtr, transformedCloudPtr1);

    Eigen::Matrix4f rotation_mat_sift;
    rotation_mat_sift = rotationSiftPipeline(transformedCloudPtr1, targetCloudPtr);
    PointCloudPtr finalTransformedCloudPtr(new PointCloud);
    PointCloud &finalTransformedCloud = *finalTransformedCloudPtr;
    pcl::transformPointCloud(sourceTransformedCloud, finalTransformedCloud, rotation_mat_sift);
    visualizeCroppedResults(targetCloudPtr, sourceCloudPtr, finalTransformedCloudPtr);
*/
    /*******************************************************/
    /******************* HARRIS PIPELINE *******************/
    /*******************************************************/
    /*
    Eigen::Matrix4f translation_mat_harris;
    translation_mat_harris = translationHarrisPipeline(sourceTransformedCloudPtr, targetCloudPtr);
    PointCloudPtr transformedCloudPtr2(new PointCloud);
    PointCloud &transformedCloud2 = *transformedCloudPtr2;
    pcl::transformPointCloud(sourceTransformedCloud, transformedCloud2, translation_mat_harris);
    visualizeCroppedResults(targetCloudPtr, sourceCloudPtr, transformedCloudPtr2);*/

    Eigen::Matrix4f rotation_mat_harris;
    rotation_mat_harris = rotationHarrisPipeline(sourceTransformedCloudPtr, targetCloudPtr);
    PointCloudPtr finalTransformedCloudPtr2(new PointCloud);
    PointCloud &finalTransformedCloud2 = *finalTransformedCloudPtr2;
    pcl::transformPointCloud(sourceTransformedCloud, finalTransformedCloud2, rotation_mat_harris);
    visualizeCroppedResults(targetCloudPtr, sourceCloudPtr, finalTransformedCloudPtr2);

    return 0;
}