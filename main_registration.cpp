#include "./include/tools.hpp"

int main(int argc, char **argv)
{
    if (argc < 1)
    {
        throw std::runtime_error("Required arguments: source_pcl_filename.pcd target_pcl_filename.pcd");
    }
    std::string folderName = "../data/";
    std::string suffixFileName = argv[1];
    std::string parametersFilename = folderName + suffixFileName;
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

    // Run the full pipeline with different seeds
    tuplePointCloudPtr PointCloudsPtrOutput;
    PointCloudPtr sourceCloudPtr(new PointCloud);
    PointCloudPtr targetCloudPtr(new PointCloud);
    PointCloudPtr transformedCloudPtr1(new PointCloud);
    PointCloudPtr transformedCloudPtr2(new PointCloud);

    // Registration pipeline with SIFT & Harris
    double seedRef(1.0), seedSource(1.0), seedCustomRotation(1.0);
    PointCloudsPtrOutput = fullPipeline(parametersList, seedRef, seedSource, seedCustomRotation);
    std::tie(sourceCloudPtr, targetCloudPtr, transformedCloudPtr1, transformedCloudPtr2) = PointCloudsPtrOutput;

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

    // Saving the results
    std::ofstream outfile;
    outfile.open("./results/data_save_run1.txt", std::ios_base::app);
    outfile << "[SIFT] MRTE : " << euclidean_distance_sift << "\n";
    outfile << "[HARRIS] MRTE : " << euclidean_distance_harris << "\n";
    outfile << "[SEED] Reference cutout : " << seedRef << "\n";
    outfile << "[SEED] Source cutout : " << seedSource << "\n";
    outfile << "[SEED] Custom rotation : " << seedCustomRotation << "\n";
    outfile.close();
    std::cout << "Results saved." << std::endl;

    // Visualization
    //visualizeCroppedResults(targetCloudPtr, sourceCloudPtr, transformedCloudPtr1);

    visualizeMultipleResults(targetCloudPtr,
                             sourceCloudPtr,
                             transformedCloudPtr1,
                             targetCloudPtr,
                             sourceCloudPtr,
                             transformedCloudPtr2);

    return 0;
}