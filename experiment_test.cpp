#include "./include/tools.hpp"

// RUN FROM THE BUILD FOLDER

int main(int argc, char **argv)
{
    /* // Input by the user
    if (argc < 2)
    {
        throw std::runtime_error("Required arguments: folder_name file_name");
    }
    std::string folderName = argv[1];
    std::string parametersFileName = argv[2];
    std::string fullParametersFilename = folderName + parametersFileName; // including path
    */

    /*
    if (argc < 1)
    {
        throw std::runtime_error("Required arguments: number of iterations");
    }
    std::string charNumberIter = argv[1];
    int numberIter = std::stoi(charNumberIter);

    // Harris runs y675_58_5025_17 data file (experiment 2)
    double seedRefHarris(0.0), seedSourceHarris(0.0), seedCustomRotationHarris(0.0);
    std::cout << "\nHARRIS RUNS\n"
              << std::endl;
    for (size_t i = 1; i <= numberIter; i++)
    {
        // Harris y675_58_5025_17
        std::string folderName = "../data/", parametersFileName = "harris_600m-40deg-FOV_rotation-Z.txt";
        std::string fullParametersFilename = folderName + parametersFileName; // including path
        //double seedRef(0.0), seedSource(0.0), seedCustomRotation(0.0);
        seedRefHarris += 0.5, seedSourceHarris += 0.5, seedCustomRotationHarris += 0.5;
        fullRegistrationTranslation(fullParametersFilename, parametersFileName, seedRefHarris, seedSourceHarris, seedCustomRotationHarris);
    }

    // Sift runs y675_58_5025_17 data file
    double seedRefSift(0.0), seedSourceSift(0.0), seedCustomRotationSift(0.0);
    std::cout << "\nSIFT RUNS\n"
              << std::endl;
    for (size_t i = 1; i <= numberIter; i++)
    {
        // SIFT y675_58_5025_17
        std::string folderName = "../data/", parametersFileName = "sift_600m-40deg-FOV_rotation-Z.txt";
        std::string fullParametersFilename = folderName + parametersFileName; // including path
        seedRefSift += 0.5, seedSourceSift += 0.5, seedCustomRotationSift += 0.5;
        fullRegistrationTranslation(fullParametersFilename, parametersFileName, seedRefSift, seedSourceSift, seedCustomRotationSift);
    }
*/
    Descriptor SHOT;
    PointCloudPtr sourceCloudPtr(new PointCloud);
    PointCloudPtr targetCloudPtr(new PointCloud);
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormalsPtr(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr targetNormalsPtr(new pcl::PointCloud<pcl::Normal>);
    LocalDescriptorSHOTPtr shotFeaturesSrc(new pcl::PointCloud<SHOTFeature>);
    LocalDescriptorSHOTPtr shotFeaturesTrg(new pcl::PointCloud<SHOTFeature>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormals(new pcl::search::KdTree<pcl::PointXYZ>);

    std::string sourcePath = "../data/y675_58_5025_17.pcd";
    std::string targetPath = "../data/reference_LM.pcd";

    sourceCloudPtr = loadingCloud(sourcePath);
    targetCloudPtr = loadingCloud(targetPath);

    /*
    sourceNormalsPtr = computeNormals(sourceCloudPtr, treeNormals, searchRadiusSHOT);
    targetNormalsPtr = computeNormals(targetCloudPtr, treeNormals, searchRadiusSHOT);

    shotFeaturesSrc = SHOT.computeSHOT(sourceCloudPtr, sourceNormalsPtr, treeNormals, searchRadiusSHOT);
    shotFeaturesTrg = SHOT.computeSHOT(sourceCloudPtr, sourceNormalsPtr, treeNormals, searchRadiusSHOT);
*/

    // Compute the normals
    /*
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(sourceCloudPtr);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormals(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(treeNormals);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normalEstimation.setRadiusSearch(10);
    normalEstimation.compute(*normals);
*/
    sourceNormalsPtr = computeNormals(sourceCloudPtr, treeNormals, searchRadiusSHOT);
    //targetNormalsPtr = computeNormals(targetCloudPtr, treeNormals, searchRadiusSHOT);

    // Setup the SHOT features
    pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, SHOTFeature> shotEstimation;
    shotEstimation.setInputCloud(sourceCloudPtr);
    shotEstimation.setInputNormals(sourceNormalsPtr);

    // Use the same KdTree from the normal estimation
    shotEstimation.setSearchMethod(treeNormals);
    //SHOTFeature shotFeaturesSrc(new pcl::PointCloud<ShotFeature>);
    shotEstimation.setRadiusSearch(2);

    // Actually compute the spin images
    shotEstimation.compute(*shotFeaturesSrc);
    std::cout << "SHOT output points.size (): " << shotFeaturesSrc->size() << std::endl;

    return 0;
}