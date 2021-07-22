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

    if (argc < 1)
    {
        throw std::runtime_error("Required arguments: number of iterations");
    }
    std::string charNumberIter = argv[1];
    int numberIter = std::stoi(charNumberIter);

    // Harris runs y675_58_5025_17 data file (experiment 2)
    std::cout << "\nHARRIS RUNS\n"
              << std::endl;
    for (size_t i = 1; i <= numberIter; i++)
    {
        // Harris y675_58_5025_17
        std::string folderName = "../data/", parametersFileName = "harris_600m-40deg-FOV_rotation-Z.txt"; 
        std::string fullParametersFilename = folderName + parametersFileName; // including path
        double seedRef(0.0), seedSource(0.0), seedCustomRotation(0.0);
        seedRef += 0.5, seedSource += 0.5, seedCustomRotation += 0.5;
        fullRegistration(fullParametersFilename, parametersFileName, seedRef, seedSource, seedCustomRotation);
    }

    // Sift runs y675_58_5025_17 data file
    std::cout << "\nSIFT RUNS\n"
              << std::endl;
    for (size_t i = 1; i <= numberIter; i++)
    {
        // SIFT y675_58_5025_17
        std::string folderName = "../data/", parametersFileName = "sift_600m-40deg-FOV_rotation-Z.txt";
        std::string fullParametersFilename = folderName + parametersFileName; // including path
        double seedRef(0.0), seedSource(0.0), seedCustomRotation(0.0);
        seedRef += 0.5, seedSource += 0.5, seedCustomRotation += 0.5;
        fullRegistration(fullParametersFilename, parametersFileName, seedRef, seedSource, seedCustomRotation);
    }

    return 0;
}