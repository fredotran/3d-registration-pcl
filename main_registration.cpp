#include "./include/tools.hpp"

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

    std::cout << "HARRIS RUNS\n"
              << std::endl;
    for (size_t i = 0; i <= numberIter; i++)
    {
        // Harris y675_58_5025_17
        std::string folderName = "../data/", parametersFileName = "harris_600m-40deg-FOV_rotation-Z.txt";
        std::string fullParametersFilename = folderName + parametersFileName; // including path
        double seedRef(0.0), seedSource(0.0), seedCustomRotation(0.0);
        seedRef += 1, seedSource += 1, seedCustomRotation += 1;
        fullRegistration(fullParametersFilename, parametersFileName, seedRef, seedSource, seedCustomRotation);
    }

    std::cout << "SIFT RUNS\n"
              << std::endl;
    for (size_t i = 0; i <= numberIter; i++)
    {
        // SIFT y675_58_5025_17
        std::string folderName = "../data/", parametersFileName = "sift_600m-40deg-FOV_rotation-Z.txt";
        std::string fullParametersFilename = folderName + parametersFileName; // including path
        double seedRef(0.0), seedSource(0.0), seedCustomRotation(0.0);
        seedRef += 1, seedSource += 1, seedCustomRotation += 1;
        fullRegistration(fullParametersFilename, parametersFileName, seedRef, seedSource, seedCustomRotation);
    }

    return 0;
}