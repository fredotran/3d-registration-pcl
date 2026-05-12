#include "../include/tools.hpp"

// RUN FROM THE BUILD FOLDER

int main(int argc, char **argv)
{
    if (argc < 1)
    {
        throw std::runtime_error("Required arguments: number of iterations");
    }
    int numberIter = std::stoi(argv[1]);

    Settings pipelineSettings = getPipelineDefaultSettings();

    // Harris runs y675_58_5025_17 data file
    std::cout << "\nHARRIS RUNS\n" << std::endl;
    for (size_t i = 1; i <= numberIter; i++)
    {
        std::string fullParametersFilename = "../data/harris_600m-40deg-FOV_rotation-Z.txt";
        std::string savedFilename = "../results/harris_results";
        double seedRef(0.5), seedSource(0.5), seedCustomRotation(0.5);
        fullRegistration(fullParametersFilename, pipelineSettings, savedFilename,
                         &seedRef, &seedSource, &seedCustomRotation);
    }

    // Sift runs y675_58_5025_17 data file
    std::cout << "\nSIFT RUNS\n" << std::endl;
    for (size_t i = 1; i <= numberIter; i++)
    {
        std::string fullParametersFilename = "../data/sift_600m-40deg-FOV_rotation-Z.txt";
        std::string savedFilename = "../results/sift_results";
        double seedRef(0.5), seedSource(0.5), seedCustomRotation(0.5);
        fullRegistration(fullParametersFilename, pipelineSettings, savedFilename,
                         &seedRef, &seedSource, &seedCustomRotation);
    }

    return 0;
}