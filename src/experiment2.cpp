#include <ctime>

#include "../include/tools.hpp"

// RUN FROM THE BUILD FOLDER

int main(int argc, char** argv) {
    time_t timetoday;
    time(&timetoday);
    std::cout << "experiment2 initiated on : " << asctime(localtime(&timetoday)) << std::endl;

    if (argc < 1) {
        throw std::runtime_error("Required arguments: number of iterations");
    }
    int numberIter = std::stoi(argv[1]);

    Settings pipelineSettings = getPipelineDefaultSettings();

    // Harris runs y675_58_5025_17 data file
    std::cout << "\nHARRIS RUNS\n" << std::endl;
    double seedRef(0.0), seedSource(0.1), seedCustomRotation(0.2);

    std::string fullParametersFilenameHarris = "../data/harris_600m-40deg-FOV_rotation-Z.txt";
    std::string savedFilenameHarris = "../results/harris_results";

    for (size_t i = 1; i <= numberIter; i++) {
        seedRef += 0.37913;
        seedSource += 0.3913;
        seedCustomRotation += 0.313;
        fullRegistration(fullParametersFilenameHarris, pipelineSettings, savedFilenameHarris,
                         &seedRef, &seedSource, &seedCustomRotation);
    }

    // Sift runs y675_58_5025_17 data file
    std::cout << "\nSIFT RUNS\n" << std::endl;

    seedRef = 0.0;
    seedSource = 0.1111;
    seedCustomRotation = 0.22222;

    std::string fullParametersFilenameSift = "../data/sift_600m-40deg-FOV_rotation-Z.txt";
    std::string savedFilenameSift = "../results/sift_results";

    for (size_t i = 1; i <= numberIter; i++) {
        seedRef += 0.37913;
        fullRegistration(fullParametersFilenameSift, pipelineSettings, savedFilenameSift, &seedRef,
                         &seedSource, &seedCustomRotation);
    }

    // Pipeline for all points
    std::cout << "\nPipeline for all points RUNS\n" << std::endl;

    seedRef = 0.0;
    seedSource = 0.22222;
    seedCustomRotation = 0.333333;

    std::string fullParametersFilenameAll = "../data/all_600m-40deg-FOV_rotation-Z.txt";
    std::string savedFilenameAll = "../results/all_results";

    for (size_t i = 1; i <= numberIter; i++) {
        seedRef += 0.37913;
        fullRegistration(fullParametersFilenameAll, pipelineSettings, savedFilenameAll, &seedRef,
                         &seedSource, &seedCustomRotation);
    }

    return 0;
}