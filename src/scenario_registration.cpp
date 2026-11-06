#include <ctime>

#include "../include/tools.hpp"

// RUN FROM THE BUILD FOLDER
// Reproduces the a priori displacement scenarios of Sabel et al., ICAS 2022
// (Table 2): a PCUAV footprint is cropped from a UAV survey cloud, displaced
// by random horizontal translation, heading/pitch/roll and scale errors, and
// registered against a PCref crop from a different survey epoch.
// Example parameter files: ../configs/scenario_a.txt, ../configs/scenario_b.txt

void printUsage(const char* programName) {
    std::cerr << "Usage: " << programName << " <parameter_file> <number_of_iterations>"
              << std::endl;
    std::cerr << "  e.g.: " << programName << " ../configs/scenario_a.txt 100" << std::endl;
}

int main(int argc, char** argv) {
    time_t timetoday;
    time(&timetoday);
    std::cout << "scenario_registration initiated on : " << asctime(localtime(&timetoday))
              << std::endl;

    if (argc < 3) {
        printUsage(argv[0]);
        return 1;
    }

    std::string fullParametersFilename = argv[1];
    int numberIter = std::stoi(argv[2]);
    std::string savedFilename = appendTimestamp("../results/scenario_results");

    Settings pipelineSettings = getPipelineDefaultSettings();

    double seedRef(0.5), seedSource(0.5), seedCustomRotation(0.5);

    for (int i = 1; i <= numberIter; i++) {
        fullRegistration(fullParametersFilename, pipelineSettings, savedFilename, &seedRef,
                         &seedSource, &seedCustomRotation);
    }

    return 0;
}
