#include "../include/tools.hpp"

// RUN FROM THE BUILD FOLDER

void printUsage(const char* programName) {
    std::cerr << "Usage: " << programName << " <number_of_iterations> [detector]" << std::endl;
    std::cerr << "  detector options: all, sift, harris, iss (default: all)" << std::endl;
    std::cerr << "  numeric options:  1=all, 2=sift, 3=harris, 4=iss" << std::endl;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    int numberIter = std::stoi(argv[1]);
    std::string detector = (argc >= 3) ? argv[2] : "all";

    // Support numeric options: 1=all, 2=sift, 3=harris, 4=iss
    if (detector == "1")
        detector = "all";
    else if (detector == "2")
        detector = "sift";
    else if (detector == "3")
        detector = "harris";
    else if (detector == "4")
        detector = "iss";

    Settings pipelineSettings = getPipelineDefaultSettings();

    auto runDetector = [&](const std::string& name, const std::string& paramFile,
                           const std::string& resultFile) {
        std::cout << "\n" << name << " RUNS\n" << std::endl;
        for (size_t i = 1; i <= numberIter; i++) {
            double seedRef(0.5), seedSource(0.5), seedCustomRotation(0.5);
            fullRegistration(paramFile, pipelineSettings, resultFile, &seedRef, &seedSource,
                             &seedCustomRotation);
        }
    };

    bool runAll = (detector == "all");

    if (runAll || detector == "harris") {
        runDetector("HARRIS", "../data/harris_600m-40deg-FOV_rotation-Z.txt",
                    "../results/harris_results");
    }
    if (runAll || detector == "sift") {
        runDetector("SIFT", "../data/sift_600m-40deg-FOV_rotation-Z.txt",
                    "../results/sift_results");
    }
    if (runAll || detector == "iss") {
        runDetector("ISS", "../data/iss_600m-40deg-FOV_rotation-Z.txt", "../results/iss_results");
    }

    return 0;
}
