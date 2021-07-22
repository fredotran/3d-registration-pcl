#include "./include/tools.hpp"
#include <ctime>

// RUN FROM THE BUILD FOLDER

int main(int argc, char **argv)
{
    time_t timetoday;
    time(&timetoday);
    std::cout << "experiment2 initiated on : " << asctime(localtime(&timetoday)) << std::endl;

    if (argc < 1)
    {
        throw std::runtime_error("Required arguments: number of iterations");
    }
    std::string charNumberIter = argv[1];
    int numberIter = std::stoi(charNumberIter);

    Settings pipelineSettings = getPipelineDefaultSettings();

    // define maps that can be filled with algorithm-specific settings (e.g. for SAC-IA)
    
    // Harris runs y675_58_5025_17 data file (experiment 2)
    std::cout << "\nHARRIS RUNS\n"
              << std::endl;
    double seedRef(0.0), seedSource(0.1), seedCustomRotation(0.2);
    // Harris y675_58_5025_17
    std::ofstream outfileHarris;
    std::string folderNameHarris = "../data/", parametersFilenameHarris = "harris_600m-40deg-FOV_rotation-Z.txt";
    std::string fullParametersFilenameHarris = folderNameHarris + parametersFilenameHarris; // including path
    std::string parametersFilenameReducedHarris = removeExtension(parametersFilenameHarris);
    std::string savedFilenameHarris = "../results/" + parametersFilenameReducedHarris + "_results";
    outfileHarris.open(savedFilenameHarris + ".csv", std::ios_base::app);
    outfileHarris << "Pipeline" << "," << "MRTE" << "," << "Bias error on x" << "," << "Bias error on y"
            << "," << "Bias error on z" << "," << "Custom angle on x" << "," << "Custom angle on y" << ","
            << "Custom angle on z" << "," << "[SEED] Reference cutout" << "," 
            << "[SEED] Source cutout" << "," << "[SEED] Custom rotation" << "," 
            << "MapString" << "," << "mapDouble" << std::endl;

    for (size_t i = 1; i <= numberIter; i++)
    {
        seedRef += 0.37913, seedSource += 0.3913, seedCustomRotation += 0.313;
        fullRegistration(fullParametersFilenameHarris, pipelineSettings, savedFilenameHarris, 
                         &seedRef, &seedSource, &seedCustomRotation);
    }
    
    // Sift runs y675_58_5025_17 data file
    std::cout << "\nSIFT RUNS\n"
              << std::endl;
    
    seedRef = 0;
    seedSource = 0.1111;
    seedCustomRotation = 0.22222;


    // SIFT y675_58_5025_17
    std::ofstream outfileSift;
    std::string folderNameSift = "../data/", parametersFilenameSift = "sift_600m-40deg-FOV_rotation-Z.txt";
    std::string fullParametersFilenameSift = folderNameSift + parametersFilenameSift; // including path
    std::string parametersFilenameReducedSift = removeExtension(parametersFilenameSift);
    std::string savedFilenameSift = "../results/" + parametersFilenameReducedSift + "_results";


    outfileSift.open(savedFilenameSift + ".csv", std::ios_base::app);
    outfileSift << "Pipeline" << "," << "MRTE" << "," << "Bias error on x" << "," << "Bias error on y"
            << "," << "Bias error on z" << "," << "Custom angle on x" << "," << "Custom angle on y" << ","
            << "Custom angle on z" << "," << "[SEED] Reference cutout" << "," 
            << "[SEED] Source cutout" << "," << "[SEED] Custom rotation" << "," 
            << "MapString" << "," << "mapDouble" << std::endl;

    for (size_t i = 1; i <= numberIter; i++)
    {
        seedRef += 0.37913, seedSource = 0.1111, seedCustomRotation = 0.22222;
        fullRegistration(fullParametersFilenameSift, pipelineSettings, savedFilenameSift, 
                         &seedRef, &seedSource, &seedCustomRotation);
    }

        // Sift runs y675_58_5025_17 data file
    std::cout << "\nPipeline for all points RUNS\n"
              << std::endl;

    seedRef = 0;
    seedSource = 0.22222;
    seedCustomRotation = 0.333333;

    // SIFT y675_58_5025_17
    std::ofstream outfile;
    std::string folderName = "../data/", parametersFilename = "all_600m-40deg-FOV_rotation-Z.txt";
    std::string fullParametersFilename = folderName + parametersFilename; // including path
    std::string parametersFilenameReduced = removeExtension(parametersFilename);
    std::string savedFilename = "../results/" + parametersFilenameReduced + "_results";


    outfile.open(savedFilename + ".csv", std::ios_base::app);
    outfile << "Pipeline" << "," << "MRTE" << "," << "Bias error on x" << "," << "Bias error on y"
            << "," << "Bias error on z" << "," << "Custom angle on x" << "," << "Custom angle on y" << ","
            << "Custom angle on z" << "," << "[SEED] Reference cutout" << "," 
            << "[SEED] Source cutout" << "," << "[SEED] Custom rotation" << "," 
            << "MapString" << "," << "mapDouble" << std::endl;

    for (size_t i = 1; i <= numberIter; i++)
    {
        seedRef += 0.37913, seedSource = 0.1111, seedCustomRotation = 0.22222;
        fullRegistration(fullParametersFilename, pipelineSettings, savedFilename, 
        &seedRef, &seedSource, &seedCustomRotation);
    }

    return 0;
}
