#include "./include/tools.hpp"
#include <ctime>

// RUN FROM THE BUILD FOLDER

int main(int argc, char **argv)
{
    time_t timetoday;
    time(&timetoday);
    std::cout << "experiment2_ds initiated on : " << asctime(localtime(&timetoday)) << std::endl;

    // Input by the user
    /* if (argc < 2)
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

    std::string fullParametersFilename = "../data/params_simple_setup_ds_all.txt"; // including path
    std::string paramFilenameSift = "../data/params_simple_setup_ds_sift.txt";
    std::string paramFilenameHarris = "../data/params_simple_setup_ds_harris.txt";
    std::string savedFilename = appendTimestamp("../results/param_search_ds_results");

    // define maps that can be filled with algorithm-specific settings (e.g. for SAC-IA)
    Settings pipelineSettings = getPipelineDefaultSettings();

    // set up all required settings
    pipelineSettings.setValue(SACIA_NUM_SAMPLES, 50.0f);
    pipelineSettings.setValue(SACIA_MIN_SAMPLE_DIST, 1.5f);
    pipelineSettings.setValue(VISUALIZER_PARAMETER, 2.0f);

    double seedRef(0.1), seedSource(0.2), seedCustomRotation(0.3);
    double inc = 0.37;

    /*
    for (size_t i = 1; i <= numberIter; i++)
    {
      
        //seedRef += inc, seedSource += inc, seedCustomRotation += inc;

        // for assessing stability of pose estimation, use same seed for several iterations and observe if pose estimation is stable
        seedRef = 0.19, seedSource = 0.344, seedCustomRotation = .7668;

        fullRegistration(fullParametersFilename, pipelineSettings, savedFilename, &seedRef, &seedSource, &seedCustomRotation);
    }

    for (size_t i = 1; i <= numberIter; i++)
    {
      
        //seedRef += inc, seedSource += inc, seedCustomRotation += inc;

        // for assessing stability of pose estimation, use same seed for several iterations and observe if pose estimation is stable
        seedRef = 0.19, seedSource = 0.344, seedCustomRotation = .7668;
        

        fullRegistration(paramFilenameSift, pipelineSettings, savedFilename, &seedRef, &seedSource, &seedCustomRotation);
    }
*/
    for (size_t i = 1; i <= numberIter; i++)
    {

        //seedRef += inc, seedSource += inc, seedCustomRotation += inc;

        // for assessing stability of pose estimation, use same seed for several iterations and observe if pose estimation is stable
        seedRef = 0.19, seedSource = 0.344, seedCustomRotation = .7668;

        fullRegistration(paramFilenameHarris, pipelineSettings, savedFilename, &seedRef, &seedSource, &seedCustomRotation);
    }

    return 0;
}