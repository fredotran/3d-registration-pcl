#include "../include/tools.hpp"
#include <ctime>

// RUN FROM THE BUILD FOLDER

int main(int argc, char **argv)
{
    time_t timetoday;
    time(&timetoday);
    std::cout << "experiment2_ds initiated on : " << asctime(localtime(&timetoday)) << std::endl;

    if (argc < 1)
    {
        throw std::runtime_error("Required arguments: number of iterations");
    }
    int numberIter = std::stoi(argv[1]);

    std::string fullParametersFilename = "../data/params_simple_setup_ds_all.txt";
    std::string paramFilenameSift = "../data/params_simple_setup_ds_sift.txt";
    std::string paramFilenameHarris = "../data/params_simple_setup_ds_harris.txt";
    std::string savedFilename = appendTimestamp("../results/param_search_ds_results");

    Settings pipelineSettings = getPipelineDefaultSettings();

    pipelineSettings.setValue(SACIA_NUM_SAMPLES, 50.0f);
    pipelineSettings.setValue(SACIA_MIN_SAMPLE_DIST, 1.5f);
    pipelineSettings.setValue(VISUALIZER_PARAMETER, 2.0f);

    double seedRef(0.1), seedSource(0.2), seedCustomRotation(0.3);
    double inc = 0.37;

    for (size_t i = 1; i <= numberIter; i++)
    {
        seedRef += inc;
        seedSource += inc;
        seedCustomRotation += inc;

        fullRegistration(paramFilenameHarris, pipelineSettings, savedFilename,
                         &seedRef, &seedSource, &seedCustomRotation);
    }

    return 0;
}