#include <ctime>

#include "../include/tools.hpp"

// RUN FROM THE BUILD FOLDER
// Author: DS, 2021-07-22

int main(int argc, char** argv) {
    time_t timetoday;
    time(&timetoday);
    std::cout << "exp_sacia_param_grid_search initiated on : " << asctime(localtime(&timetoday))
              << std::endl;

    std::string configFilenames[] = {"../data/exp_sacia_param_grid_search.txt",
                                     "../data/exp_sacia_param_grid_search_yaw110deg.txt"};

    Settings pipelineSettings = getPipelineDefaultSettings();

    double saciaNumSampleVals[] = {5, 10.0, 20, 40, 80};
    double d = 0.5;
    double saciaMinSampleDistVals[] = {0., 1.1 * d, 2.1 * d, 4.1 * d, 8.1 * d};
    double saciaNumIterVals[] = {800};
    double saciaMaxCorrDistVals[] = {0.5, 1.0, 2.0, 4.0, 8.0};
    int numberIter = 1;

    double seedRef(0.1), seedSource(0.2), seedCustomRotation(0.3);
    seedRef = 0.19;
    seedSource = 0.344;
    seedCustomRotation = 0.7668;
    double inc = 0.37;

    for (const std::string& configFilename : configFilenames) {
        for (size_t i = 1; i <= numberIter; i++) {
            seedRef += inc;
            seedSource += inc;
            seedCustomRotation += inc;

            std::string outputFilename =
                appendTimestamp("../results/exp_sacia_param_grid_search_fixedYaw");

            for (double saciaNumIter : saciaNumIterVals) {
                pipelineSettings.setValue(SACIA_NUM_ITERATIONS, saciaNumIter);

                for (double saciaNumSample : saciaNumSampleVals) {
                    pipelineSettings.setValue(SACIA_NUM_SAMPLES, saciaNumSample);

                    for (double saciaMinSampleDist : saciaMinSampleDistVals) {
                        pipelineSettings.setValue(SACIA_MIN_SAMPLE_DIST, saciaMinSampleDist);

                        for (double saciaMaxCorrDist : saciaMaxCorrDistVals) {
                            pipelineSettings.setValue(SACIA_MAX_CORRESPONDENCE_DIST,
                                                      saciaMaxCorrDist);

                            fullRegistration(configFilename, pipelineSettings, outputFilename,
                                             &seedRef, &seedSource, &seedCustomRotation);
                        }
                    }
                }
            }
        }
    }

    return 0;
}