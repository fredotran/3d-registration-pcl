#include "parameters.hpp"

Settings getPipelineDefaultSettings()
{
    Settings settings;

    settings.setValue(NORMALS_SEARCH_RADIUS, 2.0);
    settings.setValue(FPFH_SEARCH_RADIUS, 2.0);
    settings.setValue(HARRIS_SEARCH_RADIUS, 2.0);
    settings.setValue(HARRIS_THRESHOLD, 1e-6);
    settings.setValue(SIFT_MIN_SCALE_SOURCE, 0.1);
    settings.setValue(SIFT_MIN_SCALE_TARGET, 0.1);
    settings.setValue(SIFT_NUM_OCTAVES_SOURCE, 6.0);
    settings.setValue(SIFT_NUM_OCTAVES_TARGET, 6.0);
    settings.setValue(SIFT_NUM_SCALES_PER_OCTAVE_SOURCE, 4.0);
    settings.setValue(SIFT_NUM_SCALES_PER_OCTAVE_TARGET, 4.0);
    settings.setValue(SIFT_MIN_CONTRAST_SOURCE, 0.001);
    settings.setValue(SIFT_MIN_CONTRAST_TARGET, 0.001);
    settings.setValue(SACIA_MIN_SAMPLE_DIST, 3.0);
    settings.setValue(SACIA_MAX_CORRESPONDENCE_DIST, 4.0);
    settings.setValue(SACIA_NUM_ITERATIONS, 400.0);
    settings.setValue(SACIA_NUM_SAMPLES, 3.0);
    settings.setValue(VISUALIZER_PARAMETER, 0.0);
    settings.setValue(BRISK_THRESHOLD, 0.0);

    return settings;
}
