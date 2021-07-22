#include <iostream>
#include <fstream>
#include <string>
#include <bits/stdc++.h>
#include <typeinfo>
#include <tuple>
#include <cstdlib>
#include <ctime>
#include <algorithm>

#include "Settings.hpp"


#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/common/time.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

// PCL point types
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// PCL filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>

// Descriptors
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>

// Detectors
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/brisk_2d.h>

// Searching methods
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/sample_consensus_prerejective.h>

// Visualization
#include <pcl/visualization/pcl_visualizer.h>

// typedef definitions
typedef pcl::PointWithScale PointWithScale;
typedef pcl::PointCloud<PointWithScale> PtCloudPointWithScale;
typedef pcl::PointCloud<PointWithScale>::Ptr PtCloudPointWithScalePtr;

typedef pcl::PointXYZI PointWithIntensity;
typedef pcl::PointCloud<PointWithIntensity> PtCloudPointWithIntensity;
typedef pcl::PointCloud<PointWithIntensity>::Ptr PtCloudPointWithIntensityPtr;

typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;

typedef pcl::FPFHSignature33 LocalDescriptorFPFH;
typedef pcl::PointCloud<LocalDescriptorFPFH>::Ptr LocalDescriptorFPFHPtr;

typedef std::tuple<std::string, std::string, std::string, double, double, double, double, double, double, double, double, double, double> tupleParameters;
typedef std::tuple<PointCloudPtr, PointCloudPtr, PointCloudPtr, PointCloudPtr, double, double, double> tuplePointCloudPtr; 
typedef std::tuple<double, double, double> tupleOfDouble;
typedef std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> tupleOfVectorDouble;
typedef std::vector<pcl::PointXYZ> vectorPointXYZ;

typedef std::map<std::string, std::string> StringMap;
typedef std::map<std::string, double> DoubleMap;







/*
// PARAMETERS FOR KEYSTONE DATASETS
// Parameters for sift computation (source)
const float min_scale_source = 0.1;       // the standard deviation of the smallest scale in the scale space
const int n_octaves_source = 8;           // the number of octaves (i.e. doublings of scale) to compute
const int n_scales_per_octave_source = 6; // the number of scales to compute within each octave
const float min_contrast_source = 1e-6;   // the minimum contrast required for detection

// Parameters for sift computation (reference/target)
const float min_scale_target = 0.1;       // the standard deviation of the smallest scale in the scale space
const int n_octaves_target = 8;           // the number of octaves (i.e. doublings of scale) to compute
const int n_scales_per_octave_target = 6; // the number of scales to compute within each octave
const float min_contrast_target = 1e-6;   // the minimum contrast required for detection
*/



// Sample Consensus Initial Alignment parameters for KEYSTONE datasets
/*const float min_sample_dist = 0.01f;
const float max_correspondence_dist = 10.0f;
const int nr_iters = 200;*/


// =====================================
// PARAMETER NAMES
// =====================================

const std::string SACIA_MIN_SAMPLE_DIST = "sacia.minSampleDist";
const std::string SACIA_MAX_CORRESPONDENCE_DIST = "sacia.maxCorrFeatureDist";
const std::string SACIA_NUM_ITERATIONS = "sacia.numIterations";
const std::string SACIA_NUM_SAMPLES = "sacia.numSamplesPerIter";
const std::string NORMALS_SEARCH_RADIUS = "normals.searchRadius";
const std::string FPFH_SEARCH_RADIUS = "fpfh.searchRadius";
const std::string HARRIS_SEARCH_RADIUS = "harris.searchRadius";
const std::string HARRIS_THRESHOLD = "harris.threshold";
const std::string BRISK_THRESHOLD = "brisk.threshold";
const std::string SIFT_MIN_SCALE_SOURCE = "sift.minScaleSource";
const std::string SIFT_MIN_SCALE_TARGET = "sift.minScaleTarget";
const std::string SIFT_NUM_OCTAVES_SOURCE = "sift.numOctavesSource";
const std::string SIFT_NUM_OCTAVES_TARGET = "sift.numOctavesTarget";
const std::string SIFT_NUM_SCALES_PER_OCTAVE_SOURCE = "sift.numScalesPerOctaveSource";
const std::string SIFT_NUM_SCALES_PER_OCTAVE_TARGET = "sift.numScalesPerOctaveTarget";
const std::string SIFT_MIN_CONTRAST_SOURCE = "sift.minContrastSource";
const std::string SIFT_MIN_CONTRAST_TARGET = "sift.minContrastTarget";


Settings getPipelineDefaultSettings() {

    Settings settings;

    // Sample Consensus Initial Alignment (SAC-IA) parameters for LM datasets
    settings.setValue(SACIA_MIN_SAMPLE_DIST, 3.0f); // minimum geometric euclidian distance between points during random sampling
    settings.setValue(SACIA_MAX_CORRESPONDENCE_DIST, 1.0f); // max distance between NN correspondences from intermittent pose estimation to be considered for pose candidate transform ??
    settings.setValue(SACIA_NUM_ITERATIONS, 500); // number of iterations
    settings.setValue(SACIA_NUM_SAMPLES, 50); // number of samples in each iteration
        
    settings.setValue(NORMALS_SEARCH_RADIUS, 1.5);   // search radius for computation of surface normals
    settings.setValue(FPFH_SEARCH_RADIUS, 3.0);   // search radius for FPFH descriptor. IMPORTANT: must be larger than radius used to estimate the surface normals!!!
    settings.setValue(HARRIS_SEARCH_RADIUS, 2.0); // radius searching for HARRIS 3D Keypoints on target data
    settings.setValue(HARRIS_THRESHOLD, 1e-7);
    settings.setValue(BRISK_THRESHOLD, 60.0);

    // SIFT PARAMETERS FOR LM SURFACE MODEL DATASETS
    settings.setValue(SIFT_MIN_SCALE_SOURCE, 0.5);  // the standard deviation of the smallest scale in the scale space
    settings.setValue(SIFT_MIN_SCALE_TARGET, 0.5);  // the standard deviation of the smallest scale in the scale space
    settings.setValue(SIFT_NUM_OCTAVES_SOURCE, 6);  // the number of octaves (i.e. doublings of scale) to compute
    settings.setValue(SIFT_NUM_OCTAVES_TARGET, 6);  // the number of octaves (i.e. doublings of scale) to compute
    settings.setValue(SIFT_NUM_SCALES_PER_OCTAVE_SOURCE, 8); // the number of scales to compute within each octave
    settings.setValue(SIFT_NUM_SCALES_PER_OCTAVE_TARGET, 8); // the number of scales to compute within each octave
    settings.setValue(SIFT_MIN_CONTRAST_SOURCE, 1e-3);   // the minimum contrast required for detection
    settings.setValue(SIFT_MIN_CONTRAST_TARGET, 1e-3);   // the minimum contrast required for detection

    return settings;
}