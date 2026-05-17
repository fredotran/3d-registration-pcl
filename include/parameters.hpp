#pragma once

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <tuple>

#include "Settings.hpp"

// PCL point types
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// PCL filters
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

// Descriptors
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot.h>
#include <pcl/features/shot_omp.h>

// Detectors
#include <pcl/keypoints/brisk_2d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/sift_keypoint.h>

// Searching methods
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/transformation_estimation_svd.h>

// Additional detectors
#include <pcl/keypoints/iss_3d.h>

// Visualization
#include <pcl/visualization/pcl_visualizer.h>

// typedef definitions
using PointWithScale = pcl::PointWithScale;
using PtCloudPointWithScale = pcl::PointCloud<PointWithScale>;
using PtCloudPointWithScalePtr = pcl::PointCloud<PointWithScale>::Ptr;

using PointWithIntensity = pcl::PointXYZI;
using PtCloudPointWithIntensity = pcl::PointCloud<PointWithIntensity>;
using PtCloudPointWithIntensityPtr = pcl::PointCloud<PointWithIntensity>::Ptr;

using PointXYZ = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

using LocalDescriptorFPFH = pcl::FPFHSignature33;
using LocalDescriptorFPFHPtr = pcl::PointCloud<LocalDescriptorFPFH>::Ptr;

using LocalDescriptorSHOT = pcl::SHOT352;
using LocalDescriptorSHOTPtr = pcl::PointCloud<LocalDescriptorSHOT>::Ptr;

using StringMap = std::map<std::string, std::string>;
using DoubleMap = std::map<std::string, double>;

using PipelineSiftOutput =
    std::tuple<PointCloudPtr, PointCloudPtr, PointCloudPtr, PtCloudPointWithScalePtr,
               PtCloudPointWithScalePtr, Eigen::Matrix4f>;
using PipelineHarrisOutput =
    std::tuple<PointCloudPtr, PointCloudPtr, PointCloudPtr, PtCloudPointWithIntensityPtr,
               PtCloudPointWithIntensityPtr, Eigen::Matrix4f>;
using PipelineISSOutput =
    std::tuple<PointCloudPtr, PointCloudPtr, PointCloudPtr, PtCloudPointWithScalePtr,
               PtCloudPointWithScalePtr, Eigen::Matrix4f>;
using PipelineAllPointsOutput =
    std::tuple<PointCloudPtr, PointCloudPtr, PointCloudPtr, Eigen::Matrix4f>;

using TupleParameters = std::tuple<std::string, std::string, std::string, double, double, double,
                                   double, double, double, double, double, double, double>;
using TuplePointCloudPtr =
    std::tuple<PointCloudPtr, PointCloudPtr, PointCloudPtr, PointCloudPtr, double, double, double>;
using TupleOfDouble = std::tuple<double, double, double>;
using TupleOfVectorDouble =
    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>;
using VectorPointXYZ = std::vector<pcl::PointXYZ>;

// Pipeline output types (named as in original code - these are tuple types, not pointers)
using pipelineSiftOutputPtr = PipelineSiftOutput;
using pipelineHarrisOutputPtr = PipelineHarrisOutput;
using pipelineISSOutputPtr = PipelineISSOutput;
using pipelineAllPointsOutputPtr = PipelineAllPointsOutput;

// PARAMETER NAMES
constexpr const char* SACIA_MIN_SAMPLE_DIST = "sacia.minSampleDist";
constexpr const char* SACIA_MAX_CORRESPONDENCE_DIST = "sacia.maxCorrDist";
constexpr const char* SACIA_NUM_ITERATIONS = "sacia.numIterations";
constexpr const char* SACIA_NUM_SAMPLES = "sacia.numSamplesPerIter";
constexpr const char* NORMALS_SEARCH_RADIUS = "normals.searchRadius";
constexpr const char* FPFH_SEARCH_RADIUS = "fpfh.searchRadius";
constexpr const char* SHOT_SEARCH_RADIUS = "shot.searchRadius";
constexpr const char* HARRIS_SEARCH_RADIUS = "harris.searchRadius";
constexpr const char* HARRIS_THRESHOLD = "harris.threshold";
constexpr const char* BRISK_THRESHOLD = "brisk.threshold";
constexpr const char* SIFT_MIN_SCALE_SOURCE = "sift.minScaleSource";
constexpr const char* SIFT_MIN_SCALE_TARGET = "sift.minScaleTarget";
constexpr const char* SIFT_NUM_OCTAVES_SOURCE = "sift.numOctavesSource";
constexpr const char* SIFT_NUM_OCTAVES_TARGET = "sift.numOctavesTarget";
constexpr const char* SIFT_NUM_SCALES_PER_OCTAVE_SOURCE = "sift.numScalesPerOctaveSource";
constexpr const char* SIFT_NUM_SCALES_PER_OCTAVE_TARGET = "sift.numScalesPerOctaveTarget";
constexpr const char* SIFT_MIN_CONTRAST_SOURCE = "sift.minContrastSource";
constexpr const char* SIFT_MIN_CONTRAST_TARGET = "sift.minContrastTarget";
constexpr const char* VISUALIZER_PARAMETER = "visualizer.parameter";

// ICP parameters
constexpr const char* ICP_MAX_CORRESPONDENCE_DIST = "icp.maxCorrespondenceDist";
constexpr const char* ICP_MAX_ITERATIONS = "icp.maxIterations";
constexpr const char* ICP_TRANSFORMATION_EPSILON = "icp.transformationEpsilon";
constexpr const char* ICP_EUCLIDEAN_EPSILON = "icp.euclideanFitnessEpsilon";
constexpr const char* ICP_ENABLED = "icp.enabled";

// ISS detector parameters
constexpr const char* ISS_SALIENT_RADIUS = "iss.salientRadius";
constexpr const char* ISS_NON_MAX_RADIUS = "iss.nonMaxRadius";
constexpr const char* ISS_THRESHOLD_21 = "iss.threshold21";
constexpr const char* ISS_THRESHOLD_32 = "iss.threshold32";
constexpr const char* ISS_MIN_NEIGHBORS = "iss.minNeighbors";
constexpr const char* ISS_ENABLED = "iss.enabled";

// Preprocessing parameters
constexpr const char* PREPROCESSING_VOXEL_LEAF_SIZE = "preprocessing.voxelLeafSize";

// Correspondence rejection parameters
constexpr const char* REJECTION_MAX_DISTANCE = "rejection.maxDistance";

Settings getPipelineDefaultSettings();