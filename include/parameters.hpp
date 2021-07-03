#include <iostream>
#include <fstream>
#include <string>
#include <bits/stdc++.h>
#include <typeinfo>
#include <tuple>
#include <cstdlib>
#include <ctime>
#include <algorithm>

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

typedef std::tuple<std::string, double, std::string, double, double, double, double, double, double, double, double, double, double> tupleParameters;

// PARAMETERS FOR LM DATASETS
// Parameters for sift computation (source)
const float min_scale_source = 0.5;       // the standard deviation of the smallest scale in the scale space
const int n_octaves_source = 6;           // the number of octaves (i.e. doublings of scale) to compute
const int n_scales_per_octave_source = 8; // the number of scales to compute within each octave
const float min_contrast_source = 1e-3;   // the minimum contrast required for detection

// Parameters for sift computation (reference/target)
const float min_scale_target = 0.5;       // the standard deviation of the smallest scale in the scale space
const int n_octaves_target = 6;           // the number of octaves (i.e. doublings of scale) to compute
const int n_scales_per_octave_target = 8; // the number of scales to compute within each octave
const float min_contrast_target = 1e-3;   // the minimum contrast required for detection

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

// Parameters for Harris computation
const float threshold_harris = 1e-7;
// Parameters for BRISK computation
const float threshold_brisk = 60.0;

// Sample Consensus Initial Alignment parameters for LM datasets (translation)
const float min_sample_dist = 0.01f;
const float max_correspondence_dist = 10.0f;
const int nr_iters = 200;
const int nr_samples = 1;

// Sample Consensus Initial Alignment parameters for LM datasets (rotation)
const float min_sample_dist_rot = 0.01f;
const float max_correspondence_dist_rot = 1.0f;
const int nr_iters_rot = 100;
const int nr_samples_rot = 25;

// Sample Consensus Initial Alignment parameters for KEYSTONE datasets
/*const float min_sample_dist = 0.01f;
const float max_correspondence_dist = 10.0f;
const int nr_iters = 200;*/

// General parameters for radius searching
double SearchRadiusSift(2.0);   // radius searching for SIFT Keypoints on target data
double SearchRadiusHarris(2.0); // radius searching for HARRIS 3D Keypoints on target data
double searchRadiusFPFH(2.0);   // searching radius for FPFH descriptors