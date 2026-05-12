#pragma once

#include "visualization_tools.hpp"
#include "file_io.hpp"
#include "randomization.hpp"

#ifdef _OPENMP
#include <omp.h>
#endif

///////////////////////////////
///////// OPENMP HELPERS ///////
///////////////////////////////

inline int getOpenMPMaxThreads()
{
#ifdef _OPENMP
    return omp_get_max_threads();
#else
    return 1;
#endif
}

inline void printOpenMPInfo(const std::string& step_name)
{
#ifdef _OPENMP
    std::cout << "  [OpenMP] " << step_name << " using " << omp_get_max_threads() << " threads" << std::endl;
#else
    (void)step_name; // suppress unused parameter warning
#endif
}

///////////////////////////////
///////// PROTOTYPES ///////////
///////////////////////////////

// Reading point clouds & computing normals
pcl::PointCloud<pcl::PointNormal>::Ptr computePointNormals(
    PointCloudPtr inputCloudPtr,
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormals,
    double search_radius);

pcl::PointCloud<pcl::Normal>::Ptr computeNormals(
    PointCloudPtr inputCloudPtr,
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormals,
    double search_radius);

// Descriptors
LocalDescriptorFPFHPtr computeFPFH(
    PointCloudPtr inputCloudPtr,
    pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
    pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
    double search_radius);

LocalDescriptorFPFHPtr computeFPFH(
    PointCloudPtr inputCloudPtr,
    pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
    PtCloudPointWithScalePtr keypointsPtr,
    pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
    double search_radius);

LocalDescriptorFPFHPtr computeFPFH(
    PointCloudPtr inputCloudPtr,
    pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
    PtCloudPointWithIntensityPtr keypointsPtr,
    pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
    double search_radius);

// SHOT Descriptors
LocalDescriptorSHOTPtr computeSHOT(
    PointCloudPtr inputCloudPtr,
    pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
    pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
    double search_radius);

LocalDescriptorSHOTPtr computeSHOT(
    PointCloudPtr inputCloudPtr,
    pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
    PtCloudPointWithScalePtr keypointsPtr,
    pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
    double search_radius);

LocalDescriptorSHOTPtr computeSHOT(
    PointCloudPtr inputCloudPtr,
    pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
    PtCloudPointWithIntensityPtr keypointsPtr,
    pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
    double search_radius);

// Detectors
PtCloudPointWithScalePtr computeSiftKeypoints(
    pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
    float min_scale, int n_octaves, int n_scales_per_octave, float min_contrast);

PtCloudPointWithIntensityPtr computeHarris3DKeypoints(
    pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
    double search_radius, double threshold_harris);

PtCloudPointWithScalePtr computeBRISK2DKeypoints(
    pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
    double threshold_brisk);

PtCloudPointWithScalePtr computeISSKeypoints(
    pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
    double salient_radius, double non_max_radius,
    double threshold_21, double threshold_32, int min_neighbors);

// Searching methods
Eigen::Matrix4f computeSACInitialAlignment(
    PtCloudPointWithScalePtr& sourceKeypointsPtr,
    PtCloudPointWithScalePtr& targetKeypointsPtr,
    LocalDescriptorFPFHPtr sourceFPFH,
    LocalDescriptorFPFHPtr targetFPFH,
    float min_sample_distance, float max_correspondence_dist,
    int nr_iters, int nr_samples);

Eigen::Matrix4f computeSACInitialAlignment(
    PtCloudPointWithIntensityPtr& sourceKeypointsPtr,
    PtCloudPointWithIntensityPtr& targetKeypointsPtr,
    LocalDescriptorFPFHPtr& sourceFPFH,
    LocalDescriptorFPFHPtr& targetFPFH,
    float min_sample_distance, float max_correspondence_dist,
    int nr_iters, int nr_samples);

Eigen::Matrix4f computeSACInitialAlignment(
    PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr,
    LocalDescriptorFPFHPtr sourceFPFH, LocalDescriptorFPFHPtr targetFPFH,
    float min_sample_distance, float max_correspondence_dist,
    int nr_iters, int nr_samples);

// SAC-IA with SHOT descriptors
Eigen::Matrix4f computeSACInitialAlignment(
    PtCloudPointWithScalePtr& sourceKeypointsPtr,
    PtCloudPointWithScalePtr& targetKeypointsPtr,
    LocalDescriptorSHOTPtr sourceSHOT,
    LocalDescriptorSHOTPtr targetSHOT,
    float min_sample_distance, float max_correspondence_dist,
    int nr_iters, int nr_samples);

Eigen::Matrix4f computeSACInitialAlignment(
    PtCloudPointWithIntensityPtr& sourceKeypointsPtr,
    PtCloudPointWithIntensityPtr& targetKeypointsPtr,
    LocalDescriptorSHOTPtr& sourceSHOT,
    LocalDescriptorSHOTPtr& targetSHOT,
    float min_sample_distance, float max_correspondence_dist,
    int nr_iters, int nr_samples);

Eigen::Matrix4f computeSACInitialAlignment(
    PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr,
    LocalDescriptorSHOTPtr sourceSHOT, LocalDescriptorSHOTPtr targetSHOT,
    float min_sample_distance, float max_correspondence_dist,
    int nr_iters, int nr_samples);

// ICP fine registration
Eigen::Matrix4f computeICP(
    PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr,
    float max_correspondence_dist, int max_iterations,
    float transformation_epsilon, float euclidean_fitness_epsilon);

// Full pipelines
pipelineSiftOutputPtr siftPipeline(
    PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr, Settings settings);
pipelineHarrisOutputPtr harrisPipeline(
    PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr, Settings settings);
pipelineISSOutputPtr issPipeline(
    PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr, Settings settings);
pipelineAllPointsOutputPtr pipelineAllPoints(
    PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr, Settings settings);
pipelineAllPointsOutputPtr shotPipeline(
    PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr, Settings settings);

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Private helper: core FPFH computation given a pre-built cloud
namespace {

LocalDescriptorFPFHPtr computeFPFHCore(
    PointCloudPtr inputCloudPtr,
    pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
    PointCloudPtr keypointCloudPtr,
    pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
    double search_radius)
{
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::PointNormal, LocalDescriptorFPFH> fpfh_estimation;
    LocalDescriptorFPFHPtr fpfh_features(new pcl::PointCloud<LocalDescriptorFPFH>);

    std::cout << "\n- Step 3 : ";
    std::cout << "Computing FPFH features from keypoints... " << std::endl;
#ifdef _OPENMP
    fpfh_estimation.setNumberOfThreads(getOpenMPMaxThreads());
    printOpenMPInfo("FPFH estimation");
#endif

    fpfh_estimation.setSearchSurface(inputCloudPtr);
    fpfh_estimation.setInputNormals(inputCloudNormalsPtr);
    fpfh_estimation.setInputCloud(keypointCloudPtr);
    fpfh_estimation.setSearchMethod(inputTreeNormals);
    fpfh_estimation.setRadiusSearch(search_radius);
    fpfh_estimation.compute(*fpfh_features);

    std::cout << "output size (): " << fpfh_features->size() << std::endl;
    return fpfh_features;
}

// Private helper: core SAC-IA alignment given a configured SAC object
template <typename PointSource, typename PointTarget>
Eigen::Matrix4f computeSACAlignment(
    typename pcl::PointCloud<PointSource>::Ptr sourceKeypointsPtr,
    typename pcl::PointCloud<PointTarget>::Ptr targetKeypointsPtr,
    LocalDescriptorFPFHPtr sourceFPFH, LocalDescriptorFPFHPtr targetFPFH,
    float min_sample_distance, float max_correspondence_dist,
    int nr_iters, int nr_samples,
    const std::string& alignment_name)
{
    using SAC = pcl::SampleConsensusInitialAlignment<PointSource, PointTarget, LocalDescriptorFPFH>;
    typename SAC::Ptr sac_ia(new SAC);

    sac_ia->setInputSource(sourceKeypointsPtr);
    sac_ia->setInputTarget(targetKeypointsPtr);
    sac_ia->setSourceFeatures(sourceFPFH);
    sac_ia->setTargetFeatures(targetFPFH);
    sac_ia->setMinSampleDistance(min_sample_distance);
    sac_ia->setNumberOfSamples(nr_samples);
    sac_ia->setMaxCorrespondenceDistance(max_correspondence_dist);
    sac_ia->setMaximumIterations(nr_iters);

    typename pcl::PointCloud<PointSource> registration_output;
    pcl::StopWatch watch;
    std::cout << "\n- Step 4 : ";
    std::cout << "Start SAC-IA alignment with " << alignment_name << "...\n";
    sac_ia->align(registration_output);

    bool converged = sac_ia->hasConverged();
    std::cout << "\nThe point clouds " << (converged ? "has" : "has NOT") << " converged !" << std::endl;

    float fitness_score = static_cast<float>(sac_ia->getFitnessScore(max_correspondence_dist));
    Eigen::Matrix4f final_transformation = sac_ia->getFinalTransformation();

    std::cout << "Time of alignment : " << watch.getTimeSeconds() << "sec" << std::endl;
    std::cout << "Calculated transformation\n" << final_transformation << std::endl;
    std::cout << "Euclidean fitness score : " << fitness_score << std::endl;

    return final_transformation;
}

// Private helper: core SHOT computation given a pre-built cloud
LocalDescriptorSHOTPtr computeSHOTCore(
    PointCloudPtr inputCloudPtr,
    pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
    PointCloudPtr keypointCloudPtr,
    pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
    double search_radius)
{
    pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::PointNormal, LocalDescriptorSHOT> shot_estimation;
    LocalDescriptorSHOTPtr shot_features(new pcl::PointCloud<LocalDescriptorSHOT>);

    std::cout << "\n- Step 3 : ";
    std::cout << "Computing SHOT features from keypoints... " << std::endl;
#ifdef _OPENMP
    shot_estimation.setNumberOfThreads(getOpenMPMaxThreads());
    printOpenMPInfo("SHOT estimation");
#endif

    shot_estimation.setSearchSurface(inputCloudPtr);
    shot_estimation.setInputNormals(inputCloudNormalsPtr);
    shot_estimation.setInputCloud(keypointCloudPtr);
    shot_estimation.setSearchMethod(inputTreeNormals);
    shot_estimation.setRadiusSearch(search_radius);
    shot_estimation.compute(*shot_features);

    std::cout << "output size (): " << shot_features->size() << std::endl;
    return shot_features;
}

// Private helper: core SAC-IA alignment with SHOT features
template <typename PointSource, typename PointTarget>
Eigen::Matrix4f computeSACAlignmentSHOT(
    typename pcl::PointCloud<PointSource>::Ptr sourceKeypointsPtr,
    typename pcl::PointCloud<PointTarget>::Ptr targetKeypointsPtr,
    LocalDescriptorSHOTPtr sourceSHOT, LocalDescriptorSHOTPtr targetSHOT,
    float min_sample_distance, float max_correspondence_dist,
    int nr_iters, int nr_samples,
    const std::string& alignment_name)
{
    using SAC = pcl::SampleConsensusInitialAlignment<PointSource, PointTarget, LocalDescriptorSHOT>;
    typename SAC::Ptr sac_ia(new SAC);

    sac_ia->setInputSource(sourceKeypointsPtr);
    sac_ia->setInputTarget(targetKeypointsPtr);
    sac_ia->setSourceFeatures(sourceSHOT);
    sac_ia->setTargetFeatures(targetSHOT);
    sac_ia->setMinSampleDistance(min_sample_distance);
    sac_ia->setNumberOfSamples(nr_samples);
    sac_ia->setMaxCorrespondenceDistance(max_correspondence_dist);
    sac_ia->setMaximumIterations(nr_iters);

    typename pcl::PointCloud<PointSource> registration_output;
    pcl::StopWatch watch;
    std::cout << "\n- Step 4 : ";
    std::cout << "Start SAC-IA alignment with " << alignment_name << "...\n";
    sac_ia->align(registration_output);

    bool converged = sac_ia->hasConverged();
    std::cout << "\nThe point clouds " << (converged ? "has" : "has NOT") << " converged !" << std::endl;

    float fitness_score = static_cast<float>(sac_ia->getFitnessScore(max_correspondence_dist));
    Eigen::Matrix4f final_transformation = sac_ia->getFinalTransformation();

    std::cout << "Time of alignment : " << watch.getTimeSeconds() << "sec" << std::endl;
    std::cout << "Calculated transformation\n" << final_transformation << std::endl;
    std::cout << "Euclidean fitness score : " << fitness_score << std::endl;

    return final_transformation;
}

} // namespace

///////////////////////////////
////////// METHODS /////////////
///////////////////////////////

/* Compute the point normals of the input point cloud using KdTree search */
pcl::PointCloud<pcl::PointNormal>::Ptr computePointNormals(
    PointCloudPtr inputCloudPtr,
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormals,
    double search_radius)
{
    auto inputCloudNormalsPtr = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> normalEstimation;
    std::cout << "\n- Step 1 : Computing point cloud normals... " << std::endl;
#ifdef _OPENMP
    normalEstimation.setNumberOfThreads(getOpenMPMaxThreads());
    printOpenMPInfo("Normal estimation");
#endif
    normalEstimation.setInputCloud(inputCloudPtr);
    normalEstimation.setSearchMethod(treeNormals);
    normalEstimation.setRadiusSearch(search_radius);
    normalEstimation.compute(*inputCloudNormalsPtr);
    pcl::copyPointCloud(*inputCloudPtr, *inputCloudNormalsPtr);
    std::cout << "Done! " << std::endl;
    std::cout << "Normals point clouds size : " << inputCloudNormalsPtr->size() << std::endl;
    return inputCloudNormalsPtr;
}

/* Compute normals of the input point cloud using KdTree search */
pcl::PointCloud<pcl::Normal>::Ptr computeNormals(
    PointCloudPtr inputCloudPtr,
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormals,
    double search_radius)
{
    auto inputNormalsPtr = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation;
    std::cout << "Computing point cloud normals... " << std::endl;
#ifdef _OPENMP
    normalEstimation.setNumberOfThreads(getOpenMPMaxThreads());
    printOpenMPInfo("Normal estimation");
#endif
    normalEstimation.setInputCloud(inputCloudPtr);
    normalEstimation.setSearchMethod(treeNormals);
    normalEstimation.setRadiusSearch(search_radius);
    normalEstimation.compute(*inputNormalsPtr);
    pcl::copyPointCloud(*inputCloudPtr, *inputNormalsPtr);
    std::cout << "Done! " << std::endl;
    std::cout << "Normals point clouds size : " << inputNormalsPtr->size() << std::endl;
    return inputNormalsPtr;
}

///////////////////////////////
/////// DETECTORS CLASS ////////
///////////////////////////////

class Detector
{
public:
    // SIFT Detector
    PtCloudPointWithScalePtr computeSiftKeypoints(
        pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
        float min_scale, int n_octaves, int n_scales_per_octave, float min_contrast)
    {
        pcl::SIFTKeypoint<pcl::PointNormal, PointWithScale> sift;
        auto siftKeypointsPtr = PtCloudPointWithScalePtr(new PtCloudPointWithScale);
        pcl::search::KdTree<pcl::PointNormal>::Ptr treeSiftKeypoints(new pcl::search::KdTree<pcl::PointNormal>());
        std::cout << "\n- Step 2 : Computing sift keypoints from normals... " << std::endl;
        sift.setSearchMethod(treeSiftKeypoints);
        sift.setScales(min_scale, n_octaves, n_scales_per_octave);
        sift.setMinimumContrast(min_contrast);
        sift.setInputCloud(inputCloudNormalsPtr);
        pcl::StopWatch watch;
        sift.compute(*siftKeypointsPtr);
        std::cout << "Resulting sift points are of size : " << siftKeypointsPtr->size() << std::endl;
        std::cout << "Time of detection : " << watch.getTimeSeconds() << "sec" << std::endl;
        return siftKeypointsPtr;
    }

    // Harris 3D Detector
    PtCloudPointWithIntensityPtr computeHarris3DKeypoints(
        pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
        double search_radius, double threshold_harris)
    {
        pcl::HarrisKeypoint3D<pcl::PointNormal, pcl::PointXYZI> harris;
        PtCloudPointWithIntensityPtr intensityHarrisKeypointsPtr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::search::KdTree<pcl::PointNormal>::Ptr treeHarrisKeypoints(new pcl::search::KdTree<pcl::PointNormal>());
        harris.setNonMaxSupression(true);
        harris.setRefine(false);
        harris.setSearchMethod(treeHarrisKeypoints);
        harris.setRadius(search_radius);
        harris.setInputCloud(inputCloudNormalsPtr);
        harris.setThreshold(threshold_harris);
        std::cout << "\n- Step 2 : Computing Harris keypoints from normals... " << std::endl;
        pcl::StopWatch watch;
        harris.compute(*intensityHarrisKeypointsPtr);
        std::cout << "Resulting Harris points are of size : " << intensityHarrisKeypointsPtr->size() << std::endl;
        std::cout << "Time of detection : " << watch.getTimeSeconds() << "sec" << std::endl;
        return intensityHarrisKeypointsPtr;
    }

    // BRISK 2D Detector
    PtCloudPointWithScalePtr computeBRISK2DKeypoints(
        pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
        double threshold_brisk)
    {
        pcl::BriskKeypoint2D<pcl::PointNormal> brisk;
        auto intensityBriskKeypointsPtr = PtCloudPointWithScalePtr(new PtCloudPointWithScale);
        pcl::search::KdTree<pcl::PointNormal>::Ptr treeBriskKeypoints(new pcl::search::KdTree<pcl::PointNormal>());
        brisk.setSearchMethod(treeBriskKeypoints);
        brisk.setInputCloud(inputCloudNormalsPtr);
        brisk.setThreshold(threshold_brisk);
        std::cout << "\n- Step 2 : Computing BRISK keypoints from normals... " << std::endl;
        pcl::StopWatch watch;
        brisk.compute(*intensityBriskKeypointsPtr);
        std::cout << "Resulting BRISK points are of size : " << intensityBriskKeypointsPtr->size() << std::endl;
        std::cout << "Time of detection : " << watch.getTimeSeconds() << "sec" << std::endl;
        return intensityBriskKeypointsPtr;
    }

    // ISS 3D Detector
    PtCloudPointWithScalePtr computeISSKeypoints(
        pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
        double salient_radius, double non_max_radius,
        double threshold_21, double threshold_32, int min_neighbors)
    {
        pcl::ISSKeypoint3D<pcl::PointNormal, PointWithScale> iss;
        auto issKeypointsPtr = PtCloudPointWithScalePtr(new PtCloudPointWithScale);
        pcl::search::KdTree<pcl::PointNormal>::Ptr treeIssKeypoints(new pcl::search::KdTree<pcl::PointNormal>());
        std::cout << "\n- Step 2 : Computing ISS keypoints from normals... " << std::endl;
        iss.setSearchMethod(treeIssKeypoints);
        iss.setSalientRadius(salient_radius);
        iss.setNonMaxRadius(non_max_radius);
        iss.setThreshold21(threshold_21);
        iss.setThreshold32(threshold_32);
        iss.setMinNeighbors(min_neighbors);
        iss.setInputCloud(inputCloudNormalsPtr);
        pcl::StopWatch watch;
        iss.compute(*issKeypointsPtr);
        std::cout << "Resulting ISS points are of size : " << issKeypointsPtr->size() << std::endl;
        std::cout << "Time of detection : " << watch.getTimeSeconds() << "sec" << std::endl;
        return issKeypointsPtr;
    }
};

///////////////////////////////
/////// DESCRIPTOR CLASS ///////
///////////////////////////////

class Descriptor
{
public:
    // FPFH Descriptor on all the points (no keypoint filtering)
    LocalDescriptorFPFHPtr computeFPFH(
        PointCloudPtr inputCloudPtr,
        pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
        pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
        double search_radius)
    {
        return computeFPFHCore(inputCloudPtr, inputCloudNormalsPtr, inputCloudPtr, inputTreeNormals, search_radius);
    }

    // FPFH Descriptor for SIFT & BRISK detector
    LocalDescriptorFPFHPtr computeFPFH(
        PointCloudPtr inputCloudPtr,
        pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
        PtCloudPointWithScalePtr keypointsPtr,
        pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
        double search_radius)
    {
        auto keypointCloudPtr = PointCloudPtr(new PointCloud);
        copyPointCloud(*keypointsPtr, *keypointCloudPtr);
        return computeFPFHCore(inputCloudPtr, inputCloudNormalsPtr, keypointCloudPtr, inputTreeNormals, search_radius);
    }

    // FPFH Descriptor for Harris 3D detector
    LocalDescriptorFPFHPtr computeFPFH(
        PointCloudPtr inputCloudPtr,
        pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
        PtCloudPointWithIntensityPtr keypointsPtr,
        pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
        double search_radius)
    {
        auto keypointCloudPtr = PointCloudPtr(new PointCloud);
        copyPointCloud(*keypointsPtr, *keypointCloudPtr);
        return computeFPFHCore(inputCloudPtr, inputCloudNormalsPtr, keypointCloudPtr, inputTreeNormals, search_radius);
    }

    // SHOT Descriptor on all the points (no keypoint filtering)
    LocalDescriptorSHOTPtr computeSHOT(
        PointCloudPtr inputCloudPtr,
        pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
        pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
        double search_radius)
    {
        return computeSHOTCore(inputCloudPtr, inputCloudNormalsPtr, inputCloudPtr, inputTreeNormals, search_radius);
    }

    // SHOT Descriptor for SIFT & BRISK detector
    LocalDescriptorSHOTPtr computeSHOT(
        PointCloudPtr inputCloudPtr,
        pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
        PtCloudPointWithScalePtr keypointsPtr,
        pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
        double search_radius)
    {
        auto keypointCloudPtr = PointCloudPtr(new PointCloud);
        copyPointCloud(*keypointsPtr, *keypointCloudPtr);
        return computeSHOTCore(inputCloudPtr, inputCloudNormalsPtr, keypointCloudPtr, inputTreeNormals, search_radius);
    }

    // SHOT Descriptor for Harris 3D detector
    LocalDescriptorSHOTPtr computeSHOT(
        PointCloudPtr inputCloudPtr,
        pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
        PtCloudPointWithIntensityPtr keypointsPtr,
        pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
        double search_radius)
    {
        auto keypointCloudPtr = PointCloudPtr(new PointCloud);
        copyPointCloud(*keypointsPtr, *keypointCloudPtr);
        return computeSHOTCore(inputCloudPtr, inputCloudNormalsPtr, keypointCloudPtr, inputTreeNormals, search_radius);
    }
};

///////////////////////////////
/////// SEARCHING METHODS //////
///////////////////////////////

class SearchingMethods
{
public:
    // SAC-IA with SIFT keypoints
    Eigen::Matrix4f computeSACInitialAlignment(
        PtCloudPointWithScalePtr& sourceKeypointsPtr,
        PtCloudPointWithScalePtr& targetKeypointsPtr,
        LocalDescriptorFPFHPtr sourceFPFH, LocalDescriptorFPFHPtr targetFPFH,
        float min_sample_distance, float max_correspondence_dist,
        int nr_iters, int nr_samples)
    {
        return ::computeSACAlignment<PointWithScale, PointWithScale>(
            sourceKeypointsPtr, targetKeypointsPtr,
            sourceFPFH, targetFPFH,
            min_sample_distance, max_correspondence_dist,
            nr_iters, nr_samples, "SIFT keypoints");
    }

    // SAC-IA with Harris 3D keypoints
    Eigen::Matrix4f computeSACInitialAlignment(
        PtCloudPointWithIntensityPtr& sourceKeypointsPtr,
        PtCloudPointWithIntensityPtr& targetKeypointsPtr,
        LocalDescriptorFPFHPtr& sourceFPFH, LocalDescriptorFPFHPtr& targetFPFH,
        float min_sample_distance, float max_correspondence_dist,
        int nr_iters, int nr_samples)
    {
        return ::computeSACAlignment<PointWithIntensity, PointWithIntensity>(
            sourceKeypointsPtr, targetKeypointsPtr,
            sourceFPFH, targetFPFH,
            min_sample_distance, max_correspondence_dist,
            nr_iters, nr_samples, "Harris keypoints");
    }

    // SAC-IA with full point clouds
    Eigen::Matrix4f computeSACInitialAlignment(
        PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr,
        LocalDescriptorFPFHPtr sourceFPFH, LocalDescriptorFPFHPtr targetFPFH,
        float min_sample_distance, float max_correspondence_dist,
        int nr_iters, int nr_samples)
    {
        return ::computeSACAlignment<PointXYZ, PointXYZ>(
            sourceCloudPtr, targetCloudPtr,
            sourceFPFH, targetFPFH,
            min_sample_distance, max_correspondence_dist,
            nr_iters, nr_samples, "all points");
    }

    // SAC-IA with SIFT keypoints and SHOT
    Eigen::Matrix4f computeSACInitialAlignment(
        PtCloudPointWithScalePtr& sourceKeypointsPtr,
        PtCloudPointWithScalePtr& targetKeypointsPtr,
        LocalDescriptorSHOTPtr sourceSHOT, LocalDescriptorSHOTPtr targetSHOT,
        float min_sample_distance, float max_correspondence_dist,
        int nr_iters, int nr_samples)
    {
        return ::computeSACAlignmentSHOT<PointWithScale, PointWithScale>(
            sourceKeypointsPtr, targetKeypointsPtr,
            sourceSHOT, targetSHOT,
            min_sample_distance, max_correspondence_dist,
            nr_iters, nr_samples, "SIFT keypoints");
    }

    // SAC-IA with Harris 3D keypoints and SHOT
    Eigen::Matrix4f computeSACInitialAlignment(
        PtCloudPointWithIntensityPtr& sourceKeypointsPtr,
        PtCloudPointWithIntensityPtr& targetKeypointsPtr,
        LocalDescriptorSHOTPtr& sourceSHOT, LocalDescriptorSHOTPtr& targetSHOT,
        float min_sample_distance, float max_correspondence_dist,
        int nr_iters, int nr_samples)
    {
        return ::computeSACAlignmentSHOT<PointWithIntensity, PointWithIntensity>(
            sourceKeypointsPtr, targetKeypointsPtr,
            sourceSHOT, targetSHOT,
            min_sample_distance, max_correspondence_dist,
            nr_iters, nr_samples, "Harris keypoints");
    }

    // SAC-IA with full point clouds and SHOT
    Eigen::Matrix4f computeSACInitialAlignment(
        PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr,
        LocalDescriptorSHOTPtr sourceSHOT, LocalDescriptorSHOTPtr targetSHOT,
        float min_sample_distance, float max_correspondence_dist,
        int nr_iters, int nr_samples)
    {
        return ::computeSACAlignmentSHOT<PointXYZ, PointXYZ>(
            sourceCloudPtr, targetCloudPtr,
            sourceSHOT, targetSHOT,
            min_sample_distance, max_correspondence_dist,
            nr_iters, nr_samples, "all points");
    }

    // ICP fine registration
    Eigen::Matrix4f computeICP(
        PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr,
        float max_correspondence_dist, int max_iterations,
        float transformation_epsilon, float euclidean_fitness_epsilon)
    {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        pcl::PointCloud<pcl::PointXYZ> registration_output;

        icp.setInputSource(sourceCloudPtr);
        icp.setInputTarget(targetCloudPtr);
        icp.setMaxCorrespondenceDistance(max_correspondence_dist);
        icp.setMaximumIterations(max_iterations);
        icp.setTransformationEpsilon(transformation_epsilon);
        icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);

        pcl::StopWatch watch;
        std::cout << "\n- Step 5 : Starting ICP fine registration...\n";
        icp.align(registration_output);

        bool converged = icp.hasConverged();
        std::cout << "\nThe ICP " << (converged ? "has" : "has NOT") << " converged !" << std::endl;

        Eigen::Matrix4f final_transformation = icp.getFinalTransformation();
        float fitness_score = icp.getFitnessScore(max_correspondence_dist);

        std::cout << "Time of ICP alignment : " << watch.getTimeSeconds() << "sec" << std::endl;
        std::cout << "ICP transformation\n" << final_transformation << std::endl;
        std::cout << "ICP fitness score : " << fitness_score << std::endl;

        return final_transformation;
    }
};

///////////////////////////////
//////// PIPELINES /////////////
///////////////////////////////

pipelineSiftOutputPtr siftPipeline(
    PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr, Settings settings)
{
    double normalsSearchRadius = settings.getValue(NORMALS_SEARCH_RADIUS);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormals(new pcl::search::KdTree<pcl::PointXYZ>);
    auto sourceNormalsPtr = computePointNormals(sourceCloudPtr, treeNormals, normalsSearchRadius);
    auto targetNormalsPtr = computePointNormals(targetCloudPtr, treeNormals, normalsSearchRadius);

    Detector detector;
    double minScaleSource = settings.getValue(SIFT_MIN_SCALE_SOURCE);
    double numOctavesSource = settings.getValue(SIFT_NUM_OCTAVES_SOURCE);
    double numScalesPerOctaveSource = settings.getValue(SIFT_NUM_SCALES_PER_OCTAVE_SOURCE);
    double minContrastSource = settings.getValue(SIFT_MIN_CONTRAST_SOURCE);
    double minScaleTarget = settings.getValue(SIFT_MIN_SCALE_TARGET);
    double numOctavesTarget = settings.getValue(SIFT_NUM_OCTAVES_TARGET);
    double numScalesPerOctaveTarget = settings.getValue(SIFT_NUM_SCALES_PER_OCTAVE_TARGET);
    double minContrastTarget = settings.getValue(SIFT_MIN_CONTRAST_TARGET);

    auto sourceSiftKeypointsPtr = detector.computeSiftKeypoints(
        sourceNormalsPtr, minScaleSource, numOctavesSource, numScalesPerOctaveSource, minContrastSource);
    auto targetSiftKeypointsPtr = detector.computeSiftKeypoints(
        targetNormalsPtr, minScaleTarget, numOctavesTarget, numScalesPerOctaveTarget, minContrastTarget);

    Descriptor descriptor;
    double fpfhSearchRadius = settings.getValue(FPFH_SEARCH_RADIUS);
    auto sourceFPFHSift = descriptor.computeFPFH(sourceCloudPtr, sourceNormalsPtr, sourceSiftKeypointsPtr, treeNormals, fpfhSearchRadius);
    auto targetFPFHSift = descriptor.computeFPFH(targetCloudPtr, targetNormalsPtr, targetSiftKeypointsPtr, treeNormals, fpfhSearchRadius);

    SearchingMethods sac;
    double minSampleDist = settings.getValue(SACIA_MIN_SAMPLE_DIST);
    double maxCorrespondDist = settings.getValue(SACIA_MAX_CORRESPONDENCE_DIST);
    int numIterations = static_cast<int>(std::lround(settings.getValue(SACIA_NUM_ITERATIONS)));
    int numSamples = static_cast<int>(std::lround(settings.getValue(SACIA_NUM_SAMPLES)));

    std::cout << "numSamples = " << numSamples << std::endl;

    auto coarse_transformation = sac.computeSACInitialAlignment(
        sourceSiftKeypointsPtr, targetSiftKeypointsPtr,
        sourceFPFHSift, targetFPFHSift,
        minSampleDist, maxCorrespondDist, numIterations, numSamples);

    // Apply coarse transformation
    auto coarseTransformedCloudPtr = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceCloudPtr, *coarseTransformedCloudPtr, coarse_transformation);

    // Optional ICP fine registration
    Eigen::Matrix4f final_transformation = coarse_transformation;
    bool icp_enabled = false;
    if (settings.exists(ICP_ENABLED) && settings.getValue(ICP_ENABLED) > 0.5) {
        icp_enabled = true;
        double icpMaxCorrespondDist = settings.getValue(ICP_MAX_CORRESPONDENCE_DIST);
        int icpMaxIterations = static_cast<int>(std::lround(settings.getValue(ICP_MAX_ITERATIONS)));
        double icpTransformEpsilon = settings.getValue(ICP_TRANSFORMATION_EPSILON);
        double icpEuclideanEpsilon = settings.getValue(ICP_EUCLIDEAN_EPSILON);

        SearchingMethods searching;
        Eigen::Matrix4f icp_transformation = searching.computeICP(
            coarseTransformedCloudPtr, targetCloudPtr,
            icpMaxCorrespondDist, icpMaxIterations,
            icpTransformEpsilon, icpEuclideanEpsilon);

        // Combine transformations: final = icp * coarse
        final_transformation = icp_transformation * coarse_transformation;
    }

    auto transformedCloudPtr = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceCloudPtr, *transformedCloudPtr, final_transformation);

    return {sourceCloudPtr, targetCloudPtr, transformedCloudPtr, sourceSiftKeypointsPtr, targetSiftKeypointsPtr, final_transformation};
}

pipelineHarrisOutputPtr harrisPipeline(
    PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr, Settings settings)
{
    double normalsSearchRadius = settings.getValue(NORMALS_SEARCH_RADIUS);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormals(new pcl::search::KdTree<pcl::PointXYZ>);
    auto sourceNormalsPtr = computePointNormals(sourceCloudPtr, treeNormals, normalsSearchRadius);
    auto targetNormalsPtr = computePointNormals(targetCloudPtr, treeNormals, normalsSearchRadius);

    Detector detector;
    double harrisSearchRadius = settings.getValue(HARRIS_SEARCH_RADIUS);
    double harrisThreshold = settings.getValue(HARRIS_THRESHOLD);

    auto srcIntensityHarrisKeypointsPtr = detector.computeHarris3DKeypoints(sourceNormalsPtr, harrisSearchRadius, harrisThreshold);
    auto trgIntensityHarrisKeypointsPtr = detector.computeHarris3DKeypoints(targetNormalsPtr, harrisSearchRadius, harrisThreshold);

    Descriptor descriptor;
    double fpfhSearchRadius = settings.getValue(FPFH_SEARCH_RADIUS);
    auto sourceFPFHHarris = descriptor.computeFPFH(sourceCloudPtr, sourceNormalsPtr, srcIntensityHarrisKeypointsPtr, treeNormals, fpfhSearchRadius);
    auto targetFPFHHarris = descriptor.computeFPFH(targetCloudPtr, targetNormalsPtr, trgIntensityHarrisKeypointsPtr, treeNormals, fpfhSearchRadius);

    SearchingMethods sac;
    double minSampleDist = settings.getValue(SACIA_MIN_SAMPLE_DIST);
    double maxCorrespondDist = settings.getValue(SACIA_MAX_CORRESPONDENCE_DIST);
    int numIterations = static_cast<int>(std::lround(settings.getValue(SACIA_NUM_ITERATIONS)));
    int numSamples = static_cast<int>(std::lround(settings.getValue(SACIA_NUM_SAMPLES)));

    auto coarse_transformation = sac.computeSACInitialAlignment(
        srcIntensityHarrisKeypointsPtr, trgIntensityHarrisKeypointsPtr,
        sourceFPFHHarris, targetFPFHHarris,
        minSampleDist, maxCorrespondDist, numIterations, numSamples);

    // Apply coarse transformation
    auto coarseTransformedCloudPtr = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceCloudPtr, *coarseTransformedCloudPtr, coarse_transformation);

    // Optional ICP fine registration
    Eigen::Matrix4f final_transformation = coarse_transformation;
    bool icp_enabled = false;
    if (settings.exists(ICP_ENABLED) && settings.getValue(ICP_ENABLED) > 0.5) {
        icp_enabled = true;
        double icpMaxCorrespondDist = settings.getValue(ICP_MAX_CORRESPONDENCE_DIST);
        int icpMaxIterations = static_cast<int>(std::lround(settings.getValue(ICP_MAX_ITERATIONS)));
        double icpTransformEpsilon = settings.getValue(ICP_TRANSFORMATION_EPSILON);
        double icpEuclideanEpsilon = settings.getValue(ICP_EUCLIDEAN_EPSILON);

        SearchingMethods searching;
        Eigen::Matrix4f icp_transformation = searching.computeICP(
            coarseTransformedCloudPtr, targetCloudPtr,
            icpMaxCorrespondDist, icpMaxIterations,
            icpTransformEpsilon, icpEuclideanEpsilon);

        // Combine transformations: final = icp * coarse
        final_transformation = icp_transformation * coarse_transformation;
    }

    auto transformedCloudPtr = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceCloudPtr, *transformedCloudPtr, final_transformation);

    return {sourceCloudPtr, targetCloudPtr, transformedCloudPtr, srcIntensityHarrisKeypointsPtr, trgIntensityHarrisKeypointsPtr, final_transformation};
}

pipelineISSOutputPtr issPipeline(
    PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr, Settings settings)
{
    double normalsSearchRadius = settings.getValue(NORMALS_SEARCH_RADIUS);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormals(new pcl::search::KdTree<pcl::PointXYZ>);
    auto sourceNormalsPtr = computePointNormals(sourceCloudPtr, treeNormals, normalsSearchRadius);
    auto targetNormalsPtr = computePointNormals(targetCloudPtr, treeNormals, normalsSearchRadius);

    Detector detector;
    double issSalientRadius = settings.getValue(ISS_SALIENT_RADIUS);
    double issNonMaxRadius = settings.getValue(ISS_NON_MAX_RADIUS);
    double issThreshold21 = settings.getValue(ISS_THRESHOLD_21);
    double issThreshold32 = settings.getValue(ISS_THRESHOLD_32);
    int issMinNeighbors = static_cast<int>(std::lround(settings.getValue(ISS_MIN_NEIGHBORS)));

    auto sourceISSKeypointsPtr = detector.computeISSKeypoints(
        sourceNormalsPtr, issSalientRadius, issNonMaxRadius,
        issThreshold21, issThreshold32, issMinNeighbors);
    auto targetISSKeypointsPtr = detector.computeISSKeypoints(
        targetNormalsPtr, issSalientRadius, issNonMaxRadius,
        issThreshold21, issThreshold32, issMinNeighbors);

    Descriptor descriptor;
    double fpfhSearchRadius = settings.getValue(FPFH_SEARCH_RADIUS);
    auto sourceFPFHISS = descriptor.computeFPFH(sourceCloudPtr, sourceNormalsPtr, sourceISSKeypointsPtr, treeNormals, fpfhSearchRadius);
    auto targetFPFHISS = descriptor.computeFPFH(targetCloudPtr, targetNormalsPtr, targetISSKeypointsPtr, treeNormals, fpfhSearchRadius);

    SearchingMethods sac;
    double minSampleDist = settings.getValue(SACIA_MIN_SAMPLE_DIST);
    double maxCorrespondDist = settings.getValue(SACIA_MAX_CORRESPONDENCE_DIST);
    int numIterations = static_cast<int>(std::lround(settings.getValue(SACIA_NUM_ITERATIONS)));
    int numSamples = static_cast<int>(std::lround(settings.getValue(SACIA_NUM_SAMPLES)));

    auto coarse_transformation = sac.computeSACInitialAlignment(
        sourceISSKeypointsPtr, targetISSKeypointsPtr,
        sourceFPFHISS, targetFPFHISS,
        minSampleDist, maxCorrespondDist, numIterations, numSamples);

    // Apply coarse transformation
    auto coarseTransformedCloudPtr = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceCloudPtr, *coarseTransformedCloudPtr, coarse_transformation);

    // Optional ICP fine registration
    Eigen::Matrix4f final_transformation = coarse_transformation;
    bool icp_enabled = false;
    if (settings.exists(ICP_ENABLED) && settings.getValue(ICP_ENABLED) > 0.5) {
        icp_enabled = true;
        double icpMaxCorrespondDist = settings.getValue(ICP_MAX_CORRESPONDENCE_DIST);
        int icpMaxIterations = static_cast<int>(std::lround(settings.getValue(ICP_MAX_ITERATIONS)));
        double icpTransformEpsilon = settings.getValue(ICP_TRANSFORMATION_EPSILON);
        double icpEuclideanEpsilon = settings.getValue(ICP_EUCLIDEAN_EPSILON);

        SearchingMethods searching;
        Eigen::Matrix4f icp_transformation = searching.computeICP(
            coarseTransformedCloudPtr, targetCloudPtr,
            icpMaxCorrespondDist, icpMaxIterations,
            icpTransformEpsilon, icpEuclideanEpsilon);

        // Combine transformations: final = icp * coarse
        final_transformation = icp_transformation * coarse_transformation;
    }

    auto transformedCloudPtr = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceCloudPtr, *transformedCloudPtr, final_transformation);

    return {sourceCloudPtr, targetCloudPtr, transformedCloudPtr, sourceISSKeypointsPtr, targetISSKeypointsPtr, final_transformation};
}

pipelineAllPointsOutputPtr pipelineAllPoints(
    PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr, Settings settings)
{
    double normalsSearchRadius = settings.getValue(NORMALS_SEARCH_RADIUS);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormals(new pcl::search::KdTree<pcl::PointXYZ>);
    auto sourceNormalsPtr = computePointNormals(sourceCloudPtr, treeNormals, normalsSearchRadius);
    auto targetNormalsPtr = computePointNormals(targetCloudPtr, treeNormals, normalsSearchRadius);

    Descriptor descriptor;
    double fpfhSearchRadius = settings.getValue(FPFH_SEARCH_RADIUS);
    auto sourceFPFHAll = descriptor.computeFPFH(sourceCloudPtr, sourceNormalsPtr, treeNormals, fpfhSearchRadius);
    auto targetFPFHAll = descriptor.computeFPFH(targetCloudPtr, targetNormalsPtr, treeNormals, fpfhSearchRadius);

    SearchingMethods sac;
    double minSampleDist = settings.getValue(SACIA_MIN_SAMPLE_DIST);
    double maxCorrespondDist = settings.getValue(SACIA_MAX_CORRESPONDENCE_DIST);
    int numIterations = static_cast<int>(std::lround(settings.getValue(SACIA_NUM_ITERATIONS)));
    int numSamples = static_cast<int>(std::lround(settings.getValue(SACIA_NUM_SAMPLES)));

    auto coarse_transformation = sac.computeSACInitialAlignment(
        sourceCloudPtr, targetCloudPtr,
        sourceFPFHAll, targetFPFHAll,
        minSampleDist, maxCorrespondDist, numIterations, numSamples);

    // Apply coarse transformation
    auto coarseTransformedCloudPtr = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceCloudPtr, *coarseTransformedCloudPtr, coarse_transformation);

    // Optional ICP fine registration
    Eigen::Matrix4f final_transformation = coarse_transformation;
    bool icp_enabled = false;
    if (settings.exists(ICP_ENABLED) && settings.getValue(ICP_ENABLED) > 0.5) {
        icp_enabled = true;
        double icpMaxCorrespondDist = settings.getValue(ICP_MAX_CORRESPONDENCE_DIST);
        int icpMaxIterations = static_cast<int>(std::lround(settings.getValue(ICP_MAX_ITERATIONS)));
        double icpTransformEpsilon = settings.getValue(ICP_TRANSFORMATION_EPSILON);
        double icpEuclideanEpsilon = settings.getValue(ICP_EUCLIDEAN_EPSILON);

        SearchingMethods searching;
        Eigen::Matrix4f icp_transformation = searching.computeICP(
            coarseTransformedCloudPtr, targetCloudPtr,
            icpMaxCorrespondDist, icpMaxIterations,
            icpTransformEpsilon, icpEuclideanEpsilon);

        // Combine transformations: final = icp * coarse
        final_transformation = icp_transformation * coarse_transformation;
    }

    auto transformedCloudPtr = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceCloudPtr, *transformedCloudPtr, final_transformation);

    return {sourceCloudPtr, targetCloudPtr, transformedCloudPtr, final_transformation};
}

pipelineAllPointsOutputPtr shotPipeline(
    PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr, Settings settings)
{
    double normalsSearchRadius = settings.getValue(NORMALS_SEARCH_RADIUS);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormals(new pcl::search::KdTree<pcl::PointXYZ>);
    auto sourceNormalsPtr = computePointNormals(sourceCloudPtr, treeNormals, normalsSearchRadius);
    auto targetNormalsPtr = computePointNormals(targetCloudPtr, treeNormals, normalsSearchRadius);

    Descriptor descriptor;
    double shotSearchRadius = settings.getValue(SHOT_SEARCH_RADIUS);
    auto sourceSHOTAll = descriptor.computeSHOT(sourceCloudPtr, sourceNormalsPtr, treeNormals, shotSearchRadius);
    auto targetSHOTAll = descriptor.computeSHOT(targetCloudPtr, targetNormalsPtr, treeNormals, shotSearchRadius);

    SearchingMethods sac;
    double minSampleDist = settings.getValue(SACIA_MIN_SAMPLE_DIST);
    double maxCorrespondDist = settings.getValue(SACIA_MAX_CORRESPONDENCE_DIST);
    int numIterations = static_cast<int>(std::lround(settings.getValue(SACIA_NUM_ITERATIONS)));
    int numSamples = static_cast<int>(std::lround(settings.getValue(SACIA_NUM_SAMPLES)));

    auto coarse_transformation = sac.computeSACInitialAlignment(
        sourceCloudPtr, targetCloudPtr,
        sourceSHOTAll, targetSHOTAll,
        minSampleDist, maxCorrespondDist, numIterations, numSamples);

    // Apply coarse transformation
    auto coarseTransformedCloudPtr = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceCloudPtr, *coarseTransformedCloudPtr, coarse_transformation);

    // Optional ICP fine registration
    Eigen::Matrix4f final_transformation = coarse_transformation;
    bool icp_enabled = false;
    if (settings.exists(ICP_ENABLED) && settings.getValue(ICP_ENABLED) > 0.5) {
        icp_enabled = true;
        double icpMaxCorrespondDist = settings.getValue(ICP_MAX_CORRESPONDENCE_DIST);
        int icpMaxIterations = static_cast<int>(std::lround(settings.getValue(ICP_MAX_ITERATIONS)));
        double icpTransformEpsilon = settings.getValue(ICP_TRANSFORMATION_EPSILON);
        double icpEuclideanEpsilon = settings.getValue(ICP_EUCLIDEAN_EPSILON);

        SearchingMethods searching;
        Eigen::Matrix4f icp_transformation = searching.computeICP(
            coarseTransformedCloudPtr, targetCloudPtr,
            icpMaxCorrespondDist, icpMaxIterations,
            icpTransformEpsilon, icpEuclideanEpsilon);

        // Combine transformations: final = icp * coarse
        final_transformation = icp_transformation * coarse_transformation;
    }

    auto transformedCloudPtr = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceCloudPtr, *transformedCloudPtr, final_transformation);

    return {sourceCloudPtr, targetCloudPtr, transformedCloudPtr, final_transformation};
}