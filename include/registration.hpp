#pragma once

#include "file_io.hpp"
#include "randomization.hpp"
#include "visualization_tools.hpp"

#ifdef _OPENMP
#include <omp.h>
#endif

inline int getOpenMPMaxThreads() {
#ifdef _OPENMP
    return omp_get_max_threads();
#else
    return 1;
#endif
}

inline void printOpenMPInfo(const std::string& step_name) {
#ifdef _OPENMP
    std::cout << "  [OpenMP] " << step_name << " using " << omp_get_max_threads() << " threads"
              << std::endl;
#else
    (void)step_name;  // suppress unused parameter warning
#endif
}

///////////////////////////////
///////// PROTOTYPES ///////////
///////////////////////////////

pcl::PointCloud<pcl::PointNormal>::Ptr computePointNormals(
    PointCloudPtr inputCloudPtr, pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormals,
    double search_radius);

pcl::PointCloud<pcl::Normal>::Ptr computeNormals(
    PointCloudPtr inputCloudPtr, pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormals,
    double search_radius);

LocalDescriptorFPFHPtr computeFPFH(PointCloudPtr inputCloudPtr,
                                   pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                   pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
                                   double search_radius);

LocalDescriptorFPFHPtr computeFPFH(PointCloudPtr inputCloudPtr,
                                   pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                   PtCloudPointWithScalePtr keypointsPtr,
                                   pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
                                   double search_radius);

LocalDescriptorFPFHPtr computeFPFH(PointCloudPtr inputCloudPtr,
                                   pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                   PtCloudPointWithIntensityPtr keypointsPtr,
                                   pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
                                   double search_radius);

LocalDescriptorSHOTPtr computeSHOT(PointCloudPtr inputCloudPtr,
                                   pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                   pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
                                   double search_radius);

LocalDescriptorSHOTPtr computeSHOT(PointCloudPtr inputCloudPtr,
                                   pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                   PtCloudPointWithScalePtr keypointsPtr,
                                   pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
                                   double search_radius);

LocalDescriptorSHOTPtr computeSHOT(PointCloudPtr inputCloudPtr,
                                   pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                   PtCloudPointWithIntensityPtr keypointsPtr,
                                   pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
                                   double search_radius);

PtCloudPointWithScalePtr computeSiftKeypoints(
    pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr, float min_scale, int n_octaves,
    int n_scales_per_octave, float min_contrast);

PtCloudPointWithIntensityPtr computeHarris3DKeypoints(
    pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr, double search_radius,
    double threshold_harris);

PtCloudPointWithScalePtr computeBRISK2DKeypoints(
    pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr, double threshold_brisk);

PtCloudPointWithScalePtr computeISSKeypoints(
    pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr, double salient_radius,
    double non_max_radius, double threshold_21, double threshold_32, int min_neighbors);

Eigen::Matrix4f computeSACInitialAlignment(PtCloudPointWithScalePtr& sourceKeypointsPtr,
                                           PtCloudPointWithScalePtr& targetKeypointsPtr,
                                           LocalDescriptorFPFHPtr sourceFPFH,
                                           LocalDescriptorFPFHPtr targetFPFH,
                                           float min_sample_distance, float max_correspondence_dist,
                                           int nr_iters, int nr_samples);

Eigen::Matrix4f computeSACInitialAlignment(PtCloudPointWithIntensityPtr& sourceKeypointsPtr,
                                           PtCloudPointWithIntensityPtr& targetKeypointsPtr,
                                           LocalDescriptorFPFHPtr& sourceFPFH,
                                           LocalDescriptorFPFHPtr& targetFPFH,
                                           float min_sample_distance, float max_correspondence_dist,
                                           int nr_iters, int nr_samples);

Eigen::Matrix4f computeSACInitialAlignment(PointCloudPtr sourceCloudPtr,
                                           PointCloudPtr targetCloudPtr,
                                           LocalDescriptorFPFHPtr sourceFPFH,
                                           LocalDescriptorFPFHPtr targetFPFH,
                                           float min_sample_distance, float max_correspondence_dist,
                                           int nr_iters, int nr_samples);

Eigen::Matrix4f computeSACInitialAlignment(PtCloudPointWithScalePtr& sourceKeypointsPtr,
                                           PtCloudPointWithScalePtr& targetKeypointsPtr,
                                           LocalDescriptorSHOTPtr sourceSHOT,
                                           LocalDescriptorSHOTPtr targetSHOT,
                                           float min_sample_distance, float max_correspondence_dist,
                                           int nr_iters, int nr_samples);

Eigen::Matrix4f computeSACInitialAlignment(PtCloudPointWithIntensityPtr& sourceKeypointsPtr,
                                           PtCloudPointWithIntensityPtr& targetKeypointsPtr,
                                           LocalDescriptorSHOTPtr& sourceSHOT,
                                           LocalDescriptorSHOTPtr& targetSHOT,
                                           float min_sample_distance, float max_correspondence_dist,
                                           int nr_iters, int nr_samples);

Eigen::Matrix4f computeSACInitialAlignment(PointCloudPtr sourceCloudPtr,
                                           PointCloudPtr targetCloudPtr,
                                           LocalDescriptorSHOTPtr sourceSHOT,
                                           LocalDescriptorSHOTPtr targetSHOT,
                                           float min_sample_distance, float max_correspondence_dist,
                                           int nr_iters, int nr_samples);

Eigen::Matrix4f computeICP(PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr,
                           float max_correspondence_dist, int max_iterations,
                           float transformation_epsilon, float euclidean_fitness_epsilon);

pipelineSiftOutputPtr siftPipeline(PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr,
                                   Settings settings);
pipelineHarrisOutputPtr harrisPipeline(PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr,
                                       Settings settings);
pipelineISSOutputPtr issPipeline(PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr,
                                 Settings settings);
pipelineAllPointsOutputPtr pipelineAllPoints(PointCloudPtr sourceCloudPtr,
                                             PointCloudPtr targetCloudPtr, Settings settings);
pipelineAllPointsOutputPtr shotPipeline(PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr,
                                        Settings settings);

////////////////////////////////////////////////////////////////////////////////
// Anonymous namespace: template helpers that must stay in the header
////////////////////////////////////////////////////////////////////////////////

namespace {

LocalDescriptorFPFHPtr computeFPFHCore(PointCloudPtr inputCloudPtr,
                                       pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                       PointCloudPtr keypointCloudPtr,
                                       pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
                                       double search_radius) {
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

template <typename PointSource, typename PointTarget>
Eigen::Matrix4f computeSACAlignment(typename pcl::PointCloud<PointSource>::Ptr sourceKeypointsPtr,
                                    typename pcl::PointCloud<PointTarget>::Ptr targetKeypointsPtr,
                                    LocalDescriptorFPFHPtr sourceFPFH,
                                    LocalDescriptorFPFHPtr targetFPFH, float min_sample_distance,
                                    float max_correspondence_dist, int nr_iters, int nr_samples,
                                    const std::string& alignment_name) {
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
    std::cout << "\nThe point clouds " << (converged ? "has" : "has NOT") << " converged !"
              << std::endl;

    float fitness_score = static_cast<float>(sac_ia->getFitnessScore(max_correspondence_dist));
    Eigen::Matrix4f final_transformation = sac_ia->getFinalTransformation();

    std::cout << "Time of alignment : " << watch.getTimeSeconds() << "sec" << std::endl;
    std::cout << "Calculated transformation\n" << final_transformation << std::endl;
    std::cout << "Euclidean fitness score : " << fitness_score << std::endl;

    return final_transformation;
}

LocalDescriptorSHOTPtr computeSHOTCore(PointCloudPtr inputCloudPtr,
                                       pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                       PointCloudPtr keypointCloudPtr,
                                       pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
                                       double search_radius) {
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

template <typename PointSource, typename PointTarget>
Eigen::Matrix4f computeSACAlignmentSHOT(
    typename pcl::PointCloud<PointSource>::Ptr sourceKeypointsPtr,
    typename pcl::PointCloud<PointTarget>::Ptr targetKeypointsPtr,
    LocalDescriptorSHOTPtr sourceSHOT, LocalDescriptorSHOTPtr targetSHOT, float min_sample_distance,
    float max_correspondence_dist, int nr_iters, int nr_samples,
    const std::string& alignment_name) {
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
    std::cout << "\nThe point clouds " << (converged ? "has" : "has NOT") << " converged !"
              << std::endl;

    float fitness_score = static_cast<float>(sac_ia->getFitnessScore(max_correspondence_dist));
    Eigen::Matrix4f final_transformation = sac_ia->getFinalTransformation();

    std::cout << "Time of alignment : " << watch.getTimeSeconds() << "sec" << std::endl;
    std::cout << "Calculated transformation\n" << final_transformation << std::endl;
    std::cout << "Euclidean fitness score : " << fitness_score << std::endl;

    return final_transformation;
}

}  // namespace

///////////////////////////////
/////// DETECTORS CLASS ////////
///////////////////////////////

class Detector {
   public:
    PtCloudPointWithScalePtr computeSiftKeypoints(
        pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr, float min_scale, int n_octaves,
        int n_scales_per_octave, float min_contrast) {
        pcl::SIFTKeypoint<pcl::PointNormal, PointWithScale> sift;
        auto siftKeypointsPtr = PtCloudPointWithScalePtr(new PtCloudPointWithScale);
        pcl::search::KdTree<pcl::PointNormal>::Ptr treeSiftKeypoints(
            new pcl::search::KdTree<pcl::PointNormal>());
        std::cout << "\n- Step 2 : Computing sift keypoints from normals... " << std::endl;
        sift.setSearchMethod(treeSiftKeypoints);
        sift.setScales(min_scale, n_octaves, n_scales_per_octave);
        sift.setMinimumContrast(min_contrast);
        sift.setInputCloud(inputCloudNormalsPtr);
        pcl::StopWatch watch;
        sift.compute(*siftKeypointsPtr);
        std::cout << "Resulting sift points are of size : " << siftKeypointsPtr->size()
                  << std::endl;
        std::cout << "Time of detection : " << watch.getTimeSeconds() << "sec" << std::endl;
        return siftKeypointsPtr;
    }

    PtCloudPointWithIntensityPtr computeHarris3DKeypoints(
        pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr, double search_radius,
        double threshold_harris) {
        pcl::HarrisKeypoint3D<pcl::PointNormal, pcl::PointXYZI> harris;
        PtCloudPointWithIntensityPtr intensityHarrisKeypointsPtr(
            new pcl::PointCloud<pcl::PointXYZI>);
        pcl::search::KdTree<pcl::PointNormal>::Ptr treeHarrisKeypoints(
            new pcl::search::KdTree<pcl::PointNormal>());
        harris.setNonMaxSupression(true);
        harris.setRefine(false);
        harris.setSearchMethod(treeHarrisKeypoints);
        harris.setRadius(search_radius);
        harris.setInputCloud(inputCloudNormalsPtr);
        harris.setThreshold(threshold_harris);
        std::cout << "\n- Step 2 : Computing Harris keypoints from normals... " << std::endl;
        pcl::StopWatch watch;
        harris.compute(*intensityHarrisKeypointsPtr);
        std::cout << "Resulting Harris points are of size : " << intensityHarrisKeypointsPtr->size()
                  << std::endl;
        std::cout << "Time of detection : " << watch.getTimeSeconds() << "sec" << std::endl;
        return intensityHarrisKeypointsPtr;
    }

    PtCloudPointWithScalePtr computeBRISK2DKeypoints(
        pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr, double threshold_brisk) {
        pcl::BriskKeypoint2D<pcl::PointNormal> brisk;
        auto intensityBriskKeypointsPtr = PtCloudPointWithScalePtr(new PtCloudPointWithScale);
        pcl::search::KdTree<pcl::PointNormal>::Ptr treeBriskKeypoints(
            new pcl::search::KdTree<pcl::PointNormal>());
        brisk.setSearchMethod(treeBriskKeypoints);
        brisk.setInputCloud(inputCloudNormalsPtr);
        brisk.setThreshold(threshold_brisk);
        std::cout << "\n- Step 2 : Computing BRISK keypoints from normals... " << std::endl;
        pcl::StopWatch watch;
        brisk.compute(*intensityBriskKeypointsPtr);
        std::cout << "Resulting BRISK points are of size : " << intensityBriskKeypointsPtr->size()
                  << std::endl;
        std::cout << "Time of detection : " << watch.getTimeSeconds() << "sec" << std::endl;
        return intensityBriskKeypointsPtr;
    }

    PtCloudPointWithScalePtr computeISSKeypoints(
        pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr, double salient_radius,
        double non_max_radius, double threshold_21, double threshold_32, int min_neighbors) {
        pcl::ISSKeypoint3D<pcl::PointNormal, PointWithScale> iss;
        auto issKeypointsPtr = PtCloudPointWithScalePtr(new PtCloudPointWithScale);
        pcl::search::KdTree<pcl::PointNormal>::Ptr treeIssKeypoints(
            new pcl::search::KdTree<pcl::PointNormal>());
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

class Descriptor {
   public:
    LocalDescriptorFPFHPtr computeFPFH(PointCloudPtr inputCloudPtr,
                                       pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                       pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
                                       double search_radius) {
        return computeFPFHCore(inputCloudPtr, inputCloudNormalsPtr, inputCloudPtr, inputTreeNormals,
                               search_radius);
    }

    LocalDescriptorFPFHPtr computeFPFH(PointCloudPtr inputCloudPtr,
                                       pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                       PtCloudPointWithScalePtr keypointsPtr,
                                       pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
                                       double search_radius) {
        auto keypointCloudPtr = PointCloudPtr(new PointCloud);
        copyPointCloud(*keypointsPtr, *keypointCloudPtr);
        return computeFPFHCore(inputCloudPtr, inputCloudNormalsPtr, keypointCloudPtr,
                               inputTreeNormals, search_radius);
    }

    LocalDescriptorFPFHPtr computeFPFH(PointCloudPtr inputCloudPtr,
                                       pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                       PtCloudPointWithIntensityPtr keypointsPtr,
                                       pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
                                       double search_radius) {
        auto keypointCloudPtr = PointCloudPtr(new PointCloud);
        copyPointCloud(*keypointsPtr, *keypointCloudPtr);
        return computeFPFHCore(inputCloudPtr, inputCloudNormalsPtr, keypointCloudPtr,
                               inputTreeNormals, search_radius);
    }

    LocalDescriptorSHOTPtr computeSHOT(PointCloudPtr inputCloudPtr,
                                       pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                       pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
                                       double search_radius) {
        return computeSHOTCore(inputCloudPtr, inputCloudNormalsPtr, inputCloudPtr, inputTreeNormals,
                               search_radius);
    }

    LocalDescriptorSHOTPtr computeSHOT(PointCloudPtr inputCloudPtr,
                                       pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                       PtCloudPointWithScalePtr keypointsPtr,
                                       pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
                                       double search_radius) {
        auto keypointCloudPtr = PointCloudPtr(new PointCloud);
        copyPointCloud(*keypointsPtr, *keypointCloudPtr);
        return computeSHOTCore(inputCloudPtr, inputCloudNormalsPtr, keypointCloudPtr,
                               inputTreeNormals, search_radius);
    }

    LocalDescriptorSHOTPtr computeSHOT(PointCloudPtr inputCloudPtr,
                                       pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                       PtCloudPointWithIntensityPtr keypointsPtr,
                                       pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
                                       double search_radius) {
        auto keypointCloudPtr = PointCloudPtr(new PointCloud);
        copyPointCloud(*keypointsPtr, *keypointCloudPtr);
        return computeSHOTCore(inputCloudPtr, inputCloudNormalsPtr, keypointCloudPtr,
                               inputTreeNormals, search_radius);
    }
};

///////////////////////////////
/////// SEARCHING METHODS //////
///////////////////////////////

class SearchingMethods {
   public:
    Eigen::Matrix4f computeSACInitialAlignment(
        PtCloudPointWithScalePtr& sourceKeypointsPtr, PtCloudPointWithScalePtr& targetKeypointsPtr,
        LocalDescriptorFPFHPtr sourceFPFH, LocalDescriptorFPFHPtr targetFPFH,
        float min_sample_distance, float max_correspondence_dist, int nr_iters, int nr_samples) {
        return ::computeSACAlignment<PointWithScale, PointWithScale>(
            sourceKeypointsPtr, targetKeypointsPtr, sourceFPFH, targetFPFH, min_sample_distance,
            max_correspondence_dist, nr_iters, nr_samples, "SIFT keypoints");
    }

    Eigen::Matrix4f computeSACInitialAlignment(PtCloudPointWithIntensityPtr& sourceKeypointsPtr,
                                               PtCloudPointWithIntensityPtr& targetKeypointsPtr,
                                               LocalDescriptorFPFHPtr& sourceFPFH,
                                               LocalDescriptorFPFHPtr& targetFPFH,
                                               float min_sample_distance,
                                               float max_correspondence_dist, int nr_iters,
                                               int nr_samples) {
        return ::computeSACAlignment<PointWithIntensity, PointWithIntensity>(
            sourceKeypointsPtr, targetKeypointsPtr, sourceFPFH, targetFPFH, min_sample_distance,
            max_correspondence_dist, nr_iters, nr_samples, "Harris keypoints");
    }

    Eigen::Matrix4f computeSACInitialAlignment(
        PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr,
        LocalDescriptorFPFHPtr sourceFPFH, LocalDescriptorFPFHPtr targetFPFH,
        float min_sample_distance, float max_correspondence_dist, int nr_iters, int nr_samples) {
        return ::computeSACAlignment<PointXYZ, PointXYZ>(
            sourceCloudPtr, targetCloudPtr, sourceFPFH, targetFPFH, min_sample_distance,
            max_correspondence_dist, nr_iters, nr_samples, "all points");
    }

    Eigen::Matrix4f computeSACInitialAlignment(
        PtCloudPointWithScalePtr& sourceKeypointsPtr, PtCloudPointWithScalePtr& targetKeypointsPtr,
        LocalDescriptorSHOTPtr sourceSHOT, LocalDescriptorSHOTPtr targetSHOT,
        float min_sample_distance, float max_correspondence_dist, int nr_iters, int nr_samples) {
        return ::computeSACAlignmentSHOT<PointWithScale, PointWithScale>(
            sourceKeypointsPtr, targetKeypointsPtr, sourceSHOT, targetSHOT, min_sample_distance,
            max_correspondence_dist, nr_iters, nr_samples, "SIFT keypoints");
    }

    Eigen::Matrix4f computeSACInitialAlignment(PtCloudPointWithIntensityPtr& sourceKeypointsPtr,
                                               PtCloudPointWithIntensityPtr& targetKeypointsPtr,
                                               LocalDescriptorSHOTPtr& sourceSHOT,
                                               LocalDescriptorSHOTPtr& targetSHOT,
                                               float min_sample_distance,
                                               float max_correspondence_dist, int nr_iters,
                                               int nr_samples) {
        return ::computeSACAlignmentSHOT<PointWithIntensity, PointWithIntensity>(
            sourceKeypointsPtr, targetKeypointsPtr, sourceSHOT, targetSHOT, min_sample_distance,
            max_correspondence_dist, nr_iters, nr_samples, "Harris keypoints");
    }

    Eigen::Matrix4f computeSACInitialAlignment(
        PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr,
        LocalDescriptorSHOTPtr sourceSHOT, LocalDescriptorSHOTPtr targetSHOT,
        float min_sample_distance, float max_correspondence_dist, int nr_iters, int nr_samples) {
        return ::computeSACAlignmentSHOT<PointXYZ, PointXYZ>(
            sourceCloudPtr, targetCloudPtr, sourceSHOT, targetSHOT, min_sample_distance,
            max_correspondence_dist, nr_iters, nr_samples, "all points");
    }

    Eigen::Matrix4f computeICP(PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr,
                               float max_correspondence_dist, int max_iterations,
                               float transformation_epsilon, float euclidean_fitness_epsilon) {
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
