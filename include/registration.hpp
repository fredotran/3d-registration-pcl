#include "visualization_tools.hpp"
#include "file_io.hpp"
#include "randomization.hpp"

////////////////////////////////
////////// PROTOTYPES //////////
////////////////////////////////

// Reading point clouds & computing normals
pcl::PointCloud<pcl::PointNormal>::Ptr computePointNormals(PointCloudPtr inputCloudPtr,
                                                           pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormals,
                                                           const double &search_radius);
pcl::PointCloud<pcl::Normal>::Ptr computeNormals(PointCloudPtr inputCloudPtr,
                                                 pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormals,
                                                 const double &search_radius);

// Descriptors
LocalDescriptorFPFHPtr computeFPFH(PointCloudPtr inputCloudPtr,
                                   pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                   pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
                                   const double &search_radius);
LocalDescriptorFPFHPtr computeFPFH(PointCloudPtr inputCloudPtr,
                                   pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                   PtCloudPointWithScalePtr inputSiftKeypointsPtr,
                                   pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
                                   const double &search_radius);
LocalDescriptorFPFHPtr computeFPFH(PointCloudPtr inputCloudPtr,
                                   pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                   PtCloudPointWithIntensityPtr inputHarrisKeypointsPtr,
                                   pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
                                   const double &search_radius);
LocalDescriptorFPFHPtr computeFPFH(PointCloudPtr inputCloudPtr,
                                   pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                   PtCloudPointWithScalePtr inputBriskKeypointsPtr,
                                   pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
                                   const double &search_radius);

// Detectors
PtCloudPointWithScalePtr computeSiftKeypoints(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                              const float &min_scale,
                                              const int &n_octaves,
                                              const int &n_scales_per_octave,
                                              const float &min_contrast);
PointCloudPtr computeHarris3DKeypoints(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                       const float &threshold_harris);
PtCloudPointWithScalePtr computeBRISK2DKeypoints(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                                 const double &threshold_brisk);

// Searching methods
Eigen::Matrix4f computeSACInitialAlignment(PtCloudPointWithScalePtr &sourceSiftKeypointsPtr,
                                           PtCloudPointWithScalePtr &targetSiftKeypointsPtr,
                                           LocalDescriptorFPFHPtr sourceFPFH,
                                           LocalDescriptorFPFHPtr targetFPFH,
                                           const float &min_sample_distance,
                                           const float &max_correspondence_dist,
                                           const int &nr_iters,
                                           const int &nr_samples);

Eigen::Matrix4f computeSACInitialAlignment(PtCloudPointWithScalePtr &sourceSiftKeypointsPtr,
                                           PtCloudPointWithScalePtr &targetSiftKeypointsPtr,
                                           LocalDescriptorFPFHPtr &sourceFPFH,
                                           LocalDescriptorFPFHPtr &targetFPFH,
                                           const float &min_sample_distance,
                                           const float &max_correspondence_dist,
                                           const int &nr_iters);
Eigen::Matrix4f computeSACInitialAlignment(PtCloudPointWithIntensityPtr &sourceHarrisKeypointsPtr,
                                           PtCloudPointWithIntensityPtr &targetHarrisKeypointsPtr,
                                           LocalDescriptorFPFHPtr &sourceFPFH,
                                           LocalDescriptorFPFHPtr &targetFPFH,
                                           const float &min_sample_distance,
                                           const float &max_correspondence_dist,
                                           const int &nr_iters,
                                           const int &nr_samples);

// Full pipelines
pipelineSiftOutputPtr siftPipeline(PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr, Settings settings);
pipelineHarrisOutputPtr harrisPipeline(PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr, Settings settings);
pipelineAllPointsOutputPtr pipelineAllPoints(PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr, Settings settings);

/*******************************************************************************************************************************************************/
/*******************************************************************************************************************************************************/
/*******************************************************************************************************************************************************/

////////////////////////////////
//////////// METHODS ///////////
////////////////////////////////

/* Compute the point normals of the input point cloud using KdTree search */
pcl::PointCloud<pcl::PointNormal>::Ptr computePointNormals(PointCloudPtr inputCloudPtr,
                                                           pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormals,
                                                           const double &search_radius)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr(new pcl::PointCloud<pcl::PointNormal>);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> normalEstimation;
    printf("\n- Step 1 : ");
    printf("Computing point cloud normals... \n");
    normalEstimation.setInputCloud(inputCloudPtr);
    normalEstimation.setSearchMethod(treeNormals);
    normalEstimation.setRadiusSearch(search_radius);
    normalEstimation.compute(*inputCloudNormalsPtr);
    pcl::copyPointCloud(*inputCloudPtr, *inputCloudNormalsPtr);

    printf("Done! \n");
    // Check point cloud normalized size to point cloud size unnormalized
    std::cout << "Normals point clouds size : " << inputCloudNormalsPtr->size() << endl;
    return inputCloudNormalsPtr;
}

/* Compute normals of the input point cloud using KdTree search */
pcl::PointCloud<pcl::Normal>::Ptr computeNormals(PointCloudPtr inputCloudPtr,
                                                 pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormals,
                                                 const double &search_radius)
{
    pcl::PointCloud<pcl::Normal>::Ptr inputNormalsPtr(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation;
    printf("Computing point cloud normals... \n");
    normalEstimation.setInputCloud(inputCloudPtr);
    normalEstimation.setSearchMethod(treeNormals);
    normalEstimation.setRadiusSearch(search_radius);
    normalEstimation.compute(*inputNormalsPtr);
    pcl::copyPointCloud(*inputCloudPtr, *inputNormalsPtr);

    printf("Done! \n");
    // Check point cloud normalized size to point cloud size unnormalized
    std::cout << "Normals point clouds size : " << inputNormalsPtr->size() << endl;
    return inputNormalsPtr;
}

/*******************************************************/
/******************* DETECTORS CLASS *******************/
/*******************************************************/

class Detector

{
    // Store our detectors here

private:
    float min_scale;         // the standard deviation of the smallest scale in the scale space
    int n_octaves;           // the number of octaves (i.e. doublings of scale) to compute
    int n_scales_per_octave; // the number of scales to compute within each octave
    float min_contrast;      // the minimum contrast required for detection
    float threshold_harris;

public:
    // SIFT Detector
    PtCloudPointWithScalePtr computeSiftKeypoints(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                                  const float &min_scale,
                                                  const int &n_octaves,
                                                  const int &n_scales_per_octave,
                                                  const float &min_contrast)
    {
        pcl::SIFTKeypoint<pcl::PointNormal, PointWithScale> sift;
        PtCloudPointWithScalePtr siftKeypointsPtr(new PtCloudPointWithScale);
        pcl::search::KdTree<pcl::PointNormal>::Ptr treeSiftKeypoints(new pcl::search::KdTree<pcl::PointNormal>());
        printf("\n- Step 2 : ");
        printf("Computing sift keypoints from normals... \n");
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
    pcl::PointCloud<pcl::PointXYZI>::Ptr computeHarris3DKeypoints(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                                                  const double &search_radius,
                                                                  const double &threshold_harris)
    {
        pcl::HarrisKeypoint3D<pcl::PointNormal, pcl::PointXYZI> harris;
        pcl::PointCloud<pcl::PointXYZI>::Ptr intensityHarrisKeypointsPtr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::search::KdTree<pcl::PointNormal>::Ptr treeHarrisKeypoints(new pcl::search::KdTree<pcl::PointNormal>());
        harris.setNonMaxSupression(true);
        harris.setRefine(false);
        harris.setSearchMethod(treeHarrisKeypoints);
        harris.setRadius(search_radius);
        harris.setInputCloud(inputCloudNormalsPtr);
        harris.setThreshold(threshold_harris);

        printf("\n- Step 2 : ");
        printf("Computing Harris keypoints from normals... \n");
        pcl::StopWatch watch;
        harris.compute(*intensityHarrisKeypointsPtr);

        std::cout << "Resulting Harris points are of size : " << intensityHarrisKeypointsPtr->size() << std::endl;
        std::cout << "Time of detection : " << watch.getTimeSeconds() << "sec" << std::endl;

        return intensityHarrisKeypointsPtr;
    }

    // BRISK 2D Detector
    PtCloudPointWithScalePtr computeBRISK2DKeypoints(pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                                     const double &threshold_brisk)
    {
        pcl::BriskKeypoint2D<pcl::PointNormal> brisk;
        PtCloudPointWithScalePtr intensityBriskKeypointsPtr(new PtCloudPointWithScale);
        pcl::search::KdTree<pcl::PointNormal>::Ptr treeBriskKeypoints(new pcl::search::KdTree<pcl::PointNormal>());
        brisk.setSearchMethod(treeBriskKeypoints);
        brisk.setInputCloud(inputCloudNormalsPtr);
        brisk.setThreshold(threshold_harris);

        printf("\n- Step 2 : ");
        printf("Computing BRISK keypoints from normals... \n");
        pcl::StopWatch watch;
        brisk.compute(*intensityBriskKeypointsPtr);
        //pcl::copyPointCloud(*harrisKeypointsPtr, *intensityHarrisKeypointsPtr);

        std::cout << "Resulting BRISK points are of size : " << intensityBriskKeypointsPtr->size() << std::endl;
        std::cout << "Time of detection : " << watch.getTimeSeconds() << "sec" << std::endl;

        return intensityBriskKeypointsPtr;
    }
};

/*******************************************************/
/****************** DESCRIPTOR CLASS *******************/
/*******************************************************/

class Descriptor
{
    // Store our descriptors here

private:
    double search_radius;

public:
    // FPFH Descriptor on all the points
    LocalDescriptorFPFHPtr computeFPFH(PointCloudPtr inputCloudPtr,
                                       pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                       pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
                                       const double &search_radius)
    {
        // Setup the feature computation
        pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::PointNormal, LocalDescriptorFPFH> fpfh_estimation;
        LocalDescriptorFPFHPtr fpfh_features(new pcl::PointCloud<LocalDescriptorFPFH>);
        PointCloudPtr newPointCloud(new PointCloud);
        PointCloud &inputCloud = *inputCloudPtr;
        copyPointCloud(inputCloud, *newPointCloud);

        printf("\n- Step 3 : ");
        printf("Computing FPFH features from keypoints... \n");
        // Provide the original point cloud (without normals)
        fpfh_estimation.setSearchSurface(inputCloudPtr);
        // Provide the point cloud with normals
        fpfh_estimation.setInputNormals(inputCloudNormalsPtr);
        fpfh_estimation.setInputCloud(newPointCloud);
        // Use the same KdTree from the normal estimation
        fpfh_estimation.setSearchMethod(inputTreeNormals);
        fpfh_estimation.setRadiusSearch(search_radius);
        // Actually compute the spin images
        fpfh_estimation.compute(*fpfh_features);

        std::cout << "output size (): " << fpfh_features->size() << std::endl;

        return fpfh_features;
    }

    // FPFH Descriptor for SIFT & BRISK detector
    LocalDescriptorFPFHPtr computeFPFH(PointCloudPtr inputCloudPtr,
                                       pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                       PtCloudPointWithScalePtr inputSiftKeypointsPtr,
                                       pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
                                       const double &search_radius)
    {
        // Setup the feature computation
        pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::PointNormal, LocalDescriptorFPFH> fpfh_estimation;
        LocalDescriptorFPFHPtr fpfh_features(new pcl::PointCloud<LocalDescriptorFPFH>);
        PointCloudPtr newPointCloud(new PointCloud);
        PtCloudPointWithScale &inputSiftKeypoints = *inputSiftKeypointsPtr;
        copyPointCloud(inputSiftKeypoints, *newPointCloud);

        printf("\n- Step 3 : ");
        printf("Computing FPFH features from keypoints... \n");
        // Provide the original point cloud (without normals)
        fpfh_estimation.setSearchSurface(inputCloudPtr);
        // Provide the point cloud with normals
        fpfh_estimation.setInputNormals(inputCloudNormalsPtr);
        fpfh_estimation.setInputCloud(newPointCloud);
        // Use the same KdTree from the normal estimation
        fpfh_estimation.setSearchMethod(inputTreeNormals);
        fpfh_estimation.setRadiusSearch(search_radius);
        // Actually compute the spin images
        fpfh_estimation.compute(*fpfh_features);

        std::cout << "output size (): " << fpfh_features->size() << std::endl;

        return fpfh_features;
    }
    // FPFH Descriptor for Harris 3D detector
    LocalDescriptorFPFHPtr computeFPFH(PointCloudPtr inputCloudPtr,
                                       pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudNormalsPtr,
                                       PtCloudPointWithIntensityPtr inputHarrisKeypointsPtr,
                                       pcl::search::KdTree<pcl::PointXYZ>::Ptr inputTreeNormals,
                                       const double &search_radius)
    {
        // Setup the feature computation
        pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::PointNormal, LocalDescriptorFPFH> fpfh_estimation;
        LocalDescriptorFPFHPtr fpfh_features(new pcl::PointCloud<LocalDescriptorFPFH>);
        PointCloudPtr HarrisPointCloud(new PointCloud);
        PtCloudPointWithIntensity &inputHarrisKeypoints = *inputHarrisKeypointsPtr;
        copyPointCloud(inputHarrisKeypoints, *HarrisPointCloud);

        printf("\n- Step 3 : ");
        printf("Computing FPFH features from keypoints... \n");
        // Provide the original point cloud (without normals)
        fpfh_estimation.setSearchSurface(inputCloudPtr);
        // Provide the point cloud with normals
        fpfh_estimation.setInputNormals(inputCloudNormalsPtr);
        fpfh_estimation.setInputCloud(HarrisPointCloud);
        // Use the same KdTree from the normal estimation
        fpfh_estimation.setSearchMethod(inputTreeNormals);
        fpfh_estimation.setRadiusSearch(search_radius);
        // Actually compute the spin images
        fpfh_estimation.compute(*fpfh_features);

        std::cout << "output size (): " << fpfh_features->size() << std::endl;

        return fpfh_features;
    }
};

/*******************************************************/
/*************** SEARCHING METHODS CLASS ***************/
/*******************************************************/

class SearchingMethods
{
    // Store our searching methods here

private:
    float max_correspondence_dist;
    int nr_iters;

public:
    // TODO: can the computeSACInitialAlignment methods be merged into one?

    // SAC-IA method with SIFT
    Eigen::Matrix4f computeSACInitialAlignment(PtCloudPointWithScalePtr &sourceSiftKeypointsPtr,
                                               PtCloudPointWithScalePtr &targetSiftKeypointsPtr,
                                               LocalDescriptorFPFHPtr sourceFPFH,
                                               LocalDescriptorFPFHPtr targetFPFH,
                                               const float &min_sample_distance,
                                               const float &max_correspondence_dist,
                                               const int &nr_iters,
                                               const int &nr_samples)
    {
        pcl::SampleConsensusInitialAlignment<PointWithScale, PointWithScale, LocalDescriptorFPFH> sac_ia;
        sac_ia.setInputSource(sourceSiftKeypointsPtr);
        sac_ia.setInputTarget(targetSiftKeypointsPtr);
        sac_ia.setSourceFeatures(sourceFPFH);
        sac_ia.setTargetFeatures(targetFPFH);
        sac_ia.setMinSampleDistance(min_sample_distance);
        sac_ia.setNumberOfSamples(nr_samples);
        sac_ia.setMaxCorrespondenceDistance(max_correspondence_dist);
        sac_ia.setMaximumIterations(nr_iters);

        PtCloudPointWithScale registration_output;
        pcl::StopWatch watch;
        printf("\n- Step 4 : ");
        std::cout << "Start SAC-IA correspondence matching...\n";
        sac_ia.align(registration_output);

        // Retrieve correspondence rejectors vector
        //sac_ia.getCorrespondenceRejectors();
        bool convergence = sac_ia.hasConverged();
        if (convergence == true)
            std::cout << "\nThe point clouds has converged !" << endl;
        if (convergence == false)
            std::cout << "\nThe point clouds has NOT converged !" << endl;

        float fitness_score = (float)sac_ia.getFitnessScore(max_correspondence_dist);
        Eigen::Matrix4f final_transformation = Eigen::Matrix4f::Identity();
        final_transformation = sac_ia.getFinalTransformation();

        std::cout << "Time of alignment : " << watch.getTimeSeconds() << "sec" << std::endl;
        std::cout << "Calculated transformation\n"
                  << final_transformation << std::endl;
        std::cout << "Euclidian fitness score : "
                  << fitness_score << std::endl;

        return final_transformation;
    }

    // SAC-IA method with HARRIS
    Eigen::Matrix4f computeSACInitialAlignment(PtCloudPointWithIntensityPtr &sourceHarrisKeypointsPtr,
                                               PtCloudPointWithIntensityPtr &targetHarrisKeypointsPtr,
                                               LocalDescriptorFPFHPtr sourceFPFH,
                                               LocalDescriptorFPFHPtr targetFPFH,
                                               const float &min_sample_distance,
                                               const float &max_correspondence_dist,
                                               const int &nr_iters,
                                               const int &nr_samples)
    {
        pcl::SampleConsensusInitialAlignment<PointWithIntensity, PointWithIntensity, LocalDescriptorFPFH> sac_ia_harris;
        sac_ia_harris.setInputSource(sourceHarrisKeypointsPtr);
        sac_ia_harris.setInputTarget(targetHarrisKeypointsPtr);
        sac_ia_harris.setSourceFeatures(sourceFPFH);
        sac_ia_harris.setTargetFeatures(targetFPFH);
        sac_ia_harris.setMinSampleDistance(min_sample_distance);
        sac_ia_harris.setNumberOfSamples(nr_samples);
        sac_ia_harris.setMaxCorrespondenceDistance(max_correspondence_dist);
        sac_ia_harris.setMaximumIterations(nr_iters);

        PtCloudPointWithIntensity registration_output;
        pcl::StopWatch watch;
        printf("\n- Step 4 : ");
        std::cout << "Start SAC-IA alignment with HARRIS keypoints...\n";
        sac_ia_harris.align(registration_output);

        // Retrieve correspondence rejectors vector
        //sac_ia_harris.getCorrespondenceRejectors();
        bool convergence = sac_ia_harris.hasConverged();
        if (convergence == true)
            std::cout << "\nThe point clouds has converged" << endl;
        if (convergence == false)
            std::cout << "\nThe point clouds has NOT converged" << endl;

        float fitness_score = (float)sac_ia_harris.getFitnessScore(max_correspondence_dist);
        Eigen::Matrix4f final_transformation = Eigen::Matrix4f::Identity();
        final_transformation = sac_ia_harris.getFinalTransformation();

        std::cout << "Time of alignment : " << watch.getTimeSeconds() << "sec" << std::endl;
        std::cout << "Calculated transformation\n"
                  << final_transformation << std::endl;
        std::cout << "Euclidian fitness score : "
                  << fitness_score << std::endl;

        return final_transformation;
    }

    // SAC-IA method with full point clouds
    Eigen::Matrix4f computeSACInitialAlignment(PointCloudPtr sourceCloudPtr,
                                               PointCloudPtr targetCloudPtr,
                                               LocalDescriptorFPFHPtr sourceFPFH,
                                               LocalDescriptorFPFHPtr targetFPFH,
                                               const float &min_sample_distance,
                                               const float &max_correspondence_dist,
                                               const int &nr_iters,
                                               const int &nr_samples)
    {
        pcl::SampleConsensusInitialAlignment<PointXYZ, PointXYZ, LocalDescriptorFPFH> sac_ia;
        sac_ia.setInputSource(sourceCloudPtr);
        sac_ia.setInputTarget(targetCloudPtr);
        sac_ia.setSourceFeatures(sourceFPFH);
        sac_ia.setTargetFeatures(targetFPFH);
        sac_ia.setMinSampleDistance(min_sample_distance);
        sac_ia.setNumberOfSamples(nr_samples);
        sac_ia.setMaxCorrespondenceDistance(max_correspondence_dist);
        sac_ia.setMaximumIterations(nr_iters);

        PointCloud registration_output;
        pcl::StopWatch watch;
        printf("\n- Step 4 : ");
        std::cout << "Start SAC-IA alignment with SIFT keypoints...\n";
        sac_ia.align(registration_output);

        // Retrieve correspondence rejectors vector
        //sac_ia.getCorrespondenceRejectors();
        bool convergence = sac_ia.hasConverged();
        if (convergence == true)
            std::cout << "\nThe point clouds has converged !" << endl;
        if (convergence == false)
            std::cout << "\nThe point clouds has NOT converged !" << endl;

        float fitness_score = (float)sac_ia.getFitnessScore(max_correspondence_dist);
        Eigen::Matrix4f final_transformation = Eigen::Matrix4f::Identity();
        final_transformation = sac_ia.getFinalTransformation();

        std::cout << "Time of alignment : " << watch.getTimeSeconds() << "sec" << std::endl;
        std::cout << "Calculated transformation\n"
                  << final_transformation << std::endl;
        std::cout << "Euclidian fitness score : "
                  << fitness_score << std::endl;

        return final_transformation;
    }
};

/*******************************************************/
/********************* PIPELINES ***********************/
/*******************************************************/

pipelineSiftOutputPtr siftPipeline(PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr, Settings settings)
{

    double normalsSearchRadius = settings.getValue(NORMALS_SEARCH_RADIUS);

    pcl::PointCloud<pcl::PointNormal>::Ptr sourceNormalsPtr;
    pcl::PointCloud<pcl::PointNormal>::Ptr targetNormalsPtr;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormals(new pcl::search::KdTree<pcl::PointXYZ>);
    sourceNormalsPtr = computePointNormals(sourceCloudPtr, treeNormals, normalsSearchRadius);
    targetNormalsPtr = computePointNormals(targetCloudPtr, treeNormals, normalsSearchRadius);

    Detector SIFT;
    PtCloudPointWithScalePtr sourceSiftKeypointsPtr;
    PtCloudPointWithScalePtr targetSiftKeypointsPtr;
    double minScaleSource = settings.getValue(SIFT_MIN_SCALE_SOURCE);
    double numOctavesSource = settings.getValue(SIFT_NUM_OCTAVES_SOURCE);
    double numScalesPerOctaveSource = settings.getValue(SIFT_NUM_SCALES_PER_OCTAVE_SOURCE);
    double minContrastSource = settings.getValue(SIFT_MIN_CONTRAST_SOURCE);
    double minScaleTarget = settings.getValue(SIFT_MIN_SCALE_TARGET);
    double numOctavesTarget = settings.getValue(SIFT_NUM_OCTAVES_TARGET);
    double numScalesPerOctaveTarget = settings.getValue(SIFT_NUM_SCALES_PER_OCTAVE_TARGET);
    double minContrastTarget = settings.getValue(SIFT_MIN_CONTRAST_TARGET);

    sourceSiftKeypointsPtr = SIFT.computeSiftKeypoints(sourceNormalsPtr,
                                                       minScaleSource,
                                                       numOctavesSource,
                                                       numScalesPerOctaveSource,
                                                       minContrastSource);
    targetSiftKeypointsPtr = SIFT.computeSiftKeypoints(targetNormalsPtr,
                                                       minScaleTarget,
                                                       numOctavesTarget,
                                                       numScalesPerOctaveTarget,
                                                       minContrastTarget);
    Descriptor FPFHSift;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceFPFHSift(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetFPFHSift(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeFPFH(new pcl::search::KdTree<pcl::PointXYZ>);

    double fpfhSearchRadius = settings.getValue(FPFH_SEARCH_RADIUS);

    sourceFPFHSift = FPFHSift.computeFPFH(sourceCloudPtr,
                                          sourceNormalsPtr,
                                          sourceSiftKeypointsPtr,
                                          treeFPFH,
                                          fpfhSearchRadius);
    targetFPFHSift = FPFHSift.computeFPFH(targetCloudPtr,
                                          targetNormalsPtr,
                                          targetSiftKeypointsPtr,
                                          treeFPFH,
                                          fpfhSearchRadius);

    SearchingMethods SampleConsensusIASift;
    Eigen::Matrix4f final_transformation_sift = Eigen::Matrix4f::Identity();

    // get SAC-IA settings
    double minSampleDist = settings.getValue(SACIA_MIN_SAMPLE_DIST);
    double maxCorrespondDist = settings.getValue(SACIA_MAX_CORRESPONDENCE_DIST);
    int numIterations = std::lround(settings.getValue(SACIA_NUM_ITERATIONS));
    int numSamples = std::lround(settings.getValue(SACIA_NUM_SAMPLES));

    std::cout << "numSamples = " << numSamples << std::endl;

    final_transformation_sift = SampleConsensusIASift.computeSACInitialAlignment(sourceSiftKeypointsPtr,
                                                                                 targetSiftKeypointsPtr,
                                                                                 sourceFPFHSift,
                                                                                 targetFPFHSift,
                                                                                 minSampleDist,
                                                                                 maxCorrespondDist,
                                                                                 numIterations,
                                                                                 numSamples);

    PointCloudPtr transformedCloudPtr(new PointCloud);
    PointCloudPtr newSrcPtCloudPtr(new PointCloud);
    copyPointCloud(*sourceCloudPtr, *newSrcPtCloudPtr);
    pcl::transformPointCloud(*newSrcPtCloudPtr, *transformedCloudPtr, final_transformation_sift);

    return {sourceCloudPtr, targetCloudPtr, transformedCloudPtr, sourceSiftKeypointsPtr, targetSiftKeypointsPtr, final_transformation_sift};
}

pipelineHarrisOutputPtr harrisPipeline(PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr, Settings settings)
{

    double normalsSearchRadius = settings.getValue(NORMALS_SEARCH_RADIUS);

    pcl::PointCloud<pcl::PointNormal>::Ptr sourceNormalsPtr;
    pcl::PointCloud<pcl::PointNormal>::Ptr targetNormalsPtr;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormals(new pcl::search::KdTree<pcl::PointXYZ>);
    sourceNormalsPtr = computePointNormals(sourceCloudPtr, treeNormals, normalsSearchRadius);
    targetNormalsPtr = computePointNormals(targetCloudPtr, treeNormals, normalsSearchRadius);

    Detector Harris;
    PtCloudPointWithIntensityPtr srcIntensityHarrisKeypointsPtr;
    PtCloudPointWithIntensityPtr trgIntensityHarrisKeypointsPtr;

    double harrisSearchRadius = settings.getValue(HARRIS_SEARCH_RADIUS);
    double harrisThreshold = settings.getValue(HARRIS_THRESHOLD);

    srcIntensityHarrisKeypointsPtr = Harris.computeHarris3DKeypoints(sourceNormalsPtr, harrisSearchRadius, harrisThreshold);
    trgIntensityHarrisKeypointsPtr = Harris.computeHarris3DKeypoints(targetNormalsPtr, harrisSearchRadius, harrisThreshold);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceFPFHHarris(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetFPFHHarris(new pcl::PointCloud<pcl::FPFHSignature33>);

    double fpfhSearchRadius = settings.getValue(FPFH_SEARCH_RADIUS);

    Descriptor FPFHHarris;
    sourceFPFHHarris = FPFHHarris.computeFPFH(sourceCloudPtr,
                                              sourceNormalsPtr,
                                              srcIntensityHarrisKeypointsPtr,
                                              treeNormals,
                                              fpfhSearchRadius);
    targetFPFHHarris = FPFHHarris.computeFPFH(targetCloudPtr,
                                              targetNormalsPtr,
                                              trgIntensityHarrisKeypointsPtr,
                                              treeNormals,
                                              fpfhSearchRadius);

    SearchingMethods SampleConsensusIAHarris;
    Eigen::Matrix4f final_transformation_harris;

    // get SAC-IA settings
    double minSampleDist = settings.getValue(SACIA_MIN_SAMPLE_DIST);
    double maxCorrespondDist = settings.getValue(SACIA_MAX_CORRESPONDENCE_DIST);
    long numIterations = std::lround(settings.getValue(SACIA_NUM_ITERATIONS));
    long numSamples = std::lround(settings.getValue(SACIA_NUM_SAMPLES));

    final_transformation_harris = SampleConsensusIAHarris.computeSACInitialAlignment(srcIntensityHarrisKeypointsPtr,
                                                                                     trgIntensityHarrisKeypointsPtr,
                                                                                     sourceFPFHHarris,
                                                                                     targetFPFHHarris,
                                                                                     minSampleDist,
                                                                                     maxCorrespondDist,
                                                                                     numIterations,
                                                                                     numSamples);

    PointCloudPtr transformedCloudPtr(new PointCloud);
    PointCloudPtr newSrcPtCloudPtr(new PointCloud);
    copyPointCloud(*sourceCloudPtr, *newSrcPtCloudPtr);
    pcl::transformPointCloud(*newSrcPtCloudPtr, *transformedCloudPtr, final_transformation_harris);

    return {sourceCloudPtr, targetCloudPtr, transformedCloudPtr, srcIntensityHarrisKeypointsPtr, trgIntensityHarrisKeypointsPtr, final_transformation_harris};
}

pipelineAllPointsOutputPtr pipelineAllPoints(PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr, Settings settings)
{

    double normalsSearchRadius = settings.getValue(NORMALS_SEARCH_RADIUS);

    pcl::PointCloud<pcl::PointNormal>::Ptr sourceNormalsPtr;
    pcl::PointCloud<pcl::PointNormal>::Ptr targetNormalsPtr;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormals(new pcl::search::KdTree<pcl::PointXYZ>);
    sourceNormalsPtr = computePointNormals(sourceCloudPtr, treeNormals, normalsSearchRadius);
    targetNormalsPtr = computePointNormals(targetCloudPtr, treeNormals, normalsSearchRadius);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceFPFHAll(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetFPFHAll(new pcl::PointCloud<pcl::FPFHSignature33>);
    Descriptor FPFHAll;
    double fpfhSearchRadius = settings.getValue(FPFH_SEARCH_RADIUS);

    sourceFPFHAll = FPFHAll.computeFPFH(sourceCloudPtr,
                                        sourceNormalsPtr,
                                        treeNormals,
                                        fpfhSearchRadius);

    targetFPFHAll = FPFHAll.computeFPFH(targetCloudPtr,
                                        targetNormalsPtr,
                                        treeNormals,
                                        fpfhSearchRadius);

    SearchingMethods SampleConsensusIA;
    Eigen::Matrix4f final_transformation;

    // get SAC-IA settings

    double minSampleDist = settings.getValue(SACIA_MIN_SAMPLE_DIST);
    double maxCorrespondDist = settings.getValue(SACIA_MAX_CORRESPONDENCE_DIST);
    int numIterations = std::lround(settings.getValue(SACIA_NUM_ITERATIONS));
    int numSamples = std::lround(settings.getValue(SACIA_NUM_SAMPLES));

    final_transformation = SampleConsensusIA.computeSACInitialAlignment(sourceCloudPtr,
                                                                        targetCloudPtr,
                                                                        sourceFPFHAll,
                                                                        targetFPFHAll,
                                                                        minSampleDist,
                                                                        maxCorrespondDist,
                                                                        numIterations,
                                                                        numSamples);

    PointCloudPtr transformedCloudPtr(new PointCloud);
    PointCloudPtr newSrcPtCloudPtr(new PointCloud);
    copyPointCloud(*sourceCloudPtr, *newSrcPtCloudPtr);
    pcl::transformPointCloud(*newSrcPtCloudPtr, *transformedCloudPtr, final_transformation);

    return {sourceCloudPtr, targetCloudPtr, transformedCloudPtr, final_transformation};
}
