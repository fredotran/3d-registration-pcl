#include "registration.hpp"

static void applyVoxelDownsampling(PointCloudPtr& sourceCloudPtr, PointCloudPtr& targetCloudPtr,
                                   const Settings& settings) {
    if (!settings.exists(PREPROCESSING_VOXEL_LEAF_SIZE))
        return;
    float leafSize = static_cast<float>(settings.getValue(PREPROCESSING_VOXEL_LEAF_SIZE));
    if (leafSize <= 0.0f)
        return;

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(leafSize, leafSize, leafSize);

    auto srcDS = PointCloudPtr(new PointCloud);
    vg.setInputCloud(sourceCloudPtr);
    vg.filter(*srcDS);
    std::cout << "Voxel downsampling source: " << sourceCloudPtr->size() << " -> " << srcDS->size()
              << " points" << std::endl;
    sourceCloudPtr = srcDS;

    auto tgtDS = PointCloudPtr(new PointCloud);
    vg.setInputCloud(targetCloudPtr);
    vg.filter(*tgtDS);
    std::cout << "Voxel downsampling target: " << targetCloudPtr->size() << " -> " << tgtDS->size()
              << " points" << std::endl;
    targetCloudPtr = tgtDS;
}

static void applyCorrespondenceRejection(Eigen::Matrix4f& coarse_transformation,
                                         PointCloudPtr& coarseTransformedCloudPtr,
                                         PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr,
                                         const Settings& settings) {
    if (!settings.exists(REJECTION_MAX_DISTANCE))
        return;
    double rejectionDist = settings.getValue(REJECTION_MAX_DISTANCE);
    if (rejectionDist <= 0.0)
        return;

    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corrEst;
    corrEst.setInputSource(coarseTransformedCloudPtr);
    corrEst.setInputTarget(targetCloudPtr);
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    corrEst.determineCorrespondences(*correspondences);

    pcl::registration::CorrespondenceRejectorDistance rejector;
    rejector.setMaximumDistance(rejectionDist);
    rejector.setInputCorrespondences(correspondences);
    pcl::Correspondences filtered;
    rejector.getCorrespondences(filtered);

    std::cout << "Correspondence rejection: " << correspondences->size() << " -> "
              << filtered.size() << " correspondences" << std::endl;

    if (filtered.size() >= 3) {
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
        Eigen::Matrix4f refined;
        svd.estimateRigidTransformation(*coarseTransformedCloudPtr, *targetCloudPtr, filtered,
                                        refined);
        coarse_transformation = refined * coarse_transformation;
        pcl::transformPointCloud(*sourceCloudPtr, *coarseTransformedCloudPtr,
                                 coarse_transformation);
    }
}

pcl::PointCloud<pcl::PointNormal>::Ptr computePointNormals(
    PointCloudPtr inputCloudPtr, pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormals,
    double search_radius) {
    auto inputCloudNormalsPtr =
        pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
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

pcl::PointCloud<pcl::Normal>::Ptr computeNormals(
    PointCloudPtr inputCloudPtr, pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormals,
    double search_radius) {
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

pipelineSiftOutputPtr siftPipeline(PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr,
                                   Settings settings) {
    applyVoxelDownsampling(sourceCloudPtr, targetCloudPtr, settings);
    double normalsSearchRadius = settings.getValue(NORMALS_SEARCH_RADIUS);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormalsSource(
        new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormalsTarget(
        new pcl::search::KdTree<pcl::PointXYZ>);
    auto sourceNormalsPtr =
        computePointNormals(sourceCloudPtr, treeNormalsSource, normalsSearchRadius);
    auto targetNormalsPtr =
        computePointNormals(targetCloudPtr, treeNormalsTarget, normalsSearchRadius);

    Detector detector;
    double minScaleSource = settings.getValue(SIFT_MIN_SCALE_SOURCE);
    double numOctavesSource = settings.getValue(SIFT_NUM_OCTAVES_SOURCE);
    double numScalesPerOctaveSource = settings.getValue(SIFT_NUM_SCALES_PER_OCTAVE_SOURCE);
    double minContrastSource = settings.getValue(SIFT_MIN_CONTRAST_SOURCE);
    double minScaleTarget = settings.getValue(SIFT_MIN_SCALE_TARGET);
    double numOctavesTarget = settings.getValue(SIFT_NUM_OCTAVES_TARGET);
    double numScalesPerOctaveTarget = settings.getValue(SIFT_NUM_SCALES_PER_OCTAVE_TARGET);
    double minContrastTarget = settings.getValue(SIFT_MIN_CONTRAST_TARGET);

    auto sourceSiftKeypointsPtr =
        detector.computeSiftKeypoints(sourceNormalsPtr, minScaleSource, numOctavesSource,
                                      numScalesPerOctaveSource, minContrastSource);
    auto targetSiftKeypointsPtr =
        detector.computeSiftKeypoints(targetNormalsPtr, minScaleTarget, numOctavesTarget,
                                      numScalesPerOctaveTarget, minContrastTarget);

    Descriptor descriptor;
    double fpfhSearchRadius = settings.getValue(FPFH_SEARCH_RADIUS);
    auto sourceFPFHSift =
        descriptor.computeFPFH(sourceCloudPtr, sourceNormalsPtr, sourceSiftKeypointsPtr,
                               treeNormalsSource, fpfhSearchRadius);
    auto targetFPFHSift =
        descriptor.computeFPFH(targetCloudPtr, targetNormalsPtr, targetSiftKeypointsPtr,
                               treeNormalsTarget, fpfhSearchRadius);

    SearchingMethods sac;
    double minSampleDist = settings.getValue(SACIA_MIN_SAMPLE_DIST);
    double maxCorrespondDist = settings.getValue(SACIA_MAX_CORRESPONDENCE_DIST);
    int numIterations = static_cast<int>(std::lround(settings.getValue(SACIA_NUM_ITERATIONS)));
    int numSamples = static_cast<int>(std::lround(settings.getValue(SACIA_NUM_SAMPLES)));

    std::cout << "numSamples = " << numSamples << std::endl;

    auto coarse_transformation = sac.computeSACInitialAlignment(
        sourceSiftKeypointsPtr, targetSiftKeypointsPtr, sourceFPFHSift, targetFPFHSift,
        minSampleDist, maxCorrespondDist, numIterations, numSamples);

    auto coarseTransformedCloudPtr = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceCloudPtr, *coarseTransformedCloudPtr, coarse_transformation);

    applyCorrespondenceRejection(coarse_transformation, coarseTransformedCloudPtr, sourceCloudPtr,
                                 targetCloudPtr, settings);

    Eigen::Matrix4f final_transformation = coarse_transformation;
    if (settings.exists(ICP_ENABLED) && settings.getValue(ICP_ENABLED) > 0.5) {
        double icpMaxCorrespondDist = settings.getValue(ICP_MAX_CORRESPONDENCE_DIST);
        int icpMaxIterations = static_cast<int>(std::lround(settings.getValue(ICP_MAX_ITERATIONS)));
        double icpTransformEpsilon = settings.getValue(ICP_TRANSFORMATION_EPSILON);
        double icpEuclideanEpsilon = settings.getValue(ICP_EUCLIDEAN_EPSILON);

        SearchingMethods searching;
        Eigen::Matrix4f icp_transformation =
            searching.computeICP(coarseTransformedCloudPtr, targetCloudPtr, icpMaxCorrespondDist,
                                 icpMaxIterations, icpTransformEpsilon, icpEuclideanEpsilon);

        final_transformation = icp_transformation * coarse_transformation;
    }

    auto transformedCloudPtr = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceCloudPtr, *transformedCloudPtr, final_transformation);

    return {sourceCloudPtr,         targetCloudPtr,         transformedCloudPtr,
            sourceSiftKeypointsPtr, targetSiftKeypointsPtr, final_transformation};
}

pipelineHarrisOutputPtr harrisPipeline(PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr,
                                       Settings settings) {
    applyVoxelDownsampling(sourceCloudPtr, targetCloudPtr, settings);
    double normalsSearchRadius = settings.getValue(NORMALS_SEARCH_RADIUS);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormalsSource(
        new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormalsTarget(
        new pcl::search::KdTree<pcl::PointXYZ>);
    auto sourceNormalsPtr =
        computePointNormals(sourceCloudPtr, treeNormalsSource, normalsSearchRadius);
    auto targetNormalsPtr =
        computePointNormals(targetCloudPtr, treeNormalsTarget, normalsSearchRadius);

    Detector detector;
    double harrisSearchRadius = settings.getValue(HARRIS_SEARCH_RADIUS);
    double harrisThreshold = settings.getValue(HARRIS_THRESHOLD);

    auto srcIntensityHarrisKeypointsPtr =
        detector.computeHarris3DKeypoints(sourceNormalsPtr, harrisSearchRadius, harrisThreshold);
    auto trgIntensityHarrisKeypointsPtr =
        detector.computeHarris3DKeypoints(targetNormalsPtr, harrisSearchRadius, harrisThreshold);

    Descriptor descriptor;
    double fpfhSearchRadius = settings.getValue(FPFH_SEARCH_RADIUS);
    auto sourceFPFHHarris =
        descriptor.computeFPFH(sourceCloudPtr, sourceNormalsPtr, srcIntensityHarrisKeypointsPtr,
                               treeNormalsSource, fpfhSearchRadius);
    auto targetFPFHHarris =
        descriptor.computeFPFH(targetCloudPtr, targetNormalsPtr, trgIntensityHarrisKeypointsPtr,
                               treeNormalsTarget, fpfhSearchRadius);

    SearchingMethods sac;
    double minSampleDist = settings.getValue(SACIA_MIN_SAMPLE_DIST);
    double maxCorrespondDist = settings.getValue(SACIA_MAX_CORRESPONDENCE_DIST);
    int numIterations = static_cast<int>(std::lround(settings.getValue(SACIA_NUM_ITERATIONS)));
    int numSamples = static_cast<int>(std::lround(settings.getValue(SACIA_NUM_SAMPLES)));

    auto coarse_transformation = sac.computeSACInitialAlignment(
        srcIntensityHarrisKeypointsPtr, trgIntensityHarrisKeypointsPtr, sourceFPFHHarris,
        targetFPFHHarris, minSampleDist, maxCorrespondDist, numIterations, numSamples);

    auto coarseTransformedCloudPtr = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceCloudPtr, *coarseTransformedCloudPtr, coarse_transformation);

    applyCorrespondenceRejection(coarse_transformation, coarseTransformedCloudPtr, sourceCloudPtr,
                                 targetCloudPtr, settings);

    Eigen::Matrix4f final_transformation = coarse_transformation;
    if (settings.exists(ICP_ENABLED) && settings.getValue(ICP_ENABLED) > 0.5) {
        double icpMaxCorrespondDist = settings.getValue(ICP_MAX_CORRESPONDENCE_DIST);
        int icpMaxIterations = static_cast<int>(std::lround(settings.getValue(ICP_MAX_ITERATIONS)));
        double icpTransformEpsilon = settings.getValue(ICP_TRANSFORMATION_EPSILON);
        double icpEuclideanEpsilon = settings.getValue(ICP_EUCLIDEAN_EPSILON);

        SearchingMethods searching;
        Eigen::Matrix4f icp_transformation =
            searching.computeICP(coarseTransformedCloudPtr, targetCloudPtr, icpMaxCorrespondDist,
                                 icpMaxIterations, icpTransformEpsilon, icpEuclideanEpsilon);

        final_transformation = icp_transformation * coarse_transformation;
    }

    auto transformedCloudPtr = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceCloudPtr, *transformedCloudPtr, final_transformation);

    return {sourceCloudPtr,
            targetCloudPtr,
            transformedCloudPtr,
            srcIntensityHarrisKeypointsPtr,
            trgIntensityHarrisKeypointsPtr,
            final_transformation};
}

pipelineISSOutputPtr issPipeline(PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr,
                                 Settings settings) {
    applyVoxelDownsampling(sourceCloudPtr, targetCloudPtr, settings);
    double normalsSearchRadius = settings.getValue(NORMALS_SEARCH_RADIUS);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormalsSource(
        new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormalsTarget(
        new pcl::search::KdTree<pcl::PointXYZ>);
    auto sourceNormalsPtr =
        computePointNormals(sourceCloudPtr, treeNormalsSource, normalsSearchRadius);
    auto targetNormalsPtr =
        computePointNormals(targetCloudPtr, treeNormalsTarget, normalsSearchRadius);

    Detector detector;
    double issSalientRadius = settings.getValue(ISS_SALIENT_RADIUS);
    double issNonMaxRadius = settings.getValue(ISS_NON_MAX_RADIUS);
    double issThreshold21 = settings.getValue(ISS_THRESHOLD_21);
    double issThreshold32 = settings.getValue(ISS_THRESHOLD_32);
    int issMinNeighbors = static_cast<int>(std::lround(settings.getValue(ISS_MIN_NEIGHBORS)));

    auto sourceISSKeypointsPtr =
        detector.computeISSKeypoints(sourceNormalsPtr, issSalientRadius, issNonMaxRadius,
                                     issThreshold21, issThreshold32, issMinNeighbors);
    auto targetISSKeypointsPtr =
        detector.computeISSKeypoints(targetNormalsPtr, issSalientRadius, issNonMaxRadius,
                                     issThreshold21, issThreshold32, issMinNeighbors);

    Descriptor descriptor;
    double fpfhSearchRadius = settings.getValue(FPFH_SEARCH_RADIUS);
    auto sourceFPFHISS =
        descriptor.computeFPFH(sourceCloudPtr, sourceNormalsPtr, sourceISSKeypointsPtr,
                               treeNormalsSource, fpfhSearchRadius);
    auto targetFPFHISS =
        descriptor.computeFPFH(targetCloudPtr, targetNormalsPtr, targetISSKeypointsPtr,
                               treeNormalsTarget, fpfhSearchRadius);

    SearchingMethods sac;
    double minSampleDist = settings.getValue(SACIA_MIN_SAMPLE_DIST);
    double maxCorrespondDist = settings.getValue(SACIA_MAX_CORRESPONDENCE_DIST);
    int numIterations = static_cast<int>(std::lround(settings.getValue(SACIA_NUM_ITERATIONS)));
    int numSamples = static_cast<int>(std::lround(settings.getValue(SACIA_NUM_SAMPLES)));

    auto coarse_transformation = sac.computeSACInitialAlignment(
        sourceISSKeypointsPtr, targetISSKeypointsPtr, sourceFPFHISS, targetFPFHISS, minSampleDist,
        maxCorrespondDist, numIterations, numSamples);

    auto coarseTransformedCloudPtr = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceCloudPtr, *coarseTransformedCloudPtr, coarse_transformation);

    applyCorrespondenceRejection(coarse_transformation, coarseTransformedCloudPtr, sourceCloudPtr,
                                 targetCloudPtr, settings);

    Eigen::Matrix4f final_transformation = coarse_transformation;
    if (settings.exists(ICP_ENABLED) && settings.getValue(ICP_ENABLED) > 0.5) {
        double icpMaxCorrespondDist = settings.getValue(ICP_MAX_CORRESPONDENCE_DIST);
        int icpMaxIterations = static_cast<int>(std::lround(settings.getValue(ICP_MAX_ITERATIONS)));
        double icpTransformEpsilon = settings.getValue(ICP_TRANSFORMATION_EPSILON);
        double icpEuclideanEpsilon = settings.getValue(ICP_EUCLIDEAN_EPSILON);

        SearchingMethods searching;
        Eigen::Matrix4f icp_transformation =
            searching.computeICP(coarseTransformedCloudPtr, targetCloudPtr, icpMaxCorrespondDist,
                                 icpMaxIterations, icpTransformEpsilon, icpEuclideanEpsilon);

        final_transformation = icp_transformation * coarse_transformation;
    }

    auto transformedCloudPtr = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceCloudPtr, *transformedCloudPtr, final_transformation);

    return {sourceCloudPtr,        targetCloudPtr,        transformedCloudPtr,
            sourceISSKeypointsPtr, targetISSKeypointsPtr, final_transformation};
}

pipelineAllPointsOutputPtr pipelineAllPoints(PointCloudPtr sourceCloudPtr,
                                             PointCloudPtr targetCloudPtr, Settings settings) {
    applyVoxelDownsampling(sourceCloudPtr, targetCloudPtr, settings);
    double normalsSearchRadius = settings.getValue(NORMALS_SEARCH_RADIUS);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormalsSource(
        new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormalsTarget(
        new pcl::search::KdTree<pcl::PointXYZ>);
    auto sourceNormalsPtr =
        computePointNormals(sourceCloudPtr, treeNormalsSource, normalsSearchRadius);
    auto targetNormalsPtr =
        computePointNormals(targetCloudPtr, treeNormalsTarget, normalsSearchRadius);

    Descriptor descriptor;
    double fpfhSearchRadius = settings.getValue(FPFH_SEARCH_RADIUS);
    auto sourceFPFHAll = descriptor.computeFPFH(sourceCloudPtr, sourceNormalsPtr, treeNormalsSource,
                                                fpfhSearchRadius);
    auto targetFPFHAll = descriptor.computeFPFH(targetCloudPtr, targetNormalsPtr, treeNormalsTarget,
                                                fpfhSearchRadius);

    SearchingMethods sac;
    double minSampleDist = settings.getValue(SACIA_MIN_SAMPLE_DIST);
    double maxCorrespondDist = settings.getValue(SACIA_MAX_CORRESPONDENCE_DIST);
    int numIterations = static_cast<int>(std::lround(settings.getValue(SACIA_NUM_ITERATIONS)));
    int numSamples = static_cast<int>(std::lround(settings.getValue(SACIA_NUM_SAMPLES)));

    auto coarse_transformation =
        sac.computeSACInitialAlignment(sourceCloudPtr, targetCloudPtr, sourceFPFHAll, targetFPFHAll,
                                       minSampleDist, maxCorrespondDist, numIterations, numSamples);

    auto coarseTransformedCloudPtr = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceCloudPtr, *coarseTransformedCloudPtr, coarse_transformation);

    applyCorrespondenceRejection(coarse_transformation, coarseTransformedCloudPtr, sourceCloudPtr,
                                 targetCloudPtr, settings);

    Eigen::Matrix4f final_transformation = coarse_transformation;
    if (settings.exists(ICP_ENABLED) && settings.getValue(ICP_ENABLED) > 0.5) {
        double icpMaxCorrespondDist = settings.getValue(ICP_MAX_CORRESPONDENCE_DIST);
        int icpMaxIterations = static_cast<int>(std::lround(settings.getValue(ICP_MAX_ITERATIONS)));
        double icpTransformEpsilon = settings.getValue(ICP_TRANSFORMATION_EPSILON);
        double icpEuclideanEpsilon = settings.getValue(ICP_EUCLIDEAN_EPSILON);

        SearchingMethods searching;
        Eigen::Matrix4f icp_transformation =
            searching.computeICP(coarseTransformedCloudPtr, targetCloudPtr, icpMaxCorrespondDist,
                                 icpMaxIterations, icpTransformEpsilon, icpEuclideanEpsilon);

        final_transformation = icp_transformation * coarse_transformation;
    }

    auto transformedCloudPtr = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceCloudPtr, *transformedCloudPtr, final_transformation);

    return {sourceCloudPtr, targetCloudPtr, transformedCloudPtr, final_transformation};
}

pipelineAllPointsOutputPtr shotPipeline(PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr,
                                        Settings settings) {
    applyVoxelDownsampling(sourceCloudPtr, targetCloudPtr, settings);
    double normalsSearchRadius = settings.getValue(NORMALS_SEARCH_RADIUS);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormalsSource(
        new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormalsTarget(
        new pcl::search::KdTree<pcl::PointXYZ>);
    auto sourceNormalsPtr =
        computePointNormals(sourceCloudPtr, treeNormalsSource, normalsSearchRadius);
    auto targetNormalsPtr =
        computePointNormals(targetCloudPtr, treeNormalsTarget, normalsSearchRadius);

    Descriptor descriptor;
    double shotSearchRadius = settings.getValue(SHOT_SEARCH_RADIUS);
    auto sourceSHOTAll = descriptor.computeSHOT(sourceCloudPtr, sourceNormalsPtr, treeNormalsSource,
                                                shotSearchRadius);
    auto targetSHOTAll = descriptor.computeSHOT(targetCloudPtr, targetNormalsPtr, treeNormalsTarget,
                                                shotSearchRadius);

    SearchingMethods sac;
    double minSampleDist = settings.getValue(SACIA_MIN_SAMPLE_DIST);
    double maxCorrespondDist = settings.getValue(SACIA_MAX_CORRESPONDENCE_DIST);
    int numIterations = static_cast<int>(std::lround(settings.getValue(SACIA_NUM_ITERATIONS)));
    int numSamples = static_cast<int>(std::lround(settings.getValue(SACIA_NUM_SAMPLES)));

    auto coarse_transformation =
        sac.computeSACInitialAlignment(sourceCloudPtr, targetCloudPtr, sourceSHOTAll, targetSHOTAll,
                                       minSampleDist, maxCorrespondDist, numIterations, numSamples);

    auto coarseTransformedCloudPtr = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceCloudPtr, *coarseTransformedCloudPtr, coarse_transformation);

    applyCorrespondenceRejection(coarse_transformation, coarseTransformedCloudPtr, sourceCloudPtr,
                                 targetCloudPtr, settings);

    Eigen::Matrix4f final_transformation = coarse_transformation;
    if (settings.exists(ICP_ENABLED) && settings.getValue(ICP_ENABLED) > 0.5) {
        double icpMaxCorrespondDist = settings.getValue(ICP_MAX_CORRESPONDENCE_DIST);
        int icpMaxIterations = static_cast<int>(std::lround(settings.getValue(ICP_MAX_ITERATIONS)));
        double icpTransformEpsilon = settings.getValue(ICP_TRANSFORMATION_EPSILON);
        double icpEuclideanEpsilon = settings.getValue(ICP_EUCLIDEAN_EPSILON);

        SearchingMethods searching;
        Eigen::Matrix4f icp_transformation =
            searching.computeICP(coarseTransformedCloudPtr, targetCloudPtr, icpMaxCorrespondDist,
                                 icpMaxIterations, icpTransformEpsilon, icpEuclideanEpsilon);

        final_transformation = icp_transformation * coarse_transformation;
    }

    auto transformedCloudPtr = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceCloudPtr, *transformedCloudPtr, final_transformation);

    return {sourceCloudPtr, targetCloudPtr, transformedCloudPtr, final_transformation};
}
