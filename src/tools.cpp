#include "tools.hpp"

#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <ctime>
#include <iomanip>
#include <numeric>
#include <sstream>

PointCloudPtr loadingCloud(const std::string& fileName) {
    auto cloudPtr = PointCloudPtr(new PointCloud);

    std::string ext = fileName.substr(fileName.find_last_of('.') + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    int result = -1;
    if (ext == "pcd") {
        result = pcl::io::loadPCDFile<pcl::PointXYZ>(fileName, *cloudPtr);
    } else if (ext == "ply") {
        result = pcl::io::loadPLYFile<pcl::PointXYZ>(fileName, *cloudPtr);
    } else if (ext == "obj") {
        pcl::PolygonMesh mesh;
        result = pcl::io::loadOBJFile(fileName, mesh);
        if (result == 0)
            pcl::fromPCLPointCloud2(mesh.cloud, *cloudPtr);
    } else if (ext == "vtk") {
        pcl::PolygonMesh mesh;
        result = pcl::io::loadPolygonFileVTK(fileName, mesh);
        if (result == 0)
            pcl::fromPCLPointCloud2(mesh.cloud, *cloudPtr);
    } else {
        throw std::runtime_error("Unsupported file format: " + ext + " for file: " + fileName);
    }

    if (result == -1) {
        throw std::runtime_error("Failed to read file: " + fileName);
    }

    std::cout << "Loaded source file : " << cloudPtr->size() << " points." << std::endl;

    std::vector<int> nanValues;
    pcl::removeNaNFromPointCloud(*cloudPtr, *cloudPtr, nanValues);

    if (cloudPtr->empty()) {
        throw std::runtime_error("Cloud is empty after loading: " + fileName);
    }

    return cloudPtr;
}

PointCloudPtr correctedPointCloud(PointCloudPtr sourcePointCloudPtr, Eigen::Vector4f centroid,
                                  Eigen::Vector4f origin_centroid, Eigen::Vector4f new_centroid,
                                  float theta_x, float theta_y, float theta_z) {
    (void)origin_centroid;

    Eigen::Affine3f transform_translation = Eigen::Affine3f::Identity();
    pcl::compute3DCentroid(*sourcePointCloudPtr, centroid);
    transform_translation.translation() = new_centroid.head<3>() - centroid.head<3>();

    std::cout << "Transformation used for the origin correction : \n"
              << transform_translation.matrix() << std::endl;
    std::cout << "\nSource point cloud (white) " << std::endl;
    std::cout << "Corrected coordinates point cloud (green) " << std::endl;

    Eigen::Vector4f translatedCentroid;
    translatedCentroid.head<3>() = centroid.head<3>() + transform_translation.translation();
    translatedCentroid[3] = 1.0f;
    std::cout << "\nNew centroid coordinates (after correction) : \n"
              << translatedCentroid << std::endl;

    Eigen::Affine3f transform_rotation = Eigen::Affine3f::Identity();
    transform_rotation.rotate(Eigen::AngleAxisf((theta_x * M_PI) / 180, Eigen::Vector3f::UnitX()));
    transform_rotation.rotate(Eigen::AngleAxisf((theta_y * M_PI) / 180, Eigen::Vector3f::UnitY()));
    transform_rotation.rotate(Eigen::AngleAxisf((theta_z * M_PI) / 180, Eigen::Vector3f::UnitZ()));
    std::cout << "\nTransformation used for the rotation : \n"
              << transform_rotation.matrix() << std::endl;

    Eigen::Affine3f composed =
        transform_translation.inverse() * transform_rotation * transform_translation;
    auto new_cloud = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourcePointCloudPtr, *new_cloud, composed);

    return new_cloud;
}

Eigen::Affine3f customRotation(PointCloudPtr sourcePointCloudPtr, Eigen::Vector4f centroid,
                               Eigen::Vector4f origin_centroid, Eigen::Vector4f new_centroid,
                               float theta_x, float theta_y, float theta_z) {
    (void)origin_centroid;

    Eigen::Affine3f transform_translation = Eigen::Affine3f::Identity();
    pcl::compute3DCentroid(*sourcePointCloudPtr, centroid);
    transform_translation.translation() = new_centroid.head<3>() - centroid.head<3>();

    pcl::compute3DCentroid(*sourcePointCloudPtr, new_centroid);
    std::cout << "\nNew centroid coordinates (after correction) : \n" << new_centroid << std::endl;

    Eigen::Affine3f transform_rotation = Eigen::Affine3f::Identity();
    transform_rotation.rotate(Eigen::AngleAxisf((theta_x * M_PI) / 180, Eigen::Vector3f::UnitX()));
    transform_rotation.rotate(Eigen::AngleAxisf((theta_y * M_PI) / 180, Eigen::Vector3f::UnitY()));
    transform_rotation.rotate(Eigen::AngleAxisf((theta_z * M_PI) / 180, Eigen::Vector3f::UnitZ()));

    Eigen::Affine3f final_matrix =
        transform_translation.inverse() * transform_rotation * transform_translation;

    return final_matrix;
}

Eigen::Affine3f customTranslation(PointCloudPtr sourcePointCloudPtr,
                                  Eigen::Affine3f inputTransformationMatrix, float trans_x,
                                  float trans_y, float trans_z) {
    (void)sourcePointCloudPtr;
    inputTransformationMatrix.translation() << trans_x, trans_y, trans_z;
    return inputTransformationMatrix;
}

PointCloudPtr statisticalOutlierRemoval(PointCloudPtr inputCloud, int meanK, double stddevMult) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    auto filteredCloud = PointCloudPtr(new PointCloud);

    sor.setInputCloud(inputCloud);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(stddevMult);
    sor.filter(*filteredCloud);

    std::cout << "Statistical outlier removal: " << inputCloud->size() << " -> "
              << filteredCloud->size() << " points (removed "
              << inputCloud->size() - filteredCloud->size() << " outliers)" << std::endl;

    return filteredCloud;
}

PointCloudPtr cropPointCloud(PointCloudPtr cloudPtr, PointCloudPtr cloudOutPtr, double xMin,
                             double xMax, double yMin, double yMax) {
    pcl::PointXYZ ref_min_pt, ref_max_pt;
    pcl::getMinMax3D(*cloudPtr, ref_min_pt, ref_max_pt);

    Eigen::Vector4f minPoints(xMin, yMin, ref_min_pt.z, 1.0);
    Eigen::Vector4f maxPoints(xMax, yMax, ref_max_pt.z, 1.0);

    pcl::CropBox<PointXYZ> cropFilter;
    cropFilter.setInputCloud(cloudPtr);
    cropFilter.setMin(minPoints);
    cropFilter.setMax(maxPoints);
    cropFilter.filter(*cloudOutPtr);

    return cloudOutPtr;
}

PointCloudPtr computeReferenceCloud(PointCloudPtr surfaceModelCloudPtr, double* seedRef,
                                    double sourceWidth, double sourceHeight, double x_uncertainty,
                                    double y_uncertainty) {
    if (!surfaceModelCloudPtr || surfaceModelCloudPtr->empty()) {
        throw std::runtime_error(
            "computeReferenceCloud: input surface model cloud is null or empty");
    }
    auto outPointCloudPtr = PointCloudPtr(new PointCloud);

    pcl::PointXYZ ref_min_pt, ref_max_pt;
    pcl::getMinMax3D(*surfaceModelCloudPtr, ref_min_pt, ref_max_pt);
    double xBoundariesRef[2] = {ref_min_pt.x, ref_max_pt.x};
    double yBoundariesRef[2] = {ref_min_pt.y, ref_max_pt.y};

    auto coordRandomizedRef =
        randomizeReferenceCutout(seedRef, xBoundariesRef, yBoundariesRef, sourceWidth, sourceHeight,
                                 x_uncertainty, y_uncertainty);

    cropPointCloud(surfaceModelCloudPtr, outPointCloudPtr, coordRandomizedRef[0],
                   coordRandomizedRef[1], coordRandomizedRef[2], coordRandomizedRef[3]);

    return outPointCloudPtr;
}

PointCloudPtr computeSourceCloud(PointCloudPtr targetCloudPtr, double* seed, double sourceWidth,
                                 double sourceHeight, double x_uncertainty, double y_uncertainty) {
    if (!targetCloudPtr || targetCloudPtr->empty()) {
        throw std::runtime_error("computeSourceCloud: input target cloud is null or empty");
    }
    auto outPointCloudPtr = PointCloudPtr(new PointCloud);

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*targetCloudPtr, min_pt, max_pt);
    double xBoundaries[2] = {min_pt.x, max_pt.x};
    double yBoundaries[2] = {min_pt.y, max_pt.y};

    auto coordSourceRandomized =
        randomizeSourceCutout(seed, xBoundaries, yBoundaries, sourceWidth, sourceHeight);

    cropPointCloud(targetCloudPtr, outPointCloudPtr, coordSourceRandomized[0],
                   coordSourceRandomized[1], coordSourceRandomized[2], coordSourceRandomized[3]);

    return outPointCloudPtr;
}

namespace {

std::tuple<PointCloudPtr, PointCloudPtr, PointCloudPtr, Eigen::Affine3f, double, double, double>
setupPipeline(TupleParameters& parametersList, double* seedRef, double* seedSource,
              double* seedCustomRotation) {
    std::string pipelineType, typeTransformation, referenceDataFilename;
    double x_uncertainty, y_uncertainty, sourceWidth, sourceHeight;
    double x_angle_min, x_angle_max, y_angle_min, y_angle_max, z_angle_min, z_angle_max;

    std::tie(pipelineType, typeTransformation, referenceDataFilename, x_uncertainty, y_uncertainty,
             sourceWidth, sourceHeight, x_angle_min, x_angle_max, y_angle_min, y_angle_max,
             z_angle_min, z_angle_max) = parametersList;

    auto surfaceModelCloudPtr = loadingCloud(referenceDataFilename);

    auto targetCloudPtr = computeReferenceCloud(surfaceModelCloudPtr, seedRef, sourceWidth,
                                                sourceHeight, x_uncertainty, y_uncertainty);
    auto sourceCloudPtr = computeSourceCloud(targetCloudPtr, seedSource, sourceWidth, sourceHeight,
                                             x_uncertainty, y_uncertainty);

    double randomAngleOnX = randomizeDoubleUniform(seedCustomRotation, x_angle_min, x_angle_max);
    double randomAngleOnY = randomizeDoubleUniform(seedCustomRotation, y_angle_min, y_angle_max);
    double randomAngleOnZ = randomizeDoubleUniform(seedCustomRotation, z_angle_min, z_angle_max);
    std::cout << "\nangle on x : " << randomAngleOnX << "\nangle on y : " << randomAngleOnY
              << "\nangle on z : " << randomAngleOnZ << std::endl;

    Eigen::Affine3f first_transform =
        customRotation(sourceCloudPtr, Eigen::Vector4f::Zero(), Eigen::Vector4f::Zero(),
                       Eigen::Vector4f::Zero(), randomAngleOnX, randomAngleOnY, randomAngleOnZ);

    std::cout << "\nFirst transformation : \n" << first_transform.matrix() << std::endl;

    Eigen::Affine3f second_transform = Eigen::Affine3f::Identity();
    second_transform.translation() << 0.0f, 0.0f, 0.0f;

    Eigen::Affine3f final_transform = first_transform * second_transform;
    std::cout << "\nFinal transformation : \n" << final_transform.matrix() << std::endl;

    auto sourceTransformedCloudPtr = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceCloudPtr, *sourceTransformedCloudPtr, final_transform);

    return {sourceCloudPtr,  targetCloudPtr, sourceTransformedCloudPtr,
            final_transform, randomAngleOnX, randomAngleOnY,
            randomAngleOnZ};
}

template <typename PipelineOutput, int TransformIndex>
TuplePointCloudPtr fullPipelineTemplate(
    TupleParameters parametersList, double* seedRef, double* seedSource, double* seedCustomRotation,
    Settings pipelineSettings,
    std::function<PipelineOutput(PointCloudPtr, PointCloudPtr, Settings)> pipelineFunc) {
    auto [sourceCloudPtr, targetCloudPtr, sourceTransformedCloudPtr, final_transform, angleX,
          angleY, angleZ] = setupPipeline(parametersList, seedRef, seedSource, seedCustomRotation);

    auto pipelineOutput = pipelineFunc(sourceTransformedCloudPtr, targetCloudPtr, pipelineSettings);

    auto finalCloudPtr = PointCloudPtr(new PointCloud);
    pcl::transformPointCloud(*sourceCloudPtr, *finalCloudPtr,
                             std::get<TransformIndex>(pipelineOutput));

    std::cout << "\nNot displaying any results" << std::endl;

    return {sourceCloudPtr, targetCloudPtr, sourceTransformedCloudPtr, finalCloudPtr, angleX,
            angleY,         angleZ};
}

}  // namespace

TuplePointCloudPtr fullPipelineSift(TupleParameters parametersList, double* seedRef,
                                    double* seedSource, double* seedCustomRotation,
                                    Settings pipelineSettings) {
    return fullPipelineTemplate<PipelineSiftOutput, 5>(
        parametersList, seedRef, seedSource, seedCustomRotation, pipelineSettings, siftPipeline);
}

TuplePointCloudPtr fullPipelineHarris(TupleParameters parametersList, double* seedRef,
                                      double* seedSource, double* seedCustomRotation,
                                      Settings pipelineSettings) {
    return fullPipelineTemplate<PipelineHarrisOutput, 5>(
        parametersList, seedRef, seedSource, seedCustomRotation, pipelineSettings, harrisPipeline);
}

TuplePointCloudPtr fullPipelineISS(TupleParameters parametersList, double* seedRef,
                                   double* seedSource, double* seedCustomRotation,
                                   Settings pipelineSettings) {
    return fullPipelineTemplate<PipelineISSOutput, 5>(
        parametersList, seedRef, seedSource, seedCustomRotation, pipelineSettings, issPipeline);
}

TuplePointCloudPtr fullPipeline(TupleParameters parametersList, double* seedRef, double* seedSource,
                                double* seedCustomRotation, Settings pipelineSettings) {
    return fullPipelineTemplate<PipelineAllPointsOutput, 3>(parametersList, seedRef, seedSource,
                                                            seedCustomRotation, pipelineSettings,
                                                            pipelineAllPoints);
}

TuplePointCloudPtr fullPipelineShot(TupleParameters parametersList, double* seedRef,
                                    double* seedSource, double* seedCustomRotation,
                                    Settings pipelineSettings) {
    return fullPipelineTemplate<PipelineAllPointsOutput, 3>(
        parametersList, seedRef, seedSource, seedCustomRotation, pipelineSettings, shotPipeline);
}

double distance(const PointXYZ& p1, const PointXYZ& p2) {
    Eigen::Vector3f diff = p1.getVector3fMap() - p2.getVector3fMap();
    return diff.norm();
}

double distanceSquared(const PointXYZ& p1, const PointXYZ& p2) {
    Eigen::Vector3f diff = p1.getVector3fMap() - p2.getVector3fMap();
    return diff.squaredNorm();
}

TupleOfVectorDouble getCoordinates(const PointCloudPtr& srcPointCloudPtr) {
    auto& srcPointCloud = *srcPointCloudPtr;
    std::vector<double> mtre_x_src(srcPointCloud.size());
    std::vector<double> mtre_y_src(srcPointCloud.size());
    std::vector<double> mtre_z_src(srcPointCloud.size());

#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (size_t i = 0; i < srcPointCloud.size(); ++i) {
        mtre_x_src[i] = srcPointCloud[i].x;
        mtre_y_src[i] = srcPointCloud[i].y;
        mtre_z_src[i] = srcPointCloud[i].z;
    }

    return {mtre_x_src, mtre_y_src, mtre_z_src};
}

VectorPointXYZ Get3DCoordinatesXYZ(const PointCloudPtr& srcPointCloudPtr) {
    auto& srcPointCloud = *srcPointCloudPtr;
    VectorPointXYZ coords(srcPointCloud.size());

#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (size_t i = 0; i < srcPointCloud.size(); ++i) {
        coords[i] = srcPointCloud[i];
    }
    return coords;
}

TupleOfDouble registrationErrorBias(const PointCloudPtr& srcPointCloudPtr,
                                    const PointCloudPtr& transformedSrcPointCloudPtr) {
    auto [mtre_x_src, mtre_y_src, mtre_z_src] = getCoordinates(srcPointCloudPtr);
    auto [mtre_x_transformed_src, mtre_y_transformed_src, mtre_z_transformed_src] =
        getCoordinates(transformedSrcPointCloudPtr);

    std::vector<double> mtre_x(mtre_x_src.size());
    std::vector<double> mtre_y(mtre_y_src.size());
    std::vector<double> mtre_z(mtre_z_src.size());

#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (size_t i = 0; i < mtre_x_src.size(); ++i) {
        mtre_x[i] = mtre_x_src[i] - mtre_x_transformed_src[i];
        mtre_y[i] = mtre_y_src[i] - mtre_y_transformed_src[i];
        mtre_z[i] = mtre_z_src[i] - mtre_z_transformed_src[i];
    }

    if (mtre_x.empty()) {
        return {0.0, 0.0, 0.0};
    }
    double avg_x = std::accumulate(mtre_x.begin(), mtre_x.end(), 0.0) / mtre_x.size();
    double avg_y = std::accumulate(mtre_y.begin(), mtre_y.end(), 0.0) / mtre_y.size();
    double avg_z = std::accumulate(mtre_z.begin(), mtre_z.end(), 0.0) / mtre_z.size();

    return {avg_x, avg_y, avg_z};
}

double meanTargetRegistrationError(const PointCloudPtr& srcPointCloudPtr,
                                   const PointCloudPtr& transformedSrcPointCloudPtr) {
    auto coordinatesSrc = Get3DCoordinatesXYZ(srcPointCloudPtr);
    auto coordinatesSrcTransformed = Get3DCoordinatesXYZ(transformedSrcPointCloudPtr);

    double sum = 0.0;
    size_t count = coordinatesSrc.size();

#ifdef _OPENMP
#pragma omp parallel for reduction(+ : sum)
#endif
    for (size_t i = 0; i < count; ++i) {
        sum += distance(coordinatesSrc[i], coordinatesSrcTransformed[i]);
    }

    if (count == 0)
        return 0.0;
    return sum / count;
}

double rootMeanSquareError(const PointCloudPtr& srcPointCloudPtr,
                           const PointCloudPtr& transformedSrcPointCloudPtr) {
    auto coordinatesSrc = Get3DCoordinatesXYZ(srcPointCloudPtr);
    auto coordinatesSrcTransformed = Get3DCoordinatesXYZ(transformedSrcPointCloudPtr);

    double sum_squared = 0.0;
    size_t count = coordinatesSrc.size();

#ifdef _OPENMP
#pragma omp parallel for reduction(+ : sum_squared)
#endif
    for (size_t i = 0; i < count; ++i) {
        double dist = distance(coordinatesSrc[i], coordinatesSrcTransformed[i]);
        sum_squared += dist * dist;
    }

    if (count == 0)
        return 0.0;
    return std::sqrt(sum_squared / count);
}

double inlierRatio(const PointCloudPtr& srcPointCloudPtr,
                   const PointCloudPtr& transformedSrcPointCloudPtr, double threshold) {
    auto coordinatesSrc = Get3DCoordinatesXYZ(srcPointCloudPtr);
    auto coordinatesSrcTransformed = Get3DCoordinatesXYZ(transformedSrcPointCloudPtr);

    double thresholdSq = threshold * threshold;
    int inliers = 0;
    size_t count = coordinatesSrc.size();

#ifdef _OPENMP
#pragma omp parallel for reduction(+ : inliers)
#endif
    for (size_t i = 0; i < count; ++i) {
        if (distanceSquared(coordinatesSrc[i], coordinatesSrcTransformed[i]) < thresholdSq) {
            inliers++;
        }
    }

    if (count == 0)
        return 0.0;
    return static_cast<double>(inliers) / count;
}

double computePrecision(const PointCloudPtr& srcPointCloudPtr,
                        const PointCloudPtr& transformedSrcPointCloudPtr, double threshold) {
    return inlierRatio(srcPointCloudPtr, transformedSrcPointCloudPtr, threshold);
}

double computeRecall(const PointCloudPtr& srcPointCloudPtr,
                     const PointCloudPtr& transformedSrcPointCloudPtr, double threshold) {
    return inlierRatio(srcPointCloudPtr, transformedSrcPointCloudPtr, threshold);
}

double computeF1Score(const PointCloudPtr& srcPointCloudPtr,
                      const PointCloudPtr& transformedSrcPointCloudPtr, double threshold) {
    double precision = computePrecision(srcPointCloudPtr, transformedSrcPointCloudPtr, threshold);
    double recall = computeRecall(srcPointCloudPtr, transformedSrcPointCloudPtr, threshold);

    if (precision + recall < 1e-10) {
        return 0.0;
    }
    return 2.0 * precision * recall / (precision + recall);
}

std::string appendTimestamp(const std::string& filename) {
    auto t = std::time(nullptr);
    std::tm tm_buf{};
    gmtime_r(&t, &tm_buf);
    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%d-%m-%Y %H-%M-%S");
    return removeExtension(filename) + "_" + oss.str();
}

void fullRegistration(const std::string& fullParametersFilename, Settings pipelineSettings,
                      const std::string& savedFilename, double* seedRef, double* seedSource,
                      double* seedCustomRotation) {
    double initialSeedRef = *seedRef;
    double initialSeedSource = *seedSource;
    double initialSeedRot = *seedCustomRotation;

    auto parameters = readParameters(fullParametersFilename);
    auto parametersList = parametersArray(fullParametersFilename, parameters);

    std::string pipelineType, typeTransformation, referenceDataFilename;
    double x_uncertainty, y_uncertainty, sourceWidth, sourceHeight;
    double x_angle_min, x_angle_max, y_angle_min, y_angle_max, z_angle_min, z_angle_max;

    std::tie(pipelineType, typeTransformation, referenceDataFilename, x_uncertainty, y_uncertainty,
             sourceWidth, sourceHeight, x_angle_min, x_angle_max, y_angle_min, y_angle_max,
             z_angle_min, z_angle_max) = parametersList;

    TuplePointCloudPtr PointCloudsPtrOutput;

    if (pipelineType == "all") {
        std::cout << "\nVisualizer parameter : " << pipelineSettings.getValue(VISUALIZER_PARAMETER)
                  << std::endl;
        PointCloudsPtrOutput =
            fullPipeline(parametersList, seedRef, seedSource, seedCustomRotation, pipelineSettings);
    } else if (pipelineType == "sift") {
        std::cout << "\nVisualizer parameter : " << pipelineSettings.getValue(VISUALIZER_PARAMETER)
                  << std::endl;
        PointCloudsPtrOutput = fullPipelineSift(parametersList, seedRef, seedSource,
                                                seedCustomRotation, pipelineSettings);
    } else if (pipelineType == "harris") {
        std::cout << "\nVisualizer parameter : " << pipelineSettings.getValue(VISUALIZER_PARAMETER)
                  << std::endl;
        PointCloudsPtrOutput = fullPipelineHarris(parametersList, seedRef, seedSource,
                                                  seedCustomRotation, pipelineSettings);
    } else if (pipelineType == "iss") {
        std::cout << "\nVisualizer parameter : " << pipelineSettings.getValue(VISUALIZER_PARAMETER)
                  << std::endl;
        PointCloudsPtrOutput = fullPipelineISS(parametersList, seedRef, seedSource,
                                               seedCustomRotation, pipelineSettings);
    } else if (pipelineType == "shot") {
        std::cout << "\nVisualizer parameter : " << pipelineSettings.getValue(VISUALIZER_PARAMETER)
                  << std::endl;
        PointCloudsPtrOutput = fullPipelineShot(parametersList, seedRef, seedSource,
                                                seedCustomRotation, pipelineSettings);
    } else {
        std::cout << "===ERROR=== aborting: pipelineType not recognized: " << pipelineType
                  << std::endl;
        return;
    }

    PointCloudPtr sourceCloudPtr, targetCloudPtr, sourceTransformedCloudPtr, transformedCloudPtr;
    double angleX, angleY, angleZ;
    std::tie(sourceCloudPtr, targetCloudPtr, sourceTransformedCloudPtr, transformedCloudPtr, angleX,
             angleY, angleZ) = PointCloudsPtrOutput;

    std::cout << "\n-------------------------------------------------------------------------------"
                 "----------------\n"
              << "--------------------------------- Mean Target Registration Error "
                 "--------------------------------\n"
              << "---------------------------------------------------------------------------------"
                 "----------------\n";
    double mtre = meanTargetRegistrationError(sourceCloudPtr, transformedCloudPtr);
    std::cout << "[" << pipelineType
              << "] Mean Target Registration Error between original source and transformed cloud: "
              << mtre << std::endl;

    double rmse = rootMeanSquareError(sourceCloudPtr, transformedCloudPtr);
    std::cout << "[" << pipelineType << "] Root Mean Square Error: " << rmse << std::endl;

    double inlier_threshold = 1.0;
    double inlier_ratio = inlierRatio(sourceCloudPtr, transformedCloudPtr, inlier_threshold);
    double precision = computePrecision(sourceCloudPtr, transformedCloudPtr, inlier_threshold);
    double recall = computeRecall(sourceCloudPtr, transformedCloudPtr, inlier_threshold);
    double f1_score = computeF1Score(sourceCloudPtr, transformedCloudPtr, inlier_threshold);

    std::cout << "[" << pipelineType << "] Inlier ratio (threshold=" << inlier_threshold
              << "m): " << inlier_ratio << std::endl;
    std::cout << "[" << pipelineType << "] Precision: " << precision << std::endl;
    std::cout << "[" << pipelineType << "] Recall: " << recall << std::endl;
    std::cout << "[" << pipelineType << "] F1 Score: " << f1_score << std::endl;

    auto bias = registrationErrorBias(sourceCloudPtr, transformedCloudPtr);
    double bias_x, bias_y, bias_z;
    std::tie(bias_x, bias_y, bias_z) = bias;

    std::cout << "\n-------------------------------------------------------------------------------"
                 "----------------\n"
              << "------------------------------------ Registration error bias "
                 "------------------------------------\n"
              << "---------------------------------------------------------------------------------"
                 "----------------\n"
              << "\n[" << pipelineType << "] Registration error bias on x: " << bias_x << "\n"
              << "[" << pipelineType << "] Registration error bias on y: " << bias_y << "\n"
              << "[" << pipelineType << "] Registration error bias on z: " << bias_z << std::endl;

    StringMap settingsMap;
    settingsMap["pipelineType"] = pipelineType;
    settingsMap["err_MTRE"] = std::to_string(mtre);
    settingsMap["err_RMSE"] = std::to_string(rmse);
    settingsMap["err_inlier_ratio"] = std::to_string(inlier_ratio);
    settingsMap["err_precision"] = std::to_string(precision);
    settingsMap["err_recall"] = std::to_string(recall);
    settingsMap["err_f1_score"] = std::to_string(f1_score);
    settingsMap["err_bias_x"] = std::to_string(bias_x);
    settingsMap["err_bias_y"] = std::to_string(bias_y);
    settingsMap["err_bias_z"] = std::to_string(bias_z);
    settingsMap["angle_x"] = std::to_string(angleX);
    settingsMap["angle_y"] = std::to_string(angleY);
    settingsMap["angle_z"] = std::to_string(angleZ);
    settingsMap["x_uncertainty"] = std::to_string(x_uncertainty);
    settingsMap["y_uncertainty"] = std::to_string(y_uncertainty);
    settingsMap["sourceWidth"] = std::to_string(sourceWidth);
    settingsMap["sourceHeight"] = std::to_string(sourceHeight);
    settingsMap["dataset"] = referenceDataFilename;
    settingsMap["seedRef"] = std::to_string(initialSeedRef);
    settingsMap["seedSource"] = std::to_string(initialSeedSource);
    settingsMap["seedRot"] = std::to_string(initialSeedRot);

    auto pipelineSettingsMap = pipelineSettings.getAllSettings();
    for (const auto& item : pipelineSettingsMap) {
        settingsMap[item.first] = std::to_string(item.second);
    }

    saveResults(savedFilename, settingsMap);
}
