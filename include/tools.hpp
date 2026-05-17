#pragma once

#include "registration.hpp"

#ifdef _OPENMP
#include <omp.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

///////////////////////////////
//////// PROTOTYPES ///////////
///////////////////////////////

PointCloudPtr loadingCloud(const std::string& fileName);

PointCloudPtr correctedPointCloud(PointCloudPtr sourcePointCloudPtr, Eigen::Vector4f centroid,
                                  Eigen::Vector4f origin_centroid, Eigen::Vector4f new_centroid,
                                  float theta_x, float theta_y, float theta_z);
Eigen::Affine3f customRotation(PointCloudPtr sourcePointCloudPtr, Eigen::Vector4f centroid,
                               Eigen::Vector4f origin_centroid, Eigen::Vector4f new_centroid,
                               float theta_x, float theta_y, float theta_z);
Eigen::Affine3f customTranslation(PointCloudPtr sourcePointCloudPtr,
                                  Eigen::Affine3f inputTransformationMatrix, float trans_x,
                                  float trans_y, float trans_z);

PointCloudPtr statisticalOutlierRemoval(PointCloudPtr inputCloud, int meanK, double stddevMult);

PointCloudPtr cropPointCloud(PointCloudPtr cloudPtr, PointCloudPtr cloudOutPtr, double xMin,
                             double xMax, double yMin, double yMax);
PointCloudPtr computeReferenceCloud(PointCloudPtr surfaceModelCloudPtr, double* seedRef,
                                    double sourceWidth, double sourceHeight, double x_uncertainty,
                                    double y_uncertainty);
PointCloudPtr computeSourceCloud(PointCloudPtr targetCloudPtr, double* seed, double sourceWidth,
                                 double sourceHeight, double x_uncertainty, double y_uncertainty);

void fullRegistration(const std::string& fullParametersFilename, Settings pipelineSettings,
                      const std::string& savedFilename, double* seedRef, double* seedSource,
                      double* seedCustomRotation);
TuplePointCloudPtr fullPipelineSift(TupleParameters parametersList, double* seedRef,
                                    double* seedSource, double* seedCustomRotation,
                                    Settings pipelineSettings);
TuplePointCloudPtr fullPipelineHarris(TupleParameters parametersList, double* seedRef,
                                      double* seedSource, double* seedCustomRotation,
                                      Settings pipelineSettings);
TuplePointCloudPtr fullPipelineISS(TupleParameters parametersList, double* seedRef,
                                   double* seedSource, double* seedCustomRotation,
                                   Settings pipelineSettings);
TuplePointCloudPtr fullPipeline(TupleParameters parametersList, double* seedRef, double* seedSource,
                                double* seedCustomRotation, Settings pipelineSettings);
TuplePointCloudPtr fullPipelineShot(TupleParameters parametersList, double* seedRef,
                                    double* seedSource, double* seedCustomRotation,
                                    Settings pipelineSettings);

double distance(const PointXYZ& p1, const PointXYZ& p2);
double distanceSquared(const PointXYZ& p1, const PointXYZ& p2);
TupleOfVectorDouble getCoordinates(const PointCloudPtr& srcPointCloudPtr);
VectorPointXYZ Get3DCoordinatesXYZ(const PointCloudPtr& srcPointCloudPtr);

TupleOfDouble registrationErrorBias(const PointCloudPtr& srcPointCloudPtr,
                                    const PointCloudPtr& transformedSrcPointCloudPtr);
double meanTargetRegistrationError(const PointCloudPtr& srcPointCloudPtr,
                                   const PointCloudPtr& transformedSrcPointCloudPtr);
double rootMeanSquareError(const PointCloudPtr& srcPointCloudPtr,
                           const PointCloudPtr& transformedSrcPointCloudPtr);
double inlierRatio(const PointCloudPtr& srcPointCloudPtr,
                   const PointCloudPtr& transformedSrcPointCloudPtr, double threshold);
double computePrecision(const PointCloudPtr& srcPointCloudPtr,
                        const PointCloudPtr& transformedSrcPointCloudPtr, double threshold);
double computeRecall(const PointCloudPtr& srcPointCloudPtr,
                     const PointCloudPtr& transformedSrcPointCloudPtr, double threshold);
double computeF1Score(const PointCloudPtr& srcPointCloudPtr,
                      const PointCloudPtr& transformedSrcPointCloudPtr, double threshold);

std::string appendTimestamp(const std::string& filename);
