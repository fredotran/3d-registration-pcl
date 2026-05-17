#pragma once

#include "param_parsing.hpp"

// single viewers tools
void visualizeSinglePointCloud(PointCloudPtr inputCloudPtr);
void visualizeMatchingResults(PointCloudPtr inputCloudPtr, PointCloudPtr transformedCloudPtr);

void visualizeSinglePointCloudForHarris(PointCloudPtr inputCloudPtr,
                                        PtCloudPointWithIntensityPtr inputKeypointsCloudPtr);

void visualizeSinglePointCloudForBRISK(PointCloudPtr inputCloudPtr,
                                       PtCloudPointWithScalePtr inputKeypointsCloudPtr);
void visualizePtClouds(PointCloudPtr inputSrcCloudPtr, PointCloudPtr inputTrgCloudPtr);

void visualizeCroppedResults(PointCloudPtr inputCloudPtr, PointCloudPtr croppedReferenceCloudPtr,
                             PointCloudPtr croppedSourceCloudPtr);

void visualizeResults(PointCloudPtr referenceCloudPtr, PointCloudPtr sourceCloudPtr,
                      PointCloudPtr customTransformedCloudPtr, PointCloudPtr transformedCloudPtr);

// multiple viewers tools
void visualizeMultipleResultsPointCloudHarris(PointCloudPtr sourceCloudPtr,
                                              PointCloudPtr targetCloudPtr,
                                              PtCloudPointWithIntensityPtr srcKeypointsCloudPtr,
                                              PtCloudPointWithIntensityPtr trgKeypointsCloudPtr,
                                              PointCloudPtr transformedCloudPtr);

void visualizeMultipleResultsPointCloudSift(PointCloudPtr inputSrcCloudPtr,
                                            PointCloudPtr inputTrgCloudPtr,
                                            PtCloudPointWithScalePtr sourceSiftKeypointsPtr,
                                            PtCloudPointWithScalePtr targetSiftKeypointsPtr,
                                            PointCloudPtr transformedCloudPtr);

void visualizeMultipleMatchingResults(PointCloudPtr inputTrgCloudPtr1,
                                      PointCloudPtr inputTrgCloudPtr2,
                                      PointCloudPtr transformedCloudPtr1,
                                      PointCloudPtr transformedCloudPtr2);

void visualizeMultipleResults(PointCloudPtr referenceCloudPtr1, PointCloudPtr sourceCloudPtr1,
                              PointCloudPtr customTransformedCloudPtr1,
                              PointCloudPtr transformedCloudPtr1, PointCloudPtr referenceCloudPtr2,
                              PointCloudPtr sourceCloudPtr2,
                              PointCloudPtr customTransformedCloudPtr2,
                              PointCloudPtr transformedCloudPtr2);

void visualizationToolSift(PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr,
                           PointCloudPtr transformedCloudPtr,
                           PtCloudPointWithScalePtr srcSiftKeypointsCloudPtr,
                           PtCloudPointWithScalePtr trgSiftKeypointsCloudPtr,
                           Settings pipelineSettings, std::string pipelineType);

void visualizationToolHarris(PointCloudPtr sourceCloudPtr, PointCloudPtr targetCloudPtr,
                             PointCloudPtr transformedCloudPtr,
                             PtCloudPointWithIntensityPtr srcHarrisKeypointsCloudPtr,
                             PtCloudPointWithIntensityPtr trgHarrisKeypointsCloudPtr,
                             Settings pipelineSettings, std::string pipelineType);
