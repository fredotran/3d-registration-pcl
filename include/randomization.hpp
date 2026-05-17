#pragma once

#include <array>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>

double randomizeDoubleUniform(double* seed, double min, double max);
std::array<double, 2> randomizeCoordinate(double* seed, double boundariesX[2],
                                          double boundariesY[2], double bufferX, double bufferY);
std::array<double, 4> randomizeReferenceCutout(double* seed, double surfaceModelBoundariesX[2],
                                               double surfaceModelBoundariesY[2],
                                               double sourceWidth, double sourceHeight,
                                               double xUncertainty, double yUncertainty);
std::array<double, 4> randomizeSourceCutout(double* seed, double refModelBoundariesX[2],
                                            double refModelBoundariesY[2], double sourceWidth,
                                            double sourceHeight);
