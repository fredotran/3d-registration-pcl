#include "randomization.hpp"

double randomizeDoubleUniform(double* seed, double min, double max) {
    *seed += 1;
    static thread_local std::mt19937 generator(static_cast<uint32_t>(*seed));
    generator.seed(static_cast<uint32_t>(*seed));             // Re-seed for reproducibility
    std::uniform_real_distribution<> distribution(min, max);  // define distribution
    return distribution(generator);                           // generate random double
}

std::array<double, 2> randomizeCoordinate(double* seed, double boundariesX[2],
                                          double boundariesY[2], double bufferX, double bufferY) {
    double xMin = boundariesX[0] + bufferX;
    double xMax = boundariesX[1] - bufferX;
    double yMin = boundariesY[0] + bufferY;
    double yMax = boundariesY[1] - bufferY;
    if (xMin >= xMax) {
        throw std::runtime_error("randomizeCoordinate: invalid X range after applying buffer [" +
                                 std::to_string(xMin) + ", " + std::to_string(xMax) + "]");
    }
    if (yMin >= yMax) {
        throw std::runtime_error("randomizeCoordinate: invalid Y range after applying buffer [" +
                                 std::to_string(yMin) + ", " + std::to_string(yMax) + "]");
    }
    std::array<double, 2> coord;
    coord[0] = randomizeDoubleUniform(seed, xMin, xMax);
    coord[1] = randomizeDoubleUniform(seed, yMin, yMax);
    return coord;
}

std::array<double, 4> randomizeReferenceCutout(double* seed, double surfaceModelBoundariesX[2],
                                               double surfaceModelBoundariesY[2],
                                               double sourceWidth, double sourceHeight,
                                               double xUncertainty, double yUncertainty) {
    // compute width and height of reference point cloud
    double width = sourceWidth + 2 * xUncertainty;
    double height = sourceHeight + 2 * yUncertainty;
    // width and height computed as worst case, since we assume that yaw rotation of source cloud is
    // unknown
    double refSize = std::sqrt(width * width + height * height);

    // verify that reference fits in surface model
    double surfaceModelWidth = surfaceModelBoundariesX[1] - surfaceModelBoundariesX[0];
    double surfaceModelHeight = surfaceModelBoundariesY[1] - surfaceModelBoundariesY[0];
    std::cout << "\nsurfaceModelWidth/surfaceModelHeight: " << surfaceModelWidth << "/"
              << surfaceModelHeight << std::endl;
    std::cout << "Reference point cloud height=width: " << refSize << std::endl;
    if (refSize >= surfaceModelWidth * 1.01) {
        throw std::runtime_error("Reference size (" + std::to_string(refSize) +
                                 ") exceeds surface model width (" +
                                 std::to_string(surfaceModelWidth) + ")");
    }
    if (refSize >= surfaceModelHeight * 1.01) {
        throw std::runtime_error("Reference size (" + std::to_string(refSize) +
                                 ") exceeds surface model height (" +
                                 std::to_string(surfaceModelHeight) + ")");
    }

    // randomize reference point cloud centroid
    std::array<double, 2> centroid = randomizeCoordinate(
        seed, surfaceModelBoundariesX, surfaceModelBoundariesY, 0.5 * refSize, 0.5 * refSize);

    // compute reference point cloud boundaries
    double xMin = centroid[0] - refSize * 0.5;
    double xMax = centroid[0] + refSize * 0.5;
    double yMin = centroid[1] - refSize * 0.5;
    double yMax = centroid[1] + refSize * 0.5;
    std::array<double, 4> boundaries = {xMin, xMax, yMin, yMax};

    return boundaries;
}

std::array<double, 4> randomizeSourceCutout(double* seed, double refModelBoundariesX[2],
                                            double refModelBoundariesY[2], double sourceWidth,
                                            double sourceHeight) {
    // buffer computed as worst case, since we assume that yaw rotation of source cloud is unknown
    double diagonal = std::sqrt((sourceWidth * sourceWidth + sourceHeight * sourceHeight));

    // verify that source fits in reference point cloud
    double refModelWidth = refModelBoundariesX[1] - refModelBoundariesX[0];
    double refModelHeight = refModelBoundariesY[1] - refModelBoundariesY[0];
    if (diagonal >= refModelWidth * 1.01) {
        throw std::runtime_error("Source diagonal (" + std::to_string(diagonal) +
                                 ") exceeds reference model width (" +
                                 std::to_string(refModelWidth) + ")");
    }
    if (diagonal >= refModelHeight * 1.01) {
        throw std::runtime_error("Source diagonal (" + std::to_string(diagonal) +
                                 ") exceeds reference model height (" +
                                 std::to_string(refModelHeight) + ")");
    }

    // randomize source centroid
    double buffer = 0.5 * diagonal;
    std::array<double, 2> centroid =
        randomizeCoordinate(seed, refModelBoundariesX, refModelBoundariesY, buffer, buffer);

    // compute reference point cloud boundaries
    double xMin = centroid[0] - sourceWidth * 0.5;
    double xMax = centroid[0] + sourceWidth * 0.5;
    double yMin = centroid[1] - sourceHeight * 0.5;
    double yMax = centroid[1] + sourceHeight * 0.5;
    std::array<double, 4> boundaries = {xMin, xMax, yMin, yMax};

    return boundaries;
}
