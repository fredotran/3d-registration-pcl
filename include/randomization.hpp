
////////////////////////////////
////////// PROTOTYPES //////////
////////////////////////////////

// Randomization
float randomAngles(float angleMin, float angleMax);
double randomizeDoubleUniform(double *seed, double min, double max);
std::array<double, 2> randomizeCoordinate(double *seed, double boundariesX[2], double boundariesY[2], double bufferX, double bufferY);
std::array<double, 4> randomizeSourceCutout(double *seed, double refModelBoundariesX[2], double refModelBoundariesY[2],
                                            double sourceWidth, double sourceHeight);


////////////////////////////////
//////////// METHODS ///////////
////////////////////////////////

int randomAngle(int angleMin, int angleMax)
{
    srand((unsigned)time(0));
    int randomizedAngle;
    randomizedAngle = (rand() % angleMax) + angleMin;
    return randomizedAngle;
}

/* Generates a random number between min and max (uniform distribution). The seed will be incremented so that it 
can be directly used in subsequent calls. */
double randomizeDoubleUniform(double *seed, double min, double max)
{
    *seed += 1;
    std::mt19937 generator(*seed);                           //Standard mersenne_twister_engine
    std::uniform_real_distribution<> distribution(min, max); // define distribution
    return distribution(generator);                          // generate random double
}

/* Randomize a 2D coordinate within the provided boundaries under the condition that the distance between the coordinate and 
the boundaries may not be less than the specified buffers. */
std::array<double, 2> randomizeCoordinate(double *seed, double boundariesX[2], double boundariesY[2], double bufferX, double bufferY)
{
    double xMin = boundariesX[0] + bufferX;
    double xMax = boundariesX[1] - bufferX;
    double yMin = boundariesY[0] + bufferY;
    double yMax = boundariesY[1] - bufferY;
    std::array<double, 2> coord;
    coord[0] = randomizeDoubleUniform(seed, xMin, xMax);
    coord[1] = randomizeDoubleUniform(seed, yMin, yMax);
    return coord;
}

/* Returns randomized rectangular (xMin, xMax, yMin, yMax) cut-out boundaries to be used for reference point cloud extraction 
from surface model point cloud(input: surface model data boundaries, source_width, 
source_height, x_uncertainty, y_uncertainty; output: reference PC x/y limits) */
std::array<double, 4> randomizeReferenceCutout(double *seed, double surfaceModelBoundariesX[2], double surfaceModelBoundariesY[2],
                                               double sourceWidth, double sourceHeight, double xUncertainty, double yUncertainty)
{
    // compute width and height of reference point cloud
    double width = sourceWidth + 2 * xUncertainty;
    double height = sourceHeight + 2 * yUncertainty;
    // width and height computed as worst case, since we assume that yaw rotation of source cloud is unknown
    double refSize = std::sqrt(width * width + height * height);

    // verify that reference fits in surface model
    double surfaceModelWidth = surfaceModelBoundariesX[1] - surfaceModelBoundariesX[0];
    double surfaceModelHeight = surfaceModelBoundariesY[1] - surfaceModelBoundariesY[0];
    std::cout << "\nsurfaceModelWidth/surfaceModelHeight: " << surfaceModelWidth << "/" << surfaceModelHeight << endl;
    std::cout << "Reference point cloud height=width: " << refSize << endl;
    assert(refSize < surfaceModelWidth * 1.01);
    assert(refSize < surfaceModelHeight * 1.01);

    // randomize reference point cloud centroid
    std::array<double, 2> centroid = randomizeCoordinate(seed, surfaceModelBoundariesX, surfaceModelBoundariesY, 0.5 * refSize, 0.5 * refSize);

    // compute reference point cloud boundaries
    double xMin = centroid[0] - refSize * 0.5;
    double xMax = centroid[0] + refSize * 0.5;
    double yMin = centroid[1] - refSize * 0.5;
    double yMax = centroid[1] + refSize * 0.5;
    std::array<double, 4> boundaries = {xMin, xMax, yMin, yMax};

    return boundaries;
}

/* Returns randomized rectangular (xMin, xMax, yMin, yMax) cut-out boundaries to be used for source point cloud extraction
from reference point cloud */
std::array<double, 4> randomizeSourceCutout(double *seed, double refModelBoundariesX[2], double refModelBoundariesY[2],
                                            double sourceWidth, double sourceHeight)
{
    // buffer computed as worst case, since we assume that yaw rotation of source cloud is unknown
    double diagonal = std::sqrt((sourceWidth * sourceWidth + sourceHeight * sourceHeight));

    // verify that source fits in reference point cloud
    double refModelWidth = refModelBoundariesX[1] - refModelBoundariesX[0];
    double refModelHeight = refModelBoundariesY[1] - refModelBoundariesY[0];
    assert(diagonal < refModelWidth * 1.01);
    assert(diagonal < refModelHeight * 1.01);

    // randomize source centroid
    double buffer = 0.5 * diagonal;
    std::array<double, 2> centroid = randomizeCoordinate(seed, refModelBoundariesX, refModelBoundariesY, buffer, buffer);

    // compute reference point cloud boundaries
    double xMin = centroid[0] - sourceWidth * 0.5;
    double xMax = centroid[0] + sourceWidth * 0.5;
    double yMin = centroid[1] - sourceHeight * 0.5;
    double yMax = centroid[1] + sourceHeight * 0.5;
    std::array<double, 4> boundaries = {xMin, xMax, yMin, yMax};

    return boundaries;
}