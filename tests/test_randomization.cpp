#include <gtest/gtest.h>

#include <cmath>

#include "randomization.hpp"

TEST(RandomizationTest, DoubleUniformWithinRange) {
    double seed = 42.0;
    double min = 5.0;
    double max = 10.0;

    for (int i = 0; i < 100; ++i) {
        double val = randomizeDoubleUniform(&seed, min, max);
        EXPECT_GE(val, min);
        EXPECT_LE(val, max);
    }
}

TEST(RandomizationTest, DoubleUniformDifferentSeedsProduceDifferentValues) {
    double seed1 = 100.0;
    double seed2 = 200.0;
    double val1 = randomizeDoubleUniform(&seed1, 0.0, 1.0);
    double val2 = randomizeDoubleUniform(&seed2, 0.0, 1.0);
    EXPECT_NE(val1, val2);
}

TEST(RandomizationTest, DoubleUniformSameSeedReproducible) {
    double seed1 = 42.0;
    double seed2 = 42.0;
    double val1 = randomizeDoubleUniform(&seed1, 0.0, 100.0);
    double val2 = randomizeDoubleUniform(&seed2, 0.0, 100.0);
    EXPECT_DOUBLE_EQ(val1, val2);
}

TEST(RandomizationTest, DoubleUniformSeedIsIncremented) {
    double seed = 10.0;
    randomizeDoubleUniform(&seed, 0.0, 1.0);
    EXPECT_DOUBLE_EQ(seed, 11.0);
    randomizeDoubleUniform(&seed, 0.0, 1.0);
    EXPECT_DOUBLE_EQ(seed, 12.0);
}

TEST(RandomizationTest, CoordinateWithinBoundaries) {
    double seed = 50.0;
    double boundariesX[2] = {0.0, 100.0};
    double boundariesY[2] = {0.0, 200.0};
    double bufferX = 10.0;
    double bufferY = 20.0;

    for (int i = 0; i < 50; ++i) {
        auto coord = randomizeCoordinate(&seed, boundariesX, boundariesY, bufferX, bufferY);
        EXPECT_GE(coord[0], boundariesX[0] + bufferX);
        EXPECT_LE(coord[0], boundariesX[1] - bufferX);
        EXPECT_GE(coord[1], boundariesY[0] + bufferY);
        EXPECT_LE(coord[1], boundariesY[1] - bufferY);
    }
}

TEST(RandomizationTest, ReferenceCutoutThrowsWhenSizeExceedsWidth) {
    double seed = 1.0;
    double surfBoundsX[2] = {0.0, 10.0};
    double surfBoundsY[2] = {0.0, 1000.0};
    // refSize = sqrt((100+100)^2 + (100+100)^2) ≈ 283, far exceeds width=10
    double sourceWidth = 100.0;
    double sourceHeight = 100.0;
    double xUncertainty = 50.0;
    double yUncertainty = 50.0;

    EXPECT_THROW(
        randomizeReferenceCutout(&seed, surfBoundsX, surfBoundsY, sourceWidth, sourceHeight,
                                 xUncertainty, yUncertainty),
        std::runtime_error);
}

TEST(RandomizationTest, ReferenceCutoutThrowsWhenSizeExceedsHeight) {
    double seed = 1.0;
    double surfBoundsX[2] = {0.0, 1000.0};
    double surfBoundsY[2] = {0.0, 10.0};
    double sourceWidth = 100.0;
    double sourceHeight = 100.0;
    double xUncertainty = 50.0;
    double yUncertainty = 50.0;

    EXPECT_THROW(
        randomizeReferenceCutout(&seed, surfBoundsX, surfBoundsY, sourceWidth, sourceHeight,
                                 xUncertainty, yUncertainty),
        std::runtime_error);
}

TEST(RandomizationTest, ReferenceCutoutValidInput) {
    double seed = 1.0;
    double surfBoundsX[2] = {0.0, 1000.0};
    double surfBoundsY[2] = {0.0, 1000.0};
    double sourceWidth = 10.0;
    double sourceHeight = 10.0;
    double xUncertainty = 5.0;
    double yUncertainty = 5.0;

    std::array<double, 4> bounds;
    EXPECT_NO_THROW(bounds = randomizeReferenceCutout(&seed, surfBoundsX, surfBoundsY, sourceWidth,
                                                      sourceHeight, xUncertainty, yUncertainty));

    EXPECT_LT(bounds[0], bounds[1]);
    EXPECT_LT(bounds[2], bounds[3]);
    EXPECT_GE(bounds[0], surfBoundsX[0]);
    EXPECT_LE(bounds[1], surfBoundsX[1]);
    EXPECT_GE(bounds[2], surfBoundsY[0]);
    EXPECT_LE(bounds[3], surfBoundsY[1]);
}

TEST(RandomizationTest, SourceCutoutThrowsWhenDiagonalExceedsWidth) {
    double seed = 1.0;
    double refBoundsX[2] = {0.0, 10.0};
    double refBoundsY[2] = {0.0, 1000.0};
    // diagonal = sqrt(100^2 + 100^2) ≈ 141 >> 10
    double sourceWidth = 100.0;
    double sourceHeight = 100.0;

    EXPECT_THROW(randomizeSourceCutout(&seed, refBoundsX, refBoundsY, sourceWidth, sourceHeight),
                 std::runtime_error);
}

TEST(RandomizationTest, SourceCutoutThrowsWhenDiagonalExceedsHeight) {
    double seed = 1.0;
    double refBoundsX[2] = {0.0, 1000.0};
    double refBoundsY[2] = {0.0, 10.0};
    double sourceWidth = 100.0;
    double sourceHeight = 100.0;

    EXPECT_THROW(randomizeSourceCutout(&seed, refBoundsX, refBoundsY, sourceWidth, sourceHeight),
                 std::runtime_error);
}

TEST(RandomizationTest, SourceCutoutValidInput) {
    double seed = 1.0;
    double refBoundsX[2] = {0.0, 500.0};
    double refBoundsY[2] = {0.0, 500.0};
    double sourceWidth = 10.0;
    double sourceHeight = 10.0;

    std::array<double, 4> bounds;
    EXPECT_NO_THROW(
        bounds = randomizeSourceCutout(&seed, refBoundsX, refBoundsY, sourceWidth, sourceHeight));

    EXPECT_LT(bounds[0], bounds[1]);
    EXPECT_LT(bounds[2], bounds[3]);
    EXPECT_NEAR(bounds[1] - bounds[0], sourceWidth, 1e-5);
    EXPECT_NEAR(bounds[3] - bounds[2], sourceHeight, 1e-5);
}
