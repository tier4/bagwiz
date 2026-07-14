// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/projection_matrix.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace
{

using bagwiz::core::image::compute_projection_matrix;
using bagwiz::core::image::ProjectionMatrixInput;

// A real 1920x1280 plumb_bob calibration. Its projection matrix was produced by
// the `camera_calibration` package, i.e. by
// cv::getOptimalNewCameraMatrix(k, d, size, alpha=0.0), so recomputing p at
// alpha=0 must reproduce kGoldenP. This is the regression anchor for the whole
// command: if OpenCV ever changes the maths, this test is what catches it.
constexpr std::array<double, 9> kGoldenK{854.298157, 0.000000, 964.290283, 0.000000, 907.693054,
                                         646.557800, 0.000000, 0.000000,   1.000000};

const std::vector<double> kGoldenD{
  -0.143768742681, 0.031336024404, -0.001296524890, -0.001500067534, -0.003719333094};

constexpr std::array<double, 12> kGoldenP{695.399963, 0.000000,   955.277163, 0.000000,
                                          0.000000,   840.401001, 644.738561, 0.000000,
                                          0.000000,   0.000000,   1.000000,   0.000000};

constexpr std::array<double, 9> kIdentityR{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

// kGoldenP cannot be matched exactly: cv::getOptimalNewCameraMatrix's result
// shifts slightly between OpenCV versions (4.5.4 -> 4.13.0 moves fx/fy/cx/cy by
// up to 0.77px), and kGoldenP was written by an OpenCV 4.5.x-era
// camera_calibration. bagwiz builds against each distro's own OpenCV, so the
// exact value is distro-dependent by design and pinning it would break
// humble/jazzy/lyrical against each other.
//
// 2px therefore absorbs any version drift while still failing every way this
// could actually be wrong -- the nearest incorrect answer is an order of
// magnitude outside it (alpha=0.25 is 16px off, alpha=1 is 66px, a naive [k|0]
// is 159px). AlphaZeroFitsTheGoldenFileBestOfAnyAlpha below pins down the one
// thing this tolerance leaves loose.
constexpr double kFileTol = 2.0;

// Exact-comparison tolerance for claims that are pure arithmetic on our side
// (no OpenCV involved), so no version drift applies.
constexpr double kExactTol = 1e-9;

ProjectionMatrixInput golden_input()
{
  ProjectionMatrixInput in;
  in.k = kGoldenK;
  in.r = kIdentityR;
  in.p = kGoldenP;
  in.d = kGoldenD;
  in.distortion_model = "plumb_bob";
  in.width = 1920;
  in.height = 1280;
  return in;
}

// The largest absolute deviation of `p` from the golden file's own p.
double max_deviation_from_golden(const std::array<double, 12> & p)
{
  double worst = 0.0;
  for (std::size_t i = 0; i < 12; ++i) {
    worst = std::max(worst, std::abs(p[i] - kGoldenP[i]));
  }
  return worst;
}

TEST(ProjectionMatrixTest, ReproducesCameraCalibrationOutputAtAlphaZero)
{
  const auto result = compute_projection_matrix(golden_input(), 0.0);

  ASSERT_TRUE(result.ok()) << result.error;
  for (std::size_t i = 0; i < 12; ++i) {
    EXPECT_NEAR((*result.p)[i], kGoldenP[i], kFileTol) << "p[" << i << "] differs";
  }
}

// Guards the claim kFileTol is too loose to make on its own: that the default
// really is alpha=0 and not some other alpha that happens to land nearby. Every
// other alpha must fit the golden file strictly worse, which holds on any
// OpenCV version because it compares like against like.
TEST(ProjectionMatrixTest, AlphaZeroFitsTheGoldenFileBestOfAnyAlpha)
{
  const auto best = compute_projection_matrix(golden_input(), 0.0);
  ASSERT_TRUE(best.ok()) << best.error;
  const double best_deviation = max_deviation_from_golden(*best.p);

  for (const double alpha : {0.25, 0.5, 0.75, 1.0}) {
    const auto other = compute_projection_matrix(golden_input(), alpha);
    ASSERT_TRUE(other.ok()) << other.error;
    EXPECT_GT(max_deviation_from_golden(*other.p), best_deviation)
      << "alpha=" << alpha << " fits the golden file at least as well as alpha=0, so this test no "
      << "longer proves the default is alpha=0";
  }
}

TEST(ProjectionMatrixTest, AlphaOneZoomsOutFurtherThanAlphaZero)
{
  const auto tight = compute_projection_matrix(golden_input(), 0.0);
  const auto wide = compute_projection_matrix(golden_input(), 1.0);

  ASSERT_TRUE(tight.ok()) << tight.error;
  ASSERT_TRUE(wide.ok()) << wide.error;

  // alpha=1 keeps every source pixel, so the undistorted view must shrink to
  // fit: a strictly smaller focal length than the alpha=0 crop.
  EXPECT_LT((*wide.p)[0], (*tight.p)[0]);
  EXPECT_LT((*wide.p)[5], (*tight.p)[5]);
}

TEST(ProjectionMatrixTest, AlwaysZeroesTheFourthColumn)
{
  const auto result = compute_projection_matrix(golden_input(), 0.0);

  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_DOUBLE_EQ((*result.p)[3], 0.0);
  EXPECT_DOUBLE_EQ((*result.p)[7], 0.0);
  EXPECT_DOUBLE_EQ((*result.p)[11], 0.0);
  // Bottom row stays [0 0 1 0].
  EXPECT_DOUBLE_EQ((*result.p)[8], 0.0);
  EXPECT_DOUBLE_EQ((*result.p)[9], 0.0);
  EXPECT_DOUBLE_EQ((*result.p)[10], 1.0);
}

TEST(ProjectionMatrixTest, EmptyDistortionYieldsKWithZeroColumn)
{
  auto in = golden_input();
  in.d.clear();

  const auto result = compute_projection_matrix(in, 0.0);

  ASSERT_TRUE(result.ok()) << result.error;
  const std::array<double, 12> expected{kGoldenK[0], kGoldenK[1], kGoldenK[2], 0.0,
                                        kGoldenK[3], kGoldenK[4], kGoldenK[5], 0.0,
                                        kGoldenK[6], kGoldenK[7], kGoldenK[8], 0.0};
  for (std::size_t i = 0; i < 12; ++i) {
    EXPECT_DOUBLE_EQ((*result.p)[i], expected[i]) << "p[" << i << "] differs";
  }
}

TEST(ProjectionMatrixTest, AllZeroDistortionYieldsKWithZeroColumn)
{
  auto in = golden_input();
  in.d = {0.0, 0.0, 0.0, 0.0, 0.0};

  const auto result = compute_projection_matrix(in, 0.0);

  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_DOUBLE_EQ((*result.p)[0], kGoldenK[0]);
  EXPECT_DOUBLE_EQ((*result.p)[5], kGoldenK[4]);
  EXPECT_DOUBLE_EQ((*result.p)[2], kGoldenK[2]);
  EXPECT_DOUBLE_EQ((*result.p)[6], kGoldenK[5]);
}

TEST(ProjectionMatrixTest, TreatsUnsetRectificationAsIdentity)
{
  auto unset = golden_input();
  unset.r = {};  // all-zero: publishers that never set r, as UndistortHelper allows

  const auto from_unset = compute_projection_matrix(unset, 0.0);
  const auto from_identity = compute_projection_matrix(golden_input(), 0.0);

  ASSERT_TRUE(from_unset.ok()) << from_unset.error;
  ASSERT_TRUE(from_identity.ok()) << from_identity.error;

  // Comparing the two results against each other (rather than against the
  // golden file) keeps this an exact assertion on any OpenCV version: whatever
  // the maths produces, an unset r must produce it too.
  for (std::size_t i = 0; i < 12; ++i) {
    EXPECT_NEAR((*from_unset.p)[i], (*from_identity.p)[i], kExactTol) << "p[" << i << "] differs";
  }
}

// --- Guardrails: cases where recomputing p from k would be wrong ------------

TEST(ProjectionMatrixTest, RejectsStereoRectifiedRotation)
{
  auto in = golden_input();
  // A genuine rectification rotation (~5 deg about z): p belongs to stereoRectify.
  in.r = {0.996195, -0.087156, 0.0, 0.087156, 0.996195, 0.0, 0.0, 0.0, 1.0};

  const auto result = compute_projection_matrix(in, 0.0);

  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.error.find("rectification"), std::string::npos) << result.error;
}

TEST(ProjectionMatrixTest, RejectsStereoBaselineInProjectionMatrix)
{
  auto in = golden_input();
  in.p[3] = -123.456;  // -fx * baseline: a stereo right camera

  const auto result = compute_projection_matrix(in, 0.0);

  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.error.find("baseline"), std::string::npos) << result.error;
}

TEST(ProjectionMatrixTest, RejectsFisheyeDistortionModel)
{
  for (const char * model : {"equidistant", "fisheye"}) {
    auto in = golden_input();
    in.distortion_model = model;

    const auto result = compute_projection_matrix(in, 0.0);

    EXPECT_FALSE(result.ok()) << model;
    EXPECT_NE(result.error.find(model), std::string::npos) << result.error;
  }
}

TEST(ProjectionMatrixTest, AcceptsRationalPolynomial)
{
  auto in = golden_input();
  in.distortion_model = "rational_polynomial";
  in.d = {-0.14, 0.03, -0.001, -0.0015, -0.003, 0.0, 0.0, 0.0};  // 8 coefficients

  const auto result = compute_projection_matrix(in, 0.0);

  EXPECT_TRUE(result.ok()) << result.error;
}

TEST(ProjectionMatrixTest, RejectsUnknownDistortionModel)
{
  auto in = golden_input();
  in.distortion_model = "double_sphere";

  const auto result = compute_projection_matrix(in, 0.0);

  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.error.find("double_sphere"), std::string::npos) << result.error;
  // The message must name what IS supported, not just what is not.
  EXPECT_NE(result.error.find("plumb_bob"), std::string::npos) << result.error;
}

// An unsupported model must be refused on its own merits, not merely wherever
// the distortion maths would have been reached. Zero coefficients previously
// short-circuited to [k|0] before the model was ever inspected, quietly
// accepting a camera whose p this function cannot compute.
TEST(ProjectionMatrixTest, RejectsUnsupportedModelEvenWithZeroCoefficients)
{
  for (const char * model : {"equidistant", "fisheye", "double_sphere"}) {
    auto in = golden_input();
    in.distortion_model = model;
    in.d = {0.0, 0.0, 0.0, 0.0, 0.0};

    const auto result = compute_projection_matrix(in, 0.0);

    EXPECT_FALSE(result.ok()) << model << " with all-zero d must still be refused";
  }
}

TEST(ProjectionMatrixTest, RejectsUnsupportedModelEvenWithEmptyCoefficients)
{
  auto in = golden_input();
  in.distortion_model = "equidistant";
  in.d.clear();

  const auto result = compute_projection_matrix(in, 0.0);

  EXPECT_FALSE(result.ok());
}

// An empty model or "none" declares an already-undistorted camera, so p is
// [k|0] rather than an error -- these are supported, not unknown.
TEST(ProjectionMatrixTest, TreatsNoDistortionModelAsUndistorted)
{
  for (const char * model : {"", "none"}) {
    auto in = golden_input();
    in.distortion_model = model;

    const auto result = compute_projection_matrix(in, 0.0);

    ASSERT_TRUE(result.ok()) << "model='" << model << "': " << result.error;
    EXPECT_DOUBLE_EQ((*result.p)[0], kGoldenK[0]) << "model='" << model << "'";
    EXPECT_DOUBLE_EQ((*result.p)[2], kGoldenK[2]) << "model='" << model << "'";
    EXPECT_DOUBLE_EQ((*result.p)[3], 0.0);
  }
}

TEST(ProjectionMatrixTest, RejectsZeroImageSize)
{
  auto in = golden_input();
  in.width = 0;

  const auto result = compute_projection_matrix(in, 0.0);

  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.error.empty());
}

TEST(ProjectionMatrixTest, RejectsAlphaOutsideUnitRange)
{
  for (const double alpha : {-0.1, 1.1}) {
    const auto result = compute_projection_matrix(golden_input(), alpha);
    EXPECT_FALSE(result.ok()) << "alpha=" << alpha << " should be refused";
  }
}

TEST(ProjectionMatrixTest, RejectsDegenerateIntrinsics)
{
  auto in = golden_input();
  in.k[0] = 0.0;  // fx = 0

  const auto result = compute_projection_matrix(in, 0.0);

  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.error.empty());
}

TEST(ProjectionMatrixTest, RejectsNonFiniteIntrinsics)
{
  auto in = golden_input();
  in.k[2] = std::numeric_limits<double>::quiet_NaN();

  const auto result = compute_projection_matrix(in, 0.0);

  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.error.empty());
}

}  // namespace
