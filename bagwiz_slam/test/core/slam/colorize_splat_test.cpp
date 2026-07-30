// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/colorize_splat.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace
{
namespace slam = bagwiz::core::slam;

constexpr double kFx = 400.0;
constexpr double kFy = 400.0;
// Well above every semi-axis the cases below produce, so the clamp only
// engages in the test that asks for it.
constexpr double kNoClamp = 1e6;

// The two semi-axis lengths of a footprint, minor first, from the eigenvalues
// of the symmetric 2x2 C (the ellipse is { q : q^T C^-1 q <= 1 }, so the
// semi-axes are the square roots of C's eigenvalues).
std::array<double, 2> semi_axes(const slam::SplatFootprint & f)
{
  const double mean = 0.5 * (f.c_uu + f.c_vv);
  const double spread = std::hypot(0.5 * (f.c_uu - f.c_vv), f.c_uv);
  return {std::sqrt(std::max(0.0, mean - spread)), std::sqrt(std::max(0.0, mean + spread))};
}

TEST(IsotropicSplatFootprint, IsTheDiscOfTheGivenRadius)
{
  const auto f = slam::isotropic_splat_footprint(3.0);

  EXPECT_DOUBLE_EQ(f.half_extent_u(), 3.0);
  EXPECT_DOUBLE_EQ(f.half_extent_v(), 3.0);
  EXPECT_TRUE(f.contains(2.9, 0.0));
  EXPECT_TRUE(f.contains(0.0, -2.9));
  EXPECT_TRUE(f.contains(2.0, 2.0));   // |q| = 2.83 < 3
  EXPECT_FALSE(f.contains(2.2, 2.2));  // |q| = 3.11 > 3
  EXPECT_FALSE(f.contains(3.1, 0.0));
}

TEST(IsotropicSplatFootprint, NonPositiveRadiusCoversNothingBeyondTheCenter)
{
  for (const double radius : {0.0, -1.0}) {
    const auto f = slam::isotropic_splat_footprint(radius);
    EXPECT_DOUBLE_EQ(f.half_extent_u(), 0.0);
    EXPECT_DOUBLE_EQ(f.half_extent_v(), 0.0);
    EXPECT_FALSE(f.contains(0.5, 0.0));
    EXPECT_FALSE(f.contains(0.0, 0.5));
  }
}

TEST(SurfelSplatFootprint, FrontoParallelSurfelIsTheFocalLengthScaledDisc)
{
  // A surfel facing the camera head-on at the principal point: the classic
  // r_px = f * radius / z disc the elliptical footprint generalizes.
  const auto f =
    slam::surfel_splat_footprint({0.0, 0.0, 5.0}, {0.0, 0.0, 1.0}, 0.05, kFx, kFy, kNoClamp);

  const double expected = kFx * 0.05 / 5.0;  // 4 px
  EXPECT_NEAR(f.half_extent_u(), expected, 1e-9);
  EXPECT_NEAR(f.half_extent_v(), expected, 1e-9);
  EXPECT_NEAR(f.c_uv, 0.0, 1e-12);
}

TEST(SurfelSplatFootprint, NormalSignDoesNotChangeTheFootprint)
{
  // Neighborhood PCA leaves the normal's sign arbitrary, so the footprint must
  // depend only on n n^T.
  const std::array<double, 3> p = {0.3, -0.2, 4.0};
  const std::array<double, 3> n = {0.6, 0.0, 0.8};
  const auto up = slam::surfel_splat_footprint(p, n, 0.05, kFx, kFy, kNoClamp);
  const auto down =
    slam::surfel_splat_footprint(p, {-n[0], -n[1], -n[2]}, 0.05, kFx, kFy, kNoClamp);

  EXPECT_DOUBLE_EQ(up.c_uu, down.c_uu);
  EXPECT_DOUBLE_EQ(up.c_uv, down.c_uv);
  EXPECT_DOUBLE_EQ(up.c_vv, down.c_vv);
}

TEST(SurfelSplatFootprint, TiltCompressesAlongTheTiltDirectionByCosine)
{
  // Normal tilted 60 deg away from the view direction inside the x-z plane:
  // the disc foreshortens by |cos 60| = 0.5 along u and keeps its full extent
  // along v, which is exactly the over-culling the circular footprint caused.
  constexpr double kCos = 0.5;
  const double sin = std::sqrt(1.0 - kCos * kCos);
  const auto f =
    slam::surfel_splat_footprint({0.0, 0.0, 5.0}, {sin, 0.0, kCos}, 0.05, kFx, kFy, kNoClamp);

  const double full = kFx * 0.05 / 5.0;
  EXPECT_NEAR(f.half_extent_u(), full * kCos, 1e-9);
  EXPECT_NEAR(f.half_extent_v(), full, 1e-9);
  EXPECT_NEAR(f.c_uv, 0.0, 1e-12);

  // The pixel a circular footprint would have covered along the tilt
  // direction is now outside, while the perpendicular direction is untouched.
  EXPECT_FALSE(f.contains(0.75 * full, 0.0));
  EXPECT_TRUE(f.contains(0.0, 0.75 * full));
}

TEST(SurfelSplatFootprint, EdgeOnSurfelCollapsesToASegment)
{
  // Normal perpendicular to the view direction: the disc is seen exactly
  // edge-on and projects to a line segment across the tilt direction.
  const auto f =
    slam::surfel_splat_footprint({0.0, 0.0, 5.0}, {1.0, 0.0, 0.0}, 0.05, kFx, kFy, kNoClamp);

  const double full = kFy * 0.05 / 5.0;
  EXPECT_NEAR(f.half_extent_u(), 0.0, 1e-9);
  EXPECT_NEAR(f.half_extent_v(), full, 1e-9);
  // Degenerate C must not produce infinities: nothing off the segment is in.
  EXPECT_FALSE(f.contains(0.5, 0.0));
  EXPECT_FALSE(f.contains(0.5, 0.5));
  // And the segment stays bounded along its own direction — the shape test
  // alone accepts that whole line, so contains() must bound it itself rather
  // than leaning on the caller's bounding-box loop.
  EXPECT_TRUE(f.contains(0.0, 0.9 * full));
  EXPECT_FALSE(f.contains(0.0, 1.1 * full));
}

TEST(SurfelSplatFootprint, TiltedOffAxisFootprintIsRotatedNotAxisAligned)
{
  // Tilt inside a plane that is neither u nor v aligned: the ellipse's axes
  // must follow the projected normal, which shows up as a non-zero c_uv.
  const double axis = 1.0 / std::sqrt(2.0);
  const auto f = slam::surfel_splat_footprint(
    {0.0, 0.0, 5.0}, {0.6 * axis, 0.6 * axis, 0.8}, 0.05, kFx, kFy, kNoClamp);

  EXPECT_LT(std::abs(f.c_uv), f.c_uu);  // still a valid ellipse
  EXPECT_GT(std::abs(f.c_uv), 1e-6);    // but genuinely rotated

  // The minor axis matches |cos| = 0.8 of the fronto-parallel radius and the
  // major axis keeps that radius in full, whatever the ellipse's orientation.
  const double full = kFx * 0.05 / 5.0;
  const auto axes = semi_axes(f);
  EXPECT_NEAR(axes[0], full * 0.8, 1e-9);
  EXPECT_NEAR(axes[1], full, 1e-9);
}

TEST(SurfelSplatFootprint, OffAxisPerspectiveStretchesTheFootprint)
{
  // At 45 deg off the optical axis (x = z) a surfel facing its own view ray is
  // farther away by sqrt(2) but the pixel scale grows as sec^2 = 2, so its
  // extent along the off-axis direction ends up sqrt(2) times the on-axis
  // radius. Across that direction the depth alone sets the scale.
  const double axis = 1.0 / std::sqrt(2.0);
  const auto f =
    slam::surfel_splat_footprint({5.0, 0.0, 5.0}, {axis, 0.0, axis}, 0.05, kFx, kFy, kNoClamp);

  const double on_axis = kFx * 0.05 / 5.0;
  EXPECT_NEAR(f.half_extent_u(), on_axis * std::sqrt(2.0), 1e-9);
  EXPECT_NEAR(f.half_extent_v(), on_axis, 1e-9);
}

TEST(SurfelSplatFootprint, ZeroNormalFallsBackToTheAverageFocalLengthDisc)
{
  // The geometry pre-pass's "no normal" sentinel must reproduce the circular
  // footprint the rasterizers used before, radius f_avg * radius / z, so a
  // caller that supplies no normals is unaffected by this change.
  constexpr double kFyWide = 200.0;
  const auto f =
    slam::surfel_splat_footprint({0.4, -0.3, 5.0}, {0.0, 0.0, 0.0}, 0.05, kFx, kFyWide, kNoClamp);

  const double expected = 0.5 * (kFx + kFyWide) * 0.05 / 5.0;
  EXPECT_NEAR(f.half_extent_u(), expected, 1e-9);
  EXPECT_NEAR(f.half_extent_v(), expected, 1e-9);
  EXPECT_NEAR(f.c_uv, 0.0, 1e-12);
}

TEST(SurfelSplatFootprint, MajorAxisIsClampedWithoutReshapingTheEllipse)
{
  // An isolated point's spacing makes the footprint huge; the cap bounds the
  // major axis but must keep the surfel's aspect ratio, not square it up.
  constexpr double kCos = 0.5;
  constexpr double kMaxAxis = 4.0;
  const double sin = std::sqrt(1.0 - kCos * kCos);
  const auto f =
    slam::surfel_splat_footprint({0.0, 0.0, 1.0}, {sin, 0.0, kCos}, 2.0, kFx, kFy, kMaxAxis);

  const auto axes = semi_axes(f);
  EXPECT_NEAR(axes[1], kMaxAxis, 1e-9);
  EXPECT_NEAR(axes[0], kMaxAxis * kCos, 1e-9);
}

TEST(SurfelSplatFootprint, ZeroNormalFallbackIsClampedToo)
{
  constexpr double kMaxAxis = 4.0;
  const auto f =
    slam::surfel_splat_footprint({0.0, 0.0, 1.0}, {0.0, 0.0, 0.0}, 2.0, kFx, kFy, kMaxAxis);

  EXPECT_NEAR(f.half_extent_u(), kMaxAxis, 1e-9);
  EXPECT_NEAR(f.half_extent_v(), kMaxAxis, 1e-9);
}

TEST(SurfelSplatFootprint, DegenerateGeometryCoversNothingBeyondTheCenter)
{
  const std::array<double, 3> n = {0.0, 0.0, 1.0};
  // On the camera plane, behind the camera, and a zero-radius surfel.
  const std::array<slam::SplatFootprint, 3> degenerate = {
    slam::surfel_splat_footprint({0.0, 0.0, 0.0}, n, 0.05, kFx, kFy, kNoClamp),
    slam::surfel_splat_footprint({0.0, 0.0, -5.0}, n, 0.05, kFx, kFy, kNoClamp),
    slam::surfel_splat_footprint({0.0, 0.0, 5.0}, n, 0.0, kFx, kFy, kNoClamp)};
  for (const auto & bad : degenerate) {
    EXPECT_DOUBLE_EQ(bad.half_extent_u(), 0.0);
    EXPECT_DOUBLE_EQ(bad.half_extent_v(), 0.0);
    EXPECT_FALSE(bad.contains(0.5, 0.5));
  }
}

}  // namespace
