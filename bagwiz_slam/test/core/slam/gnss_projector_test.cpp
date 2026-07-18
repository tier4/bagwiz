// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/gnss_projector.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>

// Unit test for GnssProjector: WGS84 lat/lon/alt -> local ENU meters around the
// first projected fix. Verifies the origin latches on the first call (yielding
// the zero vector) and that the East/North/Up axes carry the expected sign and
// magnitude. Links GeographicLib (via bagwiz_core); no GLIM involved.
namespace
{
namespace slam = bagwiz::core::slam;

TEST(GnssProjector, FirstFixIsOriginThenRelativeEnu)
{
  slam::GnssProjector projector;
  EXPECT_FALSE(projector.has_origin());

  // First fix becomes the ENU origin -> the zero vector.
  const std::array<double, 3> origin = projector.project(35.0, 139.0, 0.0);
  EXPECT_TRUE(projector.has_origin());
  EXPECT_NEAR(origin[0], 0.0, 1e-6);
  EXPECT_NEAR(origin[1], 0.0, 1e-6);
  EXPECT_NEAR(origin[2], 0.0, 1e-6);

  // ~0.001 deg north: x (East) ~ 0, y (North) ~ 111 m, z ~ 0.
  const std::array<double, 3> north = projector.project(35.001, 139.0, 0.0);
  EXPECT_NEAR(north[0], 0.0, 1.0);
  EXPECT_GT(north[1], 100.0);
  EXPECT_LT(north[1], 120.0);
  EXPECT_NEAR(north[2], 0.0, 1.0);

  // ~0.001 deg east at lat 35: x (East) ~ cos(35) * 111 m ~ 91 m, y (North) ~ 0.
  const std::array<double, 3> east = projector.project(35.0, 139.001, 0.0);
  EXPECT_GT(east[0], 80.0);
  EXPECT_LT(east[0], 100.0);
  EXPECT_NEAR(east[1], 0.0, 1.0);

  // +10 m altitude maps to +z (Up).
  const std::array<double, 3> up = projector.project(35.0, 139.0, 10.0);
  EXPECT_NEAR(up[0], 0.0, 1e-3);
  EXPECT_NEAR(up[1], 0.0, 1e-3);
  EXPECT_NEAR(up[2], 10.0, 1e-3);
}

}  // namespace
