// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/camera_info.hpp"

#include <gtest/gtest.h>

#include <array>
#include <vector>

namespace
{

using bagwiz::core::image::CameraInfo;
using bagwiz::core::image::scale_camera_info;

CameraInfo make_test_camera_info()
{
  CameraInfo info;
  info.width = 1920;
  info.height = 1080;
  info.frame_id = "cam";
  info.distortion_model = "plumb_bob";
  info.d = {0.1, -0.2, 0.0, 0.0, 0.0};
  // K = [[fx, skew, cx], [0, fy, cy], [0, 0, 1]]
  info.k = {1000.0, 0.5, 960.0, 0.0, 1000.0, 540.0, 0.0, 0.0, 1.0};
  // R = identity
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  // P = [[fx', s', cx', tx], [0, fy', cy', ty], [0, 0, 1, tz]]
  info.p = {1000.0, 0.5, 960.0, -50.0, 0.0, 1000.0, 540.0, 0.0, 0.0, 0.0, 1.0, -0.25};
  return info;
}

TEST(CameraInfoScale, ScalesPixelCoordinateEntries)
{
  const auto info = make_test_camera_info();
  const auto scaled = scale_camera_info(info, 0.5);

  // Focal lengths and principal point scale.
  EXPECT_DOUBLE_EQ(scaled.k[0], 500.0);
  EXPECT_DOUBLE_EQ(scaled.k[1], 0.25);   // skew
  EXPECT_DOUBLE_EQ(scaled.k[2], 480.0);  // cx
  EXPECT_DOUBLE_EQ(scaled.k[4], 500.0);  // fy
  EXPECT_DOUBLE_EQ(scaled.k[5], 270.0);  // cy

  // Pixel-coordinate entries of P scale, including stereo baseline terms.
  EXPECT_DOUBLE_EQ(scaled.p[0], 500.0);
  EXPECT_DOUBLE_EQ(scaled.p[1], 0.25);
  EXPECT_DOUBLE_EQ(scaled.p[2], 480.0);
  EXPECT_DOUBLE_EQ(scaled.p[3], -25.0);  // tx scales with focal length
  EXPECT_DOUBLE_EQ(scaled.p[4], 0.0);
  EXPECT_DOUBLE_EQ(scaled.p[5], 500.0);
  EXPECT_DOUBLE_EQ(scaled.p[6], 270.0);
  EXPECT_DOUBLE_EQ(scaled.p[7], 0.0);
}

TEST(CameraInfoScale, PreservesHomogeneousAndDepthEntries)
{
  const auto info = make_test_camera_info();
  const auto scaled = scale_camera_info(info, 0.5);

  // Homogeneous coordinate must stay 1.
  EXPECT_DOUBLE_EQ(scaled.k[8], 1.0);
  EXPECT_DOUBLE_EQ(scaled.p[10], 1.0);

  // Off-diagonal zeros in K and P stay zero.
  EXPECT_DOUBLE_EQ(scaled.k[3], 0.0);
  EXPECT_DOUBLE_EQ(scaled.k[6], 0.0);
  EXPECT_DOUBLE_EQ(scaled.k[7], 0.0);
  EXPECT_DOUBLE_EQ(scaled.p[8], 0.0);
  EXPECT_DOUBLE_EQ(scaled.p[9], 0.0);

  // Depth translation tz is in meters and must not scale.
  EXPECT_DOUBLE_EQ(scaled.p[11], -0.25);

  // Distortion coefficients and rectification matrix are unchanged.
  EXPECT_EQ(scaled.d, info.d);
  EXPECT_EQ(scaled.r, info.r);
}

TEST(CameraInfoScale, IdentityScaleLeavesInfoUnchanged)
{
  const auto info = make_test_camera_info();
  const auto scaled = scale_camera_info(info, 1.0);

  EXPECT_EQ(scaled.k, info.k);
  EXPECT_EQ(scaled.p, info.p);
  EXPECT_EQ(scaled.d, info.d);
  EXPECT_EQ(scaled.r, info.r);
  EXPECT_EQ(scaled.width, info.width);
  EXPECT_EQ(scaled.height, info.height);
  EXPECT_EQ(scaled.frame_id, info.frame_id);
}

TEST(CameraInfoScale, IndependentScales)
{
  const auto info = make_test_camera_info();
  const auto scaled = scale_camera_info(info, 0.5, 0.25);

  EXPECT_DOUBLE_EQ(scaled.k[0], 500.0);  // fx * 0.5
  EXPECT_DOUBLE_EQ(scaled.k[1], 0.25);   // skew * 0.5
  EXPECT_DOUBLE_EQ(scaled.k[2], 480.0);  // cx * 0.5
  EXPECT_DOUBLE_EQ(scaled.k[4], 250.0);  // fy * 0.25
  EXPECT_DOUBLE_EQ(scaled.k[5], 135.0);  // cy * 0.25

  EXPECT_DOUBLE_EQ(scaled.p[0], 500.0);
  EXPECT_DOUBLE_EQ(scaled.p[1], 0.25);
  EXPECT_DOUBLE_EQ(scaled.p[2], 480.0);
  EXPECT_DOUBLE_EQ(scaled.p[3], -25.0);  // tx * 0.5
  EXPECT_DOUBLE_EQ(scaled.p[5], 250.0);
  EXPECT_DOUBLE_EQ(scaled.p[6], 135.0);
  EXPECT_DOUBLE_EQ(scaled.p[7], 0.0);  // ty * 0.25

  EXPECT_DOUBLE_EQ(scaled.k[8], 1.0);
  EXPECT_DOUBLE_EQ(scaled.p[10], 1.0);
  EXPECT_DOUBLE_EQ(scaled.p[11], -0.25);  // tz unchanged
}

}  // namespace
