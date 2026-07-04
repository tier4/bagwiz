// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/overlay.hpp"

#include "bagwiz/core/image/packed_raster.hpp"
#include "bagwiz/core/pointcloud/color_scheme.hpp"
#include "bagwiz/core/pointcloud/projector.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

using bagwiz::core::image::PackedRaster;
using bagwiz::core::pointcloud::ColorScheme;
using bagwiz::core::pointcloud::overlay_projected_points;
using bagwiz::core::pointcloud::ProjectedPoint;

namespace
{

// A solid width x height packed-BGR image filled with (b, g, r).
PackedRaster solid_raster(
  std::uint32_t width, std::uint32_t height, std::uint8_t b, std::uint8_t g, std::uint8_t r)
{
  PackedRaster raster;
  raster.width = width;
  raster.height = height;
  raster.encoding = "bgr8";
  const std::size_t pixels = static_cast<std::size_t>(width) * height;
  raster.bgr.resize(pixels * 3U);
  for (std::size_t i = 0; i < pixels; ++i) {
    raster.bgr[i * 3U + 0U] = std::byte{b};
    raster.bgr[i * 3U + 1U] = std::byte{g};
    raster.bgr[i * 3U + 2U] = std::byte{r};
  }
  return raster;
}

int channel_at(const PackedRaster & raster, int x, int y, int c)
{
  const std::size_t idx =
    (static_cast<std::size_t>(y) * raster.width + static_cast<std::size_t>(x)) * 3U +
    static_cast<std::size_t>(c);
  return std::to_integer<int>(raster.bgr[idx]);
}

}  // namespace

// Regression: lowering alpha must fade only the points, never the background.
// The buggy implementation alpha-blended the whole frame against an all-black
// overlay, so every point-free pixel was scaled by (1 - alpha) and the entire
// image darkened as alpha dropped.
TEST(PointCloudOverlay, PartialAlphaLeavesBackgroundPixelsUntouched)
{
  constexpr std::uint8_t kB = 100;
  constexpr std::uint8_t kG = 120;
  constexpr std::uint8_t kR = 140;
  const PackedRaster src = solid_raster(16, 16, kB, kG, kR);

  // A single point near the center; every other pixel is background.
  const std::vector<ProjectedPoint> points{ProjectedPoint{8, 8, 1.0F, 0.5F}};

  PackedRaster out;
  const std::string err = overlay_projected_points(
    src, points, /*property_min=*/0.0, /*property_max=*/1.0, ColorScheme::kJet,
    /*point_size=*/3, /*alpha=*/0.4F, out);
  ASSERT_TRUE(err.empty()) << err;
  ASSERT_EQ(out.width, src.width);
  ASSERT_EQ(out.height, src.height);

  // A corner far from the point is pure background: it must be byte-for-byte
  // identical to the source.
  EXPECT_EQ(channel_at(out, 0, 0, 0), kB);
  EXPECT_EQ(channel_at(out, 0, 0, 1), kG);
  EXPECT_EQ(channel_at(out, 0, 0, 2), kR);

  // The point itself must still be drawn (blended away from the background).
  const bool center_changed = channel_at(out, 8, 8, 0) != kB || channel_at(out, 8, 8, 1) != kG ||
                              channel_at(out, 8, 8, 2) != kR;
  EXPECT_TRUE(center_changed);
}

// Sanity: the opaque fast path (alpha == 1) also leaves the background intact.
TEST(PointCloudOverlay, OpaqueAlphaLeavesBackgroundPixelsUntouched)
{
  constexpr std::uint8_t kB = 10;
  constexpr std::uint8_t kG = 20;
  constexpr std::uint8_t kR = 30;
  const PackedRaster src = solid_raster(16, 16, kB, kG, kR);
  const std::vector<ProjectedPoint> points{ProjectedPoint{8, 8, 1.0F, 0.5F}};

  PackedRaster out;
  const std::string err =
    overlay_projected_points(src, points, 0.0, 1.0, ColorScheme::kJet, 3, 1.0F, out);
  ASSERT_TRUE(err.empty()) << err;
  EXPECT_EQ(channel_at(out, 0, 0, 0), kB);
  EXPECT_EQ(channel_at(out, 0, 0, 1), kG);
  EXPECT_EQ(channel_at(out, 0, 0, 2), kR);
}
