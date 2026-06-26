// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/undistort.hpp"

#include "bagwiz/core/image/camera_info.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace
{

using bagwiz::core::image::CameraInfo;
using bagwiz::core::image::UndistortHelper;

CameraInfo identity_camera_info(std::uint32_t w, std::uint32_t h)
{
  CameraInfo info;
  info.width = w;
  info.height = h;
  info.distortion_model = "plumb_bob";
  info.k = {
    static_cast<double>(w),
    0.0,
    static_cast<double>(w) / 2.0,
    0.0,
    static_cast<double>(h),
    static_cast<double>(h) / 2.0,
    0.0,
    0.0,
    1.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {info.k[0], 0.0, info.k[2], 0.0, 0.0, info.k[4], info.k[5], 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}

CameraInfo distorted_camera_info(std::uint32_t w, std::uint32_t h)
{
  CameraInfo info = identity_camera_info(w, h);
  info.d = {0.2, -0.1, 0.0, 0.0, 0.0};
  return info;
}

// Some monocular publishers leave CameraInfo.r zero-filled instead of identity.
CameraInfo zero_rotation_camera_info(std::uint32_t w, std::uint32_t h)
{
  CameraInfo info = identity_camera_info(w, h);
  info.r = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  return info;
}

std::vector<std::byte> solid_bgr(
  std::uint32_t w, std::uint32_t h, std::uint8_t b, std::uint8_t g, std::uint8_t r)
{
  std::vector<std::byte> out(static_cast<std::size_t>(w) * h * 3);
  for (std::size_t i = 0; i < out.size(); i += 3) {
    out[i] = static_cast<std::byte>(b);
    out[i + 1] = static_cast<std::byte>(g);
    out[i + 2] = static_cast<std::byte>(r);
  }
  return out;
}

std::vector<std::byte> gradient_bgr(std::uint32_t w, std::uint32_t h)
{
  std::vector<std::byte> out(static_cast<std::size_t>(w) * h * 3);
  for (std::uint32_t y = 0; y < h; ++y) {
    for (std::uint32_t x = 0; x < w; ++x) {
      const std::size_t i = (static_cast<std::size_t>(y) * w + x) * 3;
      out[i] = static_cast<std::byte>((x * 16) % 256);
      out[i + 1] = static_cast<std::byte>((y * 16) % 256);
      out[i + 2] = static_cast<std::byte>(((x + y) * 8) % 256);
    }
  }
  return out;
}

TEST(UndistortHelper, ZeroDistortionPreservesCenterColor)
{
  constexpr std::uint32_t kW = 16;
  constexpr std::uint32_t kH = 16;
  const auto info = identity_camera_info(kW, kH);
  UndistortHelper helper(info, kW, kH);
  const auto input = solid_bgr(kW, kH, 100, 150, 200);
  const auto output = helper.remap(input, kW * 3);
  ASSERT_EQ(output.size(), input.size());
  const std::size_t center = (kH / 2 * kW + kW / 2) * 3;
  EXPECT_EQ(output[center], input[center]);
  EXPECT_EQ(output[center + 1], input[center + 1]);
  EXPECT_EQ(output[center + 2], input[center + 2]);
}

TEST(UndistortHelper, ZeroRectificationMatrixFallsBackToIdentity)
{
  constexpr std::uint32_t kW = 16;
  constexpr std::uint32_t kH = 16;
  const auto info = zero_rotation_camera_info(kW, kH);
  UndistortHelper helper(info, kW, kH);
  const auto input = solid_bgr(kW, kH, 100, 150, 200);
  const auto output = helper.remap(input, kW * 3);
  ASSERT_EQ(output.size(), input.size());
  // A zero (unset) rectification matrix must be treated as identity, matching
  // tier4_perception_dataset. Without the guard, initUndistortRectifyMap emits
  // NaN maps and remap fills the whole image with the border color (black).
  const std::size_t center = (kH / 2 * kW + kW / 2) * 3;
  EXPECT_EQ(output[center], input[center]);
  EXPECT_EQ(output[center + 1], input[center + 1]);
  EXPECT_EQ(output[center + 2], input[center + 2]);
}

TEST(UndistortHelper, NonZeroDistortionChangesPixels)
{
  constexpr std::uint32_t kW = 16;
  constexpr std::uint32_t kH = 16;
  const auto info = distorted_camera_info(kW, kH);
  UndistortHelper helper(info, kW, kH);
  const auto input = gradient_bgr(kW, kH);
  const auto output = helper.remap(input, kW * 3);
  ASSERT_EQ(output.size(), input.size());
  EXPECT_FALSE(std::equal(output.begin(), output.end(), input.begin()));
  // At least one interior pixel should change (not just the border).
  bool interior_changed = false;
  for (std::uint32_t y = 1; y < kH - 1 && !interior_changed; ++y) {
    for (std::uint32_t x = 1; x < kW - 1; ++x) {
      const std::size_t idx = (static_cast<std::size_t>(y) * kW + x) * 3;
      if (output[idx] != input[idx]) {
        interior_changed = true;
        break;
      }
    }
  }
  EXPECT_TRUE(interior_changed);
}

TEST(UndistortHelper, ScalesToDifferentSize)
{
  constexpr std::uint32_t kSrcW = 16;
  constexpr std::uint32_t kSrcH = 16;
  constexpr std::uint32_t kDstW = 8;
  constexpr std::uint32_t kDstH = 8;
  const auto info = identity_camera_info(kSrcW, kSrcH);
  UndistortHelper helper(info, kDstW, kDstH);
  const auto input = solid_bgr(kSrcW, kSrcH, 50, 100, 150);
  const auto output = helper.remap(input, kSrcW * 3);
  EXPECT_EQ(output.size(), static_cast<std::size_t>(kDstW) * kDstH * 3);
}

}  // namespace
