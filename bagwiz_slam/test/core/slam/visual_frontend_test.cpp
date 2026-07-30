// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/visual_frontend.hpp"

#include "bagwiz/core/image/camera_info.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace
{
namespace slam = bagwiz::core::slam;
namespace image = bagwiz::core::image;

image::CameraInfo make_pinhole(std::uint32_t w = 640, std::uint32_t h = 480)
{
  image::CameraInfo info;
  info.width = w;
  info.height = h;
  info.distortion_model = "plumb_bob";
  info.d = {0, 0, 0, 0, 0};
  info.k = {500, 0, 320, 0, 500, 240, 0, 0, 1};
  return info;
}

std::vector<std::byte> black(std::uint32_t w, std::uint32_t h)
{
  return std::vector<std::byte>(static_cast<std::size_t>(w) * h * 3, std::byte{0});
}

TEST(VisualFrontend, BlackFrameYieldsNoTracks)
{
  slam::VisualFrontendConfig cfg;
  cfg.camera = make_pinhole();
  slam::VisualFrontend fe(cfg);
  const auto frame = black(640, 480);
  EXPECT_TRUE(fe.track(0, frame, 640, 480).empty());
}

TEST(VisualFrontend, TwoConsecutiveBlackFramesStillYieldNoTracks)
{
  slam::VisualFrontendConfig cfg;
  cfg.camera = make_pinhole();
  slam::VisualFrontend fe(cfg);
  const auto frame = black(640, 480);
  EXPECT_TRUE(fe.track(0, frame, 640, 480).empty());
  EXPECT_TRUE(fe.track(100'000'000, frame, 640, 480).empty());
}

TEST(VisualFrontend, MismatchedRasterSizeYieldsNoTracks)
{
  slam::VisualFrontendConfig cfg;
  cfg.camera = make_pinhole();
  slam::VisualFrontend fe(cfg);
  const auto frame = black(640, 480);
  // frame is sized for 640x480; claiming 320x240 makes width*height*3 not
  // match frame.size().
  EXPECT_TRUE(fe.track(0, frame, 320, 240).empty());
}

TEST(VisualFrontend, IsMoveConstructible)
{
  slam::VisualFrontendConfig cfg;
  cfg.camera = make_pinhole();
  slam::VisualFrontend fe(cfg);
  slam::VisualFrontend moved(std::move(fe));
  const auto frame = black(640, 480);
  EXPECT_TRUE(moved.track(0, frame, 640, 480).empty());
}

TEST(VisualFrontend, IsMoveAssignable)
{
  slam::VisualFrontendConfig cfg;
  cfg.camera = make_pinhole();
  slam::VisualFrontend fe(cfg);
  slam::VisualFrontend other(cfg);
  other = std::move(fe);
  const auto frame = black(640, 480);
  EXPECT_TRUE(other.track(0, frame, 640, 480).empty());
}

}  // namespace
