// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/video/frame_rate.hpp"

#include <gtest/gtest.h>

namespace
{
using bagwiz::core::video::derive_frame_rate;
using bagwiz::core::video::FrameRate;
using bagwiz::core::video::kDefaultFps;
using bagwiz::core::video::kMaxFps;

double as_fps(FrameRate r)
{
  return static_cast<double>(r.num) / static_cast<double>(r.den);
}

TEST(FrameRateTest, AverageSpacingOverSpan)
{
  // 11 frames spanning exactly 1.0s -> 10 intervals -> 10 fps.
  const auto r = derive_frame_rate(0, 1'000'000'000LL, 11);
  EXPECT_NEAR(as_fps(r), 10.0, 1e-6);
}

TEST(FrameRateTest, NonIntegerRateRoundedToMilliFps)
{
  // 50 frames over 5s -> 49 intervals / 5s = 9.8 fps.
  const auto r = derive_frame_rate(0, 5'000'000'000LL, 50);
  EXPECT_NEAR(as_fps(r), 9.8, 1e-3);
  EXPECT_GT(r.den, 0);
}

TEST(FrameRateTest, SingleFrameFallsBackToDefault)
{
  const auto r = derive_frame_rate(1'000, 1'000, 1);
  EXPECT_NEAR(as_fps(r), static_cast<double>(kDefaultFps), 1e-9);
}

TEST(FrameRateTest, ZeroSpanFallsBackToDefault)
{
  const auto r = derive_frame_rate(5'000, 5'000, 10);  // all messages share a timestamp
  EXPECT_NEAR(as_fps(r), static_cast<double>(kDefaultFps), 1e-9);
}

TEST(FrameRateTest, ClampsAbsurdlyHighRate)
{
  // 1000 frames within 1 microsecond -> clamp to kMaxFps.
  const auto r = derive_frame_rate(0, 1'000LL, 1000);
  EXPECT_LE(as_fps(r), static_cast<double>(kMaxFps) + 1e-6);
}

TEST(FrameRateTest, PathologicalCountDoesNotOverflow)
{
  // A huge count over a 1 ns span yields an fps so large that lround(fps*1000)
  // would overflow LONG_MAX (UB) without the pre-round clamp. The result must
  // still be a finite, in-range rational.
  const auto r = derive_frame_rate(0, 1, 10'000'000'000ULL);
  EXPECT_GT(r.den, 0);
  EXPECT_GE(as_fps(r), static_cast<double>(bagwiz::core::video::kMinFps));
  EXPECT_LE(as_fps(r), static_cast<double>(kMaxFps) + 1e-6);
}

}  // namespace
