// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/srgb.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

namespace image = bagwiz::core::image;

TEST(Srgb, DecodeAnchors)
{
  EXPECT_DOUBLE_EQ(image::srgb_u8_to_linear(0), 0.0);
  EXPECT_DOUBLE_EQ(image::srgb_u8_to_linear(255), 1.0);
  // Below the 0.04045 knee the decode is the linear toe c / 12.92.
  EXPECT_DOUBLE_EQ(image::srgb_u8_to_linear(10), (10.0 / 255.0) / 12.92);
  // Middle gray: the power segment, ((c + 0.055) / 1.055)^2.4.
  EXPECT_NEAR(image::srgb_u8_to_linear(128), 0.215861, 1e-6);
}

TEST(Srgb, DecodeIsStrictlyIncreasing)
{
  for (int v = 1; v < 256; ++v) {
    EXPECT_LT(
      image::srgb_u8_to_linear(static_cast<std::uint8_t>(v - 1)),
      image::srgb_u8_to_linear(static_cast<std::uint8_t>(v)))
      << "decode must be strictly monotone at " << v;
  }
}

TEST(Srgb, RoundTripIsLosslessForAll256Values)
{
  for (int v = 0; v < 256; ++v) {
    const auto u8 = static_cast<std::uint8_t>(v);
    EXPECT_EQ(image::linear_to_srgb_u8(image::srgb_u8_to_linear(u8)), u8);
  }
}

TEST(Srgb, EncodeMatchesTheAnalyticRound)
{
  // Half linear light is not half sRGB: round(encode(0.5) * 255) = 188.
  EXPECT_EQ(image::linear_to_srgb_u8(0.5), 188);
  // Sweep a dense grid and compare against the continuous encode + round.
  for (int i = 0; i <= 10'000; ++i) {
    const double linear = static_cast<double>(i) / 10'000.0;
    const double srgb =
      linear <= 0.0031308 ? 12.92 * linear : 1.055 * std::pow(linear, 1.0 / 2.4) - 0.055;
    const auto expected = static_cast<std::uint8_t>(std::lround(srgb * 255.0));
    EXPECT_EQ(image::linear_to_srgb_u8(linear), expected) << "at linear " << linear;
  }
}

TEST(Srgb, EncodeClampsOutOfRangeInputs)
{
  EXPECT_EQ(image::linear_to_srgb_u8(-0.25), 0);
  EXPECT_EQ(image::linear_to_srgb_u8(0.0), 0);
  EXPECT_EQ(image::linear_to_srgb_u8(1.0), 255);
  EXPECT_EQ(image::linear_to_srgb_u8(4.0), 255);
  EXPECT_EQ(image::linear_to_srgb_u8(std::numeric_limits<double>::quiet_NaN()), 0);
  EXPECT_EQ(image::linear_to_srgb_u8(std::numeric_limits<double>::infinity()), 255);
}
