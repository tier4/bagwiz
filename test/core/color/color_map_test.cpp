// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/color/color_map.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>

namespace
{
using bagwiz::core::color::apply;
using bagwiz::core::color::ColorMapName;
using bagwiz::core::color::make_color_map;
using bagwiz::core::color::normalize;

TEST(ColorMapTest, JetMapsZeroToBlue)
{
  const auto map = make_color_map(ColorMapName::kJet);
  const auto blue = apply(map, 0);
  EXPECT_GT(blue.b, blue.r);
  EXPECT_GT(blue.b, blue.g);
}

TEST(ColorMapTest, NormalizesRange)
{
  EXPECT_EQ(normalize(0.0f, 0.0f, 10.0f), 0);
  EXPECT_EQ(normalize(10.0f, 0.0f, 10.0f), 255);
  EXPECT_EQ(normalize(5.0f, 0.0f, 10.0f), 127);
}

TEST(ColorMapTest, GrayscaleIsMonotonic)
{
  const auto map = make_color_map(ColorMapName::kGrayscale);
  const auto black = apply(map, 0);
  const auto white = apply(map, 255);
  EXPECT_EQ(black.r, 0);
  EXPECT_EQ(white.r, 255);
  EXPECT_EQ(black.r, black.g);
  EXPECT_EQ(black.r, black.b);

  for (std::size_t i = 0; i < 255; ++i) {
    const auto lower = apply(map, static_cast<std::uint8_t>(i));
    const auto upper = apply(map, static_cast<std::uint8_t>(i + 1));
    EXPECT_LE(lower.r, upper.r);
  }
}

}  // namespace
