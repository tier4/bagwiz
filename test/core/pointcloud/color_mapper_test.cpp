// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/color_mapper.hpp"

#include "bagwiz/core/pointcloud/color_scheme.hpp"

#include <gtest/gtest.h>

using bagwiz::core::pointcloud::BgrColor;
using bagwiz::core::pointcloud::ColorMapper;
using bagwiz::core::pointcloud::ColorScheme;

TEST(ColorMapper, MapsMinMaxToEnds)
{
  ColorMapper mapper(ColorScheme::kViridis);
  const auto cmin = mapper.map(0.0, 0.0, 1.0);
  const auto cmax = mapper.map(1.0, 0.0, 1.0);
  const auto mid_color = mapper.map(0.5, 0.0, 1.0);
  EXPECT_NE(cmin, cmax);
  EXPECT_NE(cmin, mid_color);
}

TEST(ColorMapper, ClampsOutOfRange)
{
  ColorMapper mapper(ColorScheme::kJet);
  const auto below = mapper.map(-10.0, 0.0, 1.0);
  const auto above = mapper.map(10.0, 0.0, 1.0);
  const auto min = mapper.map(0.0, 0.0, 1.0);
  const auto max = mapper.map(1.0, 0.0, 1.0);
  EXPECT_EQ(below, min);
  EXPECT_EQ(above, max);
}

TEST(ColorMapper, MapsMidpointToExpectedIndex)
{
  ColorMapper mapper(ColorScheme::kViridis);
  const auto actual = mapper.map(0.5, 0.0, 1.0);
  // Viridis LUT index 128 in BGR order.
  const BgrColor expected{140, 144, 32};
  EXPECT_EQ(actual, expected);
}

TEST(ColorMapper, HandlesDegenerateRange)
{
  ColorMapper mapper(ColorScheme::kViridis);
  const auto degenerate = mapper.map(5.0, 1.0, 1.0);
  const auto min_value = mapper.map(0.0, 0.0, 1.0);
  EXPECT_EQ(degenerate, min_value);
}

TEST(ColorMapper, ViridisStartIsDark)
{
  ColorMapper mapper(ColorScheme::kViridis);
  const auto c = mapper.map(0.0, 0.0, 1.0);
  EXPECT_GT(c[0], c[2]);  // B > R
  EXPECT_GT(c[0], c[1]);  // B > G
}
