// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/color_mapper.hpp"

#include "bagwiz/commands/generate_video.hpp"

#include <gtest/gtest.h>

using bagwiz::commands::ColorScheme;
using bagwiz::core::pointcloud::ColorMapper;

TEST(ColorMapper, MapsMinMaxToEnds)
{
  ColorMapper mapper(ColorScheme::kViridis);
  const auto cmin = mapper.map(0.0, 0.0, 1.0);
  const auto cmax = mapper.map(1.0, 0.0, 1.0);
  const auto cmid = mapper.map(0.5, 0.0, 1.0);
  EXPECT_NE(cmin, cmax);
  EXPECT_NE(cmin, cmid);
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
