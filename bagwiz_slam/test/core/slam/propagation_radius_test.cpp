// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/propagation_radius.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

namespace
{
namespace slam = bagwiz::core::slam;

TEST(PropagationRadius, EmptySpacingsYieldNoRadius)
{
  EXPECT_EQ(slam::propagation_radius_from_spacings({}), std::nullopt);
}

TEST(PropagationRadius, SingleSpacingFollowsTheData)
{
  const std::vector<float> spacings = {0.1F};
  const auto radius = slam::propagation_radius_from_spacings(spacings);
  ASSERT_TRUE(radius.has_value());
  EXPECT_DOUBLE_EQ(*radius, 4.0 * static_cast<double>(0.1F));
}

TEST(PropagationRadius, OddCountPicksTheMiddleValue)
{
  const std::vector<float> spacings = {0.3F, 0.1F, 0.2F};
  const auto radius = slam::propagation_radius_from_spacings(spacings);
  ASSERT_TRUE(radius.has_value());
  EXPECT_DOUBLE_EQ(*radius, 4.0 * static_cast<double>(0.2F));
}

TEST(PropagationRadius, EvenCountPicksTheUpperMiddleValue)
{
  // nth_element at size / 2 selects the upper of the two middle values.
  const std::vector<float> spacings = {0.1F, 0.4F, 0.2F, 0.3F};
  const auto radius = slam::propagation_radius_from_spacings(spacings);
  ASSERT_TRUE(radius.has_value());
  EXPECT_DOUBLE_EQ(*radius, 4.0 * static_cast<double>(0.3F));
}

TEST(PropagationRadius, ZeroMedianYieldsNoRadius)
{
  const std::vector<float> spacings = {0.0F};
  EXPECT_EQ(slam::propagation_radius_from_spacings(spacings), std::nullopt);
}

TEST(PropagationRadius, NegativeMedianYieldsNoRadius)
{
  const std::vector<float> spacings = {-0.5F};
  EXPECT_EQ(slam::propagation_radius_from_spacings(spacings), std::nullopt);
}

TEST(PropagationRadius, NanMedianYieldsNoRadius)
{
  const std::vector<float> spacings = {std::numeric_limits<float>::quiet_NaN()};
  EXPECT_EQ(slam::propagation_radius_from_spacings(spacings), std::nullopt);
}

TEST(PropagationRadius, InfiniteMedianYieldsNoRadius)
{
  const std::vector<float> spacings = {std::numeric_limits<float>::infinity()};
  EXPECT_EQ(slam::propagation_radius_from_spacings(spacings), std::nullopt);
}

TEST(PropagationRadius, TinyMedianClampsToTheLowerBound)
{
  const std::vector<float> spacings = {0.001F};  // 4x = 0.004 < 0.05
  const auto radius = slam::propagation_radius_from_spacings(spacings);
  ASSERT_TRUE(radius.has_value());
  EXPECT_DOUBLE_EQ(*radius, 0.05);
}

TEST(PropagationRadius, HugeMedianClampsToTheUpperBound)
{
  const std::vector<float> spacings = {10.0F};  // 4x = 40 > 5
  const auto radius = slam::propagation_radius_from_spacings(spacings);
  ASSERT_TRUE(radius.has_value());
  EXPECT_DOUBLE_EQ(*radius, 5.0);
}

}  // namespace
