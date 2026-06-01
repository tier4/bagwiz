// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf_walk_timeline.hpp"

#include <tf2/buffer_core.hpp>
#include <tf2/time.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace
{

tf2::TimePoint t_at(std::int32_t sec)
{
  return tf2::TimePoint(std::chrono::seconds(sec));
}

// Identity-rotation transform parent->child translated by `x` along X, stamped
// at `sec`. Mirrors the make_tf helper used in tf_chain_test.
geometry_msgs::msg::TransformStamped make_tf(
  const std::string & parent, const std::string & child, std::int32_t sec, double x)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = parent;
  t.header.stamp.sec = sec;
  t.header.stamp.nanosec = 0;
  t.child_frame_id = child;
  t.transform.translation.x = x;
  t.transform.rotation.w = 1.0;
  return t;
}

// ---------------------------------------------------------------------------
// build_tf_walk_timeline
// ---------------------------------------------------------------------------

TEST(BuildTfWalkTimeline, SortsAscendingAndDeduplicates)
{
  const auto out =
    bagwiz::core::build_tf_walk_timeline({t_at(3), t_at(1), t_at(2), t_at(1), t_at(3), t_at(2)});

  ASSERT_EQ(out.size(), 3u);
  EXPECT_EQ(out[0], t_at(1));
  EXPECT_EQ(out[1], t_at(2));
  EXPECT_EQ(out[2], t_at(3));
}

TEST(BuildTfWalkTimeline, EmptyInputYieldsEmptyTimeline)
{
  EXPECT_TRUE(bagwiz::core::build_tf_walk_timeline({}).empty());
}

// ---------------------------------------------------------------------------
// resolve_tf_walk_step
// ---------------------------------------------------------------------------

// A single static hop base_link -> lidar (+1 along X). Resolving
// from=base_link, to=lidar yields base_link's origin in the lidar frame, which
// for a +1 child offset is -1 (the tf2_echo convention).
TEST(ResolveTfWalkStep, AppliesTf2EchoDirectionConvention)
{
  tf2::BufferCore buffer{std::chrono::seconds(120)};
  buffer.setTransform(make_tf("base_link", "lidar", 0, 1.0), "test", /*is_static=*/true);

  const auto step = bagwiz::core::resolve_tf_walk_step(buffer, t_at(5), "base_link", "lidar");

  ASSERT_TRUE(step.transform.has_value());
  EXPECT_TRUE(step.error.empty());
  EXPECT_DOUBLE_EQ(step.transform->transform.translation.x, -1.0);
}

// The walk does not classify static vs dynamic: a chain mixing a static leg
// (base_link -> lidar) and a dynamic leg (map -> base_link) must resolve when
// queried at the dynamic leg's stamp.
TEST(ResolveTfWalkStep, ResolvesAcrossMergedStaticAndDynamicLegs)
{
  tf2::BufferCore buffer{std::chrono::seconds(120)};
  buffer.setTransform(make_tf("base_link", "lidar", 0, 1.0), "test", /*is_static=*/true);
  buffer.setTransform(make_tf("map", "base_link", 5, 2.0), "test", /*is_static=*/false);

  const auto step = bagwiz::core::resolve_tf_walk_step(buffer, t_at(5), "map", "lidar");

  ASSERT_TRUE(step.transform.has_value());
  EXPECT_TRUE(step.error.empty());
}

// An unconnected target frame yields no transform and a populated error, rather
// than throwing, so the walk can render a warning and carry on.
TEST(ResolveTfWalkStep, ReportsUnresolvedFramesWithoutThrowing)
{
  tf2::BufferCore buffer{std::chrono::seconds(120)};
  buffer.setTransform(make_tf("a", "b", 5, 1.0), "test", /*is_static=*/false);

  const auto step = bagwiz::core::resolve_tf_walk_step(buffer, t_at(5), "a", "z");

  EXPECT_FALSE(step.transform.has_value());
  EXPECT_FALSE(step.error.empty());
  EXPECT_EQ(step.time, t_at(5));
}

}  // namespace
