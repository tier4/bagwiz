// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/static_extrinsic.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace
{

using bagwiz::core::pointcloud::resolve_static_extrinsic;

geometry_msgs::msg::TransformStamped make_tf(
  const std::string & parent, const std::string & child, double tx, double ty, double tz)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = parent;
  t.header.stamp.sec = 0;
  t.header.stamp.nanosec = 0;
  t.child_frame_id = child;
  t.transform.translation.x = tx;
  t.transform.translation.y = ty;
  t.transform.translation.z = tz;
  t.transform.rotation.w = 1.0;  // identity rotation
  return t;
}

// Fill `buffer` with a static map -> base_link -> lidar chain plus a
// disconnected odom -> wheel tree.
void add_frames(tf2::BufferCore & buffer)
{
  buffer.setTransform(make_tf("map", "base_link", 10.0, 0.0, 0.0), "test", true);
  buffer.setTransform(make_tf("base_link", "lidar", 1.0, 2.0, 3.0), "test", true);
  buffer.setTransform(make_tf("odom", "wheel", 0.0, 0.0, 1.0), "test", true);
}

constexpr std::chrono::seconds kCacheTime{60};

TEST(StaticExtrinsic, ResolvesDirectChain)
{
  tf2::BufferCore buffer{kCacheTime};
  add_frames(buffer);
  const auto result = resolve_static_extrinsic(buffer, "base_link", "lidar");
  ASSERT_TRUE(result.ok()) << result.lookup_error;
  EXPECT_TRUE(result.missing.empty());
  EXPECT_DOUBLE_EQ(result.transform.transform.translation.x, 1.0);
  EXPECT_DOUBLE_EQ(result.transform.transform.translation.y, 2.0);
  EXPECT_DOUBLE_EQ(result.transform.transform.translation.z, 3.0);
  EXPECT_DOUBLE_EQ(result.transform.transform.rotation.w, 1.0);
}

TEST(StaticExtrinsic, ResolvesComposedChain)
{
  tf2::BufferCore buffer{kCacheTime};
  add_frames(buffer);
  // map <- lidar composes map <- base_link <- lidar: (10,0,0) + (1,2,3).
  const auto result = resolve_static_extrinsic(buffer, "map", "lidar");
  ASSERT_TRUE(result.ok()) << result.lookup_error;
  EXPECT_DOUBLE_EQ(result.transform.transform.translation.x, 11.0);
  EXPECT_DOUBLE_EQ(result.transform.transform.translation.y, 2.0);
  EXPECT_DOUBLE_EQ(result.transform.transform.translation.z, 3.0);
}

TEST(StaticExtrinsic, SameFrameYieldsIdentityWhenPresent)
{
  tf2::BufferCore buffer{kCacheTime};
  add_frames(buffer);
  const auto result = resolve_static_extrinsic(buffer, "lidar", "lidar");
  ASSERT_TRUE(result.ok()) << result.lookup_error;
  EXPECT_DOUBLE_EQ(result.transform.transform.translation.x, 0.0);
  EXPECT_DOUBLE_EQ(result.transform.transform.rotation.w, 1.0);
}

TEST(StaticExtrinsic, SameFrameReportedWhenAbsent)
{
  tf2::BufferCore buffer{kCacheTime};
  add_frames(buffer);
  // tf2's same-frame lookup would answer an unverified identity; the presence
  // check must fire first so an unknown frame can never masquerade as one.
  const auto result = resolve_static_extrinsic(buffer, "ghost", "ghost");
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.missing, (std::vector<std::string>{"ghost"}));
  EXPECT_TRUE(result.lookup_error.empty());
}

TEST(StaticExtrinsic, MissingFramesReportedInTargetSourceOrder)
{
  tf2::BufferCore buffer{kCacheTime};
  add_frames(buffer);
  const auto source_only = resolve_static_extrinsic(buffer, "base_link", "ghost");
  EXPECT_EQ(source_only.missing, (std::vector<std::string>{"ghost"}));
  EXPECT_TRUE(source_only.lookup_error.empty());

  const auto target_only = resolve_static_extrinsic(buffer, "ghost", "lidar");
  EXPECT_EQ(target_only.missing, (std::vector<std::string>{"ghost"}));

  const auto both = resolve_static_extrinsic(buffer, "ghost_t", "ghost_s");
  EXPECT_EQ(both.missing, (std::vector<std::string>{"ghost_t", "ghost_s"}));
}

TEST(StaticExtrinsic, DisconnectedTreesFailTheLookup)
{
  tf2::BufferCore buffer{kCacheTime};
  add_frames(buffer);
  // Both frames exist but in separate trees: no chain between them.
  const auto result = resolve_static_extrinsic(buffer, "map", "wheel");
  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(result.missing.empty());
  EXPECT_FALSE(result.lookup_error.empty());
}

}  // namespace
