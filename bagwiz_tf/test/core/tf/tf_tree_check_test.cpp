// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf/tf_tree_check.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace
{

using bagwiz::core::validate_tf_tree;

geometry_msgs::msg::TransformStamped make_edge(
  const std::string & parent, const std::string & child)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = parent;
  ts.child_frame_id = child;
  ts.transform.rotation.w = 1.0;
  return ts;
}

std::optional<std::string> check(
  const std::vector<geometry_msgs::msg::TransformStamped> & transforms)
{
  return validate_tf_tree(
    std::span<const geometry_msgs::msg::TransformStamped>(transforms.data(), transforms.size()),
    "in 'rig.yaml'");
}

TEST(TfTreeCheckTest, AcceptsAChain)
{
  const auto err = check(
    {make_edge("base_link", "drs_base_link"), make_edge("drs_base_link", "lidar"),
     make_edge("lidar", "camera")});
  EXPECT_FALSE(err.has_value()) << *err;
}

// Several roots is legal in ROS: you simply cannot transform between the trees.
// Frames are only ever resolved within their own tree.
TEST(TfTreeCheckTest, AcceptsAForest)
{
  const auto err = check({make_edge("map_a", "a"), make_edge("map_b", "b")});
  EXPECT_FALSE(err.has_value()) << *err;
}

TEST(TfTreeCheckTest, RejectsAnEmptySet)
{
  const auto err = check({});
  ASSERT_TRUE(err.has_value());
  EXPECT_NE(err->find("no transforms"), std::string::npos) << *err;
}

// The gap this module exists to close: tf2 silently DROPS a transform carrying a
// non-finite value (logging TF_NAN_INPUT), so without this check a bag can hold a
// well-formed /tf_static whose tree is empty the moment anything uses it. `.nan`
// is a valid YAML float, so it reaches this point happily.
TEST(TfTreeCheckTest, RejectsANonFiniteTranslation)
{
  auto edge = make_edge("base_link", "lidar");
  edge.transform.translation.x = std::nan("");
  const auto err = check({edge});
  ASSERT_TRUE(err.has_value());
  EXPECT_NE(err->find("non-finite translation.x"), std::string::npos) << *err;
  EXPECT_NE(err->find("'base_link' -> 'lidar'"), std::string::npos) << *err;
}

TEST(TfTreeCheckTest, RejectsANonFiniteRotation)
{
  auto edge = make_edge("base_link", "lidar");
  edge.transform.rotation.z = std::numeric_limits<double>::infinity();
  const auto err = check({edge});
  ASSERT_TRUE(err.has_value());
  EXPECT_NE(err->find("non-finite rotation.z"), std::string::npos) << *err;
}

// A default-initialised message has an all-zero quaternion, which denotes no
// orientation at all; tf2 drops it as denormalized.
TEST(TfTreeCheckTest, RejectsAZeroLengthRotation)
{
  auto edge = make_edge("base_link", "lidar");
  edge.transform.rotation.w = 0.0;
  const auto err = check({edge});
  ASSERT_TRUE(err.has_value());
  EXPECT_NE(err->find("zero-length rotation"), std::string::npos) << *err;
}

// tf2 does NOT reject a denormalized quaternion: it stores it, and Matrix3x3 then
// builds a skewed matrix from the raw components. Silently wrong geometry is worse
// than a missing frame, so this module rejects it even though tf2 would not.
TEST(TfTreeCheckTest, RejectsADenormalizedRotationThatTf2WouldAccept)
{
  auto edge = make_edge("base_link", "lidar");
  edge.transform.rotation.w = 0.5;  // length 0.5, not 1
  const auto err = check({edge});
  ASSERT_TRUE(err.has_value());
  EXPECT_NE(err->find("rather than 1, so it is not a rotation"), std::string::npos) << *err;
}

// The flip side: the tolerance must not reject a real bag. A quaternion stored as
// float32 and widened back to double is off by ~1e-7, which has to pass.
TEST(TfTreeCheckTest, AcceptsFloat32RoundingInTheRotation)
{
  auto edge = make_edge("base_link", "lidar");
  // A unit quaternion normalised in float32, then widened.
  const auto to_f32 = [](double v) { return static_cast<double>(static_cast<float>(v)); };
  const double half = std::sqrt(0.5);
  edge.transform.rotation.z = to_f32(half);
  edge.transform.rotation.w = to_f32(half);
  const auto & q = edge.transform.rotation;
  const double length2 = (q.z * q.z) + (q.w * q.w);
  ASSERT_NE(length2, 1.0) << "fixture must actually be off by float32 rounding";
  ASSERT_LT(std::abs(length2 - 1.0), 1e-6);

  const auto err = check({edge});
  EXPECT_FALSE(err.has_value()) << *err;
}

TEST(TfTreeCheckTest, RejectsAnEmptyFrameId)
{
  EXPECT_TRUE(check({make_edge("", "lidar")}).has_value());
  const auto err = check({make_edge("base_link", "")});
  ASSERT_TRUE(err.has_value());
  EXPECT_NE(err->find("empty frame id"), std::string::npos) << *err;
}

TEST(TfTreeCheckTest, RejectsASelfEdge)
{
  const auto err = check({make_edge("base_link", "base_link")});
  ASSERT_TRUE(err.has_value());
  EXPECT_NE(err->find("is its own parent"), std::string::npos) << *err;
}

// The structural layer, delegated to validate_tf_forest so `tf tree` and this
// share one implementation.
TEST(TfTreeCheckTest, RejectsTheStructuralFaultsTheForestCheckCovers)
{
  const auto two_parents = check({make_edge("a", "shared"), make_edge("b", "shared")});
  ASSERT_TRUE(two_parents.has_value());
  EXPECT_NE(two_parents->find("has parent"), std::string::npos) << *two_parents;

  const auto opposite = check({make_edge("a", "b"), make_edge("b", "a")});
  ASSERT_TRUE(opposite.has_value());
  EXPECT_NE(opposite->find("opposite edges"), std::string::npos) << *opposite;

  const auto cycle = check({make_edge("a", "b"), make_edge("b", "c"), make_edge("c", "a")});
  ASSERT_TRUE(cycle.has_value());
  EXPECT_NE(cycle->find("directed cycle"), std::string::npos) << *cycle;
}

// Every message names the source, so a caller does not have to prefix it.
TEST(TfTreeCheckTest, NamesTheContextInEveryMessage)
{
  const auto err = check({make_edge("base_link", "base_link")});
  ASSERT_TRUE(err.has_value());
  EXPECT_NE(err->find("TF tree in 'rig.yaml':"), std::string::npos) << *err;
}

}  // namespace
