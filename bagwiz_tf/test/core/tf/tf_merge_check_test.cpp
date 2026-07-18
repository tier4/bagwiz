// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf/tf_merge_check.hpp"

#include <gtest/gtest.h>

#include <string>

namespace
{

using bagwiz::core::TfMergeConflictChecker;

TEST(TfMergeConflictChecker, RepeatedEdgeFromOneDynamicTopicIsNotAConflict)
{
  // A normal /tf time series: same edge, same topic, over and over.
  TfMergeConflictChecker checker;
  EXPECT_FALSE(checker.add("map", "base_link", "/tf", false).has_value());
  EXPECT_FALSE(checker.add("map", "base_link", "/tf", false).has_value());
  EXPECT_FALSE(checker.add("map", "base_link", "/tf", false).has_value());
}

TEST(TfMergeConflictChecker, DisjointEdgesAcrossTopicsAreNotAConflict)
{
  // Different children partitioned across a dynamic and a static topic.
  TfMergeConflictChecker checker;
  EXPECT_FALSE(checker.add("map", "base_link", "/tf", false).has_value());
  EXPECT_FALSE(checker.add("base_link", "imu_link", "/tf_static", true).has_value());
  EXPECT_FALSE(checker.add("base_link", "lidar", "/sensing/tf_static", true).has_value());
}

TEST(TfMergeConflictChecker, SameChildSameParentAcrossStaticTopicsIsAllowed)
{
  // Two static topics declare the identical edge: same parent, both static.
  // Under the chosen policy this is last-wins, not a conflict.
  TfMergeConflictChecker checker;
  EXPECT_FALSE(checker.add("base_link", "imu_link", "/tf_static", true).has_value());
  EXPECT_FALSE(checker.add("base_link", "imu_link", "/other/tf_static", true).has_value());
}

TEST(TfMergeConflictChecker, MultiParentAcrossTopicsIsAConflict)
{
  // The same child is given two different parents by two different topics.
  TfMergeConflictChecker checker;
  EXPECT_FALSE(checker.add("map", "base_link", "/tf", false).has_value());
  const auto conflict = checker.add("odom", "base_link", "/other_tf", false);
  ASSERT_TRUE(conflict.has_value());
  EXPECT_NE(conflict->find("base_link"), std::string::npos);
  EXPECT_NE(conflict->find("conflicting parents"), std::string::npos);
}

TEST(TfMergeConflictChecker, ParentChangeWithinOneTopicIsNotAConflict)
{
  // A single topic that varies the parent over time is never flagged
  // (detection is cross-topic only).
  TfMergeConflictChecker checker;
  EXPECT_FALSE(checker.add("map", "base_link", "/tf", false).has_value());
  EXPECT_FALSE(checker.add("odom", "base_link", "/tf", false).has_value());
}

TEST(TfMergeConflictChecker, StaticAndDynamicForSameChildIsAConflict)
{
  // Even with the same parent, a frame cannot be both static and dynamic.
  TfMergeConflictChecker checker;
  EXPECT_FALSE(checker.add("base_link", "imu_link", "/tf_static", true).has_value());
  const auto conflict = checker.add("base_link", "imu_link", "/tf", false);
  ASSERT_TRUE(conflict.has_value());
  EXPECT_NE(conflict->find("imu_link"), std::string::npos);
  EXPECT_NE(conflict->find("static"), std::string::npos);
  EXPECT_NE(conflict->find("dynamic"), std::string::npos);
}

TEST(TfMergeConflictChecker, DynamicThenStaticForSameChildIsAlsoAConflict)
{
  // Order independence: dynamic first, then static, still conflicts.
  TfMergeConflictChecker checker;
  EXPECT_FALSE(checker.add("base_link", "imu_link", "/tf", false).has_value());
  EXPECT_TRUE(checker.add("base_link", "imu_link", "/tf_static", true).has_value());
}

}  // namespace
