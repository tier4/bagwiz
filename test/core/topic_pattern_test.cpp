// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/topic_pattern.hpp"

#include <gtest/gtest.h>

namespace
{

using bagwiz::core::TopicPattern;

TEST(TopicPattern, EmptyMatchesEverything)
{
  TopicPattern p("");
  EXPECT_TRUE(p.match_all());
  EXPECT_TRUE(p.matches("/anything"));
  EXPECT_TRUE(p.matches(""));
}

TEST(TopicPattern, AbsolutePrefixMatch)
{
  TopicPattern p("/sensing");
  EXPECT_TRUE(p.matches("/sensing"));
  EXPECT_TRUE(p.matches("/sensing/lidar/front/points"));
  EXPECT_FALSE(p.matches("/perception/object"));
  EXPECT_FALSE(p.matches("/foo/sensing"));
}

TEST(TopicPattern, RelativeSubstringMatch)
{
  TopicPattern p("lidar/front");
  EXPECT_TRUE(p.matches("/sensing/lidar/front/points"));
  EXPECT_TRUE(p.matches("lidar/front"));
  EXPECT_FALSE(p.matches("/sensing/lidar/rear/points"));
  EXPECT_FALSE(p.matches("/perception/object"));
}

TEST(TopicPattern, SingleSegmentWildcard)
{
  TopicPattern p("/*/nebula_packets");
  EXPECT_TRUE(p.matches("/sensing/nebula_packets"));
  EXPECT_TRUE(p.matches("/foo/nebula_packets"));
  // '*' does not cross segment boundaries, so multi-segment topics are
  // rejected.
  EXPECT_FALSE(p.matches("/sensing/lidar/nebula_packets"));
  // Extra trailing segments are rejected because the pattern is
  // anchored at both ends when a wildcard is present.
  EXPECT_FALSE(p.matches("/sensing/nebula_packets/extra"));
}

TEST(TopicPattern, RelativeWildcardIsSuffixAnchored)
{
  // No leading '/' -> start is unanchored (substring), but the presence of
  // '*' anchors the end.
  TopicPattern p("lidar/*/points");
  EXPECT_TRUE(p.matches("lidar/front/points"));
  EXPECT_TRUE(p.matches("/sensing/lidar/front/points"));
  EXPECT_FALSE(p.matches("lidar/front/points/extra"));
  EXPECT_FALSE(p.matches("/sensing/lidar/front/points/extra"));
}

TEST(TopicPattern, RegexMetacharactersAreEscaped)
{
  TopicPattern p("/tf.static");
  EXPECT_TRUE(p.matches("/tf.static"));
  // The '.' is a literal, not a regex metacharacter.
  EXPECT_FALSE(p.matches("/tfXstatic"));
}

TEST(TopicPattern, DoubleWildcardStillSegmentScoped)
{
  // Two adjacent '*' characters collapse into a single segment wildcard; this
  // test pins down that there is no implicit "match across slashes" escape.
  TopicPattern p("/**/points");
  EXPECT_TRUE(p.matches("/front/points"));
  EXPECT_FALSE(p.matches("/sensing/lidar/points"));
}

}  // namespace
