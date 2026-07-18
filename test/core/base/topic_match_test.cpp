// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/base/topic_match.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace
{

using bagwiz::core::resolve_topic_patterns;
using bagwiz::core::topic_glob_match;

TEST(TopicGlobMatch, ExactMatchWithoutWildcard)
{
  EXPECT_TRUE(topic_glob_match("/foo", "/foo"));
  EXPECT_FALSE(topic_glob_match("/foo", "/bar"));
  // No substring matching: an exact pattern must match the whole name.
  EXPECT_FALSE(topic_glob_match("/foo", "/foobar"));
  EXPECT_FALSE(topic_glob_match("/foobar", "/foo"));
}

TEST(TopicGlobMatch, LoneStarMatchesEverything)
{
  EXPECT_TRUE(topic_glob_match("*", "/foo"));
  EXPECT_TRUE(topic_glob_match("*", "/sensing/camera/image"));
  EXPECT_TRUE(topic_glob_match("*", ""));
}

TEST(TopicGlobMatch, PrefixStarMatchesAcrossSlashes)
{
  EXPECT_TRUE(topic_glob_match("/sensing/*", "/sensing/camera/image"));
  EXPECT_TRUE(topic_glob_match("/sensing/*", "/sensing/lidar"));
  // The prefix before '*' must still match literally.
  EXPECT_FALSE(topic_glob_match("/sensing/*", "/perception/objects"));
  // '*' matches the empty string, so the bare prefix matches too.
  EXPECT_TRUE(topic_glob_match("/sensing/*", "/sensing/"));
}

TEST(TopicGlobMatch, SuffixStarMatchesTrailing)
{
  EXPECT_TRUE(topic_glob_match("*/image_raw", "/camera0/image_raw"));
  EXPECT_TRUE(topic_glob_match("*/image_raw", "/a/b/c/image_raw"));
  EXPECT_FALSE(topic_glob_match("*/image_raw", "/camera0/image_compressed"));
}

TEST(TopicGlobMatch, MiddleStarMatchesInfix)
{
  EXPECT_TRUE(topic_glob_match("/camera*/image", "/camera0/image"));
  EXPECT_TRUE(topic_glob_match("/camera*/image", "/camera/front/image"));
  EXPECT_FALSE(topic_glob_match("/camera*/image", "/lidar0/image"));
}

TEST(TopicGlobMatch, MultipleStars)
{
  EXPECT_TRUE(topic_glob_match("*camera*", "/sensing/camera/image"));
  EXPECT_TRUE(topic_glob_match("/*/*/image", "/sensing/cam/image"));
  EXPECT_FALSE(topic_glob_match("/*/*/image", "/sensing/image"));
}

TEST(TopicGlobMatch, ConsecutiveStarsCollapse)
{
  EXPECT_TRUE(topic_glob_match("/foo**", "/foobar"));
  EXPECT_TRUE(topic_glob_match("**", "/anything"));
}

TEST(TopicGlobMatch, EmptyPatternOnlyMatchesEmpty)
{
  EXPECT_TRUE(topic_glob_match("", ""));
  EXPECT_FALSE(topic_glob_match("", "/foo"));
}

TEST(ResolveTopicPatterns, SinglePatternMatchesSubset)
{
  const std::vector<std::string> patterns{"/sensing/*"};
  const std::vector<std::string> topics{"/sensing/camera", "/sensing/lidar", "/perception/objects"};

  const auto result = resolve_topic_patterns(patterns, topics);

  EXPECT_TRUE(result.unmatched.empty());
  EXPECT_EQ(result.matched.size(), 2U);
  EXPECT_EQ(result.matched.count("/sensing/camera"), 1U);
  EXPECT_EQ(result.matched.count("/sensing/lidar"), 1U);
  EXPECT_EQ(result.matched.count("/perception/objects"), 0U);
}

TEST(ResolveTopicPatterns, OverlappingPatternsDeduplicate)
{
  const std::vector<std::string> patterns{"/sensing/*", "*camera*"};
  const std::vector<std::string> topics{"/sensing/camera", "/sensing/lidar"};

  const auto result = resolve_topic_patterns(patterns, topics);

  // /sensing/camera is matched by both patterns but appears once.
  EXPECT_TRUE(result.unmatched.empty());
  EXPECT_EQ(result.matched.size(), 2U);
}

TEST(ResolveTopicPatterns, UnmatchedPatternsReported)
{
  const std::vector<std::string> patterns{"/foo", "/does/not/exist", "/bar*"};
  const std::vector<std::string> topics{"/foo", "/baz"};

  const auto result = resolve_topic_patterns(patterns, topics);

  EXPECT_EQ(result.matched.size(), 1U);
  EXPECT_EQ(result.matched.count("/foo"), 1U);
  ASSERT_EQ(result.unmatched.size(), 2U);
  // Preserved in input order.
  EXPECT_EQ(result.unmatched[0], "/does/not/exist");
  EXPECT_EQ(result.unmatched[1], "/bar*");
}

TEST(ResolveTopicPatterns, LoneStarMatchesAllTopics)
{
  const std::vector<std::string> patterns{"*"};
  const std::vector<std::string> topics{"/a", "/b", "/c"};

  const auto result = resolve_topic_patterns(patterns, topics);

  EXPECT_TRUE(result.unmatched.empty());
  EXPECT_EQ(result.matched.size(), 3U);
}

TEST(ResolveTopicPatterns, NoTopicsLeavesEveryPatternUnmatched)
{
  const std::vector<std::string> patterns{"*", "/foo"};
  const std::vector<std::string> topics{};

  const auto result = resolve_topic_patterns(patterns, topics);

  EXPECT_TRUE(result.matched.empty());
  EXPECT_EQ(result.unmatched.size(), 2U);
}

}  // namespace
