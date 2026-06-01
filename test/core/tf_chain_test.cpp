// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf_chain.hpp"

#include <tf2/buffer_core.hpp>
#include <tf2/time.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <vector>

namespace
{

geometry_msgs::msg::TransformStamped make_tf(
  const std::string & parent, const std::string & child, std::int32_t sec)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = parent;
  t.header.stamp.sec = sec;
  t.header.stamp.nanosec = 0;
  t.child_frame_id = child;
  t.transform.rotation.w = 1.0;
  return t;
}

// chain: root -> a -> b -> c
//   resolve_chain(a, c) returns [a, b, c]
//   resolve_chain(c, a) returns [c, b, a]
TEST(ResolveChain, AncestorPath)
{
  tf2::BufferCore buffer{std::chrono::seconds(60)};
  buffer.setTransform(make_tf("root", "a", 0), "test", true);
  buffer.setTransform(make_tf("a", "b", 0), "test", true);
  buffer.setTransform(make_tf("b", "c", 0), "test", true);

  const auto down = bagwiz::core::resolve_chain(buffer, "a", "c", tf2::TimePoint{});
  ASSERT_EQ(down.size(), 3u);
  EXPECT_EQ(down[0], "a");
  EXPECT_EQ(down[1], "b");
  EXPECT_EQ(down[2], "c");

  const auto up = bagwiz::core::resolve_chain(buffer, "c", "a", tf2::TimePoint{});
  ASSERT_EQ(up.size(), 3u);
  EXPECT_EQ(up[0], "c");
  EXPECT_EQ(up[1], "b");
  EXPECT_EQ(up[2], "a");
}

// chain: root -> {left, right}
//   resolve_chain(left, right) needs to traverse the LCA "root":
//   [left, root, right].
TEST(ResolveChain, ViaCommonAncestor)
{
  tf2::BufferCore buffer{std::chrono::seconds(60)};
  buffer.setTransform(make_tf("root", "left", 0), "test", true);
  buffer.setTransform(make_tf("root", "right", 0), "test", true);

  const auto chain = bagwiz::core::resolve_chain(buffer, "left", "right", tf2::TimePoint{});
  ASSERT_EQ(chain.size(), 3u);
  EXPECT_EQ(chain[0], "left");
  EXPECT_EQ(chain[1], "root");
  EXPECT_EQ(chain[2], "right");
}

TEST(ResolveChain, DisconnectedComponentsYieldEmpty)
{
  tf2::BufferCore buffer{std::chrono::seconds(60)};
  buffer.setTransform(make_tf("a", "b", 0), "test", true);
  buffer.setTransform(make_tf("c", "d", 0), "test", true);

  const auto chain = bagwiz::core::resolve_chain(buffer, "b", "d", tf2::TimePoint{});
  EXPECT_TRUE(chain.empty());
}

// edges of [a, b, c] over the tree root->a->b->c are:
//   (a, b): a is parent of b ⇒ (a, b)
//   (b, c): b is parent of c ⇒ (b, c)
TEST(ChainToEdges, RecoversParentChildPairs)
{
  tf2::BufferCore buffer{std::chrono::seconds(60)};
  buffer.setTransform(make_tf("root", "a", 0), "test", true);
  buffer.setTransform(make_tf("a", "b", 0), "test", true);
  buffer.setTransform(make_tf("b", "c", 0), "test", true);

  const std::vector<std::string> chain{"a", "b", "c"};
  const auto edges = bagwiz::core::chain_to_edges(buffer, chain, tf2::TimePoint{});
  ASSERT_EQ(edges.size(), 2u);
  EXPECT_EQ(edges[0].first, "a");
  EXPECT_EQ(edges[0].second, "b");
  EXPECT_EQ(edges[1].first, "b");
  EXPECT_EQ(edges[1].second, "c");
}

// For chain [c, b, a] over the same tree, traversal flips: each edge
// is still (parent, child) in TF terms even though the chain walks
// upward.
TEST(ChainToEdges, UpwardChainKeepsParentChildOrientation)
{
  tf2::BufferCore buffer{std::chrono::seconds(60)};
  buffer.setTransform(make_tf("root", "a", 0), "test", true);
  buffer.setTransform(make_tf("a", "b", 0), "test", true);
  buffer.setTransform(make_tf("b", "c", 0), "test", true);

  const std::vector<std::string> chain{"c", "b", "a"};
  const auto edges = bagwiz::core::chain_to_edges(buffer, chain, tf2::TimePoint{});
  ASSERT_EQ(edges.size(), 2u);
  EXPECT_EQ(edges[0].first, "b");
  EXPECT_EQ(edges[0].second, "c");
  EXPECT_EQ(edges[1].first, "a");
  EXPECT_EQ(edges[1].second, "b");
}

// V-shaped chain [left, root, right] crosses the LCA. Each leg is its
// own (parent=root, child=*) edge in TF.
TEST(ChainToEdges, ViaCommonAncestorEdges)
{
  tf2::BufferCore buffer{std::chrono::seconds(60)};
  buffer.setTransform(make_tf("root", "left", 0), "test", true);
  buffer.setTransform(make_tf("root", "right", 0), "test", true);

  const std::vector<std::string> chain{"left", "root", "right"};
  const auto edges = bagwiz::core::chain_to_edges(buffer, chain, tf2::TimePoint{});
  ASSERT_EQ(edges.size(), 2u);
  EXPECT_EQ(edges[0].first, "root");
  EXPECT_EQ(edges[0].second, "left");
  EXPECT_EQ(edges[1].first, "root");
  EXPECT_EQ(edges[1].second, "right");
}

TEST(ChainToEdges, EmptyAndSingletonChainsYieldEmpty)
{
  tf2::BufferCore buffer{std::chrono::seconds(60)};
  buffer.setTransform(make_tf("a", "b", 0), "test", true);

  EXPECT_TRUE(bagwiz::core::chain_to_edges(buffer, {}, tf2::TimePoint{}).empty());
  EXPECT_TRUE(bagwiz::core::chain_to_edges(buffer, {"a"}, tf2::TimePoint{}).empty());
}

}  // namespace
