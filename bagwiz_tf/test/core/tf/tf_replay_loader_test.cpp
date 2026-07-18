// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf/tf_buffer_loader.hpp"
#include "bagwiz/core/tf/tf_merge_check.hpp"
#include "bagwiz/core/tf/tf_message_wire.hpp"
#include "bagwiz/core/tf/tf_topics.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <tf2/buffer_core.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <map>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// Exercises replay_tf_topics (declared in tf_buffer_loader.hpp) over its three
// side-output modes (buffer / buffer+edges / buffer+stamps), the input-edge
// recording, and the conflict-checker / decode-failure error paths.

namespace
{

using bagwiz::core::collect_tf_topics;
using bagwiz::core::replay_tf_topics;
using bagwiz::core::TfInputEdge;
using bagwiz::core::TfReplayOutputs;
using bagwiz::core::TfTopic;

constexpr std::int64_t kTenSecondsNs = 10'000'000'000LL;
constexpr std::int64_t kTwentySecondsNs = 20'000'000'000LL;

bagwiz::io::CreateOptions mcap_options()
{
  bagwiz::io::CreateOptions o;
  o.format = bagwiz::io::Format::Mcap;
  o.layout = bagwiz::io::Layout::SingleFile;
  o.mcap_compression = "none";
  return o;
}

geometry_msgs::msg::TransformStamped make_edge(
  const std::string & parent, const std::string & child, double tx, double ty, std::int32_t sec)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = parent;
  ts.child_frame_id = child;
  ts.header.stamp.sec = sec;
  ts.header.stamp.nanosec = 0;
  ts.transform.translation.x = tx;
  ts.transform.translation.y = ty;
  ts.transform.rotation.w = 1.0;
  return ts;
}

void write_tf_message(
  bagwiz::io::BagWriter & w, const std::string & topic, std::int64_t receive_ns,
  const geometry_msgs::msg::TransformStamped & edge)
{
  const std::vector<geometry_msgs::msg::TransformStamped> edges{edge};
  const auto payload = bagwiz::core::serialize_tf_message(edges);
  w.write(topic, receive_ns, std::span<const std::byte>(payload.data(), payload.size()));
}

// Writes a bag with:
//   /tf_static  one message: base_link -> lidar (1,0,0), stamp 0
//   /tf         two messages: lidar -> wheel (0,1,0) at 10s, (0,2,0) at 20s
void write_replay_bag(const std::filesystem::path & path)
{
  auto w = bagwiz::io::open_write(path, mcap_options());
  w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf"));
  w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));
  write_tf_message(*w, "/tf_static", 0, make_edge("base_link", "lidar", 1.0, 0.0, 0));
  write_tf_message(*w, "/tf", kTenSecondsNs, make_edge("lidar", "wheel", 0.0, 1.0, 10));
  write_tf_message(*w, "/tf", kTwentySecondsNs, make_edge("lidar", "wheel", 0.0, 2.0, 20));
  w->close();
}

std::int64_t time_point_to_ns(tf2::TimePoint tp)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count();
}

TEST(ReplayTfTopics, BufferOnlyResolvesStaticAndDynamic)
{
  const std::filesystem::path bag =
    std::filesystem::temp_directory_path() / "bagwiz_tf_replay_buffer_test.mcap";
  std::filesystem::remove(bag);
  write_replay_bag(bag);

  auto reader = bagwiz::io::open_read(bag);
  const auto topics = collect_tf_topics(*reader);
  tf2::BufferCore buffer{std::chrono::hours(24 * 365)};
  TfReplayOutputs outputs;
  outputs.buffer = &buffer;
  replay_tf_topics(*reader, topics, outputs);

  const auto static_ts = buffer.lookupTransform("base_link", "lidar", tf2::TimePointZero);
  EXPECT_NEAR(static_ts.transform.translation.x, 1.0, 1e-9);
  const auto at_ten =
    buffer.lookupTransform("lidar", "wheel", tf2::TimePoint{std::chrono::seconds(10)});
  EXPECT_NEAR(at_ten.transform.translation.y, 1.0, 1e-9);
  const auto at_twenty =
    buffer.lookupTransform("lidar", "wheel", tf2::TimePoint{std::chrono::seconds(20)});
  EXPECT_NEAR(at_twenty.transform.translation.y, 2.0, 1e-9);

  std::filesystem::remove(bag);
}

TEST(ReplayTfTopics, SelectionFiltersTopics)
{
  const std::filesystem::path bag =
    std::filesystem::temp_directory_path() / "bagwiz_tf_replay_select_test.mcap";
  std::filesystem::remove(bag);
  write_replay_bag(bag);

  auto reader = bagwiz::io::open_read(bag);
  const auto all = collect_tf_topics(*reader);
  std::vector<TfTopic> dynamic_only;
  for (const auto & t : all) {
    if (!t.is_static) {
      dynamic_only.push_back(t);
    }
  }
  tf2::BufferCore buffer{std::chrono::hours(24 * 365)};
  TfReplayOutputs outputs;
  outputs.buffer = &buffer;
  replay_tf_topics(*reader, dynamic_only, outputs);

  // The static topic was not replayed, so base_link -> lidar is unknown.
  EXPECT_THROW(
    buffer.lookupTransform("base_link", "lidar", tf2::TimePointZero), tf2::TransformException);
  EXPECT_NO_THROW(
    buffer.lookupTransform("lidar", "wheel", tf2::TimePoint{std::chrono::seconds(10)}));

  std::filesystem::remove(bag);
}

TEST(ReplayTfTopics, StampsCollectEveryTransform)
{
  const std::filesystem::path bag =
    std::filesystem::temp_directory_path() / "bagwiz_tf_replay_stamps_test.mcap";
  std::filesystem::remove(bag);
  write_replay_bag(bag);

  auto reader = bagwiz::io::open_read(bag);
  const auto topics = collect_tf_topics(*reader);
  std::vector<tf2::TimePoint> stamps;
  TfReplayOutputs outputs;
  outputs.stamps = &stamps;
  replay_tf_topics(*reader, topics, outputs);

  std::vector<std::int64_t> ns;
  for (const auto tp : stamps) {
    ns.push_back(time_point_to_ns(tp));
  }
  std::sort(ns.begin(), ns.end());
  EXPECT_EQ(ns, (std::vector<std::int64_t>{0, kTenSecondsNs, kTwentySecondsNs}));

  std::filesystem::remove(bag);
}

TEST(ReplayTfTopics, EdgesByTopicCollectsDistinctEdges)
{
  const std::filesystem::path bag =
    std::filesystem::temp_directory_path() / "bagwiz_tf_replay_edges_test.mcap";
  std::filesystem::remove(bag);
  write_replay_bag(bag);

  auto reader = bagwiz::io::open_read(bag);
  const auto topics = collect_tf_topics(*reader);
  std::map<std::string, std::set<std::pair<std::string, std::string>>> edges_by_topic;
  TfReplayOutputs outputs;
  outputs.edges_by_topic = &edges_by_topic;
  replay_tf_topics(*reader, topics, outputs);

  ASSERT_EQ(edges_by_topic.size(), 2);
  // /tf published the same edge twice (10s and 20s): the set keeps one copy.
  EXPECT_EQ(
    edges_by_topic["/tf"], (std::set<std::pair<std::string, std::string>>{{"lidar", "wheel"}}));
  EXPECT_EQ(
    edges_by_topic["/tf_static"],
    (std::set<std::pair<std::string, std::string>>{{"base_link", "lidar"}}));

  std::filesystem::remove(bag);
}

TEST(ReplayTfTopics, InputEdgesRecordOnlyTheInputTopic)
{
  const std::filesystem::path bag =
    std::filesystem::temp_directory_path() / "bagwiz_tf_replay_input_edges_test.mcap";
  std::filesystem::remove(bag);
  write_replay_bag(bag);

  auto reader = bagwiz::io::open_read(bag);
  const auto topics = collect_tf_topics(*reader);
  std::vector<TfInputEdge> input_edges;
  TfReplayOutputs outputs;
  outputs.input_topic = "/tf";
  outputs.input_edges = &input_edges;
  replay_tf_topics(*reader, topics, outputs);

  ASSERT_EQ(input_edges.size(), 2);
  std::sort(
    input_edges.begin(), input_edges.end(),
    [](const TfInputEdge & a, const TfInputEdge & b) { return a.stamp_ns < b.stamp_ns; });
  EXPECT_EQ(input_edges[0].frame_id, "lidar");
  EXPECT_EQ(input_edges[0].child_frame_id, "wheel");
  EXPECT_EQ(input_edges[0].stamp_ns, kTenSecondsNs);
  EXPECT_EQ(input_edges[1].stamp_ns, kTwentySecondsNs);

  std::filesystem::remove(bag);
}

TEST(ReplayTfTopics, ConflictCheckerAbortsOnContradiction)
{
  const std::filesystem::path bag =
    std::filesystem::temp_directory_path() / "bagwiz_tf_replay_conflict_test.mcap";
  std::filesystem::remove(bag);
  {
    auto w = bagwiz::io::open_write(bag, mcap_options());
    w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));
    w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static_extra"));
    write_tf_message(*w, "/tf_static", 0, make_edge("base", "child", 0.0, 0.0, 0));
    write_tf_message(*w, "/tf_static_extra", 1, make_edge("other", "child", 0.0, 0.0, 0));
    w->close();
  }

  auto reader = bagwiz::io::open_read(bag);
  const auto topics = collect_tf_topics(*reader);
  bagwiz::core::TfMergeConflictChecker checker;
  TfReplayOutputs outputs;
  outputs.conflict_checker = &checker;
  try {
    replay_tf_topics(*reader, topics, outputs);
    FAIL() << "expected a TF merge conflict";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("TF merge conflict:"), std::string::npos);
  }

  std::filesystem::remove(bag);
}

TEST(ReplayTfTopics, DecodeFailureThrows)
{
  const std::filesystem::path bag =
    std::filesystem::temp_directory_path() / "bagwiz_tf_replay_decode_test.mcap";
  std::filesystem::remove(bag);
  {
    auto w = bagwiz::io::open_write(bag, mcap_options());
    w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf"));
    const std::array<std::byte, 4> garbage{
      std::byte{0x00}, std::byte{0x01}, std::byte{0x02}, std::byte{0x03}};
    w->write("/tf", 0, std::span<const std::byte>(garbage.data(), garbage.size()));
    w->close();
  }

  auto reader = bagwiz::io::open_read(bag);
  const auto topics = collect_tf_topics(*reader);
  TfReplayOutputs outputs;
  try {
    replay_tf_topics(*reader, topics, outputs);
    FAIL() << "expected a decode failure";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(
      std::string(e.what()).find("Failed to decode TF message on '/tf':"), std::string::npos);
  }

  std::filesystem::remove(bag);
}

TEST(ReplayTfTopics, NoOutputsIsANoOpDecodePass)
{
  const std::filesystem::path bag =
    std::filesystem::temp_directory_path() / "bagwiz_tf_replay_noop_test.mcap";
  std::filesystem::remove(bag);
  write_replay_bag(bag);

  auto reader = bagwiz::io::open_read(bag);
  const auto topics = collect_tf_topics(*reader);
  EXPECT_NO_THROW(replay_tf_topics(*reader, topics, TfReplayOutputs{}));

  std::filesystem::remove(bag);
}

}  // namespace
