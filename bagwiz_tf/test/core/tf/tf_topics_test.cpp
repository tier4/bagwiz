// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf/tf_topics.hpp"

#include "bagwiz/core/tf/tf_message_wire.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>

#include <filesystem>
#include <map>
#include <span>
#include <string>
#include <vector>

namespace
{

using bagwiz::core::collect_tf_topics;
using bagwiz::core::is_static_tf_topic;
using bagwiz::core::TfTopic;

bagwiz::io::CreateOptions mcap_options()
{
  bagwiz::io::CreateOptions o;
  o.format = bagwiz::io::Format::Mcap;
  o.layout = bagwiz::io::Layout::SingleFile;
  o.mcap_compression = "none";
  return o;
}

geometry_msgs::msg::TransformStamped make_edge(
  const std::string & parent, const std::string & child)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = parent;
  ts.child_frame_id = child;
  ts.transform.rotation.w = 1.0;
  return ts;
}

// Writes a bag with a dynamic /tf topic, a static /tf_static topic, and one
// non-TF topic (/chatter).
void write_mixed_bag(const std::filesystem::path & path)
{
  auto w = bagwiz::io::open_write(path, mcap_options());
  w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf"));
  w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));
  bagwiz::io::TopicInfo string_topic;
  string_topic.name = "/chatter";
  string_topic.type = "std_msgs/msg/String";
  string_topic.serialization_format = "cdr";
  w->declare_topic(string_topic);

  std::vector<geometry_msgs::msg::TransformStamped> edges;
  edges.push_back(make_edge("base_link", "lidar"));
  const auto payload = bagwiz::core::serialize_tf_message(edges);
  w->write("/tf", 0, std::span<const std::byte>(payload.data(), payload.size()));
  w->write("/tf_static", 0, std::span<const std::byte>(payload.data(), payload.size()));
  w->close();
}

TEST(IsStaticTfTopic, StaticSuffixMatches)
{
  EXPECT_TRUE(is_static_tf_topic("/tf_static"));
  EXPECT_TRUE(is_static_tf_topic("/foo/tf_static"));
  // The test is a pure suffix match: no leading '/' is required.
  EXPECT_TRUE(is_static_tf_topic("tf_static"));
  EXPECT_TRUE(is_static_tf_topic("xtf_static"));
}

TEST(IsStaticTfTopic, NonStaticNames)
{
  EXPECT_FALSE(is_static_tf_topic("/tf"));
  EXPECT_FALSE(is_static_tf_topic(""));
  EXPECT_FALSE(is_static_tf_topic("/tf_stat"));
  EXPECT_FALSE(is_static_tf_topic("/tf_static2"));
  EXPECT_FALSE(is_static_tf_topic("tf_stat"));
}

TEST(CollectTfTopics, PicksTfTopicsWithStaticFlag)
{
  const std::filesystem::path bag =
    std::filesystem::temp_directory_path() / "bagwiz_tf_topics_test.mcap";
  std::filesystem::remove(bag);
  write_mixed_bag(bag);

  auto reader = bagwiz::io::open_read(bag);
  const std::vector<TfTopic> topics = collect_tf_topics(*reader);

  std::map<std::string, bool> is_static_by_name;
  for (const auto & t : topics) {
    is_static_by_name.emplace(t.name, t.is_static);
  }
  ASSERT_EQ(is_static_by_name.size(), 2);  // /chatter (non-TF) is excluded
  EXPECT_FALSE(is_static_by_name.at("/tf"));
  EXPECT_TRUE(is_static_by_name.at("/tf_static"));

  std::filesystem::remove(bag);
}

TEST(CollectTfTopics, EmptyWhenNoTfTopic)
{
  const std::filesystem::path bag =
    std::filesystem::temp_directory_path() / "bagwiz_tf_topics_empty_test.mcap";
  std::filesystem::remove(bag);
  {
    auto w = bagwiz::io::open_write(bag, mcap_options());
    bagwiz::io::TopicInfo string_topic;
    string_topic.name = "/chatter";
    string_topic.type = "std_msgs/msg/String";
    string_topic.serialization_format = "cdr";
    w->declare_topic(string_topic);
    w->close();
  }

  auto reader = bagwiz::io::open_read(bag);
  EXPECT_TRUE(collect_tf_topics(*reader).empty());

  std::filesystem::remove(bag);
}

}  // namespace
