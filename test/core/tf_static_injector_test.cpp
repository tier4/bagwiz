// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Tests for the helpers backing `bagwiz tf inject-static`.
//
// `serialize_tf_message` is verified by serialising a known vector of
// TransformStamped through the introspection typesupport and reading
// the bytes back with the schema-driven decoder. `collect_tf_static_from_bag`
// is exercised end-to-end by writing a small MCAP bag with two
// /tf_static messages that share one edge and differ on another, then
// asserting the dedupe semantics (last-writer-wins, deterministic
// emission order by (parent, child)).

#include "bagwiz/core/tf_static_injector.hpp"

#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/tf_message_wire.hpp"
#include "bagwiz/core/tf_value_extract.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <vector>

namespace
{

constexpr const char * kTfMessageType = "tf2_msgs/msg/TFMessage";

geometry_msgs::msg::TransformStamped make_edge(
  const std::string & parent, const std::string & child, double tx, std::int32_t sec = 0,
  std::uint32_t nsec = 0)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = parent;
  ts.header.stamp.sec = sec;
  ts.header.stamp.nanosec = nsec;
  ts.child_frame_id = child;
  ts.transform.translation.x = tx;
  ts.transform.translation.y = 0.0;
  ts.transform.translation.z = 0.0;
  ts.transform.rotation.x = 0.0;
  ts.transform.rotation.y = 0.0;
  ts.transform.rotation.z = 0.0;
  ts.transform.rotation.w = 1.0;
  return ts;
}

class TfStaticInjectorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_tf_static_injector_" +
                std::to_string(::testing::UnitTest::GetInstance()->random_seed()) + "_" +
                std::to_string(reinterpret_cast<std::uintptr_t>(this)));
    std::filesystem::create_directories(tmp_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
};

std::vector<geometry_msgs::msg::TransformStamped> decode_tf_payload(
  std::span<const std::byte> payload)
{
  bagwiz::io::TopicInfo info;
  info.name = "/tf_static";
  info.type = kTfMessageType;
  info.serialization_format = "cdr";
  info.schema_encoding = "ros2msg";
  info.schema_text = bagwiz::core::kTfMessageWireSchema;
  auto open = bagwiz::core::decoder::open_decoder(info);
  if (!open.ok()) {
    return {};
  }
  const auto decoded = open.decoder->decode(payload);
  if (!decoded.ok()) {
    return {};
  }
  return bagwiz::core::extract_tf_message(*decoded.value);
}

}  // namespace

TEST_F(TfStaticInjectorTest, SerializeTfMessageRoundTripsSingleEdge)
{
  std::vector<geometry_msgs::msg::TransformStamped> input;
  input.push_back(make_edge("map", "base_link", 1.5, /*sec=*/42, /*nsec=*/123U));

  const auto cdr = bagwiz::core::serialize_tf_message(input);
  ASSERT_GE(cdr.size(), 4U);  // at least the CDR encapsulation header

  const auto out = decode_tf_payload(cdr);
  ASSERT_EQ(out.size(), 1U);
  EXPECT_EQ(out[0].header.frame_id, "map");
  EXPECT_EQ(out[0].child_frame_id, "base_link");
  EXPECT_EQ(out[0].header.stamp.sec, 42);
  EXPECT_EQ(out[0].header.stamp.nanosec, 123U);
  EXPECT_DOUBLE_EQ(out[0].transform.translation.x, 1.5);
  EXPECT_DOUBLE_EQ(out[0].transform.rotation.w, 1.0);
}

TEST_F(TfStaticInjectorTest, SerializeTfMessagePreservesMultipleEdges)
{
  std::vector<geometry_msgs::msg::TransformStamped> input;
  input.push_back(make_edge("map", "odom", 1.0));
  input.push_back(make_edge("odom", "base_link", 2.0));
  input.push_back(make_edge("base_link", "lidar", 3.0));

  const auto cdr = bagwiz::core::serialize_tf_message(input);
  const auto out = decode_tf_payload(cdr);
  ASSERT_EQ(out.size(), 3U);
  EXPECT_EQ(out[0].child_frame_id, "odom");
  EXPECT_EQ(out[1].child_frame_id, "base_link");
  EXPECT_EQ(out[2].child_frame_id, "lidar");
  EXPECT_DOUBLE_EQ(out[1].transform.translation.x, 2.0);
}

TEST_F(TfStaticInjectorTest, CollectFromBagDedupesLastWriterWins)
{
  const auto bag_path = tmp_dir_ / "from.mcap";

  bagwiz::io::TopicInfo topic;
  topic.name = "/tf_static";
  topic.type = kTfMessageType;
  topic.serialization_format = "cdr";
  topic.schema_encoding = "ros2msg";
  topic.schema_text = bagwiz::core::kTfMessageWireSchema;

  std::vector<geometry_msgs::msg::TransformStamped> first;
  first.push_back(make_edge("map", "odom", 1.0));
  const auto first_cdr = bagwiz::core::serialize_tf_message(first);

  std::vector<geometry_msgs::msg::TransformStamped> second;
  second.push_back(make_edge("map", "odom", 9.0));  // overwrites earlier edge
  second.push_back(make_edge("odom", "base_link", 2.0));
  const auto second_cdr = bagwiz::core::serialize_tf_message(second);

  {
    bagwiz::io::CreateOptions copts;
    copts.format = bagwiz::io::Format::Mcap;
    copts.layout = bagwiz::io::Layout::SingleFile;
    copts.mcap_compression = "none";
    auto writer = bagwiz::io::open_write(bag_path, copts);
    writer->declare_topic(topic);
    writer->write(
      topic.name, /*timestamp_ns=*/1'000'000'000LL,
      std::span<const std::byte>(first_cdr.data(), first_cdr.size()));
    writer->write(
      topic.name, /*timestamp_ns=*/2'000'000'000LL,
      std::span<const std::byte>(second_cdr.data(), second_cdr.size()));
    writer->close();
  }

  const auto collected = bagwiz::core::collect_tf_static_from_bag(bag_path);
  ASSERT_EQ(collected.by_topic.size(), 1U);
  const auto & merged = collected.by_topic.at("/tf_static");
  ASSERT_EQ(merged.size(), 2U);

  auto find_edge = [&](const std::string & parent, const std::string & child) {
    return std::find_if(merged.begin(), merged.end(), [&](const auto & t) {
      return t.header.frame_id == parent && t.child_frame_id == child;
    });
  };
  const auto map_odom = find_edge("map", "odom");
  ASSERT_NE(map_odom, merged.end());
  EXPECT_DOUBLE_EQ(map_odom->transform.translation.x, 9.0);

  const auto odom_bl = find_edge("odom", "base_link");
  ASSERT_NE(odom_bl, merged.end());
  EXPECT_DOUBLE_EQ(odom_bl->transform.translation.x, 2.0);

  ASSERT_TRUE(collected.source_topic_info.contains("/tf_static"));
  EXPECT_EQ(collected.source_topic_info.at("/tf_static").type, kTfMessageType);
}

TEST_F(TfStaticInjectorTest, CollectFromBagSkipsMalformedEdges)
{
  const auto bag_path = tmp_dir_ / "from.mcap";
  bagwiz::io::TopicInfo topic;
  topic.name = "/tf_static";
  topic.type = kTfMessageType;
  topic.serialization_format = "cdr";
  topic.schema_encoding = "ros2msg";
  topic.schema_text = bagwiz::core::kTfMessageWireSchema;

  std::vector<geometry_msgs::msg::TransformStamped> msg;
  msg.push_back(make_edge("map", "odom", 1.0));
  msg.push_back(make_edge("", "ghost", 2.0));   // missing parent — dropped
  msg.push_back(make_edge("orphan", "", 3.0));  // missing child  — dropped
  const auto cdr = bagwiz::core::serialize_tf_message(msg);

  {
    bagwiz::io::CreateOptions copts;
    copts.format = bagwiz::io::Format::Mcap;
    copts.layout = bagwiz::io::Layout::SingleFile;
    copts.mcap_compression = "none";
    auto writer = bagwiz::io::open_write(bag_path, copts);
    writer->declare_topic(topic);
    writer->write(
      topic.name, /*timestamp_ns=*/1'000'000'000LL,
      std::span<const std::byte>(cdr.data(), cdr.size()));
    writer->close();
  }

  const auto collected = bagwiz::core::collect_tf_static_from_bag(bag_path);
  ASSERT_EQ(collected.by_topic.size(), 1U);
  const auto & merged = collected.by_topic.at("/tf_static");
  ASSERT_EQ(merged.size(), 1U);
  EXPECT_EQ(merged[0].header.frame_id, "map");
  EXPECT_EQ(merged[0].child_frame_id, "odom");
}

TEST_F(TfStaticInjectorTest, CollectFromBagReturnsEmptyWhenNoStaticTopic)
{
  const auto bag_path = tmp_dir_ / "from.mcap";
  bagwiz::io::TopicInfo non_static;
  non_static.name = "/tf";  // dynamic, not /tf_static
  non_static.type = kTfMessageType;
  non_static.serialization_format = "cdr";
  non_static.schema_encoding = "ros2msg";
  non_static.schema_text = bagwiz::core::kTfMessageWireSchema;

  std::vector<geometry_msgs::msg::TransformStamped> msg;
  msg.push_back(make_edge("map", "odom", 1.0));
  const auto cdr = bagwiz::core::serialize_tf_message(msg);

  {
    bagwiz::io::CreateOptions copts;
    copts.format = bagwiz::io::Format::Mcap;
    copts.layout = bagwiz::io::Layout::SingleFile;
    copts.mcap_compression = "none";
    auto writer = bagwiz::io::open_write(bag_path, copts);
    writer->declare_topic(non_static);
    writer->write(
      non_static.name, /*timestamp_ns=*/1'000'000'000LL,
      std::span<const std::byte>(cdr.data(), cdr.size()));
    writer->close();
  }

  const auto collected = bagwiz::core::collect_tf_static_from_bag(bag_path);
  EXPECT_TRUE(collected.by_topic.empty());
  EXPECT_TRUE(collected.source_topic_info.empty());
}
