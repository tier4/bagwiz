// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf_message_wire.hpp"

#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/tf_value_extract.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace
{

using bagwiz::core::kTfMessageWireSchema;
using bagwiz::core::make_tf_message_topic_info;
using bagwiz::core::serialize_tf_message;

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

// Round-trip helper: decode a serialized TFMessage CDR payload back into
// TransformStamped values using the schema-driven decoder.
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

TEST(TfMessageWire, SchemaContainsAllReferencedTypes)
{
  ASSERT_NE(kTfMessageWireSchema, nullptr);
  const std::string s = kTfMessageWireSchema;
  EXPECT_NE(s.find("geometry_msgs/TransformStamped[] transforms"), std::string::npos);
  EXPECT_NE(s.find("MSG: geometry_msgs/TransformStamped"), std::string::npos);
  EXPECT_NE(s.find("MSG: std_msgs/Header"), std::string::npos);
  EXPECT_NE(s.find("MSG: geometry_msgs/Transform"), std::string::npos);
  EXPECT_NE(s.find("MSG: geometry_msgs/Vector3"), std::string::npos);
  EXPECT_NE(s.find("MSG: geometry_msgs/Quaternion"), std::string::npos);
  EXPECT_NE(s.find("builtin_interfaces/Time stamp"), std::string::npos);
}

TEST(MakeTfMessageTopicInfo, PopulatesAllFields)
{
  const auto info = make_tf_message_topic_info("/trajectory/tf");
  EXPECT_EQ(info.name, "/trajectory/tf");
  EXPECT_EQ(info.type, "tf2_msgs/msg/TFMessage");
  EXPECT_EQ(info.serialization_format, "cdr");
  EXPECT_EQ(info.schema_encoding, "ros2msg");
  EXPECT_FALSE(info.schema_text.empty());
  EXPECT_EQ(info.schema_text, kTfMessageWireSchema);
}

TEST(MakeTfMessageTopicInfo, AcceptsStringView)
{
  const std::string_view name = "/dyn_tf";
  const auto info = make_tf_message_topic_info(name);
  EXPECT_EQ(info.name, "/dyn_tf");
}

TEST(MakeTfMessageTopicInfo, LeavesOptionalFieldsEmpty)
{
  const auto info = make_tf_message_topic_info("/tf");
  EXPECT_TRUE(info.offered_qos_profiles.empty());
  EXPECT_TRUE(info.type_description_hash.empty());
}

// `serialize_tf_message` is verified by serialising a known vector of
// TransformStamped through the introspection typesupport and reading
// the bytes back with the schema-driven decoder.
TEST(SerializeTfMessage, RoundTripsSingleEdge)
{
  std::vector<geometry_msgs::msg::TransformStamped> input;
  input.push_back(make_edge("map", "base_link", 1.5, /*sec=*/42, /*nsec=*/123U));

  const auto cdr = serialize_tf_message(input);
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

TEST(SerializeTfMessage, PreservesMultipleEdges)
{
  std::vector<geometry_msgs::msg::TransformStamped> input;
  input.push_back(make_edge("map", "odom", 1.0));
  input.push_back(make_edge("odom", "base_link", 2.0));
  input.push_back(make_edge("base_link", "lidar", 3.0));

  const auto cdr = serialize_tf_message(input);
  const auto out = decode_tf_payload(cdr);
  ASSERT_EQ(out.size(), 3U);
  EXPECT_EQ(out[0].child_frame_id, "odom");
  EXPECT_EQ(out[1].child_frame_id, "base_link");
  EXPECT_EQ(out[2].child_frame_id, "lidar");
  EXPECT_DOUBLE_EQ(out[1].transform.translation.x, 2.0);
}

}  // namespace
