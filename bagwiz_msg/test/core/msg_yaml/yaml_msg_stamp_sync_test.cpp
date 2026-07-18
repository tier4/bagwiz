// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/msg_yaml/yaml_msg_stamp_sync.hpp"

#include "bagwiz/core/msg_schema/parser.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <limits>
#include <string>

namespace ms = bagwiz::core::msg_schema;

TEST(YamlMsgStampSync, SyncsTopLevelHeaderStamp)
{
  const std::string text =
    "std_msgs/Header header\n"
    "string data\n"
    "===\n"
    "MSG: std_msgs/Header\n"
    "builtin_interfaces/Time stamp\n"
    "string frame_id\n";

  const auto parse = ms::parse_schema("test_pkg/msg/WithHeader", text);
  ASSERT_TRUE(parse.ok()) << parse.error;
  ASSERT_TRUE(parse.schema.has_value());

  YAML::Node root = YAML::Load(
    "header:\n"
    "  stamp:\n"
    "    sec: 1\n"
    "    nanosec: 2\n"
    "  frame_id: 'map'\n"
    "data: 'hello'\n");

  const std::int64_t ns = 1'700'000'000'123'456'789LL;
  const auto r = bagwiz::core::sync_top_level_header_stamp_to_time(*parse.schema, root, ns);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(root["header"]["stamp"]["sec"].as<std::int32_t>(), 1700000000);
  EXPECT_EQ(root["header"]["stamp"]["nanosec"].as<std::uint32_t>(), 123456789U);
}

TEST(YamlMsgStampSync, FailsWhenRootTypeHasNoHeader)
{
  const auto parse = ms::parse_schema("std_msgs/msg/String", "string data\n");
  ASSERT_TRUE(parse.ok()) << parse.error;
  ASSERT_TRUE(parse.schema.has_value());

  YAML::Node root = YAML::Load("data: 'hello'\n");
  const auto r = bagwiz::core::sync_top_level_header_stamp_to_time(*parse.schema, root, 0);
  ASSERT_FALSE(r.ok);
  EXPECT_NE(r.error.find("top-level 'header'"), std::string::npos) << r.error;
}

TEST(YamlMsgStampSync, FailsWhenYamlHeaderIsNotMapping)
{
  const std::string text =
    "std_msgs/Header header\n"
    "===\n"
    "MSG: std_msgs/Header\n"
    "builtin_interfaces/Time stamp\n"
    "string frame_id\n";

  const auto parse = ms::parse_schema("test_pkg/msg/WithHeader", text);
  ASSERT_TRUE(parse.ok()) << parse.error;
  ASSERT_TRUE(parse.schema.has_value());

  YAML::Node root = YAML::Load("header: 42\n");
  const auto r = bagwiz::core::sync_top_level_header_stamp_to_time(*parse.schema, root, 0);
  ASSERT_FALSE(r.ok);
  EXPECT_NE(r.error.find("YAML 'header' must be a mapping"), std::string::npos) << r.error;
}

TEST(YamlMsgStampSync, FailsWhenYamlStampIsNotMapping)
{
  const std::string text =
    "std_msgs/Header header\n"
    "===\n"
    "MSG: std_msgs/Header\n"
    "builtin_interfaces/Time stamp\n"
    "string frame_id\n";

  const auto parse = ms::parse_schema("test_pkg/msg/WithHeader", text);
  ASSERT_TRUE(parse.ok()) << parse.error;
  ASSERT_TRUE(parse.schema.has_value());

  YAML::Node root = YAML::Load(
    "header:\n"
    "  stamp: 42\n"
    "  frame_id: 'map'\n");
  const auto r = bagwiz::core::sync_top_level_header_stamp_to_time(*parse.schema, root, 0);
  ASSERT_FALSE(r.ok);
  EXPECT_NE(r.error.find("YAML 'header.stamp' must be a mapping"), std::string::npos) << r.error;
}

TEST(YamlMsgStampSync, NormalizesNegativeNanoseconds)
{
  const std::string text =
    "std_msgs/Header header\n"
    "===\n"
    "MSG: std_msgs/Header\n"
    "builtin_interfaces/Time stamp\n"
    "string frame_id\n";

  const auto parse = ms::parse_schema("test_pkg/msg/WithHeader", text);
  ASSERT_TRUE(parse.ok()) << parse.error;
  ASSERT_TRUE(parse.schema.has_value());

  YAML::Node root = YAML::Load(
    "header:\n"
    "  stamp:\n"
    "    sec: 0\n"
    "    nanosec: 0\n"
    "  frame_id: 'map'\n");

  const auto r = bagwiz::core::sync_top_level_header_stamp_to_time(*parse.schema, root, -1);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(root["header"]["stamp"]["sec"].as<std::int32_t>(), -1);
  EXPECT_EQ(root["header"]["stamp"]["nanosec"].as<std::uint32_t>(), 999999999U);
}

TEST(YamlMsgStampSync, FailsWhenSecondsOverflowInt32)
{
  const std::string text =
    "std_msgs/Header header\n"
    "===\n"
    "MSG: std_msgs/Header\n"
    "builtin_interfaces/Time stamp\n"
    "string frame_id\n";

  const auto parse = ms::parse_schema("test_pkg/msg/WithHeader", text);
  ASSERT_TRUE(parse.ok()) << parse.error;
  ASSERT_TRUE(parse.schema.has_value());

  YAML::Node root = YAML::Load(
    "header:\n"
    "  stamp:\n"
    "    sec: 0\n"
    "    nanosec: 0\n"
    "  frame_id: 'map'\n");

  const std::int64_t too_large =
    (static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()) + 1) * 1000000000LL;
  const auto r = bagwiz::core::sync_top_level_header_stamp_to_time(*parse.schema, root, too_large);
  ASSERT_FALSE(r.ok);
  EXPECT_NE(r.error.find("out of range"), std::string::npos) << r.error;
}
