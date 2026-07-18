// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/msg_yaml/yaml_msg_validate.hpp"

#include "bagwiz/core/msg_schema/parser.hpp"
#include "bagwiz/core/msg_yaml/ros2_yaml_to_cdr.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

namespace ms = bagwiz::core::msg_schema;

TEST(YamlMsgValidate, StringOkEchoStyle)
{
  const auto pr = ms::parse_message("std_msgs", "String", "string data\n");
  ASSERT_TRUE(pr.ok()) << pr.error;
  YAML::Node root = YAML::Load("data: 'hello'");
  auto r = bagwiz::core::validate_ros2_yaml_for_message_schema(*pr.schema, root);
  EXPECT_TRUE(r.ok) << r.error;
}

TEST(YamlMsgValidate, StringMissingFieldFails)
{
  const auto pr = ms::parse_message("std_msgs", "String", "string data\n");
  ASSERT_TRUE(pr.ok()) << pr.error;
  YAML::Node root = YAML::Load("{}");
  auto r = bagwiz::core::validate_ros2_yaml_for_message_schema(*pr.schema, root);
  EXPECT_FALSE(r.ok);
}

TEST(YamlMsgValidate, StringUnknownFieldFails)
{
  const auto pr = ms::parse_message("std_msgs", "String", "string data\n");
  ASSERT_TRUE(pr.ok()) << pr.error;
  YAML::Node root = YAML::Load("data: 'x'\nnope: 1\n");
  auto r = bagwiz::core::validate_ros2_yaml_for_message_schema(*pr.schema, root);
  EXPECT_FALSE(r.ok);
}

TEST(Ros2YamlToCdr, StringRoundTripPayloadNonEmpty)
{
  const auto pr = ms::parse_message("std_msgs", "String", "string data\n");
  ASSERT_TRUE(pr.ok()) << pr.error;
  YAML::Node root = YAML::Load("data: 'joined'");
  const auto vz = bagwiz::core::validate_ros2_yaml_for_message_schema(*pr.schema, root);
  ASSERT_TRUE(vz.ok) << vz.error;
  const auto bytes = bagwiz::core::ros2_yaml_to_cdr_bytes("std_msgs/msg/String", *pr.schema, root);
  ASSERT_TRUE(bytes.ok) << bytes.error;
  EXPECT_GT(bytes.cdr.size(), 10U);
}
