// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf_message_wire.hpp"

#include <gtest/gtest.h>

#include <string>
#include <string_view>

namespace
{

using bagwiz::core::kTfMessageWireSchema;
using bagwiz::core::make_tf_message_topic_info;

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

}  // namespace
