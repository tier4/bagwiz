// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/msg_yaml/msg_definition_resolver.hpp"

#include <gtest/gtest.h>

#include <string>

namespace
{

// Re-resolve every test input to confirm the cache returns identical
// results — guards against the cache leaking state across calls.
bagwiz::core::ResolvedMessageDefinition resolve_twice(const std::string & type)
{
  const auto first = bagwiz::core::resolve_message_definition(type);
  const auto second = bagwiz::core::resolve_message_definition(type);
  EXPECT_EQ(first.text, second.text);
  EXPECT_EQ(first.encoding, second.encoding);
  return first;
}

}  // namespace

TEST(MsgDefinitionResolver, ResolvesStdMsgsString)
{
  const auto r = resolve_twice("std_msgs/msg/String");
  ASSERT_FALSE(r.text.empty());
  EXPECT_EQ(r.encoding, "ros2msg");
  // String.msg is just `string data`. No deps, so no separator.
  EXPECT_NE(r.text.find("string data"), std::string::npos);
  EXPECT_EQ(
    r.text.find("================================================================================"),
    std::string::npos)
    << "String.msg has no deps; text should not contain a separator";
}

TEST(MsgDefinitionResolver, ResolvesStdMsgsHeaderWithBuiltinInterfacesTimeDep)
{
  // std_msgs/Header references builtin_interfaces/Time. The resolver
  // must follow that reference and emit the Time .msg as a separated
  // dependency block.
  const auto r = resolve_twice("std_msgs/msg/Header");
  ASSERT_FALSE(r.text.empty());
  EXPECT_EQ(r.encoding, "ros2msg");
  EXPECT_NE(r.text.find("builtin_interfaces/Time stamp"), std::string::npos);
  EXPECT_NE(r.text.find("MSG: builtin_interfaces/Time"), std::string::npos);
  EXPECT_NE(r.text.find("int32 sec"), std::string::npos);
  EXPECT_NE(r.text.find("uint32 nanosec"), std::string::npos);
}

TEST(MsgDefinitionResolver, ResolvesSensorMsgsImuWithTransitiveDeps)
{
  // sensor_msgs/Imu references std_msgs/Header,
  // geometry_msgs/Quaternion, and geometry_msgs/Vector3. Each of these
  // must appear in the assembled output exactly once even though Imu
  // references Vector3 multiple times (angular_velocity and
  // linear_acceleration both use Vector3).
  const auto r = resolve_twice("sensor_msgs/msg/Imu");
  ASSERT_FALSE(r.text.empty());
  EXPECT_EQ(r.encoding, "ros2msg");

  const auto count_occurrences = [](const std::string & hay, const std::string & needle) {
    std::size_t count = 0;
    std::size_t pos = 0;
    while ((pos = hay.find(needle, pos)) != std::string::npos) {
      ++count;
      pos += needle.size();
    }
    return count;
  };
  EXPECT_EQ(count_occurrences(r.text, "MSG: std_msgs/Header"), 1U);
  EXPECT_EQ(count_occurrences(r.text, "MSG: geometry_msgs/Vector3"), 1U);
  EXPECT_EQ(count_occurrences(r.text, "MSG: geometry_msgs/Quaternion"), 1U);
  EXPECT_EQ(count_occurrences(r.text, "MSG: builtin_interfaces/Time"), 1U);
}

TEST(MsgDefinitionResolver, ResolvesSamePackageShorthand)
{
  // diagnostic_msgs/DiagnosticStatus uses `KeyValue[] values` — the
  // bare `KeyValue` token must resolve as `diagnostic_msgs/KeyValue`,
  // not be misclassified as an unknown type.
  const auto r = resolve_twice("diagnostic_msgs/msg/DiagnosticStatus");
  ASSERT_FALSE(r.text.empty());
  EXPECT_EQ(r.encoding, "ros2msg");
  EXPECT_NE(r.text.find("MSG: diagnostic_msgs/KeyValue"), std::string::npos);
}

TEST(MsgDefinitionResolver, AcceptsLegacyShortFormInput)
{
  // The resolver should accept either the canonical `pkg/msg/Type`
  // form or the legacy `pkg/Type` form on input.
  const auto a = resolve_twice("std_msgs/msg/Header");
  const auto b = resolve_twice("std_msgs/Header");
  EXPECT_EQ(a.text, b.text);
  EXPECT_EQ(a.encoding, b.encoding);
}

TEST(MsgDefinitionResolver, ReturnsEmptyForUnknownPackage)
{
  // Resolution must fail cleanly (empty result, no exception) when a
  // package isn't installed.
  const auto r = resolve_twice("nonexistent_pkg_qwxyz/msg/Type");
  EXPECT_TRUE(r.text.empty());
  EXPECT_TRUE(r.encoding.empty());
}

TEST(MsgDefinitionResolver, ReturnsEmptyForMalformedTypeName)
{
  EXPECT_TRUE(bagwiz::core::resolve_message_definition("").text.empty());
  EXPECT_TRUE(bagwiz::core::resolve_message_definition("just_a_word").text.empty());
  EXPECT_TRUE(bagwiz::core::resolve_message_definition("/").text.empty());
}

TEST(MsgDefinitionResolver, OutputBeginsWithTopLevelBody)
{
  // The first line of the output must be the top-level type's .msg
  // text (specifically NOT a `MSG:` header — that's only for deps).
  // This matches the rosbag2 mcap convention: top-level body comes
  // first, dependencies follow after the 80-`=` separator.
  const auto r = resolve_twice("std_msgs/msg/Header");
  ASSERT_FALSE(r.text.empty());
  EXPECT_NE(r.text.substr(0, 5), "MSG: ")
    << "Top-level body must not be prefixed with a MSG: header";
}
