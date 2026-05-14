// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/bag_topic_plan.hpp"

#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <string>
#include <utility>
#include <vector>

namespace
{

using bagwiz::core::decide_topic_write;
using bagwiz::core::TopicWriteAction;

bagwiz::io::TopicInfo make_topic(std::string name, std::string type)
{
  bagwiz::io::TopicInfo t;
  t.name = std::move(name);
  t.type = std::move(type);
  t.serialization_format = "cdr";
  return t;
}

TEST(DecideTopicWrite, DeclaresNewWhenAbsent)
{
  const std::vector<bagwiz::io::TopicInfo> topics = {
    make_topic("/other", "std_msgs/msg/String"),
  };
  const auto d = decide_topic_write(topics, "/trajectory/tf", "tf2_msgs/msg/TFMessage", 0, false);
  EXPECT_EQ(d.action, TopicWriteAction::kDeclareNew);
  EXPECT_EQ(d.existing_count, 0);
  EXPECT_FALSE(d.reason.empty());
}

TEST(DecideTopicWrite, KeepsExistingDeclarationWhenEmpty)
{
  const std::vector<bagwiz::io::TopicInfo> topics = {
    make_topic("/trajectory/tf", "tf2_msgs/msg/TFMessage"),
  };
  const auto d = decide_topic_write(topics, "/trajectory/tf", "tf2_msgs/msg/TFMessage", 0, false);
  EXPECT_EQ(d.action, TopicWriteAction::kDeclareKeep);
  EXPECT_EQ(d.existing_count, 0);
}

TEST(DecideTopicWrite, AbortsOnConflictWithoutForce)
{
  const std::vector<bagwiz::io::TopicInfo> topics = {
    make_topic("/trajectory/tf", "tf2_msgs/msg/TFMessage"),
  };
  const auto d = decide_topic_write(topics, "/trajectory/tf", "tf2_msgs/msg/TFMessage", 42, false);
  EXPECT_EQ(d.action, TopicWriteAction::kConflictAbort);
  EXPECT_EQ(d.existing_count, 42);
  EXPECT_NE(d.reason.find("--force"), std::string::npos);
}

TEST(DecideTopicWrite, SuppressesAndDeclaresUnderForce)
{
  const std::vector<bagwiz::io::TopicInfo> topics = {
    make_topic("/trajectory/tf", "tf2_msgs/msg/TFMessage"),
  };
  const auto d = decide_topic_write(topics, "/trajectory/tf", "tf2_msgs/msg/TFMessage", 17, true);
  EXPECT_EQ(d.action, TopicWriteAction::kDeclareAndSuppress);
  EXPECT_EQ(d.existing_count, 17);
}

TEST(DecideTopicWrite, FlagsTypeMismatchEvenUnderForce)
{
  const std::vector<bagwiz::io::TopicInfo> topics = {
    make_topic("/trajectory/tf", "geometry_msgs/msg/PoseStamped"),
  };
  const auto d = decide_topic_write(topics, "/trajectory/tf", "tf2_msgs/msg/TFMessage", 1, true);
  EXPECT_EQ(d.action, TopicWriteAction::kTypeMismatch);
  EXPECT_NE(d.reason.find("incompatible"), std::string::npos);
}

TEST(DecideTopicWrite, FlagsTypeMismatchEvenWithoutMessages)
{
  // Type mismatch precedes the count==0 fast-path: if the type is
  // wrong we never want to silently overwrite the declaration.
  const std::vector<bagwiz::io::TopicInfo> topics = {
    make_topic("/trajectory/tf", "geometry_msgs/msg/PoseStamped"),
  };
  const auto d = decide_topic_write(topics, "/trajectory/tf", "tf2_msgs/msg/TFMessage", 0, false);
  EXPECT_EQ(d.action, TopicWriteAction::kTypeMismatch);
}

}  // namespace
