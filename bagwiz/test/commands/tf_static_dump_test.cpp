// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/tf_static_dump.hpp"

#include "bagwiz/core/tf/tf_message_wire.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace
{

using bagwiz::commands::run_tf_static_dump;

constexpr const char * kTfMessageType = "tf2_msgs/msg/TFMessage";

geometry_msgs::msg::TransformStamped make_edge(
  const std::string & parent, const std::string & child, double tx)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = parent;
  ts.header.stamp.sec = 42;
  ts.header.stamp.nanosec = 7;
  ts.child_frame_id = child;
  ts.transform.translation.x = tx;
  ts.transform.rotation.w = 1.0;
  return ts;
}

bagwiz::io::TopicInfo tf_topic_info(const std::string & name)
{
  bagwiz::io::TopicInfo t;
  t.name = name;
  t.type = kTfMessageType;
  t.serialization_format = "cdr";
  t.schema_encoding = "ros2msg";
  t.schema_text = bagwiz::core::kTfMessageWireSchema;
  return t;
}

void write_tf_message(
  bagwiz::io::BagWriter & writer, const std::string & topic, std::int64_t stamp_ns,
  const std::vector<geometry_msgs::msg::TransformStamped> & transforms)
{
  const auto cdr = bagwiz::core::serialize_tf_message(
    std::span<const geometry_msgs::msg::TransformStamped>(transforms.data(), transforms.size()));
  writer.write(topic, stamp_ns, std::span<const std::byte>(cdr.data(), cdr.size()));
}

bagwiz::io::CreateOptions mcap_options()
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = "none";
  return options;
}

// A bag with one static topic (map->odom, odom->base_link) plus a dynamic /tf
// topic that must not reach the dump.
void write_bag(const std::filesystem::path & path)
{
  auto writer = bagwiz::io::open_write(path, mcap_options());
  writer->declare_topic(tf_topic_info("/tf_static"));
  writer->declare_topic(tf_topic_info("/tf"));
  write_tf_message(
    *writer, "/tf_static", 1'000'000'000LL,
    {make_edge("map", "odom", 1.0), make_edge("odom", "base_link", 2.0)});
  write_tf_message(*writer, "/tf", 2'000'000'000LL, {make_edge("odom", "wheel", 9.0)});
  writer->close();
}

void write_bag_without_static_tf(const std::filesystem::path & path)
{
  bagwiz::io::TopicInfo clock;
  clock.name = "/clock";
  clock.type = "std_msgs/msg/String";
  clock.serialization_format = "cdr";

  constexpr std::array<std::byte, 4> kPayload{
    std::byte{0xDE}, std::byte{0xAD}, std::byte{0xBE}, std::byte{0xEF}};

  auto writer = bagwiz::io::open_write(path, mcap_options());
  writer->declare_topic(clock);
  writer->write(
    "/clock", 1'000'000'000LL, std::span<const std::byte>(kPayload.data(), kPayload.size()));
  writer->close();
}

class TfStaticDumpTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_tf_static_dump_" +
                std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()));
    std::filesystem::remove_all(tmp_dir_);
    std::filesystem::create_directories(tmp_dir_);
  }
  void TearDown() override { std::filesystem::remove_all(tmp_dir_); }

  static std::string read_all(const std::filesystem::path & path)
  {
    std::ifstream in(path, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  }

  std::filesystem::path tmp_dir_;
};

TEST_F(TfStaticDumpTest, WritesTheStaticTreeToTheOutputPath)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "tf_static.yaml";
  write_bag(bag);

  ASSERT_EQ(run_tf_static_dump(bag, out, /*overwrite=*/false), 0);

  const YAML::Node doc = YAML::Load(read_all(out));
  ASSERT_TRUE(doc["map"]["odom"]);
  ASSERT_TRUE(doc["odom"]["base_link"]);
  EXPECT_EQ(doc["map"]["odom"]["x"].as<double>(), 1.0);
  EXPECT_EQ(doc["odom"]["base_link"]["x"].as<double>(), 2.0);
  // Identity rotation, and the source stamp (42s/7ns) has nowhere to go.
  EXPECT_EQ(doc["map"]["odom"]["yaw"].as<double>(), 0.0);
  EXPECT_EQ(read_all(out).find("stamp"), std::string::npos);
}

// A dynamic /tf topic describes a moving frame, which a static config cannot
// represent, so it must be left out entirely.
TEST_F(TfStaticDumpTest, IgnoresDynamicTfTopics)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "tf_static.yaml";
  write_bag(bag);

  ASSERT_EQ(run_tf_static_dump(bag, out, /*overwrite=*/false), 0);

  const YAML::Node doc = YAML::Load(read_all(out));
  EXPECT_FALSE(doc["odom"]["wheel"]);
  EXPECT_EQ(doc.size(), 2U);
}

TEST_F(TfStaticDumpTest, WritesTheSameTextToStdoutWhenNoOutputIsGiven)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "tf_static.yaml";
  write_bag(bag);

  ASSERT_EQ(run_tf_static_dump(bag, out, /*overwrite=*/false), 0);
  const std::string from_file = read_all(out);

  testing::internal::CaptureStdout();
  ASSERT_EQ(run_tf_static_dump(bag, std::nullopt, /*overwrite=*/false), 0);
  const std::string from_stdout = testing::internal::GetCapturedStdout();

  EXPECT_FALSE(from_stdout.empty());
  EXPECT_EQ(from_stdout, from_file);
}

// Static TF is latched: the first message already carries the whole tree, so the
// dump reads only that one and never sees the later republications.
TEST_F(TfStaticDumpTest, UsesTheFirstMessageOfEachStaticTopic)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "tf_static.yaml";
  {
    auto writer = bagwiz::io::open_write(bag, mcap_options());
    writer->declare_topic(tf_topic_info("/tf_static"));
    write_tf_message(*writer, "/tf_static", 1'000'000'000LL, {make_edge("map", "odom", 1.0)});
    // Republications a real recording accumulates. Were they read, the last one
    // would win and x would be 3.0.
    write_tf_message(*writer, "/tf_static", 2'000'000'000LL, {make_edge("map", "odom", 2.0)});
    write_tf_message(*writer, "/tf_static", 3'000'000'000LL, {make_edge("map", "odom", 3.0)});
    writer->close();
  }

  ASSERT_EQ(run_tf_static_dump(bag, out, /*overwrite=*/false), 0);

  const YAML::Node doc = YAML::Load(read_all(out));
  EXPECT_EQ(doc["map"]["odom"]["x"].as<double>(), 1.0);
}

// The schema has no topic dimension, so several *tf_static topics have to fuse
// into one tree.
TEST_F(TfStaticDumpTest, MergesEveryStaticTopicIntoOneTree)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "tf_static.yaml";
  {
    auto writer = bagwiz::io::open_write(bag, mcap_options());
    writer->declare_topic(tf_topic_info("/tf_static"));
    writer->declare_topic(tf_topic_info("/sensing/tf_static"));
    write_tf_message(
      *writer, "/tf_static", 1'000'000'000LL, {make_edge("base_link", "sensor", 1.0)});
    write_tf_message(
      *writer, "/sensing/tf_static", 2'000'000'000LL, {make_edge("sensor", "lidar", 2.0)});
    writer->close();
  }

  ASSERT_EQ(run_tf_static_dump(bag, out, /*overwrite=*/false), 0);

  const YAML::Node doc = YAML::Load(read_all(out));
  EXPECT_EQ(doc["base_link"]["sensor"]["x"].as<double>(), 1.0);
  EXPECT_EQ(doc["sensor"]["lidar"]["x"].as<double>(), 2.0);
}

// One child cannot sit under two parents in this schema, and guessing which
// topic is right would silently discard the other, so the run aborts.
TEST_F(TfStaticDumpTest, FailsWhenStaticTopicsDisagreeOnAParent)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "tf_static.yaml";
  {
    auto writer = bagwiz::io::open_write(bag, mcap_options());
    writer->declare_topic(tf_topic_info("/tf_static"));
    writer->declare_topic(tf_topic_info("/sensing/tf_static"));
    write_tf_message(
      *writer, "/tf_static", 1'000'000'000LL, {make_edge("base_link", "lidar", 1.0)});
    write_tf_message(
      *writer, "/sensing/tf_static", 2'000'000'000LL, {make_edge("sensor_kit", "lidar", 2.0)});
    writer->close();
  }

  EXPECT_EQ(run_tf_static_dump(bag, out, /*overwrite=*/false), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

// Two topics declaring the SAME parent for a child is not a contradiction; it is
// the normal result of one broadcaster's set overlapping another's, and it has to
// collapse to a single entry. Which of two differing VALUES survives is left
// unspecified: it follows the bag's topic order, and bagwiz resolves TF merges on
// topology alone (see core::TfMergeConflictChecker, which likewise compares
// parents rather than values).
TEST_F(TfStaticDumpTest, CollapsesTheSameEdgeFromTwoStaticTopics)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "tf_static.yaml";
  {
    auto writer = bagwiz::io::open_write(bag, mcap_options());
    writer->declare_topic(tf_topic_info("/tf_static"));
    writer->declare_topic(tf_topic_info("/sensing/tf_static"));
    write_tf_message(
      *writer, "/tf_static", 1'000'000'000LL, {make_edge("base_link", "lidar", 1.0)});
    write_tf_message(
      *writer, "/sensing/tf_static", 2'000'000'000LL, {make_edge("base_link", "lidar", 5.0)});
    writer->close();
  }

  ASSERT_EQ(run_tf_static_dump(bag, out, /*overwrite=*/false), 0);

  const YAML::Node doc = YAML::Load(read_all(out));
  EXPECT_EQ(doc.size(), 1U);
  ASSERT_TRUE(doc["base_link"]["lidar"]);
  EXPECT_EQ(doc["base_link"].size(), 1U);
  // One of the two, never an invented third value.
  const double x = doc["base_link"]["lidar"]["x"].as<double>();
  EXPECT_TRUE(x == 1.0 || x == 5.0) << x;
}

TEST_F(TfStaticDumpTest, FailsWhenTheBagHasNoStaticTf)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "tf_static.yaml";
  write_bag_without_static_tf(bag);

  EXPECT_EQ(run_tf_static_dump(bag, out, /*overwrite=*/false), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

TEST_F(TfStaticDumpTest, RefusesAnExistingOutputWithoutOverwrite)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "tf_static.yaml";
  write_bag(bag);
  std::ofstream(out) << "precious\n";

  EXPECT_EQ(run_tf_static_dump(bag, out, /*overwrite=*/false), 1);
  EXPECT_EQ(read_all(out), "precious\n");
}

TEST_F(TfStaticDumpTest, HonoursOverwrite)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "tf_static.yaml";
  write_bag(bag);
  std::ofstream(out) << "stale\n";

  ASSERT_EQ(run_tf_static_dump(bag, out, /*overwrite=*/true), 0);
  EXPECT_TRUE(YAML::Load(read_all(out))["map"]["odom"]);
}

// prepare_output_path() deletes an existing path under --overwrite, so claiming
// the output before the read would let a bag with no static TF destroy the
// user's file and then fail. Only the read can tell the two cases apart.
TEST_F(TfStaticDumpTest, DoesNotClobberOutputWhenTheBagHasNoStaticTf)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "tf_static.yaml";
  write_bag_without_static_tf(bag);
  std::ofstream(out) << "precious\n";

  EXPECT_EQ(run_tf_static_dump(bag, out, /*overwrite=*/true), 1);
  EXPECT_EQ(read_all(out), "precious\n");
}

TEST_F(TfStaticDumpTest, LeavesTheBagUnmodified)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "tf_static.yaml";
  write_bag(bag);
  const std::string before = read_all(bag);

  ASSERT_EQ(run_tf_static_dump(bag, out, /*overwrite=*/false), 0);

  EXPECT_EQ(read_all(bag), before);
}

}  // namespace
