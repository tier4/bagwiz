// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagcli/io/bag_io.hpp"

#include <gtest/gtest.h>

// MCAP_IMPLEMENTATION lives in src/io/mcap_reader.cpp (bagcli_core). Tests
// only need the declarations here.
#include <mcap/writer.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace
{

// Write a self-contained .mcap file with two topics and a handful of
// messages. Returns the path to the created file.
std::filesystem::path write_fixture_mcap(const std::filesystem::path & dir)
{
  std::filesystem::create_directories(dir);
  const auto path = dir / "bag_0.mcap";

  mcap::McapWriter writer;
  mcap::McapWriterOptions opts("ros2");
  opts.compression = mcap::Compression::None;

  const auto status = writer.open(path.string(), opts);
  EXPECT_TRUE(status.ok()) << status.message;

  mcap::Schema schema_string("std_msgs/msg/String", "ros2msg", "string data");
  writer.addSchema(schema_string);

  mcap::Schema schema_int("std_msgs/msg/Int32", "ros2msg", "int32 data");
  writer.addSchema(schema_int);

  mcap::Channel chan_foo("/foo", "cdr", schema_string.id);
  chan_foo.metadata["offered_qos_profiles"] = "- history: 3\n";
  writer.addChannel(chan_foo);

  mcap::Channel chan_bar("/bar", "cdr", schema_int.id);
  writer.addChannel(chan_bar);

  const std::array<std::uint8_t, 4> payload{0xDE, 0xAD, 0xBE, 0xEF};

  for (int i = 0; i < 3; ++i) {
    mcap::Message msg;
    msg.channelId = chan_foo.id;
    msg.sequence = static_cast<uint32_t>(i);
    msg.logTime = static_cast<mcap::Timestamp>(1'000'000'000ULL + i);
    msg.publishTime = msg.logTime;
    msg.data = reinterpret_cast<const std::byte *>(payload.data());
    msg.dataSize = payload.size();
    writer.write(msg);
  }
  for (int i = 0; i < 2; ++i) {
    mcap::Message msg;
    msg.channelId = chan_bar.id;
    msg.sequence = static_cast<uint32_t>(i);
    msg.logTime = static_cast<mcap::Timestamp>(2'000'000'000ULL + i);
    msg.publishTime = msg.logTime;
    msg.data = reinterpret_cast<const std::byte *>(payload.data());
    msg.dataSize = payload.size();
    writer.write(msg);
  }

  writer.close();
  return path;
}

std::filesystem::path write_fixture_directory(const std::filesystem::path & dir)
{
  std::filesystem::create_directories(dir);
  write_fixture_mcap(dir);

  const auto metadata_path = dir / "metadata.yaml";
  std::ofstream f(metadata_path);
  f << "rosbag2_bagfile_information:\n"
    << "  version: 6\n"
    << "  storage_identifier: mcap\n"
    << "  relative_file_paths:\n"
    << "    - bag_0.mcap\n"
    << "  topics_with_message_count:\n"
    << "    - topic_metadata:\n"
    << "        name: /foo\n"
    << "        type: std_msgs/msg/String\n"
    << "        serialization_format: cdr\n"
    << "        offered_qos_profiles: ''\n"
    << "      message_count: 3\n"
    << "    - topic_metadata:\n"
    << "        name: /bar\n"
    << "        type: std_msgs/msg/Int32\n"
    << "        serialization_format: cdr\n"
    << "        offered_qos_profiles: ''\n"
    << "      message_count: 2\n";
  f.close();
  return dir;
}

class McapReaderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagcli_mcap_test_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
    std::filesystem::create_directories(tmp_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
};

}  // namespace

TEST_F(McapReaderTest, OpensSingleFileAndListsTopics)
{
  const auto path = write_fixture_mcap(tmp_dir_ / "single");

  auto reader = bagcli::io::open_read(path);
  const auto topics = reader->topics();
  ASSERT_EQ(topics.size(), 2U);

  bool seen_foo = false;
  bool seen_bar = false;
  for (const auto & t : topics) {
    if (t.name == "/foo") {
      seen_foo = true;
      EXPECT_EQ(t.type, "std_msgs/msg/String");
      EXPECT_EQ(t.serialization_format, "cdr");
    } else if (t.name == "/bar") {
      seen_bar = true;
      EXPECT_EQ(t.type, "std_msgs/msg/Int32");
    }
  }
  EXPECT_TRUE(seen_foo);
  EXPECT_TRUE(seen_bar);
}

TEST_F(McapReaderTest, IteratesMessagesInTimeOrder)
{
  const auto path = write_fixture_mcap(tmp_dir_ / "iter");

  auto reader = bagcli::io::open_read(path);

  std::vector<std::pair<std::string, int64_t>> seen;
  bagcli::io::RawMessage msg;
  while (reader->next(msg)) {
    seen.emplace_back(std::string(msg.topic->name), msg.timestamp_ns);
    EXPECT_EQ(msg.payload.size(), 4U);
  }

  ASSERT_EQ(seen.size(), 5U);
  for (std::size_t i = 1; i < seen.size(); ++i) {
    EXPECT_LE(seen[i - 1].second, seen[i].second);
  }
}

TEST_F(McapReaderTest, FilterByTopic)
{
  const auto path = write_fixture_mcap(tmp_dir_ / "filter");

  auto reader = bagcli::io::open_read(path);
  bagcli::io::ReadFilter f;
  f.topics = {"/bar"};
  reader->set_filter(f);

  int count = 0;
  bagcli::io::RawMessage msg;
  while (reader->next(msg)) {
    EXPECT_EQ(std::string(msg.topic->name), "/bar");
    ++count;
  }
  EXPECT_EQ(count, 2);
}

TEST_F(McapReaderTest, StatsFromSummary)
{
  const auto path = write_fixture_mcap(tmp_dir_ / "stats");

  auto reader = bagcli::io::open_read(path);
  const auto stats = reader->compute_stats();

  EXPECT_TRUE(stats.from_summary);
  EXPECT_EQ(stats.total_messages, 5);
  EXPECT_EQ(stats.per_topic.at("/foo"), 3);
  EXPECT_EQ(stats.per_topic.at("/bar"), 2);
}

TEST_F(McapReaderTest, OpensDirectoryWithMetadata)
{
  const auto dir = write_fixture_directory(tmp_dir_ / "dir");

  auto reader = bagcli::io::open_read(dir);
  EXPECT_EQ(reader->topics().size(), 2U);

  int count = 0;
  bagcli::io::RawMessage msg;
  while (reader->next(msg)) {
    ++count;
  }
  EXPECT_EQ(count, 5);
}

TEST_F(McapReaderTest, RejectsDirectoryWithoutMetadata)
{
  const auto dir = tmp_dir_ / "no_meta";
  std::filesystem::create_directories(dir);
  EXPECT_THROW(bagcli::io::open_read(dir), std::exception);
}

TEST_F(McapReaderTest, RejectsMissingPath)
{
  EXPECT_THROW(bagcli::io::open_read(tmp_dir_ / "does_not_exist.mcap"), std::exception);
}
