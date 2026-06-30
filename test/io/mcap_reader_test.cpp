// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

// MCAP_IMPLEMENTATION lives in src/io/mcap_reader.cpp (bagwiz_core). Tests
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
    const auto write_status = writer.write(msg);
    EXPECT_TRUE(write_status.ok()) << write_status.message;
  }
  for (int i = 0; i < 2; ++i) {
    mcap::Message msg;
    msg.channelId = chan_bar.id;
    msg.sequence = static_cast<uint32_t>(i);
    msg.logTime = static_cast<mcap::Timestamp>(2'000'000'000ULL + i);
    msg.publishTime = msg.logTime;
    msg.data = reinterpret_cast<const std::byte *>(payload.data());
    msg.dataSize = payload.size();
    const auto write_status = writer.write(msg);
    EXPECT_TRUE(write_status.ok()) << write_status.message;
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

// Metadata.yaml carries the full summary block (message_count + starting_time
// + duration + topics_with_message_count). The shard file is intentionally
// not created: any code path that opens it will fail loudly.
std::filesystem::path write_fixture_directory_summary_only(const std::filesystem::path & dir)
{
  std::filesystem::create_directories(dir);

  const auto metadata_path = dir / "metadata.yaml";
  std::ofstream f(metadata_path);
  f << "rosbag2_bagfile_information:\n"
    << "  version: 6\n"
    << "  storage_identifier: mcap\n"
    << "  duration:\n"
    << "    nanoseconds: 1000000001\n"
    << "  starting_time:\n"
    << "    nanoseconds_since_epoch: 1000000000\n"
    << "  message_count: 5\n"
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
      ("bagwiz_mcap_test_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
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

  auto reader = bagwiz::io::open_read(path);
  const auto topics = reader->topics();
  ASSERT_EQ(topics.size(), 2U);

  bool seen_foo = false;
  bool seen_bar = false;
  for (const auto & t : topics) {
    if (t.name == "/foo") {
      seen_foo = true;
      EXPECT_EQ(t.type, "std_msgs/msg/String");
      EXPECT_EQ(t.serialization_format, "cdr");
      // Schema bytes from the fixture's mcap::Schema(... "ros2msg",
      // "string data") are surfaced through TopicInfo.
      EXPECT_EQ(t.schema_encoding, "ros2msg");
      EXPECT_EQ(t.schema_text, "string data");
    } else if (t.name == "/bar") {
      seen_bar = true;
      EXPECT_EQ(t.type, "std_msgs/msg/Int32");
      EXPECT_EQ(t.schema_encoding, "ros2msg");
      EXPECT_EQ(t.schema_text, "int32 data");
    }
  }
  EXPECT_TRUE(seen_foo);
  EXPECT_TRUE(seen_bar);
}

TEST_F(McapReaderTest, DirectoryPopulatesSchemasOnDemand)
{
  // metadata.yaml does not carry schemas — they live only inside the
  // individual shard files. topics() before populate_schemas()
  // therefore returns empty schemas, and populate_schemas() backfills
  // them by reading shard 0 on demand.
  const auto dir = write_fixture_directory(tmp_dir_ / "schemas_lazy");

  auto reader = bagwiz::io::open_read(dir);
  for (const auto & t : reader->topics()) {
    EXPECT_TRUE(t.schema_text.empty()) << t.name << " had unexpected schema before lazy load";
    EXPECT_TRUE(t.schema_encoding.empty()) << t.name;
  }

  reader->populate_schemas();
  bool found_foo_schema = false;
  bool found_bar_schema = false;
  for (const auto & t : reader->topics()) {
    if (t.name == "/foo") {
      EXPECT_EQ(t.schema_text, "string data");
      EXPECT_EQ(t.schema_encoding, "ros2msg");
      found_foo_schema = true;
    } else if (t.name == "/bar") {
      EXPECT_EQ(t.schema_text, "int32 data");
      EXPECT_EQ(t.schema_encoding, "ros2msg");
      found_bar_schema = true;
    }
  }
  EXPECT_TRUE(found_foo_schema);
  EXPECT_TRUE(found_bar_schema);
}

TEST_F(McapReaderTest, DirectoryNextBackfillsSchemaOpportunistically)
{
  // Even without an explicit populate_schemas() call, iteration through
  // next() should backfill schemas onto the shared TopicInfo so consumers
  // that follow the iterator see them.
  const auto dir = write_fixture_directory(tmp_dir_ / "schemas_via_next");

  auto reader = bagwiz::io::open_read(dir);
  bagwiz::io::RawMessage msg;
  ASSERT_TRUE(reader->next(msg));
  // The TopicInfo* the message points at is owned by the shard reader; it
  // must carry the schema from the MCAP shard.
  EXPECT_FALSE(msg.topic->schema_text.empty());
  EXPECT_EQ(msg.topic->schema_encoding, "ros2msg");
}

TEST_F(McapReaderTest, IteratesMessagesInTimeOrder)
{
  const auto path = write_fixture_mcap(tmp_dir_ / "iter");

  auto reader = bagwiz::io::open_read(path);

  std::vector<std::pair<std::string, int64_t>> seen;
  bagwiz::io::RawMessage msg;
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

  auto reader = bagwiz::io::open_read(path);
  bagwiz::io::ReadFilter f;
  f.topics = {"/bar"};
  reader->set_filter(f);

  int count = 0;
  bagwiz::io::RawMessage msg;
  while (reader->next(msg)) {
    EXPECT_EQ(std::string(msg.topic->name), "/bar");
    ++count;
  }
  EXPECT_EQ(count, 2);
}

TEST_F(McapReaderTest, StatsFromSummary)
{
  const auto path = write_fixture_mcap(tmp_dir_ / "stats");

  auto reader = bagwiz::io::open_read(path);
  const auto stats = reader->compute_stats();

  EXPECT_TRUE(stats.from_summary);
  EXPECT_EQ(stats.total_messages, 5);
  EXPECT_EQ(stats.per_topic.at("/foo"), 3);
  EXPECT_EQ(stats.per_topic.at("/bar"), 2);
}

TEST_F(McapReaderTest, OpensDirectoryWithMetadata)
{
  const auto dir = write_fixture_directory(tmp_dir_ / "dir");

  auto reader = bagwiz::io::open_read(dir);
  EXPECT_EQ(reader->topics().size(), 2U);

  int count = 0;
  bagwiz::io::RawMessage msg;
  while (reader->next(msg)) {
    ++count;
  }
  EXPECT_EQ(count, 5);
}

TEST_F(McapReaderTest, DirectoryStatsServedFromMetadataWithoutOpeningShards)
{
  // The fixture writes only metadata.yaml; the shard file is absent. If
  // open_read or compute_stats touches the shard, it will throw. This is
  // the proof that the metadata fast path is actually taken.
  const auto dir = write_fixture_directory_summary_only(tmp_dir_ / "summary_only");

  auto reader = bagwiz::io::open_read(dir);
  EXPECT_EQ(reader->topics().size(), 2U);

  const auto stats = reader->compute_stats();
  EXPECT_TRUE(stats.from_summary);
  EXPECT_EQ(stats.total_messages, 5);
  EXPECT_EQ(stats.start_ns, 1'000'000'000LL);
  EXPECT_EQ(stats.end_ns, 2'000'000'001LL);
  EXPECT_EQ(stats.per_topic.at("/foo"), 3);
  EXPECT_EQ(stats.per_topic.at("/bar"), 2);
}

TEST_F(McapReaderTest, TopicCountsFromSummary)
{
  const auto path = write_fixture_mcap(tmp_dir_ / "topic_counts");

  auto reader = bagwiz::io::open_read(path);
  const auto counts = reader->compute_topic_counts(std::vector<std::string>{"/foo"});

  EXPECT_EQ(counts.size(), 1U);
  EXPECT_EQ(counts.at("/foo"), 3);
}

TEST_F(McapReaderTest, TopicCountsForMultipleTopics)
{
  const auto path = write_fixture_mcap(tmp_dir_ / "topic_counts_multi");

  auto reader = bagwiz::io::open_read(path);
  const auto counts =
    reader->compute_topic_counts(std::vector<std::string>{"/foo", "/bar", "/unknown"});

  EXPECT_EQ(counts.size(), 2U);
  EXPECT_EQ(counts.at("/foo"), 3);
  EXPECT_EQ(counts.at("/bar"), 2);
}

TEST_F(McapReaderTest, DirectoryTopicCountsServedFromMetadataWithoutOpeningShards)
{
  const auto dir = write_fixture_directory_summary_only(tmp_dir_ / "topic_counts_summary_only");

  auto reader = bagwiz::io::open_read(dir);
  const auto counts = reader->compute_topic_counts(std::vector<std::string>{"/foo", "/bar"});

  EXPECT_EQ(counts.size(), 2U);
  EXPECT_EQ(counts.at("/foo"), 3);
  EXPECT_EQ(counts.at("/bar"), 2);
}

TEST_F(McapReaderTest, TimeExtentFromSummary)
{
  const auto path = write_fixture_mcap(tmp_dir_ / "time_extent");

  auto reader = bagwiz::io::open_read(path);
  const auto extent = reader->compute_time_extent();

  EXPECT_TRUE(extent.has_data);
  EXPECT_EQ(extent.start_ns, 1'000'000'000LL);
  EXPECT_EQ(extent.end_ns, 2'000'000'001LL);
}

TEST_F(McapReaderTest, TimeExtentEmptyBag)
{
  std::filesystem::create_directories(tmp_dir_ / "time_extent_empty");
  const auto path = tmp_dir_ / "time_extent_empty" / "empty.mcap";

  mcap::McapWriter writer;
  mcap::McapWriterOptions opts("ros2");
  opts.compression = mcap::Compression::None;
  const auto status = writer.open(path.string(), opts);
  EXPECT_TRUE(status.ok()) << status.message;
  writer.close();

  auto reader = bagwiz::io::open_read(path);
  const auto extent = reader->compute_time_extent();

  EXPECT_FALSE(extent.has_data);
  EXPECT_EQ(extent.start_ns, 0);
  EXPECT_EQ(extent.end_ns, 0);
}

TEST_F(McapReaderTest, DirectoryTimeExtentServedFromMetadataWithoutOpeningShards)
{
  const auto dir = write_fixture_directory_summary_only(tmp_dir_ / "time_extent_summary_only");

  auto reader = bagwiz::io::open_read(dir);
  const auto extent = reader->compute_time_extent();

  EXPECT_TRUE(extent.has_data);
  EXPECT_EQ(extent.start_ns, 1'000'000'000LL);
  EXPECT_EQ(extent.end_ns, 2'000'000'001LL);
}

TEST_F(McapReaderTest, RejectsDirectoryWithoutMetadata)
{
  const auto dir = tmp_dir_ / "no_meta";
  std::filesystem::create_directories(dir);
  EXPECT_THROW(bagwiz::io::open_read(dir), std::exception);
}

TEST_F(McapReaderTest, RejectsMissingPath)
{
  EXPECT_THROW(bagwiz::io::open_read(tmp_dir_ / "does_not_exist.mcap"), std::exception);
}
