// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagcli/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace
{

// Deterministic 4-byte payload. The CDR envelope would be larger in real
// ROS 2 messages, but bagcli writers do not interpret the payload so any
// byte sequence is valid for round-trip testing.
constexpr std::array<std::uint8_t, 4> kPayload{0xDE, 0xAD, 0xBE, 0xEF};

bagcli::io::TopicInfo make_topic(std::string name, std::string type)
{
  bagcli::io::TopicInfo t;
  t.name = std::move(name);
  t.type = std::move(type);
  t.serialization_format = "cdr";
  return t;
}

void write_fixture(
  const std::filesystem::path & path, bagcli::io::CreateOptions options,
  const std::vector<bagcli::io::TopicInfo> & topics,
  const std::vector<std::pair<std::string, int64_t>> & messages)
{
  auto writer = bagcli::io::open_write(path, options);
  for (const auto & t : topics) {
    writer->declare_topic(t);
  }
  for (const auto & [topic, ts] : messages) {
    writer->write(
      topic, ts,
      std::span<const std::byte>(
        reinterpret_cast<const std::byte *>(kPayload.data()), kPayload.size()));
  }
  writer->close();
}

void verify_round_trip(const std::filesystem::path & path)
{
  auto reader = bagcli::io::open_read(path);

  const auto topics = reader->topics();
  ASSERT_EQ(topics.size(), 2U);

  bool seen_foo = false;
  bool seen_bar = false;
  for (const auto & t : topics) {
    if (t.name == "/foo") {
      seen_foo = true;
      EXPECT_EQ(t.type, "std_msgs/msg/String");
    } else if (t.name == "/bar") {
      seen_bar = true;
      EXPECT_EQ(t.type, "std_msgs/msg/Int32");
    }
  }
  EXPECT_TRUE(seen_foo);
  EXPECT_TRUE(seen_bar);

  int foo_count = 0;
  int bar_count = 0;
  bagcli::io::RawMessage msg;
  int64_t last_ts = -1;
  while (reader->next(msg)) {
    EXPECT_GE(msg.timestamp_ns, last_ts);
    last_ts = msg.timestamp_ns;
    EXPECT_EQ(msg.payload.size(), kPayload.size());
    if (msg.topic->name == "/foo") {
      ++foo_count;
    } else if (msg.topic->name == "/bar") {
      ++bar_count;
    }
  }
  EXPECT_EQ(foo_count, 3);
  EXPECT_EQ(bar_count, 2);

  const auto stats = reader->compute_stats();
  EXPECT_EQ(stats.total_messages, 5);
  EXPECT_EQ(stats.per_topic.at("/foo"), 3);
  EXPECT_EQ(stats.per_topic.at("/bar"), 2);
}

class WriterRoundTripTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagcli_writer_test_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
    std::filesystem::create_directories(tmp_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::vector<bagcli::io::TopicInfo> topics_ = {
    make_topic("/foo", "std_msgs/msg/String"), make_topic("/bar", "std_msgs/msg/Int32")};

  std::vector<std::pair<std::string, int64_t>> messages_ = {
    {"/foo", 1'000'000'000LL},
    {"/foo", 1'000'000'001LL},
    {"/foo", 1'000'000'002LL},
    {"/bar", 2'000'000'000LL},
    {"/bar", 2'000'000'001LL}};

  std::filesystem::path tmp_dir_;
};

}  // namespace

TEST_F(WriterRoundTripTest, McapSingleFile)
{
  const auto path = tmp_dir_ / "out.mcap";
  bagcli::io::CreateOptions options;
  options.format = bagcli::io::Format::Mcap;
  options.layout = bagcli::io::Layout::SingleFile;
  options.mcap_compression = "none";  // keep test predictable & fast
  write_fixture(path, options, topics_, messages_);
  verify_round_trip(path);
}

TEST_F(WriterRoundTripTest, McapDirectory)
{
  const auto dir = tmp_dir_ / "mcap_dir";
  bagcli::io::CreateOptions options;
  options.format = bagcli::io::Format::Mcap;
  options.layout = bagcli::io::Layout::Directory;
  options.mcap_compression = "none";
  write_fixture(dir, options, topics_, messages_);

  // metadata.yaml should exist and the directory should round-trip.
  EXPECT_TRUE(std::filesystem::exists(dir / "metadata.yaml"));
  verify_round_trip(dir);
}

TEST_F(WriterRoundTripTest, McapZstdCompression)
{
  const auto path = tmp_dir_ / "zstd.mcap";
  bagcli::io::CreateOptions options;
  options.format = bagcli::io::Format::Mcap;
  options.layout = bagcli::io::Layout::SingleFile;
  options.mcap_compression = "zstd";
  write_fixture(path, options, topics_, messages_);
  verify_round_trip(path);
}

TEST_F(WriterRoundTripTest, Sqlite3SingleFile)
{
  const auto path = tmp_dir_ / "out.db3";
  bagcli::io::CreateOptions options;
  options.format = bagcli::io::Format::Sqlite3;
  options.layout = bagcli::io::Layout::SingleFile;
  write_fixture(path, options, topics_, messages_);
  verify_round_trip(path);
}

TEST_F(WriterRoundTripTest, Sqlite3Directory)
{
  const auto dir = tmp_dir_ / "sqlite_dir";
  bagcli::io::CreateOptions options;
  options.format = bagcli::io::Format::Sqlite3;
  options.layout = bagcli::io::Layout::Directory;
  write_fixture(dir, options, topics_, messages_);

  EXPECT_TRUE(std::filesystem::exists(dir / "metadata.yaml"));
  verify_round_trip(dir);
}

TEST_F(WriterRoundTripTest, AutoLayoutFromExtension)
{
  // .mcap → SingleFile + Mcap
  const auto mcap_path = tmp_dir_ / "auto.mcap";
  write_fixture(mcap_path, {}, topics_, messages_);
  verify_round_trip(mcap_path);

  // .db3 → SingleFile + Sqlite3
  const auto db_path = tmp_dir_ / "auto.db3";
  write_fixture(db_path, {}, topics_, messages_);
  verify_round_trip(db_path);

  // Bare name → Directory
  const auto dir_path = tmp_dir_ / "auto_dir";
  write_fixture(dir_path, {}, topics_, messages_);
  EXPECT_TRUE(std::filesystem::exists(dir_path / "metadata.yaml"));
  verify_round_trip(dir_path);
}
