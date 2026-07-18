// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/topics.hpp"

#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

namespace
{

constexpr std::array<std::uint8_t, 4> kPayload{0x11, 0x22, 0x33, 0x44};

// Materialise a small single-file MCAP with the given topic names (all of the
// same type) and one message on the first topic.
void seed_bag(const std::filesystem::path & path, const std::vector<std::string> & topic_names)
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = "none";
  auto writer = bagwiz::io::open_write(path, options);
  for (const auto & name : topic_names) {
    bagwiz::io::TopicInfo t;
    t.name = name;
    t.type = "std_msgs/msg/Int32";
    t.serialization_format = "cdr";
    writer->declare_topic(t);
  }
  writer->write(
    topic_names.front(), 1'000'000'000LL,
    std::span<const std::byte>(
      reinterpret_cast<const std::byte *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        kPayload.data()),
      kPayload.size()));
  writer->close();
}

class TopicsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_topics_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
    std::filesystem::create_directories(tmp_dir_);
    bag_path_ = tmp_dir_ / "topics.mcap";
    seed_bag(bag_path_, {"/alpha", "/beta"});
    reader_ = bagwiz::io::open_read(bag_path_);
  }

  void TearDown() override
  {
    reader_.reset();
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
  std::filesystem::path bag_path_;
  std::unique_ptr<bagwiz::io::BagReader> reader_;
};

constexpr const char * kLogger = "bagwiz.test.topics";

}  // namespace

TEST_F(TopicsTest, SnapshotTopicNamesMatchesReaderTopicOrder)
{
  const auto names = bagwiz::io::snapshot_topic_names(*reader_);

  // The contract is "reader order, preserved" — not declaration order (the
  // storage layer is free to reorder its topic index).
  std::vector<std::string> expected;
  for (const auto & t : reader_->topics()) {
    expected.push_back(t.name);
  }
  EXPECT_EQ(names, expected);
  EXPECT_EQ(names.size(), 2u);
}

TEST_F(TopicsTest, FindTopicReturnsMatchingTopicInfo)
{
  const auto * info = bagwiz::io::find_topic(*reader_, "/beta");
  ASSERT_NE(info, nullptr);
  EXPECT_EQ(info->name, "/beta");
  EXPECT_EQ(info->type, "std_msgs/msg/Int32");
}

TEST_F(TopicsTest, FindTopicReturnsNullOnMiss)
{
  EXPECT_EQ(bagwiz::io::find_topic(*reader_, "/missing"), nullptr);
}

TEST_F(TopicsTest, FindTopicOrLogReturnsMatchingTopicInfo)
{
  const auto * info = bagwiz::io::find_topic_or_log(*reader_, "/alpha", bag_path_, kLogger);
  ASSERT_NE(info, nullptr);
  EXPECT_EQ(info->name, "/alpha");
}

TEST_F(TopicsTest, FindTopicOrLogReturnsNullOnMiss)
{
  EXPECT_EQ(bagwiz::io::find_topic_or_log(*reader_, "/missing", bag_path_, kLogger), nullptr);
}
