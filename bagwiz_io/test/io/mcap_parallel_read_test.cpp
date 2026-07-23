// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <tuple>
#include <vector>

// The parallel indexed mcap read path (BAGWIZ_READ_THREADS > 1) must emit the
// exact same message sequence as the synchronous libmcap iteration
// (BAGWIZ_READ_THREADS=0): same topics, same timestamps, same payload bytes,
// same order — including log-time ties and out-of-order log times that span
// chunk boundaries.
namespace
{

using Record = std::tuple<std::string, std::int64_t, std::vector<std::byte>>;

bagwiz::io::TopicInfo topic_info(const std::string & name)
{
  bagwiz::io::TopicInfo t;
  t.name = name;
  t.type = "std_msgs/msg/ByteMultiArray";
  t.serialization_format = "cdr";
  return t;
}

std::vector<std::byte> payload_bytes(int seed, std::size_t size)
{
  std::vector<std::byte> out(size);
  for (std::size_t i = 0; i < size; ++i) {
    out[i] = static_cast<std::byte>((seed * 31 + static_cast<int>(i)) & 0xFF);
  }
  return out;
}

// A zstd bag with tiny chunks (so nearly every message gets its own chunk),
// two topics, log-time ties across topics, and periodic log-time inversions
// that make chunk time ranges overlap.
void write_fixture(const std::filesystem::path & path)
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = "zstd";
  options.mcap_chunk_size = 64;  // force one chunk per message or two

  auto writer = bagwiz::io::open_write(path, options);
  writer->declare_topic(topic_info("/a"));
  writer->declare_topic(topic_info("/b"));
  int seed = 0;
  for (int i = 0; i < 20; ++i) {
    const std::int64_t ts = 1000 + i * 10;
    const auto a = payload_bytes(seed++, 40);
    const auto b = payload_bytes(seed++, 40);
    writer->write("/a", ts, std::span<const std::byte>(a.data(), a.size()));
    // /b shares /a's timestamp: a log-time tie across chunks.
    writer->write("/b", ts, std::span<const std::byte>(b.data(), b.size()));
    if (i % 5 == 4) {
      // An out-of-order message: earlier log time written later, so chunk
      // time ranges overlap and LogTimeOrder must reorder across chunks.
      const auto c = payload_bytes(seed++, 40);
      writer->write("/a", ts - 7, std::span<const std::byte>(c.data(), c.size()));
    }
  }
  writer->close();
}

std::vector<Record> read_all(
  const std::filesystem::path & path, const char * read_threads,
  const std::optional<bagwiz::io::ReadFilter> & filter = std::nullopt,
  std::optional<std::size_t> stop_after = std::nullopt)
{
  ::setenv("BAGWIZ_READ_THREADS", read_threads, 1);
  std::vector<Record> out;
  auto reader = bagwiz::io::open_read(path);
  if (filter.has_value()) {
    reader->set_filter(*filter);
  }
  bagwiz::io::RawMessage raw;
  while (reader->next(raw)) {
    out.emplace_back(
      raw.topic->name, raw.timestamp_ns,
      std::vector<std::byte>(raw.payload.begin(), raw.payload.end()));
    if (stop_after.has_value() && out.size() >= *stop_after) {
      break;  // early stop: the reader (and its worker pool) is destroyed mid-run
    }
  }
  ::unsetenv("BAGWIZ_READ_THREADS");
  return out;
}

class McapParallelReadTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_ = std::filesystem::temp_directory_path() /
           ("bagwiz_mcap_parallel_" +
            std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()));
    std::filesystem::remove_all(tmp_);
    std::filesystem::create_directories(tmp_);
    bag_ = tmp_ / "fixture.mcap";
    write_fixture(bag_);
  }
  void TearDown() override { std::filesystem::remove_all(tmp_); }

  std::filesystem::path tmp_;
  std::filesystem::path bag_;
};

}  // namespace

TEST_F(McapParallelReadTest, FullReadMatchesSynchronousPath)
{
  const auto serial = read_all(bag_, "0");
  const auto parallel = read_all(bag_, "4");
  ASSERT_EQ(serial.size(), 44u);  // 20 * 2 + 4 out-of-order extras
  EXPECT_EQ(serial, parallel);
}

TEST_F(McapParallelReadTest, TopicFilterMatchesSynchronousPath)
{
  bagwiz::io::ReadFilter filter;
  filter.topics = {"/b"};
  const auto serial = read_all(bag_, "0", filter);
  const auto parallel = read_all(bag_, "4", filter);
  ASSERT_EQ(serial.size(), 20u);
  EXPECT_EQ(serial, parallel);
}

TEST_F(McapParallelReadTest, TimeRangeMatchesSynchronousPath)
{
  bagwiz::io::ReadFilter filter;
  filter.start_ns = 1050;  // inclusive
  filter.end_ns = 1150;    // exclusive
  const auto serial = read_all(bag_, "0", filter);
  const auto parallel = read_all(bag_, "4", filter);
  ASSERT_FALSE(serial.empty());
  EXPECT_EQ(serial, parallel);
}

TEST_F(McapParallelReadTest, EarlyStopDestroysCleanly)
{
  const auto serial = read_all(bag_, "0", std::nullopt, 3);
  const auto parallel = read_all(bag_, "4", std::nullopt, 3);
  ASSERT_EQ(serial.size(), 3u);
  EXPECT_EQ(serial, parallel);
}

TEST_F(McapParallelReadTest, SingleReadThreadFallsBackToSynchronousPath)
{
  const auto fallback = read_all(bag_, "1");
  const auto parallel = read_all(bag_, "4");
  EXPECT_EQ(fallback, parallel);
}
