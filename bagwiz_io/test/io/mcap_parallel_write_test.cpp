// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/bag_io.hpp"

#include <mcap/reader.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <tuple>
#include <vector>

// The parallel mcap write path (BAGWIZ_WRITE_THREADS > 1, compressed output)
// must produce deterministic files — byte-identical across worker counts,
// because each chunk is compressed one-shot and chunks are written in emission
// order — and files that stay fully compatible with the read path and with
// libmcap's strict summary parser.
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

// Compressible deterministic payload (a repeating pattern with a seed mixed
// in), so zstd/lz4 actually shrink it and the compressed-chunk path — not the
// uncompressed fallback — is what most assertions exercise.
std::vector<std::byte> payload_bytes(int seed, std::size_t size)
{
  std::vector<std::byte> out(size);
  for (std::size_t i = 0; i < size; ++i) {
    out[i] = static_cast<std::byte>((seed + static_cast<int>(i) / 7) & 0xFF);
  }
  return out;
}

// Incompressible payload (xorshift PRNG output), to exercise the
// store-uncompressed fallback for chunks that do not shrink.
std::vector<std::byte> random_payload(std::uint64_t & state, std::size_t size)
{
  std::vector<std::byte> out(size);
  for (std::size_t i = 0; i < size; ++i) {
    state ^= state << 13;
    state ^= state >> 7;
    state ^= state << 17;
    out[i] = static_cast<std::byte>(state & 0xFF);
  }
  return out;
}

struct Message
{
  std::string topic;
  std::int64_t timestamp_ns;
  std::vector<std::byte> payload;
};

// Two interleaved topics over many small chunks, with a log-time inversion
// so chunk time ranges overlap.
std::vector<Message> standard_messages()
{
  std::vector<Message> out;
  int seed = 0;
  for (int i = 0; i < 40; ++i) {
    const std::int64_t ts = 1000 + i * 10;
    out.push_back({"/a", ts, payload_bytes(seed++, 100)});
    out.push_back({"/b", ts, payload_bytes(seed++, 100)});
    if (i % 7 == 6) {
      out.push_back({"/a", ts - 3, payload_bytes(seed++, 100)});
    }
  }
  return out;
}

void write_bag(
  const std::filesystem::path & path, const std::vector<Message> & messages,
  const std::string & compression, std::uint32_t chunk_size, const char * write_threads)
{
  if (write_threads != nullptr) {
    ::setenv("BAGWIZ_WRITE_THREADS", write_threads, 1);
  } else {
    ::unsetenv("BAGWIZ_WRITE_THREADS");
  }
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = compression;
  options.mcap_chunk_size = chunk_size;

  auto writer = bagwiz::io::open_write(path, options);
  writer->declare_topic(topic_info("/a"));
  writer->declare_topic(topic_info("/b"));
  for (const auto & m : messages) {
    writer->write(
      m.topic, m.timestamp_ns, std::span<const std::byte>(m.payload.data(), m.payload.size()));
  }
  writer->close();
  ::unsetenv("BAGWIZ_WRITE_THREADS");
}

std::vector<Record> read_all(const std::filesystem::path & path)
{
  std::vector<Record> out;
  auto reader = bagwiz::io::open_read(path);
  bagwiz::io::RawMessage raw;
  while (reader->next(raw)) {
    out.emplace_back(
      raw.topic->name, raw.timestamp_ns,
      std::vector<std::byte>(raw.payload.begin(), raw.payload.end()));
  }
  return out;
}

std::vector<std::byte> file_bytes(const std::filesystem::path & path)
{
  std::ifstream in(path, std::ios::binary);
  const std::vector<char> chars{
    std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
  std::vector<std::byte> out(chars.size());
  std::memcpy(out.data(), chars.data(), chars.size());
  return out;
}

// The reader emits in log-time order, so an out-of-order write sequence comes
// back sorted by timestamp, not in write order. Compare order-insensitively.
std::vector<Record> sorted(std::vector<Record> records)
{
  std::sort(records.begin(), records.end());
  return records;
}

std::vector<Record> as_records(const std::vector<Message> & messages)
{
  std::vector<Record> out;
  out.reserve(messages.size());
  for (const auto & m : messages) {
    out.emplace_back(m.topic, m.timestamp_ns, m.payload);
  }
  return out;
}

class McapParallelWriteTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_ = std::filesystem::temp_directory_path() /
           ("bagwiz_mcap_parallel_write_" +
            std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()));
    std::filesystem::remove_all(tmp_);
    std::filesystem::create_directories(tmp_);
  }
  void TearDown() override
  {
    ::unsetenv("BAGWIZ_WRITE_THREADS");
    std::filesystem::remove_all(tmp_);
  }

  std::filesystem::path tmp_;
};

TEST_F(McapParallelWriteTest, ZstdOutputIsByteIdenticalAcrossWorkerCounts)
{
  const auto messages = standard_messages();
  const auto two = tmp_ / "two.mcap";
  const auto eight = tmp_ / "eight.mcap";
  write_bag(two, messages, "zstd", 512, "2");
  write_bag(eight, messages, "zstd", 512, "8");
  EXPECT_EQ(file_bytes(two), file_bytes(eight));
}

TEST_F(McapParallelWriteTest, Lz4OutputIsByteIdenticalAcrossWorkerCounts)
{
  const auto messages = standard_messages();
  const auto two = tmp_ / "two.mcap";
  const auto eight = tmp_ / "eight.mcap";
  write_bag(two, messages, "lz4", 512, "2");
  write_bag(eight, messages, "lz4", 512, "8");
  EXPECT_EQ(file_bytes(two), file_bytes(eight));
}

TEST_F(McapParallelWriteTest, ParallelOutputMatchesSerialSemantically)
{
  const auto messages = standard_messages();
  const auto serial = tmp_ / "serial.mcap";
  const auto parallel = tmp_ / "parallel.mcap";
  write_bag(serial, messages, "zstd", 512, "1");
  write_bag(parallel, messages, "zstd", 512, "8");
  const auto expected = read_all(serial);
  ASSERT_EQ(expected.size(), messages.size());
  EXPECT_EQ(read_all(parallel), expected);
}

TEST_F(McapParallelWriteTest, ZstdRoundTripPreservesMessages)
{
  const auto messages = standard_messages();
  const auto bag = tmp_ / "bag.mcap";
  write_bag(bag, messages, "zstd", 512, "4");
  EXPECT_EQ(sorted(read_all(bag)), sorted(as_records(messages)));
}

TEST_F(McapParallelWriteTest, Lz4RoundTripPreservesMessages)
{
  const auto messages = standard_messages();
  const auto bag = tmp_ / "bag.mcap";
  write_bag(bag, messages, "lz4", 512, "4");
  EXPECT_EQ(sorted(read_all(bag)), sorted(as_records(messages)));
}

// A strict summary parse: offsets and the summary CRC must round-trip through
// libmcap, the chunk indexes must agree with the written codec, and the
// statistics must carry the per-channel message counts.
TEST_F(McapParallelWriteTest, SummaryParsesStrictlyAndCountsMatch)
{
  const auto messages = standard_messages();
  const auto bag = tmp_ / "bag.mcap";
  write_bag(bag, messages, "zstd", 512, "8");

  mcap::McapReader reader;
  ASSERT_TRUE(reader.open(bag.string()).ok());
  ASSERT_TRUE(reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan).ok());

  const auto & chunk_indexes = reader.chunkIndexes();
  ASSERT_GT(chunk_indexes.size(), 1u);  // 512-byte chunks over 85 messages
  for (const auto & ci : chunk_indexes) {
    EXPECT_EQ(ci.compression, "zstd");
    EXPECT_GT(ci.compressedSize, 0u);
    EXPECT_GT(ci.uncompressedSize, ci.compressedSize);  // compressible fixture
  }

  const auto stats = reader.statistics();
  ASSERT_TRUE(stats.has_value());
  EXPECT_EQ(stats->messageCount, messages.size());
  EXPECT_EQ(stats->chunkCount, chunk_indexes.size());
  std::uint64_t a_count = 0;
  std::uint64_t b_count = 0;
  for (const auto & m : messages) {
    ++(m.topic == "/a" ? a_count : b_count);
  }
  ASSERT_EQ(stats->channelMessageCounts.size(), 2u);
  std::uint64_t total = 0;
  for (const auto & [channel_id, count] : stats->channelMessageCounts) {
    EXPECT_TRUE(count == a_count || count == b_count);
    total += count;
  }
  EXPECT_EQ(total, messages.size());
}

TEST_F(McapParallelWriteTest, IncompressibleChunkFallsBackToUncompressed)
{
  std::uint64_t state = 0x9E3779B97F4A7C15ull;
  std::vector<Message> messages;
  for (int i = 0; i < 8; ++i) {
    messages.push_back({"/a", 1000 + i * 10, random_payload(state, 1024)});
  }
  const auto bag = tmp_ / "bag.mcap";
  write_bag(bag, messages, "zstd", 512, "4");
  EXPECT_EQ(read_all(bag).size(), messages.size());

  mcap::McapReader reader;
  ASSERT_TRUE(reader.open(bag.string()).ok());
  ASSERT_TRUE(reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan).ok());
  for (const auto & ci : reader.chunkIndexes()) {
    EXPECT_TRUE(ci.compression.empty()) << "random payload must not be stored compressed";
  }
}

TEST_F(McapParallelWriteTest, OversizedSingleMessageGetsItsOwnChunk)
{
  std::vector<Message> messages = {{"/a", 1000, payload_bytes(7, 3 * 512)}};
  const auto bag = tmp_ / "bag.mcap";
  write_bag(bag, messages, "zstd", 512, "4");
  EXPECT_EQ(read_all(bag), std::vector<Record>({{"/a", 1000, messages[0].payload}}));

  mcap::McapReader reader;
  ASSERT_TRUE(reader.open(bag.string()).ok());
  ASSERT_TRUE(reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan).ok());
  EXPECT_EQ(reader.chunkIndexes().size(), 1u);
}

TEST_F(McapParallelWriteTest, ExactChunkBoundaryStillFlushes)
{
  // One message record = 31 bytes of header/body + payload, so a 225-byte
  // payload lands the staging buffer exactly on the 256-byte chunk size.
  std::vector<Message> messages;
  for (int i = 0; i < 3; ++i) {
    messages.push_back({"/a", 1000 + i * 10, payload_bytes(i, 225)});
  }
  const auto bag = tmp_ / "bag.mcap";
  write_bag(bag, messages, "zstd", 256, "4");
  EXPECT_EQ(read_all(bag).size(), messages.size());

  mcap::McapReader reader;
  ASSERT_TRUE(reader.open(bag.string()).ok());
  ASSERT_TRUE(reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan).ok());
  EXPECT_EQ(reader.chunkIndexes().size(), messages.size());
}

TEST_F(McapParallelWriteTest, EmptyBagIsReadable)
{
  const auto bag = tmp_ / "bag.mcap";
  write_bag(bag, {}, "zstd", 512, "4");
  EXPECT_TRUE(read_all(bag).empty());

  mcap::McapReader reader;
  ASSERT_TRUE(reader.open(bag.string()).ok());
  ASSERT_TRUE(reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan).ok());
  EXPECT_TRUE(reader.chunkIndexes().empty());
  const auto stats = reader.statistics();
  ASSERT_TRUE(stats.has_value());
  EXPECT_EQ(stats->messageCount, 0u);
}

TEST_F(McapParallelWriteTest, SingleMessage)
{
  std::vector<Message> messages = {{"/b", 42, payload_bytes(3, 16)}};
  const auto bag = tmp_ / "bag.mcap";
  write_bag(bag, messages, "zstd", 512, "4");
  EXPECT_EQ(read_all(bag), std::vector<Record>({{"/b", 42, messages[0].payload}}));
}

// Unset, empty, zero, and unparsable BAGWIZ_WRITE_THREADS values must all
// still produce a valid bag (graceful fallback, no crash, no refusal).
TEST_F(McapParallelWriteTest, EnvKnobFallbackValuesStillWriteValidBags)
{
  const auto messages = standard_messages();
  for (const char * value : std::vector<const char *>{nullptr, "", "0", "abc"}) {
    const auto bag = tmp_ / (std::string("bag_") + (value == nullptr ? "unset" : value) + ".mcap");
    ASSERT_NO_THROW(write_bag(bag, messages, "zstd", 512, value))
      << "value=" << (value == nullptr ? "<unset>" : value);
    EXPECT_EQ(read_all(bag).size(), messages.size())
      << "value=" << (value == nullptr ? "<unset>" : value);
  }
}

}  // namespace
