// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/mcap_passthrough.hpp"

#include "bagwiz/io/bag_io.hpp"

#include <mcap/reader.hpp>
#include <mcap/writer.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <tuple>
#include <vector>

// The chunk pass-through rewrite must (a) copy untouched chunks byte-for-byte
// — preserving the input's chunk compression — while re-encoding exactly the
// chunks the edit touches, (b) produce an mcap whose summary parses strictly
// and whose linear (file-order) scan stays self-describing even when the
// edit dropped the chunks that embedded the Schema/Channel records, and (c)
// return nullopt (leaving no output behind) for every input shape that needs
// the decoded rewrite pipeline.
namespace
{

using bagwiz::io::mcap_passthrough_rewrite;
using bagwiz::io::McapPassthroughEdit;

using Record = std::tuple<std::string, std::int64_t, std::vector<std::byte>>;

std::vector<std::byte> payload_bytes(int seed, std::size_t size)
{
  std::vector<std::byte> out(size);
  for (std::size_t i = 0; i < size; ++i) {
    out[i] = static_cast<std::byte>((seed * 31 + static_cast<int>(i)) & 0xFF);
  }
  return out;
}

mcap::Compression to_compression(const std::string & name)
{
  if (name == "zstd") {
    return mcap::Compression::Zstd;
  }
  if (name == "lz4") {
    return mcap::Compression::Lz4;
  }
  return mcap::Compression::None;
}

struct FixtureMessage
{
  std::string topic;
  std::uint64_t log_time;
  std::vector<std::byte> payload;
};

// One fixture chunk = one inner vector; chunk boundaries are forced with
// closeLastChunk() so tests control exactly which chunk carries what.
using FixtureChunks = std::vector<std::vector<FixtureMessage>>;

void write_fixture(
  const std::filesystem::path & path, const FixtureChunks & chunks, const std::string & compression,
  const std::vector<std::string> & topics)
{
  mcap::McapWriterOptions options("ros2");
  options.compression = to_compression(compression);
  // libmcap silently stores chunks whose payload does not shrink as
  // uncompressed; the tiny pseudo-random fixtures here never shrink, so
  // force the codec to make the compression-preservation assertions real.
  options.forceCompression = true;
  options.noChunkCRC = true;

  mcap::McapWriter writer;
  ASSERT_TRUE(writer.open(path.string(), options).ok());

  mcap::Schema schema("std_msgs/msg/ByteMultiArray", "ros2msg", "byte[] data");
  writer.addSchema(schema);
  std::map<std::string, mcap::ChannelId> channel_ids;
  for (const auto & topic : topics) {
    mcap::Channel channel(topic, "cdr", schema.id);
    writer.addChannel(channel);
    channel_ids[topic] = channel.id;
  }

  for (const auto & chunk : chunks) {
    for (const auto & msg : chunk) {
      mcap::Message m;
      m.channelId = channel_ids.at(msg.topic);
      m.sequence = 0;
      m.logTime = msg.log_time;
      m.publishTime = msg.log_time;
      m.data = msg.payload.data();
      m.dataSize = msg.payload.size();
      ASSERT_TRUE(writer.write(m).ok());
    }
    writer.closeLastChunk();
  }
  writer.close();
}

// The standard 3-chunk fixture: /a lives in every chunk, /b only in the
// middle one. Strictly increasing log times keep decoded-order comparisons
// total (equal log times may legally reorder between rewrite paths).
FixtureChunks standard_chunks()
{
  int seed = 0;
  auto msg = [&seed](const std::string & topic, std::uint64_t t) {
    return FixtureMessage{topic, t, payload_bytes(seed++, 48)};
  };
  return {
    {msg("/a", 1000), msg("/a", 1010), msg("/a", 1020)},
    {msg("/a", 1100), msg("/b", 1110), msg("/a", 1120)},
    {msg("/a", 1200), msg("/a", 1210), msg("/a", 1220)},
  };
}

std::vector<Record> expected_records(const FixtureChunks & chunks, const McapPassthroughEdit & edit)
{
  std::vector<Record> out;
  for (const auto & chunk : chunks) {
    for (const auto & msg : chunk) {
      if (edit.drop_topics.count(msg.topic) != 0) {
        continue;
      }
      const std::uint64_t start = edit.start_ns ? static_cast<std::uint64_t>(*edit.start_ns) : 0;
      const std::uint64_t end =
        edit.end_ns ? static_cast<std::uint64_t>(*edit.end_ns) : mcap::MaxTime;
      if (msg.log_time < start || msg.log_time >= end) {
        continue;
      }
      auto it = edit.rename.find(msg.topic);
      const std::string & name = it != edit.rename.end() ? it->second : msg.topic;
      out.emplace_back(name, static_cast<std::int64_t>(msg.log_time), msg.payload);
    }
  }
  std::sort(out.begin(), out.end(), [](const Record & a, const Record & b) {
    return std::get<1>(a) < std::get<1>(b);
  });
  return out;
}

std::vector<Record> read_bag(const std::filesystem::path & path)
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

// A strict summary parse of the output is part of every positive assertion:
// offsets and the summary CRC must round-trip through libmcap.
struct ChunkSpan
{
  std::uint64_t start = 0;
  std::uint64_t length = 0;  // chunk record + its MessageIndex records
  std::uint64_t start_time = 0;
  std::uint64_t end_time = 0;
  std::string compression;
};

std::vector<ChunkSpan> chunk_spans(const std::filesystem::path & path)
{
  mcap::McapReader reader;
  EXPECT_TRUE(reader.open(path.string()).ok());
  EXPECT_TRUE(reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan).ok());
  std::vector<ChunkSpan> spans;
  for (const auto & ci : reader.chunkIndexes()) {
    spans.push_back(
      {ci.chunkStartOffset, ci.chunkLength + ci.messageIndexLength, ci.messageStartTime,
       ci.messageEndTime, ci.compression});
  }
  reader.close();
  std::sort(spans.begin(), spans.end(), [](const ChunkSpan & a, const ChunkSpan & b) {
    return a.start < b.start;
  });
  return spans;
}

std::vector<std::byte> read_range(
  const std::filesystem::path & path, std::uint64_t offset, std::uint64_t length)
{
  std::ifstream f(path, std::ios::binary);
  EXPECT_TRUE(static_cast<bool>(f));
  f.seekg(static_cast<std::streamoff>(offset));
  std::vector<std::byte> out(static_cast<std::size_t>(length));
  f.read(reinterpret_cast<char *>(out.data()), static_cast<std::streamsize>(length));
  EXPECT_EQ(static_cast<std::uint64_t>(f.gcount()), length);
  return out;
}

// Assert the output chunk whose messageStartTime is `start_time` is a
// byte-for-byte copy (chunk record + MessageIndex records) of the input
// chunk with the same time bound.
void expect_chunk_verbatim(
  const std::filesystem::path & input, const std::filesystem::path & output,
  std::uint64_t start_time)
{
  const auto in_spans = chunk_spans(input);
  const auto out_spans = chunk_spans(output);
  const auto find = [start_time](const std::vector<ChunkSpan> & spans) {
    return std::find_if(spans.begin(), spans.end(), [start_time](const ChunkSpan & s) {
      return s.start_time == start_time;
    });
  };
  const auto in_it = find(in_spans);
  const auto out_it = find(out_spans);
  ASSERT_NE(in_it, in_spans.end());
  ASSERT_NE(out_it, out_spans.end());
  ASSERT_EQ(in_it->length, out_it->length);
  EXPECT_EQ(
    read_range(input, in_it->start, in_it->length),
    read_range(output, out_it->start, out_it->length));
}

class McapPassthroughTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_ = std::filesystem::temp_directory_path() /
           ("bagwiz_mcap_passthrough_" +
            std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()));
    std::filesystem::remove_all(tmp_);
    std::filesystem::create_directories(tmp_);
    input_ = tmp_ / "input.mcap";
    output_ = tmp_ / "output.mcap";
  }
  void TearDown() override { std::filesystem::remove_all(tmp_); }

  std::filesystem::path tmp_;
  std::filesystem::path input_;
  std::filesystem::path output_;
};

}  // namespace

TEST_F(McapPassthroughTest, KeepAllCopiesEveryChunkVerbatim)
{
  const auto chunks = standard_chunks();
  write_fixture(input_, chunks, "zstd", {"/a", "/b"});

  McapPassthroughEdit edit;
  std::string reason;
  const auto result = mcap_passthrough_rewrite(input_, output_, edit, &reason);
  ASSERT_TRUE(result.has_value()) << reason;

  EXPECT_EQ(result->chunks_copied, 3u);
  EXPECT_EQ(result->chunks_reencoded, 0u);
  EXPECT_EQ(result->chunks_dropped, 0u);
  EXPECT_EQ(result->messages_written, 9u);
  EXPECT_EQ(result->chunk_compression, "zstd");
  EXPECT_EQ(result->start_ns, 1000);
  EXPECT_EQ(result->end_ns, 1220);

  for (const std::uint64_t t : {1000u, 1100u, 1200u}) {
    expect_chunk_verbatim(input_, output_, t);
  }
  EXPECT_EQ(read_bag(output_), expected_records(chunks, edit));
}

TEST_F(McapPassthroughTest, DropReencodesOnlyTheTouchedChunk)
{
  const auto chunks = standard_chunks();
  write_fixture(input_, chunks, "zstd", {"/a", "/b"});

  McapPassthroughEdit edit;
  edit.drop_topics = {"/b"};
  std::string reason;
  const auto result = mcap_passthrough_rewrite(input_, output_, edit, &reason);
  ASSERT_TRUE(result.has_value()) << reason;

  EXPECT_EQ(result->chunks_copied, 2u);
  EXPECT_EQ(result->chunks_reencoded, 1u);
  EXPECT_EQ(result->messages_written, 8u);
  EXPECT_EQ(result->chunk_compression, "zstd");
  ASSERT_EQ(result->topics.size(), 1u);
  EXPECT_EQ(result->topics[0].name, "/a");
  EXPECT_EQ(result->per_topic_counts.at("/a"), 8);
  EXPECT_EQ(result->per_topic_counts.count("/b"), 0u);

  expect_chunk_verbatim(input_, output_, 1000);
  expect_chunk_verbatim(input_, output_, 1200);
  const auto spans = chunk_spans(output_);
  ASSERT_EQ(spans.size(), 3u);
  EXPECT_EQ(spans[1].compression, "zstd");  // re-encoded with its own codec
  EXPECT_EQ(read_bag(output_), expected_records(chunks, edit));
}

TEST_F(McapPassthroughTest, DropEverythingProducesValidEmptyBag)
{
  write_fixture(input_, standard_chunks(), "zstd", {"/a", "/b"});

  McapPassthroughEdit edit;
  edit.drop_topics = {"/a", "/b"};
  std::string reason;
  const auto result = mcap_passthrough_rewrite(input_, output_, edit, &reason);
  ASSERT_TRUE(result.has_value()) << reason;

  EXPECT_EQ(result->messages_written, 0u);
  EXPECT_EQ(result->chunks_dropped, 3u);
  EXPECT_TRUE(result->topics.empty());
  EXPECT_TRUE(chunk_spans(output_).empty());
  EXPECT_TRUE(read_bag(output_).empty());
}

TEST_F(McapPassthroughTest, TrimAlignedWithChunkBoundsCopiesWithoutReencoding)
{
  const auto chunks = standard_chunks();
  write_fixture(input_, chunks, "zstd", {"/a", "/b"});

  McapPassthroughEdit edit;
  edit.start_ns = 1100;
  edit.end_ns = 1121;  // chunk 2 spans [1100, 1120]
  std::string reason;
  const auto result = mcap_passthrough_rewrite(input_, output_, edit, &reason);
  ASSERT_TRUE(result.has_value()) << reason;

  EXPECT_EQ(result->chunks_copied, 1u);
  EXPECT_EQ(result->chunks_reencoded, 0u);
  EXPECT_EQ(result->chunks_dropped, 2u);
  EXPECT_EQ(result->messages_written, 3u);
  expect_chunk_verbatim(input_, output_, 1100);
  EXPECT_EQ(read_bag(output_), expected_records(chunks, edit));
}

TEST_F(McapPassthroughTest, TrimStraddlingReencodesOnlyTheEdgeChunks)
{
  const auto chunks = standard_chunks();
  write_fixture(input_, chunks, "zstd", {"/a", "/b"});

  McapPassthroughEdit edit;
  edit.start_ns = 1010;
  edit.end_ns = 1211;
  std::string reason;
  const auto result = mcap_passthrough_rewrite(input_, output_, edit, &reason);
  ASSERT_TRUE(result.has_value()) << reason;

  EXPECT_EQ(result->chunks_copied, 1u);     // chunk 2, fully inside
  EXPECT_EQ(result->chunks_reencoded, 2u);  // chunks 1 and 3, straddling
  EXPECT_EQ(result->messages_written, 7u);  // drops /a@1000 and /a@1220
  expect_chunk_verbatim(input_, output_, 1100);
  EXPECT_EQ(read_bag(output_), expected_records(chunks, edit));
}

TEST_F(McapPassthroughTest, TrimEndBoundIsExclusive)
{
  const auto chunks = standard_chunks();
  write_fixture(input_, chunks, "zstd", {"/a", "/b"});

  McapPassthroughEdit edit;
  edit.end_ns = 1220;  // equals the last message's log time: must drop it
  std::string reason;
  const auto result = mcap_passthrough_rewrite(input_, output_, edit, &reason);
  ASSERT_TRUE(result.has_value()) << reason;

  // Chunk 3's messageEndTime (1220, inclusive) equals the exclusive end
  // bound, so the chunk must be re-encoded, not copied.
  EXPECT_EQ(result->chunks_copied, 2u);
  EXPECT_EQ(result->chunks_reencoded, 1u);
  EXPECT_EQ(result->messages_written, 8u);
  EXPECT_EQ(result->end_ns, 1210);
  EXPECT_EQ(read_bag(output_), expected_records(chunks, edit));
}

TEST_F(McapPassthroughTest, TrimEmptyWindowKeepsTopicsAndWritesNoMessages)
{
  write_fixture(input_, standard_chunks(), "zstd", {"/a", "/b"});

  McapPassthroughEdit edit;
  edit.start_ns = 1100;
  edit.end_ns = 1100;
  std::string reason;
  const auto result = mcap_passthrough_rewrite(input_, output_, edit, &reason);
  ASSERT_TRUE(result.has_value()) << reason;

  EXPECT_EQ(result->messages_written, 0u);
  ASSERT_EQ(result->topics.size(), 2u);  // channels survive; only messages go
  EXPECT_TRUE(result->per_topic_counts.empty());
  EXPECT_TRUE(read_bag(output_).empty());
}

TEST_F(McapPassthroughTest, TrimPastFirstChunkKeepsFileOrderReadability)
{
  const auto chunks = standard_chunks();
  write_fixture(input_, chunks, "zstd", {"/a", "/b"});

  // The dropped head chunk is the one that embedded every Schema/Channel
  // record; the re-emitted top-level block must keep a linear file-order
  // scan self-describing.
  McapPassthroughEdit edit;
  edit.start_ns = 1100;
  std::string reason;
  const auto result = mcap_passthrough_rewrite(input_, output_, edit, &reason);
  ASSERT_TRUE(result.has_value()) << reason;

  mcap::McapReader reader;
  ASSERT_TRUE(reader.open(output_.string()).ok());
  std::vector<std::string> problems;
  auto view =
    reader.readMessages([&problems](const mcap::Status & s) { problems.push_back(s.message); });
  std::vector<Record> scanned;
  for (const auto & mv : view) {
    scanned.emplace_back(
      mv.channel->topic, static_cast<std::int64_t>(mv.message.logTime),
      std::vector<std::byte>(mv.message.data, mv.message.data + mv.message.dataSize));
  }
  reader.close();
  EXPECT_TRUE(problems.empty()) << problems.front();
  std::sort(scanned.begin(), scanned.end(), [](const Record & a, const Record & b) {
    return std::get<1>(a) < std::get<1>(b);
  });
  EXPECT_EQ(scanned, expected_records(chunks, edit));
}

TEST_F(McapPassthroughTest, RenameRewritesTheChannelAndPreservesItsId)
{
  const auto chunks = standard_chunks();
  write_fixture(input_, chunks, "zstd", {"/a", "/b"});

  mcap::McapReader in_reader;
  ASSERT_TRUE(in_reader.open(input_.string()).ok());
  ASSERT_TRUE(in_reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan).ok());
  mcap::ChannelId b_id = 0;
  for (const auto & [id, channel] : in_reader.channels()) {
    if (channel->topic == "/b") {
      b_id = id;
    }
  }
  in_reader.close();

  McapPassthroughEdit edit;
  edit.rename = {{"/b", "/b_renamed"}};
  std::string reason;
  const auto result = mcap_passthrough_rewrite(input_, output_, edit, &reason);
  ASSERT_TRUE(result.has_value()) << reason;

  // Only the middle chunk carries /b; the rest copy verbatim.
  EXPECT_EQ(result->chunks_copied, 2u);
  EXPECT_EQ(result->chunks_reencoded, 1u);
  EXPECT_EQ(result->messages_renamed, 1u);
  expect_chunk_verbatim(input_, output_, 1000);
  expect_chunk_verbatim(input_, output_, 1200);
  EXPECT_EQ(read_bag(output_), expected_records(chunks, edit));

  mcap::McapReader out_reader;
  ASSERT_TRUE(out_reader.open(output_.string()).ok());
  ASSERT_TRUE(out_reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan).ok());
  bool found = false;
  for (const auto & [id, channel] : out_reader.channels()) {
    EXPECT_NE(channel->topic, "/b");
    if (channel->topic == "/b_renamed") {
      EXPECT_EQ(id, b_id);  // ids are preserved verbatim
      found = true;
    }
  }
  out_reader.close();
  EXPECT_TRUE(found);
}

TEST_F(McapPassthroughTest, UncompressedInputStaysUncompressed)
{
  const auto chunks = standard_chunks();
  write_fixture(input_, chunks, "", {"/a", "/b"});

  McapPassthroughEdit edit;
  edit.drop_topics = {"/b"};
  std::string reason;
  const auto result = mcap_passthrough_rewrite(input_, output_, edit, &reason);
  ASSERT_TRUE(result.has_value()) << reason;

  EXPECT_EQ(result->chunk_compression, "");
  for (const auto & span : chunk_spans(output_)) {
    EXPECT_TRUE(span.compression.empty() || span.compression == "none");
  }
  EXPECT_EQ(read_bag(output_), expected_records(chunks, edit));
}

TEST_F(McapPassthroughTest, Lz4InputStaysLz4OnBothPaths)
{
  const auto chunks = standard_chunks();
  write_fixture(input_, chunks, "lz4", {"/a", "/b"});

  McapPassthroughEdit edit;
  edit.drop_topics = {"/b"};
  std::string reason;
  const auto result = mcap_passthrough_rewrite(input_, output_, edit, &reason);
  ASSERT_TRUE(result.has_value()) << reason;

  EXPECT_EQ(result->chunks_copied, 2u);
  EXPECT_EQ(result->chunks_reencoded, 1u);
  EXPECT_EQ(result->chunk_compression, "lz4");
  for (const auto & span : chunk_spans(output_)) {
    EXPECT_EQ(span.compression, "lz4");
  }
  EXPECT_EQ(read_bag(output_), expected_records(chunks, edit));
}

TEST_F(McapPassthroughTest, UnchunkedInputFallsBack)
{
  mcap::McapWriterOptions options("ros2");
  options.noChunking = true;
  mcap::McapWriter writer;
  ASSERT_TRUE(writer.open(input_.string(), options).ok());
  mcap::Schema schema("std_msgs/msg/ByteMultiArray", "ros2msg", "byte[] data");
  writer.addSchema(schema);
  mcap::Channel channel("/a", "cdr", schema.id);
  writer.addChannel(channel);
  const auto payload = payload_bytes(1, 16);
  mcap::Message m;
  m.channelId = channel.id;
  m.sequence = 0;
  m.logTime = 1000;
  m.publishTime = 1000;
  m.data = payload.data();
  m.dataSize = payload.size();
  ASSERT_TRUE(writer.write(m).ok());
  writer.close();

  std::string reason;
  const auto result = mcap_passthrough_rewrite(input_, output_, {}, &reason);
  EXPECT_FALSE(result.has_value());
  EXPECT_FALSE(reason.empty());
  EXPECT_FALSE(std::filesystem::exists(output_));
}

// Attachment/Metadata records are omitted from the output — exactly what the
// decoded pipeline does silently — and counted for the caller's warning.
// rosbag2 stamps every recording with a Metadata record, so refusing these
// inputs would disable the fast path on every rosbag2-recorded mcap.
TEST_F(McapPassthroughTest, AttachmentAndMetadataAreSkippedAndCounted)
{
  const auto chunks = standard_chunks();
  mcap::McapWriterOptions options("ros2");
  options.compression = to_compression("zstd");
  options.forceCompression = true;
  options.noChunkCRC = true;
  mcap::McapWriter writer;
  ASSERT_TRUE(writer.open(input_.string(), options).ok());
  mcap::Metadata metadata;
  metadata.name = "rosbag2";
  metadata.metadata = {{"serialization_format", "cdr"}};
  ASSERT_TRUE(writer.write(metadata).ok());
  mcap::Schema schema("std_msgs/msg/ByteMultiArray", "ros2msg", "byte[] data");
  writer.addSchema(schema);
  std::map<std::string, mcap::ChannelId> channel_ids;
  for (const auto & topic : {"/a", "/b"}) {
    mcap::Channel channel(topic, "cdr", schema.id);
    writer.addChannel(channel);
    channel_ids[topic] = channel.id;
  }
  for (const auto & chunk : chunks) {
    for (const auto & msg : chunk) {
      mcap::Message m;
      m.channelId = channel_ids.at(msg.topic);
      m.sequence = 0;
      m.logTime = msg.log_time;
      m.publishTime = msg.log_time;
      m.data = msg.payload.data();
      m.dataSize = msg.payload.size();
      ASSERT_TRUE(writer.write(m).ok());
    }
    writer.closeLastChunk();
  }
  mcap::Attachment attachment;
  attachment.logTime = 1;
  attachment.createTime = 1;
  attachment.name = "calibration.txt";
  attachment.mediaType = "text/plain";
  const auto payload = payload_bytes(2, 8);
  attachment.data = payload.data();
  attachment.dataSize = payload.size();
  ASSERT_TRUE(writer.write(attachment).ok());
  writer.close();

  std::string reason;
  const auto result = mcap_passthrough_rewrite(input_, output_, {}, &reason);
  ASSERT_TRUE(result.has_value()) << reason;
  EXPECT_EQ(result->attachments_skipped, 1u);
  EXPECT_EQ(result->metadata_skipped, 1u);
  EXPECT_EQ(result->chunks_copied, 3u);
  EXPECT_EQ(result->messages_written, 9u);
  EXPECT_EQ(read_bag(output_), expected_records(chunks, {}));

  // The rebuilt summary reports zero attachments/metadata, matching the
  // omitted records.
  mcap::McapReader out_reader;
  ASSERT_TRUE(out_reader.open(output_.string()).ok());
  ASSERT_TRUE(out_reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan).ok());
  ASSERT_TRUE(out_reader.statistics().has_value());
  EXPECT_EQ(out_reader.statistics()->attachmentCount, 0u);
  EXPECT_EQ(out_reader.statistics()->metadataCount, 0u);
  out_reader.close();
}

TEST_F(McapPassthroughTest, DuplicateTopicNamesFallBack)
{
  mcap::McapWriterOptions options("ros2");
  mcap::McapWriter writer;
  ASSERT_TRUE(writer.open(input_.string(), options).ok());
  mcap::Schema schema("std_msgs/msg/ByteMultiArray", "ros2msg", "byte[] data");
  writer.addSchema(schema);
  mcap::Channel first("/dup", "cdr", schema.id);
  mcap::Channel second("/dup", "cdr", schema.id);
  writer.addChannel(first);
  writer.addChannel(second);
  const auto payload = payload_bytes(3, 16);
  for (const auto id : {first.id, second.id}) {
    mcap::Message m;
    m.channelId = id;
    m.sequence = 0;
    m.logTime = 1000u + id;
    m.publishTime = m.logTime;
    m.data = payload.data();
    m.dataSize = payload.size();
    ASSERT_TRUE(writer.write(m).ok());
  }
  writer.close();

  std::string reason;
  const auto result = mcap_passthrough_rewrite(input_, output_, {}, &reason);
  EXPECT_FALSE(result.has_value());
  EXPECT_FALSE(std::filesystem::exists(output_));
}

TEST_F(McapPassthroughTest, RenameTargetCollisionFallsBack)
{
  write_fixture(input_, standard_chunks(), "zstd", {"/a", "/b"});

  McapPassthroughEdit edit;
  edit.rename = {{"/b", "/a"}};
  std::string reason;
  const auto result = mcap_passthrough_rewrite(input_, output_, edit, &reason);
  EXPECT_FALSE(result.has_value());
  EXPECT_FALSE(std::filesystem::exists(output_));
}

TEST_F(McapPassthroughTest, SchemaLessChannelPassesThrough)
{
  // Hand-built via the static record writers: mcap 0.8.0's high-level
  // McapWriter refuses to write messages on a schema-less channel
  // (schemaId 0), but the file shape itself is spec-legal on every version
  // and the engine must preserve it.
  mcap::FileWriter out;
  ASSERT_TRUE(out.open(input_.string()).ok());
  mcap::McapWriter::writeMagic(out);
  mcap::McapWriter::write(out, mcap::Header{"ros2", "crafted"});

  mcap::Channel channel("/raw", "cdr", 0);
  channel.id = 1;
  const auto payload = payload_bytes(4, 16);

  mcap::BufferWriter blob;
  {
    mcap::Channel copy = channel;
    mcap::McapWriter::write(blob, copy);
  }
  const std::uint64_t msg_offset = blob.size();
  mcap::Message m;
  m.channelId = channel.id;
  m.sequence = 0;
  m.logTime = 1000;
  m.publishTime = 1000;
  m.data = payload.data();
  m.dataSize = payload.size();
  mcap::McapWriter::write(blob, m);
  blob.end();

  mcap::Chunk chunk;
  chunk.messageStartTime = 1000;
  chunk.messageEndTime = 1000;
  chunk.uncompressedSize = blob.size();
  chunk.uncompressedCrc = 0;
  chunk.compression = "";
  chunk.compressedSize = blob.size();
  chunk.records = blob.data();
  const std::uint64_t chunk_start = out.size();
  mcap::McapWriter::write(out, chunk);
  const std::uint64_t index_start = out.size();
  mcap::MessageIndex mi;
  mi.channelId = channel.id;
  mi.records = {{1000, msg_offset}};
  mcap::ChunkIndex ci;
  ci.messageStartTime = 1000;
  ci.messageEndTime = 1000;
  ci.chunkStartOffset = chunk_start;
  ci.chunkLength = index_start - chunk_start;
  ci.messageIndexOffsets[channel.id] = out.size();
  mcap::McapWriter::write(out, mi);
  ci.messageIndexLength = out.size() - index_start;
  ci.compression = "";
  ci.compressedSize = blob.size();
  ci.uncompressedSize = blob.size();

  mcap::McapWriter::write(out, mcap::DataEnd{0});
  const std::uint64_t summary_start = out.size();
  {
    mcap::Channel copy = channel;
    mcap::McapWriter::write(out, copy);
  }
  mcap::McapWriter::write(out, ci);
  mcap::Statistics stats;
  stats.messageCount = 1;
  stats.schemaCount = 0;
  stats.channelCount = 1;
  stats.attachmentCount = 0;
  stats.metadataCount = 0;
  stats.chunkCount = 1;
  stats.messageStartTime = 1000;
  stats.messageEndTime = 1000;
  stats.channelMessageCounts = {{channel.id, 1}};
  mcap::McapWriter::write(out, stats);
  mcap::McapWriter::write(out, mcap::Footer{summary_start, 0}, /*crcEnabled=*/false);
  mcap::McapWriter::writeMagic(out);
  out.end();

  std::string reason;
  const auto result = mcap_passthrough_rewrite(input_, output_, {}, &reason);
  ASSERT_TRUE(result.has_value()) << reason;
  ASSERT_EQ(result->topics.size(), 1u);
  EXPECT_EQ(result->topics[0].name, "/raw");
  EXPECT_TRUE(result->topics[0].type.empty());
  EXPECT_EQ(result->messages_written, 1u);
  EXPECT_EQ(read_bag(output_).size(), 1u);
}

TEST_F(McapPassthroughTest, MissingMessageIndexesForceReencodeNotFallback)
{
  mcap::McapWriterOptions options("ros2");
  options.noMessageIndex = true;
  mcap::McapWriter writer;
  ASSERT_TRUE(writer.open(input_.string(), options).ok());
  mcap::Schema schema("std_msgs/msg/ByteMultiArray", "ros2msg", "byte[] data");
  writer.addSchema(schema);
  mcap::Channel channel("/a", "cdr", schema.id);
  writer.addChannel(channel);
  std::vector<std::vector<std::byte>> payloads;
  for (int i = 0; i < 3; ++i) {
    payloads.push_back(payload_bytes(10 + i, 32));
    mcap::Message m;
    m.channelId = channel.id;
    m.sequence = 0;
    m.logTime = static_cast<std::uint64_t>(1000 + i * 10);
    m.publishTime = m.logTime;
    m.data = payloads.back().data();
    m.dataSize = payloads.back().size();
    ASSERT_TRUE(writer.write(m).ok());
  }
  writer.close();

  std::string reason;
  const auto result = mcap_passthrough_rewrite(input_, output_, {}, &reason);
  ASSERT_TRUE(result.has_value()) << reason;
  EXPECT_EQ(result->chunks_copied, 0u);
  EXPECT_EQ(result->chunks_reencoded, 1u);
  EXPECT_EQ(result->messages_written, 3u);
  const auto spans = chunk_spans(output_);
  ASSERT_EQ(spans.size(), 1u);
  EXPECT_GT(spans[0].length, 0u);
  EXPECT_EQ(read_bag(output_).size(), 3u);
}

// Hand-built mcap: the renamed channel's Channel record is embedded in a
// chunk that carries none of that channel's messages, so the engine cannot
// strip it and must abort (removing the partial output) instead of shipping
// a verbatim chunk whose embedded record still declares the old name.
TEST_F(McapPassthroughTest, RenameFallsBackWhenChannelRecordIsNotLocatable)
{
  mcap::FileWriter out;
  ASSERT_TRUE(out.open(input_.string()).ok());
  mcap::McapWriter::writeMagic(out);
  mcap::McapWriter::write(out, mcap::Header{"ros2", "crafted"});

  mcap::Schema schema("std_msgs/msg/ByteMultiArray", "ros2msg", "byte[] data");
  schema.id = 1;
  mcap::Channel renamed_channel("/x", "cdr", schema.id);
  renamed_channel.id = 1;
  mcap::Channel other_channel("/y", "cdr", schema.id);
  other_channel.id = 2;
  const auto payload = payload_bytes(5, 16);

  std::vector<mcap::ChunkIndex> chunk_indexes;
  const auto write_chunk = [&](
                             const std::vector<mcap::Channel> & embedded,
                             mcap::ChannelId msg_channel, std::uint64_t log_time) {
    mcap::BufferWriter blob;
    for (const auto & channel : embedded) {
      mcap::Channel copy = channel;
      mcap::McapWriter::write(blob, copy);
    }
    const std::uint64_t msg_offset = blob.size();
    mcap::Message m;
    m.channelId = msg_channel;
    m.sequence = 0;
    m.logTime = log_time;
    m.publishTime = log_time;
    m.data = payload.data();
    m.dataSize = payload.size();
    mcap::McapWriter::write(blob, m);
    blob.end();

    mcap::Chunk chunk;
    chunk.messageStartTime = log_time;
    chunk.messageEndTime = log_time;
    chunk.uncompressedSize = blob.size();
    chunk.uncompressedCrc = 0;
    chunk.compression = "";
    chunk.compressedSize = blob.size();
    chunk.records = blob.data();
    const std::uint64_t chunk_start = out.size();
    mcap::McapWriter::write(out, chunk);
    const std::uint64_t index_start = out.size();
    mcap::MessageIndex mi;
    mi.channelId = msg_channel;
    mi.records = {{log_time, msg_offset}};
    mcap::ChunkIndex ci;
    ci.messageStartTime = log_time;
    ci.messageEndTime = log_time;
    ci.chunkStartOffset = chunk_start;
    ci.chunkLength = index_start - chunk_start;
    ci.messageIndexOffsets[msg_channel] = out.size();
    mcap::McapWriter::write(out, mi);
    ci.messageIndexLength = out.size() - index_start;
    ci.compression = "";
    ci.compressedSize = blob.size();
    ci.uncompressedSize = blob.size();
    chunk_indexes.push_back(ci);
  };

  // Chunk 1 embeds /x's Channel record but carries only a /y message; chunk
  // 2 carries /x's message with no embedded Channel record at all.
  write_chunk({renamed_channel, other_channel}, other_channel.id, 1000);
  write_chunk({}, renamed_channel.id, 1100);

  mcap::McapWriter::write(out, mcap::DataEnd{0});
  const std::uint64_t summary_start = out.size();
  {
    mcap::Schema copy = schema;
    mcap::McapWriter::write(out, copy);
  }
  for (const auto & channel : {renamed_channel, other_channel}) {
    mcap::Channel copy = channel;
    mcap::McapWriter::write(out, copy);
  }
  for (const auto & ci : chunk_indexes) {
    mcap::McapWriter::write(out, ci);
  }
  // readSummary(NoFallbackScan) refuses summaries without a Statistics
  // record, so even the crafted fixture must carry one.
  mcap::Statistics stats;
  stats.messageCount = 2;
  stats.schemaCount = 1;
  stats.channelCount = 2;
  stats.attachmentCount = 0;
  stats.metadataCount = 0;
  stats.chunkCount = 2;
  stats.messageStartTime = 1000;
  stats.messageEndTime = 1100;
  stats.channelMessageCounts = {{1, 1}, {2, 1}};
  mcap::McapWriter::write(out, stats);
  mcap::McapWriter::write(out, mcap::Footer{summary_start, 0}, /*crcEnabled=*/false);
  mcap::McapWriter::writeMagic(out);
  out.end();

  McapPassthroughEdit edit;
  edit.rename = {{"/x", "/x_new"}};
  std::string reason;
  const auto result = mcap_passthrough_rewrite(input_, output_, edit, &reason);
  EXPECT_FALSE(result.has_value());
  EXPECT_NE(reason.find("rename"), std::string::npos) << reason;
  EXPECT_FALSE(std::filesystem::exists(output_));
}

TEST_F(McapPassthroughTest, EmptyBagPassesThrough)
{
  mcap::McapWriterOptions options("ros2");
  mcap::McapWriter writer;
  ASSERT_TRUE(writer.open(input_.string(), options).ok());
  mcap::Schema schema("std_msgs/msg/ByteMultiArray", "ros2msg", "byte[] data");
  writer.addSchema(schema);
  mcap::Channel channel("/a", "cdr", schema.id);
  writer.addChannel(channel);
  writer.close();

  std::string reason;
  const auto result = mcap_passthrough_rewrite(input_, output_, {}, &reason);
  ASSERT_TRUE(result.has_value()) << reason;
  EXPECT_EQ(result->messages_written, 0u);
  EXPECT_TRUE(read_bag(output_).empty());
  EXPECT_TRUE(chunk_spans(output_).empty());
}
