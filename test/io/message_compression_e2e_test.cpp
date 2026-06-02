// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/bag_io.hpp"

#include <mcap/writer.hpp>

#include <gtest/gtest.h>
#include <sqlite3.h>
#include <yaml-cpp/yaml.h>
#include <zstd.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{

// Deterministic payloads — CDR shape is irrelevant because bagwiz does not
// interpret the bytes, but using distinct payloads per topic makes a
// mis-routed decompression easy to spot.
constexpr std::array<std::uint8_t, 6> kPayloadFoo{0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE};
constexpr std::array<std::uint8_t, 4> kPayloadBar{0x01, 0x02, 0x03, 0x04};

template <std::size_t N>
std::vector<std::byte> as_byte_vector(const std::array<std::uint8_t, N> & raw)
{
  std::vector<std::byte> out(N);
  for (std::size_t i = 0; i < N; ++i) {
    out.at(i) = std::byte{raw.at(i)};
  }
  return out;
}

// Compress `plain` with libzstd in single-shot mode. rosbag2's MESSAGE-mode
// writer always emits frames whose header carries the original size, so we
// mirror that path here.
std::vector<std::byte> zstd_compress(const std::vector<std::byte> & plain)
{
  const std::size_t bound = ZSTD_compressBound(plain.size());
  std::vector<std::byte> compressed(bound);
  const std::size_t written = ZSTD_compress(
    compressed.data(), compressed.size(), plain.data(), plain.size(), /*compressionLevel=*/1);
  EXPECT_FALSE(static_cast<bool>(ZSTD_isError(written))) << ZSTD_getErrorName(written);
  compressed.resize(written);
  return compressed;
}

// Hand-emit a metadata.yaml that matches rosbag2's schema closely enough
// for `io::load_metadata_yaml` to populate the fields bag_factory inspects:
// storage_identifier, compression_mode/format, relative_file_paths, topics.
void write_metadata_yaml(
  const std::filesystem::path & dir, std::string_view storage_id, std::string_view shard_rel,
  std::string_view compression_mode, std::string_view compression_format,
  const std::vector<std::pair<std::string, std::string>> & topics)
{
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "rosbag2_bagfile_information" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "version" << YAML::Value << 5;
  out << YAML::Key << "storage_identifier" << YAML::Value << std::string(storage_id);
  out << YAML::Key << "duration" << YAML::Value << YAML::BeginMap << YAML::Key << "nanoseconds"
      << YAML::Value << 0 << YAML::EndMap;
  out << YAML::Key << "starting_time" << YAML::Value << YAML::BeginMap << YAML::Key
      << "nanoseconds_since_epoch" << YAML::Value << 0 << YAML::EndMap;
  out << YAML::Key << "message_count" << YAML::Value << static_cast<int>(topics.size());
  out << YAML::Key << "topics_with_message_count" << YAML::Value << YAML::BeginSeq;
  for (const auto & [name, type] : topics) {
    out << YAML::BeginMap;
    out << YAML::Key << "topic_metadata" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << name;
    out << YAML::Key << "type" << YAML::Value << type;
    out << YAML::Key << "serialization_format" << YAML::Value << "cdr";
    out << YAML::Key << "offered_qos_profiles" << YAML::Value << "";
    out << YAML::EndMap;
    out << YAML::Key << "message_count" << YAML::Value << 1;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
  out << YAML::Key << "compression_format" << YAML::Value << std::string(compression_format);
  out << YAML::Key << "compression_mode" << YAML::Value << std::string(compression_mode);
  out << YAML::Key << "relative_file_paths" << YAML::Value << YAML::BeginSeq
      << std::string(shard_rel) << YAML::EndSeq;
  out << YAML::Key << "files" << YAML::Value << YAML::BeginSeq;
  out << YAML::BeginMap;
  out << YAML::Key << "path" << YAML::Value << std::string(shard_rel);
  out << YAML::Key << "starting_time" << YAML::Value << YAML::BeginMap << YAML::Key
      << "nanoseconds_since_epoch" << YAML::Value << 0 << YAML::EndMap;
  out << YAML::Key << "duration" << YAML::Value << YAML::BeginMap << YAML::Key << "nanoseconds"
      << YAML::Value << 0 << YAML::EndMap;
  out << YAML::Key << "message_count" << YAML::Value << static_cast<int>(topics.size());
  out << YAML::EndMap;
  out << YAML::EndSeq;
  out << YAML::EndMap;
  out << YAML::EndMap;

  std::ofstream f(dir / "metadata.yaml");
  f << out.c_str();
}

// Build a 1-shard SQLite3 directory bag whose `messages.data` blobs are
// pre-compressed zstd frames. Mirrors what rosbag2's
// SequentialCompressionWriter (mode=MESSAGE, format=zstd) emits.
void make_sqlite_message_zstd_bag(const std::filesystem::path & dir)
{
  std::filesystem::create_directories(dir);
  const auto shard_path = dir / "shard_0.db3";

  sqlite3 * db = nullptr;
  ASSERT_EQ(sqlite3_open(shard_path.string().c_str(), &db), SQLITE_OK);

  const char * schema_sql =
    "CREATE TABLE topics (id INTEGER PRIMARY KEY, name TEXT NOT NULL, "
    "type TEXT NOT NULL, serialization_format TEXT NOT NULL, "
    "offered_qos_profiles TEXT NOT NULL);"
    "CREATE TABLE messages (id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, "
    "timestamp INTEGER NOT NULL, data BLOB NOT NULL);";
  ASSERT_EQ(sqlite3_exec(db, schema_sql, nullptr, nullptr, nullptr), SQLITE_OK);

  ASSERT_EQ(
    sqlite3_exec(
      db,
      "INSERT INTO topics (id, name, type, serialization_format, offered_qos_profiles) "
      "VALUES (1, '/foo', 'std_msgs/msg/String', 'cdr', '');"
      "INSERT INTO topics (id, name, type, serialization_format, offered_qos_profiles) "
      "VALUES (2, '/bar', 'std_msgs/msg/Int32', 'cdr', '');",
      nullptr, nullptr, nullptr),
    SQLITE_OK);

  const auto foo_plain = as_byte_vector(kPayloadFoo);
  const auto bar_plain = as_byte_vector(kPayloadBar);
  const auto foo_compressed = zstd_compress(foo_plain);
  const auto bar_compressed = zstd_compress(bar_plain);

  sqlite3_stmt * stmt = nullptr;
  ASSERT_EQ(
    sqlite3_prepare_v2(
      db, "INSERT INTO messages (topic_id, timestamp, data) VALUES (?, ?, ?)", -1, &stmt, nullptr),
    SQLITE_OK);
  const auto bind = [&](int topic_id, int64_t ts, const std::vector<std::byte> & blob) {
    sqlite3_reset(stmt);
    sqlite3_bind_int64(stmt, 1, topic_id);
    sqlite3_bind_int64(stmt, 2, ts);
    sqlite3_bind_blob(stmt, 3, blob.data(), static_cast<int>(blob.size()), SQLITE_TRANSIENT);
    ASSERT_EQ(sqlite3_step(stmt), SQLITE_DONE);
  };
  bind(1, 1'000'000'000LL, foo_compressed);
  bind(2, 2'000'000'000LL, bar_compressed);
  sqlite3_finalize(stmt);
  sqlite3_close(db);

  write_metadata_yaml(
    dir, "sqlite3", "shard_0.db3", "message", "zstd",
    {{"/foo", "std_msgs/msg/String"}, {"/bar", "std_msgs/msg/Int32"}});
}

// Build a 1-shard MCAP directory bag whose per-message payloads are
// pre-compressed zstd frames. The MCAP container itself is unchunked /
// uncompressed; the compression lives strictly at the message body level,
// matching rosbag2's MESSAGE-mode MCAP output.
void make_mcap_message_zstd_bag(const std::filesystem::path & dir)
{
  std::filesystem::create_directories(dir);
  const auto shard_path = dir / "shard_0.mcap";

  mcap::McapWriter writer;
  mcap::McapWriterOptions wopts("ros2");
  wopts.compression = mcap::Compression::None;
  const auto status = writer.open(shard_path.string(), wopts);
  ASSERT_TRUE(status.ok()) << status.message;

  mcap::Schema foo_schema("std_msgs/msg/String", "", "");
  writer.addSchema(foo_schema);
  mcap::Channel foo_channel("/foo", "cdr", foo_schema.id);
  writer.addChannel(foo_channel);

  mcap::Schema bar_schema("std_msgs/msg/Int32", "", "");
  writer.addSchema(bar_schema);
  mcap::Channel bar_channel("/bar", "cdr", bar_schema.id);
  writer.addChannel(bar_channel);

  const auto foo_plain = as_byte_vector(kPayloadFoo);
  const auto bar_plain = as_byte_vector(kPayloadBar);
  const auto foo_compressed = zstd_compress(foo_plain);
  const auto bar_compressed = zstd_compress(bar_plain);

  const auto write_msg = [&](mcap::ChannelId chan, int64_t ts, const std::vector<std::byte> & d) {
    mcap::Message msg;
    msg.channelId = chan;
    msg.sequence = 0;
    msg.logTime = static_cast<mcap::Timestamp>(ts);
    msg.publishTime = msg.logTime;
    msg.data = d.data();
    msg.dataSize = d.size();
    const auto s = writer.write(msg);
    ASSERT_TRUE(s.ok()) << s.message;
  };
  write_msg(foo_channel.id, 1'000'000'000LL, foo_compressed);
  write_msg(bar_channel.id, 2'000'000'000LL, bar_compressed);
  writer.close();

  write_metadata_yaml(
    dir, "mcap", "shard_0.mcap", "message", "zstd",
    {{"/foo", "std_msgs/msg/String"}, {"/bar", "std_msgs/msg/Int32"}});
}

class MessageCompressionE2ETest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_msg_compression_" +
                std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
    std::filesystem::create_directories(tmp_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
};

// Verifies the iterator yields payloads byte-equal to `expected` keyed by
// topic name. Tolerates either log-time order or topic-grouped order.
void verify_payloads_match(
  const std::filesystem::path & bag,
  const std::vector<std::pair<std::string, std::vector<std::byte>>> & expected)
{
  auto reader = bagwiz::io::open_read(bag);
  std::vector<std::pair<std::string, std::vector<std::byte>>> seen;
  bagwiz::io::RawMessage msg;
  while (reader->next(msg)) {
    std::vector<std::byte> bytes(msg.payload.begin(), msg.payload.end());
    seen.emplace_back(msg.topic->name, std::move(bytes));
  }
  ASSERT_EQ(seen.size(), expected.size());
  for (const auto & [topic, payload] : expected) {
    bool matched = false;
    for (const auto & [s_topic, s_payload] : seen) {
      if (s_topic == topic && s_payload == payload) {
        matched = true;
        break;
      }
    }
    EXPECT_TRUE(matched) << "no decompressed match for topic " << topic;
  }
}

}  // namespace

TEST_F(MessageCompressionE2ETest, Sqlite3MessageModeZstdDecompresses)
{
  const auto bag = tmp_dir_ / "sqlite_msg_zstd";
  make_sqlite_message_zstd_bag(bag);

  verify_payloads_match(
    bag, {{"/foo", as_byte_vector(kPayloadFoo)}, {"/bar", as_byte_vector(kPayloadBar)}});
}

TEST_F(MessageCompressionE2ETest, McapMessageModeZstdDecompresses)
{
  const auto bag = tmp_dir_ / "mcap_msg_zstd";
  make_mcap_message_zstd_bag(bag);

  verify_payloads_match(
    bag, {{"/foo", as_byte_vector(kPayloadFoo)}, {"/bar", as_byte_vector(kPayloadBar)}});
}

TEST_F(MessageCompressionE2ETest, RejectsMessageModeWithNonZstdFormat)
{
  // Same on-disk bytes as the zstd path; only the declared format differs.
  // The factory must refuse before opening the storage file, since silently
  // returning compressed bytes would corrupt callers.
  const auto bag = tmp_dir_ / "msg_lz4_reject";
  make_sqlite_message_zstd_bag(bag);
  // Overwrite metadata.yaml with a different declared format.
  write_metadata_yaml(
    bag, "sqlite3", "shard_0.db3", "message", "lz4",
    {{"/foo", "std_msgs/msg/String"}, {"/bar", "std_msgs/msg/Int32"}});

  EXPECT_THROW(bagwiz::io::open_read(bag), std::runtime_error);
}

// FILE-mode zstd over sqlite3 (the whole-database `.db3.zstd` envelope) is now
// accepted and transparently decompressed on read — see
// file_compression_e2e_test.cpp for the full coverage of that path.

TEST_F(MessageCompressionE2ETest, AcceptsFileModeOnMcap)
{
  // MCAP `compression_mode: file` is rosbag2's label for storage-internal
  // chunk compression, which libmcap decompresses transparently. The factory
  // must accept it without constructing a MessageDecompressor.
  const auto bag = tmp_dir_ / "mcap_chunk_compressed";
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::Directory;
  options.mcap_compression = "zstd";

  bagwiz::io::TopicInfo topic;
  topic.name = "/foo";
  topic.type = "std_msgs/msg/String";
  topic.serialization_format = "cdr";

  auto writer = bagwiz::io::open_write(bag, options);
  writer->declare_topic(topic);
  const auto plain = as_byte_vector(kPayloadFoo);
  writer->write("/foo", 1'000'000'000LL, std::span<const std::byte>(plain.data(), plain.size()));
  writer->close();

  // Round-trip succeeds; libmcap returns the plain bytes already.
  verify_payloads_match(bag, {{"/foo", plain}});
}
