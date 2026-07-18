// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// End-to-end coverage for rosbag2 `compression_mode: FILE` zstd bags: the
// whole `.db3` database is wrapped in a `.db3.zstd` envelope. bagwiz must
// transparently decompress it on read so every command (all of which funnel
// through io::open_read) accepts it like any other bag.

#include "bagwiz/io/bag_io.hpp"

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

std::vector<std::byte> slurp(const std::filesystem::path & path)
{
  std::ifstream in(path, std::ios::binary | std::ios::ate);
  EXPECT_TRUE(in.good());
  const auto size = in.tellg();
  in.seekg(0);
  std::vector<std::byte> buf(static_cast<std::size_t>(size));
  in.read(reinterpret_cast<char *>(buf.data()), size);
  return buf;
}

// Compress `plain` whole-buffer (single-shot) and write it to `path`. Mirrors
// rosbag2's ZstdCompressor::compress_uri, which wraps the entire `.db3` file.
void write_zstd_file(const std::filesystem::path & path, const std::vector<std::byte> & plain)
{
  const std::size_t bound = ZSTD_compressBound(plain.size());
  std::vector<std::byte> compressed(bound);
  const std::size_t written = ZSTD_compress(
    compressed.data(), compressed.size(), plain.data(), plain.size(), /*compressionLevel=*/3);
  ASSERT_FALSE(static_cast<bool>(ZSTD_isError(written))) << ZSTD_getErrorName(written);
  compressed.resize(written);

  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(out.good());
  out.write(
    reinterpret_cast<const char *>(compressed.data()),
    static_cast<std::streamsize>(compressed.size()));
  out.flush();
  ASSERT_TRUE(out.good());
}

// Write a plain (uncompressed) rosbag2-shaped `.db3` with two topics and two
// messages whose payloads are the raw kPayload* bytes.
void write_plain_db3(const std::filesystem::path & db3_path)
{
  sqlite3 * db = nullptr;
  ASSERT_EQ(sqlite3_open(db3_path.string().c_str(), &db), SQLITE_OK);

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

  const auto foo = as_byte_vector(kPayloadFoo);
  const auto bar = as_byte_vector(kPayloadBar);

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
  bind(1, 1'000'000'000LL, foo);
  bind(2, 2'000'000'000LL, bar);
  sqlite3_finalize(stmt);
  sqlite3_close(db);
}

// Emit a metadata.yaml describing a FILE-compressed sqlite3 bag: the logical
// name (`<stem>.db3`) lives in `files[].path` while the on-disk name
// (`<stem>.db3.zstd`) lives in `relative_file_paths` — exactly how rosbag2
// records a FILE-compressed bag.
void write_file_compressed_metadata(
  const std::filesystem::path & dir, std::string_view logical_name, std::string_view on_disk_name,
  std::string_view compression_format)
{
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "rosbag2_bagfile_information" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "version" << YAML::Value << 5;
  out << YAML::Key << "storage_identifier" << YAML::Value << "sqlite3";
  out << YAML::Key << "duration" << YAML::Value << YAML::BeginMap << YAML::Key << "nanoseconds"
      << YAML::Value << 1'000'000'000 << YAML::EndMap;
  out << YAML::Key << "starting_time" << YAML::Value << YAML::BeginMap << YAML::Key
      << "nanoseconds_since_epoch" << YAML::Value << 1'000'000'000 << YAML::EndMap;
  out << YAML::Key << "message_count" << YAML::Value << 2;
  out << YAML::Key << "topics_with_message_count" << YAML::Value << YAML::BeginSeq;
  for (const auto & [name, type] : std::vector<std::pair<std::string, std::string>>{
         {"/foo", "std_msgs/msg/String"}, {"/bar", "std_msgs/msg/Int32"}}) {
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
  out << YAML::Key << "compression_mode" << YAML::Value << "FILE";
  out << YAML::Key << "relative_file_paths" << YAML::Value << YAML::BeginSeq
      << std::string(on_disk_name) << YAML::EndSeq;
  out << YAML::Key << "files" << YAML::Value << YAML::BeginSeq;
  out << YAML::BeginMap;
  out << YAML::Key << "path" << YAML::Value << std::string(logical_name);
  out << YAML::Key << "starting_time" << YAML::Value << YAML::BeginMap << YAML::Key
      << "nanoseconds_since_epoch" << YAML::Value << 1'000'000'000 << YAML::EndMap;
  out << YAML::Key << "duration" << YAML::Value << YAML::BeginMap << YAML::Key << "nanoseconds"
      << YAML::Value << 1'000'000'000 << YAML::EndMap;
  out << YAML::Key << "message_count" << YAML::Value << 2;
  out << YAML::EndMap;
  out << YAML::EndSeq;
  out << YAML::EndMap;
  out << YAML::EndMap;

  std::ofstream f(dir / "metadata.yaml");
  f << out.c_str();
}

// Build a complete FILE-compressed sqlite3 directory bag and return the path
// to the bare `.db3.zstd` shard (used for the single-file test).
std::filesystem::path make_file_compressed_bag(
  const std::filesystem::path & dir, std::string_view compression_format = "zstd")
{
  std::filesystem::create_directories(dir);
  const auto plain_db3 = dir / "shard_0.db3";
  write_plain_db3(plain_db3);

  const auto bytes = slurp(plain_db3);
  const auto zstd_path = dir / "shard_0.db3.zstd";
  write_zstd_file(zstd_path, bytes);

  // Remove the plain db3: a real FILE-compressed bag only ships the envelope.
  std::filesystem::remove(plain_db3);

  write_file_compressed_metadata(dir, "shard_0.db3", "shard_0.db3.zstd", compression_format);
  return zstd_path;
}

// Emit a minimal but parseable metadata.yaml for a directory bag carrying the
// given rosbag2-layer compression settings. load_metadata_yaml needs a
// storage_identifier and at least one listed file path; nothing here is
// decompressed. Used to exercise is_file_compressed_bag's directory branch for
// non-FILE modes without materialising a real database.
void write_dir_metadata(
  const std::filesystem::path & dir, std::string_view compression_mode,
  std::string_view compression_format)
{
  std::filesystem::create_directories(dir);
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "rosbag2_bagfile_information" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "storage_identifier" << YAML::Value << "sqlite3";
  out << YAML::Key << "relative_file_paths" << YAML::Value << YAML::BeginSeq << "shard_0.db3"
      << YAML::EndSeq;
  out << YAML::Key << "compression_mode" << YAML::Value << std::string(compression_mode);
  out << YAML::Key << "compression_format" << YAML::Value << std::string(compression_format);
  out << YAML::EndMap;
  out << YAML::EndMap;
  std::ofstream f(dir / "metadata.yaml");
  f << out.c_str();
}

void verify_payloads_match(
  bagwiz::io::BagReader & reader,
  const std::vector<std::pair<std::string, std::vector<std::byte>>> & expected)
{
  std::vector<std::pair<std::string, std::vector<std::byte>>> seen;
  bagwiz::io::RawMessage msg;
  while (reader.next(msg)) {
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

class FileCompressionE2ETest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_file_compression_" +
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

}  // namespace

TEST_F(FileCompressionE2ETest, DirectoryBagDecompressesOnRead)
{
  const auto bag = tmp_dir_ / "dir_bag";
  make_file_compressed_bag(bag);

  auto reader = bagwiz::io::open_read(bag);
  verify_payloads_match(
    *reader, {{"/foo", as_byte_vector(kPayloadFoo)}, {"/bar", as_byte_vector(kPayloadBar)}});
}

TEST_F(FileCompressionE2ETest, DirectoryBagTopicsAvailableFromMetadata)
{
  const auto bag = tmp_dir_ / "dir_bag_topics";
  make_file_compressed_bag(bag);

  // topics() / compute_stats() answer from metadata.yaml alone — no shard
  // decompression required.
  auto reader = bagwiz::io::open_read(bag);
  const auto topics = reader->topics();
  ASSERT_EQ(topics.size(), 2U);

  const auto stats = reader->compute_stats();
  EXPECT_EQ(stats.total_messages, 2);
}

TEST_F(FileCompressionE2ETest, SingleFileEnvelopeDecompressesOnRead)
{
  const auto bag = tmp_dir_ / "single_file";
  const auto zstd_path = make_file_compressed_bag(bag);

  // Open the bare `.db3.zstd` directly (no surrounding metadata.yaml used).
  auto reader = bagwiz::io::open_read(zstd_path);
  verify_payloads_match(
    *reader, {{"/foo", as_byte_vector(kPayloadFoo)}, {"/bar", as_byte_vector(kPayloadBar)}});
}

TEST_F(FileCompressionE2ETest, DetectFormatResolvesInnerSqlite3)
{
  const auto bag = tmp_dir_ / "detect";
  const auto zstd_path = make_file_compressed_bag(bag);

  // Directory: via metadata.yaml storage_identifier.
  EXPECT_EQ(bagwiz::io::detect_format(bag), bagwiz::io::Format::Sqlite3);
  // Single file: via the `.db3.zstd` extension, without decompressing.
  EXPECT_EQ(bagwiz::io::detect_format(zstd_path), bagwiz::io::Format::Sqlite3);
}

TEST_F(FileCompressionE2ETest, RejectsNonZstdFileCompressionOnSqlite3)
{
  // A FILE-mode bag declaring a non-zstd format must fail fast on open.
  const auto bag = tmp_dir_ / "lz4_reject";
  make_file_compressed_bag(bag, "lz4");

  EXPECT_THROW(bagwiz::io::open_read(bag), std::runtime_error);
}

// is_file_compressed_bag flags exactly the bags whose contents cannot be read
// without a whole-database decompress: FILE-mode directory envelopes and bare
// `.db3.zstd` single files. These are the cases latency-sensitive callers (shell
// completion, in-place rewrite guards) must skip.
TEST_F(FileCompressionE2ETest, IsFileCompressedBagDetectsFileModeEnvelopes)
{
  const auto bag = tmp_dir_ / "file_mode";
  const auto zstd_path = make_file_compressed_bag(bag);

  EXPECT_TRUE(bagwiz::io::is_file_compressed_bag(bag));
  EXPECT_TRUE(bagwiz::io::is_file_compressed_bag(zstd_path));
}

// MESSAGE-mode (per-message zstd), uncompressed bags, and missing paths are all
// cheap to read, so is_file_compressed_bag must leave them alone.
TEST_F(FileCompressionE2ETest, IsFileCompressedBagRejectsMessageModeUncompressedAndMissing)
{
  const auto message_bag = tmp_dir_ / "message_mode";
  write_dir_metadata(message_bag, "MESSAGE", "zstd");
  EXPECT_FALSE(bagwiz::io::is_file_compressed_bag(message_bag));

  const auto plain_dir = tmp_dir_ / "plain_dir";
  write_dir_metadata(plain_dir, "", "");
  EXPECT_FALSE(bagwiz::io::is_file_compressed_bag(plain_dir));

  const auto plain_db3 = tmp_dir_ / "plain.db3";
  write_plain_db3(plain_db3);
  EXPECT_FALSE(bagwiz::io::is_file_compressed_bag(plain_db3));

  EXPECT_FALSE(bagwiz::io::is_file_compressed_bag(tmp_dir_ / "does_not_exist"));
}
