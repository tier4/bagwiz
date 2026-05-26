// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>
#include <sqlite3.h>
#include <yaml-cpp/yaml.h>

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
// ROS 2 messages, but bagwiz writers do not interpret the payload so any
// byte sequence is valid for round-trip testing.
constexpr std::array<std::uint8_t, 4> kPayload{0xDE, 0xAD, 0xBE, 0xEF};

bagwiz::io::TopicInfo make_topic(std::string name, std::string type)
{
  bagwiz::io::TopicInfo t;
  t.name = std::move(name);
  t.type = std::move(type);
  t.serialization_format = "cdr";
  return t;
}

bagwiz::io::TopicInfo make_topic_with_schema(
  std::string name, std::string type, std::string schema_text)
{
  auto t = make_topic(std::move(name), std::move(type));
  t.schema_encoding = "ros2msg";
  t.schema_text = std::move(schema_text);
  return t;
}

void write_fixture(
  const std::filesystem::path & path, bagwiz::io::CreateOptions options,
  const std::vector<bagwiz::io::TopicInfo> & topics,
  const std::vector<std::pair<std::string, int64_t>> & messages)
{
  auto writer = bagwiz::io::open_write(path, options);
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
  auto reader = bagwiz::io::open_read(path);

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
  bagwiz::io::RawMessage msg;
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
      ("bagwiz_writer_test_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
    std::filesystem::create_directories(tmp_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::vector<bagwiz::io::TopicInfo> topics_ = {
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
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = "none";  // keep test predictable & fast
  write_fixture(path, options, topics_, messages_);
  verify_round_trip(path);
}

TEST_F(WriterRoundTripTest, McapDirectory)
{
  const auto dir = tmp_dir_ / "mcap_dir";
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::Directory;
  options.mcap_compression = "none";
  write_fixture(dir, options, topics_, messages_);

  // metadata.yaml should exist and the directory should round-trip.
  EXPECT_TRUE(std::filesystem::exists(dir / "metadata.yaml"));
  verify_round_trip(dir);
}

TEST_F(WriterRoundTripTest, McapZstdCompression)
{
  const auto path = tmp_dir_ / "zstd.mcap";
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = "zstd";
  write_fixture(path, options, topics_, messages_);
  verify_round_trip(path);
}

TEST_F(WriterRoundTripTest, Sqlite3SingleFile)
{
  const auto path = tmp_dir_ / "out.db3";
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Sqlite3;
  options.layout = bagwiz::io::Layout::SingleFile;
  write_fixture(path, options, topics_, messages_);
  verify_round_trip(path);
}

TEST_F(WriterRoundTripTest, Sqlite3Directory)
{
  const auto dir = tmp_dir_ / "sqlite_dir";
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Sqlite3;
  options.layout = bagwiz::io::Layout::Directory;
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

TEST_F(WriterRoundTripTest, McapSchemaRoundTrips)
{
  // Self-describing write: schema_text passed at declare_topic() should
  // come back out verbatim when the bag is reopened. This is the central
  // payoff of MCAP self-description support — bagwiz-written MCAPs
  // carry message definitions forward instead of the empty schema.data
  // the writer used to emit before that support landed.
  const std::vector<bagwiz::io::TopicInfo> topics = {
    make_topic_with_schema("/foo", "std_msgs/msg/String", "string data"),
    make_topic_with_schema("/bar", "std_msgs/msg/Int32", "int32 data"),
  };

  const auto path = tmp_dir_ / "with_schemas.mcap";
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = "none";
  write_fixture(path, options, topics, messages_);

  auto reader = bagwiz::io::open_read(path);
  bool found_foo = false;
  bool found_bar = false;
  for (const auto & t : reader->topics()) {
    if (t.name == "/foo") {
      EXPECT_EQ(t.schema_text, "string data");
      EXPECT_EQ(t.schema_encoding, "ros2msg");
      found_foo = true;
    } else if (t.name == "/bar") {
      EXPECT_EQ(t.schema_text, "int32 data");
      EXPECT_EQ(t.schema_encoding, "ros2msg");
      found_bar = true;
    }
  }
  EXPECT_TRUE(found_foo);
  EXPECT_TRUE(found_bar);
}

TEST_F(WriterRoundTripTest, McapEmptySchemaStillSucceeds)
{
  // Callers that don't fill TopicInfo::schema_text must keep working —
  // the writer emits an empty Schema record, and the reader surfaces
  // empty schema_text without erroring.
  const auto path = tmp_dir_ / "no_schemas.mcap";
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = "none";
  write_fixture(path, options, topics_, messages_);

  auto reader = bagwiz::io::open_read(path);
  for (const auto & t : reader->topics()) {
    EXPECT_TRUE(t.schema_text.empty()) << t.name;
  }
}

TEST_F(WriterRoundTripTest, Sqlite3PreservesSchemaFieldsViaMessageDefinitionsTable)
{
  // SQLite3 v4 storage carries schema bytes in the `message_definitions`
  // table (added in Iron). bagwiz writes the v4 layout unconditionally
  // and round-trips schema_text / schema_encoding accordingly. Topics
  // sharing a type point at one shared message_definitions row.
  const std::vector<bagwiz::io::TopicInfo> topics = {
    make_topic_with_schema("/foo", "std_msgs/msg/String", "string data"),
    make_topic_with_schema("/bar", "std_msgs/msg/Int32", "int32 data"),
  };

  const auto path = tmp_dir_ / "schema_sqlite.db3";
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Sqlite3;
  options.layout = bagwiz::io::Layout::SingleFile;
  write_fixture(path, options, topics, messages_);

  auto reader = bagwiz::io::open_read(path);
  std::size_t matched = 0;
  for (const auto & t : reader->topics()) {
    if (t.name == "/foo") {
      EXPECT_EQ(t.schema_text, "string data") << "schema_text not preserved for /foo";
      EXPECT_EQ(t.schema_encoding, "ros2msg");
      ++matched;
    } else if (t.name == "/bar") {
      EXPECT_EQ(t.schema_text, "int32 data") << "schema_text not preserved for /bar";
      EXPECT_EQ(t.schema_encoding, "ros2msg");
      ++matched;
    }
  }
  EXPECT_EQ(matched, 2U);
}

TEST_F(WriterRoundTripTest, Sqlite3WritesIronCompatibleV4Layout)
{
  // Verify the on-disk layout matches Iron rosbag2's expected
  // schema_version=4: `topics.type_description_hash` column exists,
  // `message_definitions` table exists, and `schema(schema_version)`
  // reports 4. This is the structural contract Iron+ rosbag2 readers
  // probe for via `field_exists` / `table_exists`.
  bagwiz::io::TopicInfo t;
  t.name = "/imu";
  t.type = "sensor_msgs/msg/Imu";
  t.serialization_format = "cdr";
  t.schema_encoding = "ros2msg";
  t.schema_text = "stub";
  t.type_description_hash = "RIHS01_deadbeef";

  const auto path = tmp_dir_ / "iron_layout.db3";
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Sqlite3;
  options.layout = bagwiz::io::Layout::SingleFile;
  const std::vector<bagwiz::io::TopicInfo> topics = {t};
  const std::vector<std::pair<std::string, int64_t>> empty_msgs;
  write_fixture(path, options, topics, empty_msgs);

  // Probe the on-disk schema directly with sqlite3 — this is the
  // identical interrogation Iron's rosbag2 reader performs.
  sqlite3 * db = nullptr;
  ASSERT_EQ(sqlite3_open(path.string().c_str(), &db), SQLITE_OK);

  const auto column_exists = [&](const std::string & table, const std::string & col) {
    sqlite3_stmt * stmt = nullptr;
    EXPECT_EQ(
      sqlite3_prepare_v2(db, ("PRAGMA table_info('" + table + "')").c_str(), -1, &stmt, nullptr),
      SQLITE_OK);
    bool found = false;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      const auto * name = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
      if (name != nullptr && col == name) {
        found = true;
        break;
      }
    }
    sqlite3_finalize(stmt);
    return found;
  };
  const auto table_exists = [&](const std::string & name) {
    sqlite3_stmt * stmt = nullptr;
    EXPECT_EQ(
      sqlite3_prepare_v2(
        db, "SELECT 1 FROM sqlite_master WHERE type='table' AND name=?", -1, &stmt, nullptr),
      SQLITE_OK);
    sqlite3_bind_text(stmt, 1, name.c_str(), -1, SQLITE_TRANSIENT);
    const bool found = sqlite3_step(stmt) == SQLITE_ROW;
    sqlite3_finalize(stmt);
    return found;
  };

  EXPECT_TRUE(column_exists("topics", "type_description_hash"));
  EXPECT_TRUE(table_exists("message_definitions"));
  EXPECT_TRUE(column_exists("message_definitions", "encoded_message_definition"));

  // Verify the schema_version row.
  sqlite3_stmt * stmt = nullptr;
  ASSERT_EQ(
    sqlite3_prepare_v2(db, "SELECT schema_version FROM schema", -1, &stmt, nullptr), SQLITE_OK);
  ASSERT_EQ(sqlite3_step(stmt), SQLITE_ROW);
  EXPECT_EQ(sqlite3_column_int(stmt, 0), 4);
  sqlite3_finalize(stmt);

  // Verify the message_definitions row content matches what we wrote.
  ASSERT_EQ(
    sqlite3_prepare_v2(
      db,
      "SELECT topic_type, encoding, encoded_message_definition, type_description_hash "
      "FROM message_definitions",
      -1, &stmt, nullptr),
    SQLITE_OK);
  ASSERT_EQ(sqlite3_step(stmt), SQLITE_ROW);
  EXPECT_EQ(
    std::string(reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0))),
    "sensor_msgs/msg/Imu");
  EXPECT_EQ(std::string(reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1))), "ros2msg");
  EXPECT_EQ(std::string(reinterpret_cast<const char *>(sqlite3_column_text(stmt, 2))), "stub");
  EXPECT_EQ(
    std::string(reinterpret_cast<const char *>(sqlite3_column_text(stmt, 3))), "RIHS01_deadbeef");
  sqlite3_finalize(stmt);

  // type_description_hash must also be on the topics row.
  ASSERT_EQ(
    sqlite3_prepare_v2(db, "SELECT type_description_hash FROM topics", -1, &stmt, nullptr),
    SQLITE_OK);
  ASSERT_EQ(sqlite3_step(stmt), SQLITE_ROW);
  EXPECT_EQ(
    std::string(reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0))), "RIHS01_deadbeef");
  sqlite3_finalize(stmt);

  sqlite3_close(db);
}

// rosbag2's metadata reader (rosbag2_storage humble 0.15.16
// metadata_io.cpp::convert<BagMetadata>::decode) requires the `files:`
// sequence whenever `version >= 5`, throwing
//   Exception on parsing info file: invalid node; first invalid key: "files"
// when it is missing. Pin the schema so `ros2 bag info <dir>` keeps working
// across the supported ROS 2 distros (humble + jazzy).
namespace
{

YAML::Node load_metadata_info(const std::filesystem::path & dir)
{
  const auto metadata_path = dir / "metadata.yaml";
  EXPECT_TRUE(std::filesystem::exists(metadata_path)) << metadata_path;
  const auto root = YAML::LoadFile(metadata_path.string());
  return root["rosbag2_bagfile_information"];
}

void expect_files_section_matches_summary(
  const YAML::Node & info, const std::string & expected_path, int64_t expected_start_ns,
  int64_t expected_end_ns, int64_t expected_message_count)
{
  ASSERT_TRUE(info["version"]) << "metadata.yaml is missing the `version` key";
  EXPECT_GE(info["version"].as<int>(), 5)
    << "metadata version must be >= 5 to advertise the `files:` schema";

  const auto files = info["files"];
  ASSERT_TRUE(files) << "metadata.yaml is missing the `files:` sequence";
  ASSERT_TRUE(files.IsSequence());
  ASSERT_EQ(files.size(), 1U);

  const auto file = files[0];
  EXPECT_EQ(file["path"].as<std::string>(), expected_path);
  EXPECT_EQ(file["starting_time"]["nanoseconds_since_epoch"].as<int64_t>(), expected_start_ns);
  EXPECT_EQ(file["duration"]["nanoseconds"].as<int64_t>(), expected_end_ns - expected_start_ns);
  EXPECT_EQ(file["message_count"].as<int64_t>(), expected_message_count);

  // `relative_file_paths:` is still emitted for older readers; both must agree
  // on the shard name so legacy and modern consumers see the same bag layout.
  const auto rel_paths = info["relative_file_paths"];
  ASSERT_TRUE(rel_paths) << "metadata.yaml is missing `relative_file_paths`";
  ASSERT_TRUE(rel_paths.IsSequence());
  ASSERT_EQ(rel_paths.size(), 1U);
  EXPECT_EQ(rel_paths[0].as<std::string>(), expected_path);
}

}  // namespace

TEST_F(WriterRoundTripTest, Sqlite3DirectoryMetadataDeclaresFilesSection)
{
  const auto dir = tmp_dir_ / "sqlite_metadata";
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Sqlite3;
  options.layout = bagwiz::io::Layout::Directory;
  write_fixture(dir, options, topics_, messages_);

  const auto info = load_metadata_info(dir);
  expect_files_section_matches_summary(
    info, dir.filename().string() + "_0.db3", /*start=*/1'000'000'000LL,
    /*end=*/2'000'000'001LL, /*messages=*/5);
}

TEST_F(WriterRoundTripTest, McapDirectoryMetadataDeclaresFilesSection)
{
  const auto dir = tmp_dir_ / "mcap_metadata";
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::Directory;
  options.mcap_compression = "none";
  write_fixture(dir, options, topics_, messages_);

  const auto info = load_metadata_info(dir);
  expect_files_section_matches_summary(
    info, dir.filename().string() + "_0.mcap", /*start=*/1'000'000'000LL,
    /*end=*/2'000'000'001LL, /*messages=*/5);
}

TEST_F(WriterRoundTripTest, Sqlite3DedupsMessageDefinitionsByType)
{
  // Two topics on the same type must produce exactly one
  // message_definitions row — matching upstream rosbag2 behaviour
  // (one row per unique topic_type, not per topic).
  const std::vector<bagwiz::io::TopicInfo> topics = {
    make_topic_with_schema("/a", "std_msgs/msg/String", "string data"),
    make_topic_with_schema("/b", "std_msgs/msg/String", "string data"),
  };

  const auto path = tmp_dir_ / "dedup_msgdef.db3";
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Sqlite3;
  options.layout = bagwiz::io::Layout::SingleFile;
  const std::vector<std::pair<std::string, int64_t>> empty_msgs;
  write_fixture(path, options, topics, empty_msgs);

  sqlite3 * db = nullptr;
  ASSERT_EQ(sqlite3_open(path.string().c_str(), &db), SQLITE_OK);
  sqlite3_stmt * stmt = nullptr;
  ASSERT_EQ(
    sqlite3_prepare_v2(db, "SELECT COUNT(*) FROM message_definitions", -1, &stmt, nullptr),
    SQLITE_OK);
  ASSERT_EQ(sqlite3_step(stmt), SQLITE_ROW);
  EXPECT_EQ(sqlite3_column_int(stmt, 0), 1);
  sqlite3_finalize(stmt);
  sqlite3_close(db);
}
