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

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

namespace
{

// Minimal rosbag2-compatible .db3 fixture. We only create the tables that
// the reader actually queries (topics, messages) — the extra metadata/
// schema tables rosbag2 maintains are not needed for read-side testing.
std::filesystem::path write_fixture_db3(const std::filesystem::path & dir)
{
  std::filesystem::create_directories(dir);
  const auto path = dir / "bag_0.db3";

  sqlite3 * db = nullptr;
  int rc = sqlite3_open(path.string().c_str(), &db);
  EXPECT_EQ(rc, SQLITE_OK);

  const char * schema =
    "CREATE TABLE topics("
    "  id INTEGER PRIMARY KEY,"
    "  name TEXT NOT NULL,"
    "  type TEXT NOT NULL,"
    "  serialization_format TEXT NOT NULL,"
    "  offered_qos_profiles TEXT NOT NULL);"
    "CREATE TABLE messages("
    "  id INTEGER PRIMARY KEY,"
    "  topic_id INTEGER NOT NULL,"
    "  timestamp INTEGER NOT NULL,"
    "  data BLOB NOT NULL);"
    "CREATE INDEX timestamp_idx ON messages (timestamp ASC);";
  char * errmsg = nullptr;
  rc = sqlite3_exec(db, schema, nullptr, nullptr, &errmsg);
  EXPECT_EQ(rc, SQLITE_OK) << (errmsg ? errmsg : "");
  sqlite3_free(errmsg);

  rc = sqlite3_exec(
    db,
    "INSERT INTO topics(id, name, type, serialization_format, offered_qos_profiles) VALUES"
    "  (1, '/foo', 'std_msgs/msg/String', 'cdr', ''),"
    "  (2, '/bar', 'std_msgs/msg/Int32', 'cdr', '');",
    nullptr, nullptr, nullptr);
  EXPECT_EQ(rc, SQLITE_OK);

  const std::array<std::uint8_t, 4> payload{0xDE, 0xAD, 0xBE, 0xEF};

  sqlite3_stmt * stmt = nullptr;
  rc = sqlite3_prepare_v2(
    db, "INSERT INTO messages(topic_id, timestamp, data) VALUES (?, ?, ?);", -1, &stmt, nullptr);
  EXPECT_EQ(rc, SQLITE_OK);

  // 3 messages on /foo (timestamps 1e9, 1e9+1, 1e9+2)
  for (int i = 0; i < 3; ++i) {
    sqlite3_bind_int64(stmt, 1, 1);
    sqlite3_bind_int64(stmt, 2, 1'000'000'000LL + i);
    sqlite3_bind_blob(stmt, 3, payload.data(), static_cast<int>(payload.size()), SQLITE_STATIC);
    EXPECT_EQ(sqlite3_step(stmt), SQLITE_DONE);
    sqlite3_reset(stmt);
  }
  // 2 messages on /bar (timestamps 2e9, 2e9+1)
  for (int i = 0; i < 2; ++i) {
    sqlite3_bind_int64(stmt, 1, 2);
    sqlite3_bind_int64(stmt, 2, 2'000'000'000LL + i);
    sqlite3_bind_blob(stmt, 3, payload.data(), static_cast<int>(payload.size()), SQLITE_STATIC);
    EXPECT_EQ(sqlite3_step(stmt), SQLITE_DONE);
    sqlite3_reset(stmt);
  }
  sqlite3_finalize(stmt);
  sqlite3_close(db);
  return path;
}

// Same shape as `write_fixture_db3` but with zero message rows. Exercises the
// empty-bag branch of compute_stats(), where MIN/MAX(timestamp) are SQL NULL
// and start_ns/end_ns must stay 0.
std::filesystem::path write_fixture_db3_no_messages(const std::filesystem::path & dir)
{
  std::filesystem::create_directories(dir);
  const auto path = dir / "bag_0.db3";

  sqlite3 * db = nullptr;
  EXPECT_EQ(sqlite3_open(path.string().c_str(), &db), SQLITE_OK);

  const char * schema =
    "CREATE TABLE topics("
    "  id INTEGER PRIMARY KEY,"
    "  name TEXT NOT NULL,"
    "  type TEXT NOT NULL,"
    "  serialization_format TEXT NOT NULL,"
    "  offered_qos_profiles TEXT NOT NULL);"
    "CREATE TABLE messages("
    "  id INTEGER PRIMARY KEY,"
    "  topic_id INTEGER NOT NULL,"
    "  timestamp INTEGER NOT NULL,"
    "  data BLOB NOT NULL);"
    "CREATE INDEX timestamp_idx ON messages (timestamp ASC);"
    "INSERT INTO topics(id, name, type, serialization_format, offered_qos_profiles) VALUES"
    "  (1, '/foo', 'std_msgs/msg/String', 'cdr', '');";
  char * errmsg = nullptr;
  EXPECT_EQ(sqlite3_exec(db, schema, nullptr, nullptr, &errmsg), SQLITE_OK)
    << (errmsg ? errmsg : "");
  sqlite3_free(errmsg);

  sqlite3_close(db);
  return path;
}

// Iron+ schema_version=4 fixture. Same row data as `write_fixture_db3`
// but with the v4 column on `topics` and the `message_definitions`
// table populated, so the reader can backfill schema_text /
// schema_encoding / type_description_hash from the bag itself.
std::filesystem::path write_fixture_db3_v4(const std::filesystem::path & dir)
{
  std::filesystem::create_directories(dir);
  const auto path = dir / "bag_0.db3";

  sqlite3 * db = nullptr;
  EXPECT_EQ(sqlite3_open(path.string().c_str(), &db), SQLITE_OK);

  const char * schema =
    "CREATE TABLE schema(schema_version INTEGER PRIMARY KEY, ros_distro TEXT NOT NULL);"
    "INSERT INTO schema(schema_version, ros_distro) VALUES (4, 'iron');"
    "CREATE TABLE topics("
    "  id INTEGER PRIMARY KEY,"
    "  name TEXT NOT NULL,"
    "  type TEXT NOT NULL,"
    "  serialization_format TEXT NOT NULL,"
    "  offered_qos_profiles TEXT NOT NULL,"
    "  type_description_hash TEXT NOT NULL);"
    "CREATE TABLE message_definitions("
    "  id INTEGER PRIMARY KEY,"
    "  topic_type TEXT NOT NULL,"
    "  encoding TEXT NOT NULL,"
    "  encoded_message_definition TEXT NOT NULL,"
    "  type_description_hash TEXT NOT NULL);"
    "CREATE TABLE messages("
    "  id INTEGER PRIMARY KEY,"
    "  topic_id INTEGER NOT NULL,"
    "  timestamp INTEGER NOT NULL,"
    "  data BLOB NOT NULL);"
    "CREATE INDEX timestamp_idx ON messages (timestamp ASC);"
    "INSERT INTO topics(id, name, type, serialization_format, offered_qos_profiles, "
    "type_description_hash) VALUES"
    "  (1, '/foo', 'std_msgs/msg/String', 'cdr', '', 'RIHS01_aaa'),"
    "  (2, '/bar', 'std_msgs/msg/Int32', 'cdr', '', 'RIHS01_bbb');"
    "INSERT INTO message_definitions(id, topic_type, encoding, encoded_message_definition, "
    "type_description_hash) VALUES"
    "  (1, 'std_msgs/msg/String', 'ros2msg', 'string data', 'RIHS01_aaa'),"
    "  (2, 'std_msgs/msg/Int32', 'ros2msg', 'int32 data', 'RIHS01_bbb');";
  char * errmsg = nullptr;
  EXPECT_EQ(sqlite3_exec(db, schema, nullptr, nullptr, &errmsg), SQLITE_OK)
    << (errmsg ? errmsg : "");
  sqlite3_free(errmsg);

  sqlite3_close(db);
  return path;
}

std::filesystem::path write_fixture_directory(const std::filesystem::path & dir)
{
  std::filesystem::create_directories(dir);
  write_fixture_db3(dir);

  std::ofstream f(dir / "metadata.yaml");
  f << "rosbag2_bagfile_information:\n"
    << "  version: 6\n"
    << "  storage_identifier: sqlite3\n"
    << "  relative_file_paths:\n"
    << "    - bag_0.db3\n"
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

// Metadata.yaml carries the full summary block but the .db3 is intentionally
// missing. Any code path that opens the shard will fail loudly, proving the
// metadata fast path is the one actually used.
std::filesystem::path write_fixture_directory_summary_only(const std::filesystem::path & dir)
{
  std::filesystem::create_directories(dir);

  std::ofstream f(dir / "metadata.yaml");
  f << "rosbag2_bagfile_information:\n"
    << "  version: 6\n"
    << "  storage_identifier: sqlite3\n"
    << "  duration:\n"
    << "    nanoseconds: 1000000001\n"
    << "  starting_time:\n"
    << "    nanoseconds_since_epoch: 1000000000\n"
    << "  message_count: 5\n"
    << "  relative_file_paths:\n"
    << "    - bag_0.db3\n"
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

class Sqlite3ReaderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_sqlite3_test_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
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

TEST_F(Sqlite3ReaderTest, OpensSingleFileAndListsTopics)
{
  const auto path = write_fixture_db3(tmp_dir_ / "single");

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
    } else if (t.name == "/bar") {
      seen_bar = true;
      EXPECT_EQ(t.type, "std_msgs/msg/Int32");
    }
    // SQLite3 has no slot for schema bytes; they stay empty even after
    // populate_schemas() (which is a no-op on this reader).
    EXPECT_TRUE(t.schema_text.empty()) << t.name;
    EXPECT_TRUE(t.schema_encoding.empty()) << t.name;
  }
  EXPECT_TRUE(seen_foo);
  EXPECT_TRUE(seen_bar);

  // populate_schemas() must be safe to call and remain a no-op.
  reader->populate_schemas();
  for (const auto & t : reader->topics()) {
    EXPECT_TRUE(t.schema_text.empty());
    EXPECT_TRUE(t.schema_encoding.empty());
  }
}

TEST_F(Sqlite3ReaderTest, ReadsIronSchemaV4WithMessageDefinitions)
{
  // Iron+ rosbag2 SQLite3 (schema_version=4) embeds per-type
  // self-description in the `message_definitions` table and a
  // `type_description_hash` column on `topics`. The reader must
  // backfill TopicInfo's schema_text / schema_encoding /
  // type_description_hash so callers don't need a local typestore
  // overlay to decode messages.
  const auto path = write_fixture_db3_v4(tmp_dir_ / "v4");

  auto reader = bagwiz::io::open_read(path);
  const auto topics = reader->topics();
  ASSERT_EQ(topics.size(), 2U);

  bool seen_foo = false;
  bool seen_bar = false;
  for (const auto & t : topics) {
    if (t.name == "/foo") {
      seen_foo = true;
      EXPECT_EQ(t.type, "std_msgs/msg/String");
      EXPECT_EQ(t.schema_text, "string data");
      EXPECT_EQ(t.schema_encoding, "ros2msg");
      EXPECT_EQ(t.type_description_hash, "RIHS01_aaa");
    } else if (t.name == "/bar") {
      seen_bar = true;
      EXPECT_EQ(t.type, "std_msgs/msg/Int32");
      EXPECT_EQ(t.schema_text, "int32 data");
      EXPECT_EQ(t.schema_encoding, "ros2msg");
      EXPECT_EQ(t.type_description_hash, "RIHS01_bbb");
    }
  }
  EXPECT_TRUE(seen_foo);
  EXPECT_TRUE(seen_bar);
}

TEST_F(Sqlite3ReaderTest, ReadsLegacyV3WithoutMessageDefinitions)
{
  // Humble (schema_version=3) bags have no type_description_hash column
  // and no message_definitions table. The reader must still load the
  // 5-column form correctly and leave schema_text empty.
  const auto path = write_fixture_db3(tmp_dir_ / "v3");

  auto reader = bagwiz::io::open_read(path);
  const auto topics = reader->topics();
  ASSERT_EQ(topics.size(), 2U);

  for (const auto & t : topics) {
    EXPECT_TRUE(t.schema_text.empty()) << t.name;
    EXPECT_TRUE(t.schema_encoding.empty()) << t.name;
    EXPECT_TRUE(t.type_description_hash.empty()) << t.name;
  }
}

TEST_F(Sqlite3ReaderTest, IteratesMessagesInTimeOrder)
{
  const auto path = write_fixture_db3(tmp_dir_ / "iter");

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

TEST_F(Sqlite3ReaderTest, FilterByTopic)
{
  const auto path = write_fixture_db3(tmp_dir_ / "filter");

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

TEST_F(Sqlite3ReaderTest, FilterByTimeRange)
{
  const auto path = write_fixture_db3(tmp_dir_ / "timefilter");

  auto reader = bagwiz::io::open_read(path);
  bagwiz::io::ReadFilter f;
  // Keep only the /bar messages (timestamps >= 2e9).
  f.start_ns = 2'000'000'000LL;
  reader->set_filter(f);

  int count = 0;
  bagwiz::io::RawMessage msg;
  while (reader->next(msg)) {
    EXPECT_EQ(std::string(msg.topic->name), "/bar");
    EXPECT_GE(msg.timestamp_ns, 2'000'000'000LL);
    ++count;
  }
  EXPECT_EQ(count, 2);
}

TEST_F(Sqlite3ReaderTest, Stats)
{
  const auto path = write_fixture_db3(tmp_dir_ / "stats");

  auto reader = bagwiz::io::open_read(path);
  const auto stats = reader->compute_stats();

  EXPECT_FALSE(stats.from_summary);
  EXPECT_EQ(stats.total_messages, 5);
  EXPECT_EQ(stats.per_topic.at("/foo"), 3);
  EXPECT_EQ(stats.per_topic.at("/bar"), 2);
  EXPECT_EQ(stats.start_ns, 1'000'000'000LL);
  EXPECT_EQ(stats.end_ns, 2'000'000'001LL);
}

TEST_F(Sqlite3ReaderTest, StatsEmptyBag)
{
  // A bag with no message rows: MIN/MAX(timestamp) come back as SQL NULL, so
  // compute_stats() must leave the time extent at 0 (rather than reading a
  // bogus 0 out of a NULL column) and report zero counts.
  const auto path = write_fixture_db3_no_messages(tmp_dir_ / "empty");

  auto reader = bagwiz::io::open_read(path);
  const auto stats = reader->compute_stats();

  EXPECT_FALSE(stats.from_summary);
  EXPECT_EQ(stats.total_messages, 0);
  EXPECT_TRUE(stats.per_topic.empty());
  EXPECT_EQ(stats.start_ns, 0);
  EXPECT_EQ(stats.end_ns, 0);
}

TEST_F(Sqlite3ReaderTest, DirectoryStatsServedFromMetadataWithoutOpeningShards)
{
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

TEST_F(Sqlite3ReaderTest, TopicCounts)
{
  const auto path = write_fixture_db3(tmp_dir_ / "topic_counts");

  auto reader = bagwiz::io::open_read(path);
  const auto counts =
    reader->compute_topic_counts(std::vector<std::string>{"/foo", "/bar", "/unknown"});

  EXPECT_EQ(counts.size(), 2U);
  EXPECT_EQ(counts.at("/foo"), 3);
  EXPECT_EQ(counts.at("/bar"), 2);
}

TEST_F(Sqlite3ReaderTest, TopicCountsEmptyBag)
{
  const auto path = write_fixture_db3_no_messages(tmp_dir_ / "topic_counts_empty");

  auto reader = bagwiz::io::open_read(path);
  const auto counts = reader->compute_topic_counts(std::vector<std::string>{"/foo"});

  EXPECT_TRUE(counts.empty());
}

TEST_F(Sqlite3ReaderTest, DirectoryTopicCountsServedFromMetadataWithoutOpeningShards)
{
  const auto dir = write_fixture_directory_summary_only(tmp_dir_ / "topic_counts_summary_only");

  auto reader = bagwiz::io::open_read(dir);
  const auto counts = reader->compute_topic_counts(std::vector<std::string>{"/foo", "/bar"});

  EXPECT_EQ(counts.size(), 2U);
  EXPECT_EQ(counts.at("/foo"), 3);
  EXPECT_EQ(counts.at("/bar"), 2);
}

TEST_F(Sqlite3ReaderTest, TimeExtent)
{
  const auto path = write_fixture_db3(tmp_dir_ / "time_extent");

  auto reader = bagwiz::io::open_read(path);
  const auto extent = reader->compute_time_extent();

  EXPECT_TRUE(extent.has_data);
  EXPECT_EQ(extent.start_ns, 1'000'000'000LL);
  EXPECT_EQ(extent.end_ns, 2'000'000'001LL);
}

TEST_F(Sqlite3ReaderTest, TimeExtentEmptyBag)
{
  const auto path = write_fixture_db3_no_messages(tmp_dir_ / "time_extent_empty");

  auto reader = bagwiz::io::open_read(path);
  const auto extent = reader->compute_time_extent();

  EXPECT_FALSE(extent.has_data);
  EXPECT_EQ(extent.start_ns, 0);
  EXPECT_EQ(extent.end_ns, 0);
}

TEST_F(Sqlite3ReaderTest, DirectoryTimeExtentServedFromMetadataWithoutOpeningShards)
{
  const auto dir = write_fixture_directory_summary_only(tmp_dir_ / "time_extent_summary_only");

  auto reader = bagwiz::io::open_read(dir);
  const auto extent = reader->compute_time_extent();

  EXPECT_TRUE(extent.has_data);
  EXPECT_EQ(extent.start_ns, 1'000'000'000LL);
  EXPECT_EQ(extent.end_ns, 2'000'000'001LL);
}

TEST_F(Sqlite3ReaderTest, OpensDirectoryWithMetadata)
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
