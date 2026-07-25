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

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

// The parallel db3 read path (BAGWIZ_READ_THREADS > 1) must emit the exact
// same message sequence as the single-connection scan (BAGWIZ_READ_THREADS=0):
// same topics, same timestamps, same payload bytes, same order — including
// timestamp ties within and across topics.
//
// BAGWIZ_DB3_SLICE_BYTES is forced tiny so these small fixtures still span
// many slices; at the production default (32 MiB) they would collapse to the
// serial path and prove nothing.
namespace
{

using Record = std::tuple<std::string, std::int64_t, std::vector<std::byte>>;

std::vector<std::byte> payload_for(int id, std::size_t size)
{
  std::vector<std::byte> out(size);
  for (std::size_t i = 0; i < size; ++i) {
    out[i] = static_cast<std::byte>((id * 31 + static_cast<int>(i)) & 0xFF);
  }
  return out;
}

// A rosbag2-shaped fixture: two topics, 44 messages, a run of shared
// timestamps across topics, and a run of shared timestamps within one topic.
// `with_timestamp_index` mirrors what rosbag2 writes; passing false covers the
// bag shape that must fall back to the serial scan.
std::filesystem::path write_fixture_db3(
  const std::filesystem::path & dir, bool with_timestamp_index = true)
{
  std::filesystem::create_directories(dir);
  const auto path = dir / "bag_0.db3";

  sqlite3 * db = nullptr;
  EXPECT_EQ(sqlite3_open(path.string().c_str(), &db), SQLITE_OK);
  std::string schema =
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
    "  data BLOB NOT NULL);";
  if (with_timestamp_index) {
    schema += "CREATE INDEX timestamp_idx ON messages (timestamp ASC);";
  }
  EXPECT_EQ(sqlite3_exec(db, schema.c_str(), nullptr, nullptr, nullptr), SQLITE_OK);
  EXPECT_EQ(
    sqlite3_exec(
      db,
      "INSERT INTO topics(id, name, type, serialization_format, offered_qos_profiles) VALUES"
      "  (1, '/foo', 'std_msgs/msg/String', 'cdr', ''),"
      "  (2, '/bar', 'std_msgs/msg/Int32', 'cdr', '');",
      nullptr, nullptr, nullptr),
    SQLITE_OK);

  sqlite3_stmt * stmt = nullptr;
  EXPECT_EQ(
    sqlite3_prepare_v2(
      db, "INSERT INTO messages(topic_id, timestamp, data) VALUES (?, ?, ?);", -1, &stmt, nullptr),
    SQLITE_OK);
  const auto insert = [&](std::int64_t topic_id, std::int64_t ts, int id) {
    const auto blob = payload_for(id, 32);
    sqlite3_bind_int64(stmt, 1, topic_id);
    sqlite3_bind_int64(stmt, 2, ts);
    sqlite3_bind_blob(stmt, 3, blob.data(), static_cast<int>(blob.size()), SQLITE_TRANSIENT);
    EXPECT_EQ(sqlite3_step(stmt), SQLITE_DONE);
    sqlite3_reset(stmt);
  };

  int id = 0;
  // 20 interleaved pairs: /foo and /bar share every timestamp (cross-topic tie).
  for (int i = 0; i < 20; ++i) {
    const std::int64_t ts = 1'000'000'000LL + i * 1000;
    insert(1, ts, id++);
    insert(2, ts, id++);
  }
  // 4 messages on /foo that all share one timestamp (within-topic tie).
  for (int i = 0; i < 4; ++i) {
    insert(1, 1'000'000'000LL + 20 * 1000, id++);
  }
  sqlite3_finalize(stmt);
  sqlite3_close(db);
  return path;
}

std::vector<Record> read_all(
  const std::filesystem::path & path, const char * read_threads,
  const std::optional<bagwiz::io::ReadFilter> & filter = std::nullopt,
  std::optional<std::size_t> stop_after = std::nullopt)
{
  ::setenv("BAGWIZ_READ_THREADS", read_threads, 1);
  ::setenv("BAGWIZ_DB3_SLICE_BYTES", "1", 1);  // one slice per few rows
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
  ::unsetenv("BAGWIZ_DB3_SLICE_BYTES");
  return out;
}

class Sqlite3ParallelReadTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_ = std::filesystem::temp_directory_path() /
           ("bagwiz_db3_parallel_" +
            std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()));
    std::filesystem::remove_all(tmp_);
    bag_ = write_fixture_db3(tmp_);
  }
  void TearDown() override { std::filesystem::remove_all(tmp_); }

  std::filesystem::path tmp_;
  std::filesystem::path bag_;
};

}  // namespace

TEST_F(Sqlite3ParallelReadTest, FullReadMatchesSerialPath)
{
  const auto serial = read_all(bag_, "0");
  const auto parallel = read_all(bag_, "4");
  ASSERT_EQ(serial.size(), 44u);  // 20 pairs + 4 same-timestamp extras
  EXPECT_EQ(serial, parallel);
}

TEST_F(Sqlite3ParallelReadTest, TopicFilterMatchesSerialPath)
{
  bagwiz::io::ReadFilter filter;
  filter.topics = {"/bar"};
  const auto serial = read_all(bag_, "0", filter);
  const auto parallel = read_all(bag_, "4", filter);
  ASSERT_EQ(serial.size(), 20u);
  EXPECT_EQ(serial, parallel);
}

TEST_F(Sqlite3ParallelReadTest, TimeRangeMatchesSerialPath)
{
  bagwiz::io::ReadFilter filter;
  filter.start_ns = 1'000'005'000;  // inclusive
  filter.end_ns = 1'000'015'000;    // exclusive
  const auto serial = read_all(bag_, "0", filter);
  const auto parallel = read_all(bag_, "4", filter);
  ASSERT_FALSE(serial.empty());
  EXPECT_EQ(serial, parallel);
}

TEST_F(Sqlite3ParallelReadTest, TopicAndTimeFilterMatchSerialPath)
{
  bagwiz::io::ReadFilter filter;
  filter.topics = {"/foo"};
  filter.start_ns = 1'000'005'000;
  filter.end_ns = 1'000'021'000;
  const auto serial = read_all(bag_, "0", filter);
  const auto parallel = read_all(bag_, "4", filter);
  ASSERT_FALSE(serial.empty());
  EXPECT_EQ(serial, parallel);
}

TEST_F(Sqlite3ParallelReadTest, PayloadSkipMatchesSerialPath)
{
  bagwiz::io::ReadFilter filter;
  filter.payload_topics = {"/foo"};
  const auto serial = read_all(bag_, "0", filter);
  const auto parallel = read_all(bag_, "4", filter);
  ASSERT_EQ(serial.size(), 44u);
  EXPECT_EQ(serial, parallel);
  // /bar's payloads are skipped on both paths.
  for (const auto & [name, ts, payload] : parallel) {
    if (name == "/bar") {
      EXPECT_TRUE(payload.empty());
    } else {
      EXPECT_FALSE(payload.empty());
    }
  }
}

TEST_F(Sqlite3ParallelReadTest, EarlyStopDestroysCleanly)
{
  const auto serial = read_all(bag_, "0", std::nullopt, 3);
  const auto parallel = read_all(bag_, "4", std::nullopt, 3);
  ASSERT_EQ(serial.size(), 3u);
  EXPECT_EQ(serial, parallel);
}

TEST_F(Sqlite3ParallelReadTest, SingleReadThreadFallsBackToSerialPath)
{
  EXPECT_EQ(read_all(bag_, "1"), read_all(bag_, "4"));
}

TEST_F(Sqlite3ParallelReadTest, BagWithoutTimestampIndexFallsBackToSerialPath)
{
  const auto unindexed = write_fixture_db3(tmp_ / "unindexed", false);
  const auto serial = read_all(unindexed, "0");
  const auto parallel = read_all(unindexed, "4");
  ASSERT_EQ(serial.size(), 44u);
  EXPECT_EQ(serial, parallel);
}

TEST_F(Sqlite3ParallelReadTest, EmptyBagReadsAsEmptyOnBothPaths)
{
  const auto dir = tmp_ / "empty";
  std::filesystem::create_directories(dir);
  const auto path = dir / "bag_0.db3";
  sqlite3 * db = nullptr;
  ASSERT_EQ(sqlite3_open(path.string().c_str(), &db), SQLITE_OK);
  ASSERT_EQ(
    sqlite3_exec(
      db,
      "CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL,"
      "  serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL);"
      "CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL,"
      "  timestamp INTEGER NOT NULL, data BLOB NOT NULL);"
      "CREATE INDEX timestamp_idx ON messages (timestamp ASC);",
      nullptr, nullptr, nullptr),
    SQLITE_OK);
  sqlite3_close(db);

  EXPECT_TRUE(read_all(path, "0").empty());
  EXPECT_TRUE(read_all(path, "4").empty());
}

TEST_F(Sqlite3ParallelReadTest, ProductionSliceSizeCollapsesToSerialPath)
{
  // Without the tiny BAGWIZ_DB3_SLICE_BYTES override the fixture is far below
  // one 32 MiB slice, so the reader must stay on the serial scan and still
  // produce the same output.
  ::setenv("BAGWIZ_READ_THREADS", "4", 1);
  ::unsetenv("BAGWIZ_DB3_SLICE_BYTES");
  std::vector<Record> out;
  auto reader = bagwiz::io::open_read(bag_);
  bagwiz::io::RawMessage raw;
  while (reader->next(raw)) {
    out.emplace_back(
      raw.topic->name, raw.timestamp_ns,
      std::vector<std::byte>(raw.payload.begin(), raw.payload.end()));
  }
  ::unsetenv("BAGWIZ_READ_THREADS");
  EXPECT_EQ(out, read_all(bag_, "0"));
}
