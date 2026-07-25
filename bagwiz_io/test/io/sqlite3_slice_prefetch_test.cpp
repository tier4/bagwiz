// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "io/sqlite3_slice_prefetch.hpp"  // NOLINT(build/include_subdir) src-local header under test

#include <gtest/gtest.h>
#include <sqlite3.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <vector>

// SliceScanner materializes a fixed schedule of half-open timestamp ranges on
// a worker pool, one read-only connection per worker, and hands the slices to
// the consumer strictly in schedule order.
namespace
{

using bagwiz::io::detail::PrefetchedSlice;
using bagwiz::io::detail::SliceRef;
using bagwiz::io::detail::SliceScanner;
using bagwiz::io::detail::SliceScanSpec;

// Payload byte 0 identifies the message, so a test can assert which rows came
// back and in which order.
std::vector<std::byte> payload_for(int id)
{
  return {static_cast<std::byte>(id & 0xFF), std::byte{0xAA}, std::byte{0xBB}};
}

// 10 messages at timestamps 1000, 1010, ... 1090, alternating between topic 1
// and topic 2, plus one row on topic 9 which the topics table does not list.
std::filesystem::path write_fixture(const std::filesystem::path & dir)
{
  std::filesystem::create_directories(dir);
  const auto path = dir / "slices.db3";
  sqlite3 * db = nullptr;
  EXPECT_EQ(sqlite3_open(path.string().c_str(), &db), SQLITE_OK);
  EXPECT_EQ(
    sqlite3_exec(
      db,
      "CREATE TABLE messages("
      "  id INTEGER PRIMARY KEY,"
      "  topic_id INTEGER NOT NULL,"
      "  timestamp INTEGER NOT NULL,"
      "  data BLOB NOT NULL);"
      "CREATE INDEX timestamp_idx ON messages (timestamp ASC);",
      nullptr, nullptr, nullptr),
    SQLITE_OK);

  sqlite3_stmt * stmt = nullptr;
  EXPECT_EQ(
    sqlite3_prepare_v2(
      db, "INSERT INTO messages(topic_id, timestamp, data) VALUES (?, ?, ?);", -1, &stmt, nullptr),
    SQLITE_OK);
  for (int i = 0; i < 10; ++i) {
    const auto blob = payload_for(i);
    sqlite3_bind_int64(stmt, 1, (i % 2 == 0) ? 1 : 2);
    sqlite3_bind_int64(stmt, 2, 1000 + i * 10);
    sqlite3_bind_blob(stmt, 3, blob.data(), static_cast<int>(blob.size()), SQLITE_TRANSIENT);
    EXPECT_EQ(sqlite3_step(stmt), SQLITE_DONE);
    sqlite3_reset(stmt);
  }
  // A row on an unknown topic, at a timestamp inside the covered range.
  const auto orphan = payload_for(99);
  sqlite3_bind_int64(stmt, 1, 9);
  sqlite3_bind_int64(stmt, 2, 1045);
  sqlite3_bind_blob(stmt, 3, orphan.data(), static_cast<int>(orphan.size()), SQLITE_TRANSIENT);
  EXPECT_EQ(sqlite3_step(stmt), SQLITE_DONE);
  sqlite3_finalize(stmt);
  sqlite3_close(db);
  return path;
}

SliceScanSpec spec_for_topics_1_and_2()
{
  SliceScanSpec spec;
  spec.known_topic_ids = {1, 2};
  return spec;
}

// Five slices of 20 ns each covering [1000, 1100), with unbounded outer edges.
std::vector<SliceRef> five_slices()
{
  std::vector<SliceRef> schedule;
  for (int k = 0; k < 5; ++k) {
    SliceRef slice;
    slice.start_ns =
      k == 0 ? std::optional<std::int64_t>() : std::optional<std::int64_t>(1000 + k * 20);
    slice.end_ns =
      k == 4 ? std::optional<std::int64_t>() : std::optional<std::int64_t>(1000 + (k + 1) * 20);
    schedule.push_back(slice);
  }
  return schedule;
}

// Drain every slice in order and flatten to (timestamp, first payload byte).
// A payload-skipped row is tagged -1.
std::vector<std::pair<std::int64_t, int>> drain(SliceScanner & scanner)
{
  std::vector<std::pair<std::int64_t, int>> out;
  for (std::size_t i = 0; i < scanner.size(); ++i) {
    PrefetchedSlice slice = scanner.get(i);
    EXPECT_TRUE(slice.error.empty()) << slice.error;
    for (const auto & rec : slice.records) {
      const int tag = rec.size == 0
                        ? -1
                        : static_cast<int>(std::to_integer<std::uint8_t>(slice.blobs[rec.offset]));
      out.emplace_back(rec.timestamp_ns, tag);
    }
    scanner.recycle(std::move(slice.blobs), std::move(slice.records));
  }
  return out;
}

class Sqlite3SlicePrefetchTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_ = std::filesystem::temp_directory_path() /
           ("bagwiz_db3_slices_" +
            std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()));
    std::filesystem::remove_all(tmp_);
    bag_ = write_fixture(tmp_);
  }
  void TearDown() override { std::filesystem::remove_all(tmp_); }

  std::filesystem::path tmp_;
  std::filesystem::path bag_;
};

}  // namespace

TEST_F(Sqlite3SlicePrefetchTest, EmitsEveryRowOnceInTimestampOrder)
{
  SliceScanner scanner(bag_, spec_for_topics_1_and_2(), five_slices(), 4);
  const auto rows = drain(scanner);
  ASSERT_EQ(rows.size(), 10u);
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(rows[static_cast<std::size_t>(i)].first, 1000 + i * 10);
    EXPECT_EQ(rows[static_cast<std::size_t>(i)].second, i);
  }
}

TEST_F(Sqlite3SlicePrefetchTest, DropsRowsWhoseTopicIsNotKnown)
{
  SliceScanner scanner(bag_, spec_for_topics_1_and_2(), five_slices(), 4);
  const auto rows = drain(scanner);
  for (const auto & [ts, tag] : rows) {
    EXPECT_NE(ts, 1045) << "the orphan row on topic 9 must not be emitted";
    EXPECT_NE(tag, 99);
  }
}

TEST_F(Sqlite3SlicePrefetchTest, OneWorkerProducesTheSameSequenceAsFour)
{
  SliceScanner one(bag_, spec_for_topics_1_and_2(), five_slices(), 1);
  const auto serial = drain(one);
  SliceScanner four(bag_, spec_for_topics_1_and_2(), five_slices(), 4);
  const auto parallel = drain(four);
  EXPECT_EQ(serial, parallel);
}

TEST_F(Sqlite3SlicePrefetchTest, HonoursTheTopicClause)
{
  SliceScanSpec spec = spec_for_topics_1_and_2();
  spec.topic_clause = "topic_id IN (1)";
  SliceScanner scanner(bag_, spec, five_slices(), 4);
  const auto rows = drain(scanner);
  ASSERT_EQ(rows.size(), 5u);  // even-indexed messages only
  for (const auto & [ts, tag] : rows) {
    EXPECT_EQ(tag % 2, 0);
    EXPECT_EQ(ts, 1000 + tag * 10);
  }
}

TEST_F(Sqlite3SlicePrefetchTest, SkipsPayloadForTopicsOutsideTheAllowList)
{
  SliceScanSpec spec = spec_for_topics_1_and_2();
  spec.payload_filter_active = true;
  spec.payload_topic_ids = {1};
  SliceScanner scanner(bag_, spec, five_slices(), 4);
  const auto rows = drain(scanner);
  ASSERT_EQ(rows.size(), 10u);
  for (std::size_t i = 0; i < rows.size(); ++i) {
    // Topic 1 carries the even-indexed messages and keeps its payload; topic
    // 2's rows come back with size 0 (tag -1) and never touch the blob column.
    EXPECT_EQ(rows[i].second, i % 2 == 0 ? static_cast<int>(i) : -1);
  }
}

TEST_F(Sqlite3SlicePrefetchTest, ReportsAnErrorForAnUnreadableDatabase)
{
  SliceScanner scanner(tmp_ / "does_not_exist.db3", spec_for_topics_1_and_2(), five_slices(), 2);
  const PrefetchedSlice slice = scanner.get(0);
  EXPECT_FALSE(slice.error.empty());
}

TEST_F(Sqlite3SlicePrefetchTest, DestroyingMidDrainJoinsCleanly)
{
  SliceScanner scanner(bag_, spec_for_topics_1_and_2(), five_slices(), 4);
  const PrefetchedSlice first = scanner.get(0);
  EXPECT_TRUE(first.error.empty());
  // Falls out of scope with slices 1..4 outstanding: the destructor must
  // cancel and join without hanging.
}
