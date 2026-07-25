// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/bag_io.hpp"
#include "io/env_tuning.hpp"  // NOLINT(build/include_subdir) src-local header under test

#include <gtest/gtest.h>
#include <sqlite3.h>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <span>
#include <string>
#include <vector>

// New .db3 files are written with a 32 KiB page size: measured fastest across
// payload-heavy, small-message, and page-straddling bags, and smaller on disk
// than 64 KiB. BAGWIZ_DB3_PAGE_SIZE overrides it, but SQLite silently ignores
// anything that is not a power of two in [512, 65536], so an invalid value must
// be rejected here rather than applied and forgotten.
namespace
{

constexpr const char * kEnv = "BAGWIZ_DB3_PAGE_SIZE";
constexpr std::uint32_t kDefaultPageSize = 32768;

using bagwiz::io::detail::resolve_db3_page_size;

// The page size SQLite actually recorded in the file header.
std::int64_t page_size_of(const std::filesystem::path & path)
{
  sqlite3 * db = nullptr;
  EXPECT_EQ(sqlite3_open_v2(path.string().c_str(), &db, SQLITE_OPEN_READONLY, nullptr), SQLITE_OK);
  sqlite3_stmt * stmt = nullptr;
  EXPECT_EQ(sqlite3_prepare_v2(db, "PRAGMA page_size;", -1, &stmt, nullptr), SQLITE_OK);
  std::int64_t value = 0;
  if (sqlite3_step(stmt) == SQLITE_ROW) {
    value = sqlite3_column_int64(stmt, 0);
  }
  sqlite3_finalize(stmt);
  sqlite3_close(db);
  return value;
}

bagwiz::io::TopicInfo topic_info(const std::string & name)
{
  bagwiz::io::TopicInfo t;
  t.name = name;
  t.type = "std_msgs/msg/ByteMultiArray";
  t.serialization_format = "cdr";
  return t;
}

// Write a tiny two-message bag through the public factory.
void write_bag(const std::filesystem::path & path)
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Sqlite3;
  options.layout = bagwiz::io::Layout::SingleFile;
  auto writer = bagwiz::io::open_write(path, options);
  writer->declare_topic(topic_info("/foo"));
  const std::vector<std::byte> payload{
    std::byte{0xDE}, std::byte{0xAD}, std::byte{0xBE}, std::byte{0xEF}};
  writer->write("/foo", 1'000'000'000, std::span<const std::byte>(payload.data(), payload.size()));
  writer->write("/foo", 1'000'000'001, std::span<const std::byte>(payload.data(), payload.size()));
  writer->close();
}

class Sqlite3PageSizeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ::unsetenv(kEnv);
    tmp_ = std::filesystem::temp_directory_path() /
           ("bagwiz_db3_pagesize_" +
            std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()));
    std::filesystem::remove_all(tmp_);
    std::filesystem::create_directories(tmp_);
  }
  void TearDown() override
  {
    ::unsetenv(kEnv);
    std::filesystem::remove_all(tmp_);
  }

  std::filesystem::path tmp_;
};

}  // namespace

TEST_F(Sqlite3PageSizeTest, DefaultsTo32KiB)
{
  EXPECT_EQ(resolve_db3_page_size("test"), kDefaultPageSize);
}

TEST_F(Sqlite3PageSizeTest, AcceptsAnyPowerOfTwoSqliteSupports)
{
  for (const auto * value : {"512", "4096", "8192", "16384", "32768", "65536"}) {
    ::setenv(kEnv, value, 1);
    EXPECT_EQ(resolve_db3_page_size("test"), static_cast<std::uint32_t>(std::atoi(value))) << value;
  }
}

TEST_F(Sqlite3PageSizeTest, RejectsValuesSqliteWouldSilentlyIgnore)
{
  // Not a power of two, below the floor, above the ceiling, and unparsable —
  // SQLite would ignore each of these and leave the 4 KiB default in place.
  for (const auto * value : {"5000", "12345", "256", "131072", "banana", "-4096"}) {
    ::setenv(kEnv, value, 1);
    EXPECT_EQ(resolve_db3_page_size("test"), kDefaultPageSize) << value;
  }
}

TEST_F(Sqlite3PageSizeTest, WrittenBagCarriesTheDefaultPageSize)
{
  const auto path = tmp_ / "default.db3";
  write_bag(path);
  EXPECT_EQ(page_size_of(path), static_cast<std::int64_t>(kDefaultPageSize));
}

TEST_F(Sqlite3PageSizeTest, EnvOverrideReachesTheWrittenFile)
{
  ::setenv(kEnv, "8192", 1);
  const auto path = tmp_ / "override.db3";
  write_bag(path);
  EXPECT_EQ(page_size_of(path), 8192);
}

TEST_F(Sqlite3PageSizeTest, InvalidOverrideStillProducesTheDefaultNotSqlitesFallback)
{
  // The regression this guards: a rejected value must fall back to 32 KiB, not
  // silently leave SQLite's own 4 KiB default in the file.
  ::setenv(kEnv, "5000", 1);
  const auto path = tmp_ / "invalid.db3";
  write_bag(path);
  EXPECT_EQ(page_size_of(path), static_cast<std::int64_t>(kDefaultPageSize));
}

TEST_F(Sqlite3PageSizeTest, BagRoundTripsAtEveryPageSize)
{
  for (const auto * value : {"4096", "32768", "65536"}) {
    ::setenv(kEnv, value, 1);
    const auto path = tmp_ / ("rt_" + std::string(value) + ".db3");
    write_bag(path);

    auto reader = bagwiz::io::open_read(path);
    bagwiz::io::RawMessage raw;
    std::vector<std::int64_t> stamps;
    std::vector<std::vector<std::byte>> payloads;
    while (reader->next(raw)) {
      stamps.push_back(raw.timestamp_ns);
      payloads.emplace_back(raw.payload.begin(), raw.payload.end());
    }
    ASSERT_EQ(stamps.size(), 2u) << value;
    EXPECT_EQ(stamps[0], 1'000'000'000) << value;
    EXPECT_EQ(stamps[1], 1'000'000'001) << value;
    EXPECT_EQ(payloads[0].size(), 4u) << value;
    EXPECT_EQ(payloads[0], payloads[1]) << value;
  }
}
