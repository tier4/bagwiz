// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/broken_bag.hpp"

#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <span>
#include <string>
#include <vector>

namespace
{

namespace fs = std::filesystem;

constexpr std::array<std::uint8_t, 4> kPayload{0xDE, 0xAD, 0xBE, 0xEF};

// Write a minimal, valid single-message bag in the requested format/layout.
void write_bag(const fs::path & path, bagwiz::io::Format format, bagwiz::io::Layout layout)
{
  bagwiz::io::CreateOptions options;
  options.format = format;
  options.layout = layout;
  options.mcap_compression = "none";
  auto writer = bagwiz::io::open_write(path, options);
  bagwiz::io::TopicInfo t;
  t.name = "/t";
  t.type = "std_msgs/msg/String";
  t.serialization_format = "cdr";
  writer->declare_topic(t);
  writer->write(
    "/t", 1'000'000'000LL,
    std::span<const std::byte>(
      reinterpret_cast<const std::byte *>(kPayload.data()), kPayload.size()));
  writer->close();
}

void write_mcap_file(const fs::path & path)
{
  write_bag(path, bagwiz::io::Format::Mcap, bagwiz::io::Layout::SingleFile);
}

void write_sqlite3_file(const fs::path & path)
{
  write_bag(path, bagwiz::io::Format::Sqlite3, bagwiz::io::Layout::SingleFile);
}

void write_mcap_dir(const fs::path & path)
{
  write_bag(path, bagwiz::io::Format::Mcap, bagwiz::io::Layout::Directory);
}

void truncate_file(const fs::path & path, std::uintmax_t size)
{
  std::error_code ec;
  fs::resize_file(path, size, ec);
  ASSERT_FALSE(ec) << "resize_file failed: " << ec.message();
}

void overwrite_leading_bytes(const fs::path & path, std::size_t count, char value)
{
  std::fstream f(path, std::ios::binary | std::ios::in | std::ios::out);
  ASSERT_TRUE(f.is_open());
  const std::vector<char> garbage(count, value);
  f.write(garbage.data(), static_cast<std::streamsize>(garbage.size()));
}

// Replace every occurrence of `from` with `to` in a text file.
void replace_in_file(const fs::path & path, const std::string & from, const std::string & to)
{
  std::ifstream in(path, std::ios::binary);
  ASSERT_TRUE(in.is_open());
  std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  in.close();

  std::string::size_type pos = 0;
  bool replaced = false;
  while ((pos = content.find(from, pos)) != std::string::npos) {
    content.replace(pos, from.size(), to);
    pos += to.size();
    replaced = true;
  }
  ASSERT_TRUE(replaced) << "substring not found: " << from;

  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(out.is_open());
  out << content;
}

class BrokenBagTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = fs::temp_directory_path() /
               ("bagwiz_broken_test_" +
                std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()));
    std::error_code ec;
    fs::remove_all(tmp_dir_, ec);
    fs::create_directories(tmp_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    fs::remove_all(tmp_dir_, ec);
  }

  fs::path tmp_dir_;
};

}  // namespace

// --- diagnose_bag: healthy bags -------------------------------------------

TEST_F(BrokenBagTest, HealthyMcapSingleFileIsNotBroken)
{
  const auto path = tmp_dir_ / "good.mcap";
  write_mcap_file(path);
  EXPECT_EQ(bagwiz::core::diagnose_bag(path), std::nullopt);
}

TEST_F(BrokenBagTest, HealthySqlite3SingleFileIsNotBroken)
{
  const auto path = tmp_dir_ / "good.db3";
  write_sqlite3_file(path);
  EXPECT_EQ(bagwiz::core::diagnose_bag(path), std::nullopt);
}

TEST_F(BrokenBagTest, HealthyMcapDirectoryIsNotBroken)
{
  const auto dir = tmp_dir_ / "good_dir";
  write_mcap_dir(dir);
  EXPECT_EQ(bagwiz::core::diagnose_bag(dir), std::nullopt);
}

TEST_F(BrokenBagTest, HealthyMcapPassesDeepScan)
{
  const auto path = tmp_dir_ / "good.mcap";
  write_mcap_file(path);
  EXPECT_EQ(bagwiz::core::diagnose_bag(path, /*deep=*/true), std::nullopt);
}

// --- diagnose_bag: broken bags --------------------------------------------

TEST_F(BrokenBagTest, TruncatedMcapIsBroken)
{
  const auto path = tmp_dir_ / "trunc.mcap";
  write_mcap_file(path);
  // Keep only the leading magic; the footer / summary are gone, so the bag is
  // no longer openable as MCAP.
  truncate_file(path, 8);
  EXPECT_TRUE(bagwiz::core::diagnose_bag(path).has_value());
}

TEST_F(BrokenBagTest, GarbageFileIsBroken)
{
  const auto path = tmp_dir_ / "garbage.mcap";
  std::ofstream f(path, std::ios::binary);
  const std::array<char, 64> junk{};
  f.write(junk.data(), junk.size());
  f.close();
  EXPECT_TRUE(bagwiz::core::diagnose_bag(path).has_value());
}

TEST_F(BrokenBagTest, CorruptSqlite3HeaderIsBroken)
{
  const auto path = tmp_dir_ / "corrupt.db3";
  write_sqlite3_file(path);
  // Clobber the "SQLite format 3\0" magic so the file can no longer be
  // recognized or opened as a database.
  overwrite_leading_bytes(path, 16, '\xFF');
  EXPECT_TRUE(bagwiz::core::diagnose_bag(path).has_value());
}

// The core requirement: a directory bag whose metadata.yaml statistics
// disagree with the actual records is NOT broken. Only an unreadable storage
// container is.
TEST_F(BrokenBagTest, MetadataStatsMismatchIsNotBroken)
{
  const auto dir = tmp_dir_ / "mismatch_dir";
  write_mcap_dir(dir);
  // The bag holds exactly one message; rewrite the recorded counts to a value
  // that disagrees with the shard contents.
  replace_in_file(dir / "metadata.yaml", "message_count: 1", "message_count: 999");
  EXPECT_EQ(bagwiz::core::diagnose_bag(dir), std::nullopt);
}

// --- discover_bags ---------------------------------------------------------

TEST_F(BrokenBagTest, DiscoverSingleFileInput)
{
  const auto path = tmp_dir_ / "solo.mcap";
  write_mcap_file(path);
  const auto units = bagwiz::core::discover_bags(path);
  ASSERT_EQ(units.size(), 1U);
  EXPECT_EQ(units[0].path, path);
  EXPECT_FALSE(units[0].is_directory_bag);
}

TEST_F(BrokenBagTest, DiscoverDirectoryBagInput)
{
  const auto dir = tmp_dir_ / "solo_dir";
  write_mcap_dir(dir);
  const auto units = bagwiz::core::discover_bags(dir);
  ASSERT_EQ(units.size(), 1U);
  EXPECT_EQ(units[0].path, dir);
  EXPECT_TRUE(units[0].is_directory_bag);
}

TEST_F(BrokenBagTest, DiscoverRecursesAndDoesNotDescendIntoDirectoryBags)
{
  const auto file_a = tmp_dir_ / "a.mcap";
  const auto file_b = tmp_dir_ / "sub" / "b.db3";
  const auto dir_bag = tmp_dir_ / "dir_bag";
  fs::create_directories(tmp_dir_ / "sub");
  write_mcap_file(file_a);
  write_sqlite3_file(file_b);
  write_mcap_dir(dir_bag);  // contains metadata.yaml + a .mcap shard

  const auto units = bagwiz::core::discover_bags(tmp_dir_);

  ASSERT_EQ(units.size(), 3U);
  // Sorted by path: a.mcap, dir_bag, sub/b.db3.
  EXPECT_EQ(units[0].path, file_a);
  EXPECT_FALSE(units[0].is_directory_bag);
  EXPECT_EQ(units[1].path, dir_bag);
  EXPECT_TRUE(units[1].is_directory_bag);
  EXPECT_EQ(units[2].path, file_b);
  EXPECT_FALSE(units[2].is_directory_bag);

  // The shard files inside the directory bag must not appear as separate
  // units.
  for (const auto & unit : units) {
    if (unit.path == dir_bag) {
      continue;
    }
    const auto rel = fs::relative(unit.path, dir_bag);
    EXPECT_TRUE(rel.empty() || rel.native().rfind("..", 0) == 0)
      << "shard leaked as a unit: " << unit.path;
  }
}

TEST_F(BrokenBagTest, DiscoverIgnoresNonBagFiles)
{
  std::ofstream(tmp_dir_ / "notes.txt") << "hello";
  std::ofstream(tmp_dir_ / "archive.zip") << "PK";
  const auto units = bagwiz::core::discover_bags(tmp_dir_);
  EXPECT_TRUE(units.empty());
}

TEST_F(BrokenBagTest, DiscoverMissingPathIsEmpty)
{
  const auto units = bagwiz::core::discover_bags(tmp_dir_ / "does_not_exist");
  EXPECT_TRUE(units.empty());
}

// --- delete_bag ------------------------------------------------------------

TEST_F(BrokenBagTest, DeleteFileUnitRemovesFile)
{
  const auto path = tmp_dir_ / "doomed.mcap";
  write_mcap_file(path);
  const auto ec = bagwiz::core::delete_bag({path, /*is_directory_bag=*/false});
  EXPECT_FALSE(ec) << ec.message();
  EXPECT_FALSE(fs::exists(path));
}

TEST_F(BrokenBagTest, DeleteDirectoryUnitRemovesTree)
{
  const auto dir = tmp_dir_ / "doomed_dir";
  write_mcap_dir(dir);
  const auto ec = bagwiz::core::delete_bag({dir, /*is_directory_bag=*/true});
  EXPECT_FALSE(ec) << ec.message();
  EXPECT_FALSE(fs::exists(dir));
}
