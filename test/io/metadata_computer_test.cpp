// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/metadata_computer.hpp"

#include "bagwiz/io/metadata_yaml.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace
{

// Magic prefixes recognised by bagwiz::io::detect_format(). Synthetic
// fixtures only need the magic — the MetadataComputer never reads past
// the first 16 bytes. Stored as `char` so ofstream::write can consume
// them directly without a reinterpret_cast.
constexpr std::array<char, 6> kMcapMagic{static_cast<char>(0x89), 'M', 'C', 'A', 'P', '0'};
constexpr const char * kSqliteMagic = "SQLite format 3";

void write_mcap_magic_file(const std::filesystem::path & path)
{
  std::ofstream f(path, std::ios::binary);
  // Surface I/O failures as exceptions so a fixture-side bug fails the
  // test cleanly rather than producing a zero-byte file that later
  // confuses the "magic bytes do not match" assertion.
  f.exceptions(std::ios::failbit | std::ios::badbit);
  f.write(kMcapMagic.data(), static_cast<std::streamsize>(kMcapMagic.size()));
  const std::array<char, 16> pad{};
  f.write(pad.data(), static_cast<std::streamsize>(pad.size()));
}

void write_sqlite_magic_file(const std::filesystem::path & path)
{
  std::ofstream f(path, std::ios::binary);
  f.exceptions(std::ios::failbit | std::ios::badbit);
  const auto len = std::strlen(kSqliteMagic);
  f.write(kSqliteMagic, static_cast<std::streamsize>(len));
  f.put('\0');
  const std::array<char, 16> pad{};
  f.write(pad.data(), static_cast<std::streamsize>(pad.size()));
}

// Exception assertion that doesn't trip cppcoreguidelines-avoid-goto.
// gtest's EXPECT_THROW expands to a try/catch with a goto out of the
// catch block; rebuilding the check by hand keeps the call site clean.
template <typename F>
void expect_throws_runtime_error(F && fn, const char * context)
{
  try {
    std::forward<F>(fn)();
    ADD_FAILURE() << "expected std::runtime_error: " << context;
  } catch (const std::runtime_error &) {
    // Expected — swallow.
  }
}

class MetadataComputerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_metacomp_test_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
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

TEST_F(MetadataComputerTest, ReconstructsMcapDirectory)
{
  const auto dir = tmp_dir_ / "mcap_dir";
  std::filesystem::create_directories(dir);
  write_mcap_magic_file(dir / "bag_0.mcap");

  const auto md = bagwiz::io::MetadataComputer::compute(dir);
  EXPECT_EQ(md.storage_identifier, "mcap");
  ASSERT_EQ(md.relative_file_paths.size(), 1U);
  EXPECT_EQ(md.relative_file_paths[0].filename(), "bag_0.mcap");
  // The reconstructed metadata is intentionally summary-less so the
  // ShardReader fallback computes stats lazily on the first request.
  EXPECT_FALSE(md.has_summary);
  EXPECT_TRUE(md.topics.empty());
}

TEST_F(MetadataComputerTest, ReconstructsSqliteDirectory)
{
  const auto dir = tmp_dir_ / "sqlite_dir";
  std::filesystem::create_directories(dir);
  write_sqlite_magic_file(dir / "bag_0.db3");

  const auto md = bagwiz::io::MetadataComputer::compute(dir);
  EXPECT_EQ(md.storage_identifier, "sqlite3");
  ASSERT_EQ(md.relative_file_paths.size(), 1U);
  EXPECT_EQ(md.relative_file_paths[0].filename(), "bag_0.db3");
}

TEST_F(MetadataComputerTest, OrdersShardsByTrailingIndex)
{
  // Detects the classic lexicographic-sort bug: `_10` must come after
  // `_2`, not between `_1` and `_2`.
  const auto dir = tmp_dir_ / "multi";
  std::filesystem::create_directories(dir);
  write_mcap_magic_file(dir / "bag_10.mcap");
  write_mcap_magic_file(dir / "bag_2.mcap");
  write_mcap_magic_file(dir / "bag_0.mcap");
  write_mcap_magic_file(dir / "bag_1.mcap");

  const auto md = bagwiz::io::MetadataComputer::compute(dir);
  ASSERT_EQ(md.relative_file_paths.size(), 4U);
  EXPECT_EQ(md.relative_file_paths[0].filename(), "bag_0.mcap");
  EXPECT_EQ(md.relative_file_paths[1].filename(), "bag_1.mcap");
  EXPECT_EQ(md.relative_file_paths[2].filename(), "bag_2.mcap");
  EXPECT_EQ(md.relative_file_paths[3].filename(), "bag_10.mcap");
}

TEST_F(MetadataComputerTest, RejectsMixedFormats)
{
  const auto dir = tmp_dir_ / "mixed";
  std::filesystem::create_directories(dir);
  write_mcap_magic_file(dir / "a_0.mcap");
  write_sqlite_magic_file(dir / "b_0.db3");

  expect_throws_runtime_error(
    [&] { (void)bagwiz::io::MetadataComputer::compute(dir); }, "mixed .mcap/.db3");
}

TEST_F(MetadataComputerTest, RejectsEmptyDirectory)
{
  const auto dir = tmp_dir_ / "empty";
  std::filesystem::create_directories(dir);
  expect_throws_runtime_error(
    [&] { (void)bagwiz::io::MetadataComputer::compute(dir); }, "empty directory");
}

TEST_F(MetadataComputerTest, RejectsNonDirectory)
{
  const auto file = tmp_dir_ / "not_a_dir.mcap";
  write_mcap_magic_file(file);
  expect_throws_runtime_error(
    [&] { (void)bagwiz::io::MetadataComputer::compute(file); }, "non-directory path");
}

TEST_F(MetadataComputerTest, RejectsBadMagicBytes)
{
  // .mcap extension but the file actually carries the SQLite magic.
  // The first-shard magic check must catch the lie without scanning
  // beyond the first 16 bytes.
  const auto dir = tmp_dir_ / "bad_magic";
  std::filesystem::create_directories(dir);
  write_sqlite_magic_file(dir / "fake_0.mcap");
  expect_throws_runtime_error(
    [&] { (void)bagwiz::io::MetadataComputer::compute(dir); }, "magic/ext mismatch");
}

TEST_F(MetadataComputerTest, IgnoresNonBagFiles)
{
  const auto dir = tmp_dir_ / "with_extras";
  std::filesystem::create_directories(dir);
  write_mcap_magic_file(dir / "bag_0.mcap");
  std::ofstream(dir / "README.md") << "notes\n";
  std::ofstream(dir / "other.txt") << "stuff\n";

  const auto md = bagwiz::io::MetadataComputer::compute(dir);
  EXPECT_EQ(md.storage_identifier, "mcap");
  ASSERT_EQ(md.relative_file_paths.size(), 1U);
  EXPECT_EQ(md.relative_file_paths[0].filename(), "bag_0.mcap");
}

TEST_F(MetadataComputerTest, OrdersUnindexedShardsAfterIndexedOnes)
{
  // Defensive: an unindexed file (`extra.mcap`) sorts after indexed
  // shards so well-formed multi-shard bags get a deterministic order
  // even if a stray file sneaks in.
  const auto dir = tmp_dir_ / "with_unindexed";
  std::filesystem::create_directories(dir);
  write_mcap_magic_file(dir / "extra.mcap");
  write_mcap_magic_file(dir / "bag_0.mcap");
  write_mcap_magic_file(dir / "bag_1.mcap");

  const auto md = bagwiz::io::MetadataComputer::compute(dir);
  ASSERT_EQ(md.relative_file_paths.size(), 3U);
  EXPECT_EQ(md.relative_file_paths[0].filename(), "bag_0.mcap");
  EXPECT_EQ(md.relative_file_paths[1].filename(), "bag_1.mcap");
  EXPECT_EQ(md.relative_file_paths[2].filename(), "extra.mcap");
}
