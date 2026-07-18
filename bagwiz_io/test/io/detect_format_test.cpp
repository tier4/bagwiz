// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <span>
#include <string>

namespace
{

constexpr std::array<std::uint8_t, 4> kPayload{0xDE, 0xAD, 0xBE, 0xEF};

void write_mcap(const std::filesystem::path & path, bagwiz::io::Layout layout)
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
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

void write_sqlite3(const std::filesystem::path & path, bagwiz::io::Layout layout)
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Sqlite3;
  options.layout = layout;
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

class DetectFormatTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_detect_test_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
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

TEST_F(DetectFormatTest, McapSingleFileByMagic)
{
  const auto path = tmp_dir_ / "real.mcap";
  write_mcap(path, bagwiz::io::Layout::SingleFile);
  EXPECT_EQ(bagwiz::io::detect_format(path), bagwiz::io::Format::Mcap);
}

TEST_F(DetectFormatTest, Sqlite3SingleFileByMagic)
{
  const auto path = tmp_dir_ / "real.db3";
  write_sqlite3(path, bagwiz::io::Layout::SingleFile);
  EXPECT_EQ(bagwiz::io::detect_format(path), bagwiz::io::Format::Sqlite3);
}

// The whole point of magic-byte detection: extensions must not influence
// the result. Renaming an MCAP file to .db3 still classifies as Mcap, and
// vice versa.
TEST_F(DetectFormatTest, ExtensionIsIgnored)
{
  const auto src_mcap = tmp_dir_ / "src.mcap";
  write_mcap(src_mcap, bagwiz::io::Layout::SingleFile);
  const auto fake_db3 = tmp_dir_ / "fake.db3";
  std::filesystem::rename(src_mcap, fake_db3);
  EXPECT_EQ(bagwiz::io::detect_format(fake_db3), bagwiz::io::Format::Mcap);

  const auto src_db3 = tmp_dir_ / "src.db3";
  write_sqlite3(src_db3, bagwiz::io::Layout::SingleFile);
  const auto fake_mcap = tmp_dir_ / "fake.mcap";
  std::filesystem::rename(src_db3, fake_mcap);
  EXPECT_EQ(bagwiz::io::detect_format(fake_mcap), bagwiz::io::Format::Sqlite3);
}

TEST_F(DetectFormatTest, ExtensionlessFile)
{
  const auto src = tmp_dir_ / "src.mcap";
  write_mcap(src, bagwiz::io::Layout::SingleFile);
  const auto bare = tmp_dir_ / "bare";
  std::filesystem::rename(src, bare);
  EXPECT_EQ(bagwiz::io::detect_format(bare), bagwiz::io::Format::Mcap);
}

TEST_F(DetectFormatTest, McapDirectoryByMetadata)
{
  const auto dir = tmp_dir_ / "mcap_dir";
  write_mcap(dir, bagwiz::io::Layout::Directory);
  EXPECT_EQ(bagwiz::io::detect_format(dir), bagwiz::io::Format::Mcap);
}

TEST_F(DetectFormatTest, Sqlite3DirectoryByMetadata)
{
  const auto dir = tmp_dir_ / "db_dir";
  write_sqlite3(dir, bagwiz::io::Layout::Directory);
  EXPECT_EQ(bagwiz::io::detect_format(dir), bagwiz::io::Format::Sqlite3);
}

TEST_F(DetectFormatTest, MissingPathReturnsAuto)
{
  EXPECT_EQ(bagwiz::io::detect_format(tmp_dir_ / "does_not_exist"), bagwiz::io::Format::Auto);
}

TEST_F(DetectFormatTest, GarbageFileReturnsAuto)
{
  const auto path = tmp_dir_ / "garbage.bin";
  std::ofstream f(path, std::ios::binary);
  const std::array<char, 32> junk{};
  f.write(junk.data(), junk.size());
  f.close();
  EXPECT_EQ(bagwiz::io::detect_format(path), bagwiz::io::Format::Auto);
}

TEST_F(DetectFormatTest, ShortFileReturnsAuto)
{
  // Shorter than either magic prefix; sniff must give up cleanly.
  const auto path = tmp_dir_ / "short.bin";
  std::ofstream f(path, std::ios::binary);
  f.write("AB", 2);
  f.close();
  EXPECT_EQ(bagwiz::io::detect_format(path), bagwiz::io::Format::Auto);
}

TEST_F(DetectFormatTest, EmptyDirectoryReturnsAuto)
{
  const auto dir = tmp_dir_ / "empty_dir";
  std::filesystem::create_directories(dir);
  EXPECT_EQ(bagwiz::io::detect_format(dir), bagwiz::io::Format::Auto);
}

// `infer_format_from_extension` is intended for output paths chosen by the
// user — the file does not need to exist. Only the extension is consulted,
// so renaming is irrelevant here (callers that need magic-byte truth use
// `detect_format` instead).
TEST(InferFormatFromExtension, McapExtension)
{
  EXPECT_EQ(bagwiz::io::infer_format_from_extension("/tmp/out.mcap"), bagwiz::io::Format::Mcap);
}

TEST(InferFormatFromExtension, Sqlite3Extension)
{
  EXPECT_EQ(bagwiz::io::infer_format_from_extension("/tmp/out.db3"), bagwiz::io::Format::Sqlite3);
}

TEST(InferFormatFromExtension, DirectoryStyleReturnsAuto)
{
  EXPECT_EQ(bagwiz::io::infer_format_from_extension("/tmp/out_dir"), bagwiz::io::Format::Auto);
  EXPECT_EQ(bagwiz::io::infer_format_from_extension("/tmp/out_dir/"), bagwiz::io::Format::Auto);
}

TEST(InferFormatFromExtension, UnknownExtensionReturnsAuto)
{
  EXPECT_EQ(bagwiz::io::infer_format_from_extension("/tmp/out.bag"), bagwiz::io::Format::Auto);
  EXPECT_EQ(bagwiz::io::infer_format_from_extension("/tmp/out.txt"), bagwiz::io::Format::Auto);
}

TEST(InferFormatFromExtension, UppercaseExtensionReturnsAuto)
{
  // Case-sensitive on purpose — uppercase extensions are rare enough in
  // rosbag2 tooling that we'd rather force `--storage` than guess.
  EXPECT_EQ(bagwiz::io::infer_format_from_extension("/tmp/out.MCAP"), bagwiz::io::Format::Auto);
}
