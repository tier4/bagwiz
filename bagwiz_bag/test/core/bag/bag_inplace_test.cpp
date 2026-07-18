// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/bag/bag_inplace.hpp"

#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <span>
#include <stdexcept>
#include <string>
#include <system_error>

namespace
{

class BagInplaceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_bag_inplace_" +
                std::to_string(::testing::UnitTest::GetInstance()->random_seed()) + "_" +
                std::to_string(
                  reinterpret_cast<std::uintptr_t>(
                    this)));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    std::filesystem::create_directories(tmp_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
};

void write_text_file(const std::filesystem::path & path, const std::string & body)
{
  std::ofstream out(path);
  out << body;
}

std::string read_text_file(const std::filesystem::path & path)
{
  std::ifstream in(path);
  std::string body((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  return body;
}

}  // namespace

TEST_F(BagInplaceTest, ReplacesSingleFileLayout)
{
  const auto target = tmp_dir_ / "single.mcap";
  write_text_file(target, "OLD-CONTENT");

  bagwiz::core::write_bag_inplace(
    target, [](const std::filesystem::path & tmp) { write_text_file(tmp, "NEW-CONTENT"); });

  ASSERT_TRUE(std::filesystem::exists(target));
  EXPECT_FALSE(std::filesystem::is_directory(target));
  EXPECT_EQ(read_text_file(target), "NEW-CONTENT");
}

TEST_F(BagInplaceTest, ReplacesDirectoryLayout)
{
  const auto target = tmp_dir_ / "rosbag2_dir";
  std::filesystem::create_directory(target);
  write_text_file(target / "metadata.yaml", "OLD");

  bagwiz::core::write_bag_inplace(target, [](const std::filesystem::path & tmp) {
    std::filesystem::create_directory(tmp);
    write_text_file(tmp / "metadata.yaml", "NEW");
    write_text_file(tmp / "shard.mcap", "shard");
  });

  ASSERT_TRUE(std::filesystem::is_directory(target));
  EXPECT_EQ(read_text_file(target / "metadata.yaml"), "NEW");
  EXPECT_TRUE(std::filesystem::exists(target / "shard.mcap"));
}

TEST_F(BagInplaceTest, CleansUpTmpAndPreservesFinalOnWriterException)
{
  const auto target = tmp_dir_ / "guarded.mcap";
  write_text_file(target, "ORIGINAL");

  EXPECT_THROW(
    bagwiz::core::write_bag_inplace(
      target,
      [](const std::filesystem::path & tmp) {
        write_text_file(tmp, "halfway");
        throw std::runtime_error("writer fails mid-way");
      }),
    std::runtime_error);

  // Original is preserved.
  ASSERT_TRUE(std::filesystem::exists(target));
  EXPECT_EQ(read_text_file(target), "ORIGINAL");

  // No leftover tmp file in the parent directory.
  for (const auto & entry : std::filesystem::directory_iterator(tmp_dir_)) {
    const auto name = entry.path().filename().string();
    EXPECT_EQ(name.find(".bagwiz-inplace-tmp-"), std::string::npos)
      << "leaked tmp path: " << entry.path();
  }
}

TEST_F(BagInplaceTest, ErrorsWhenWriterProducesNothing)
{
  const auto target = tmp_dir_ / "untouched.mcap";
  write_text_file(target, "ORIGINAL");

  EXPECT_THROW(
    bagwiz::core::write_bag_inplace(
      target, [](const std::filesystem::path & /*tmp*/) { /* writes nothing */ }),
    std::runtime_error);

  // Original must remain — we error before deleting it.
  ASSERT_TRUE(std::filesystem::exists(target));
  EXPECT_EQ(read_text_file(target), "ORIGINAL");
}

TEST_F(BagInplaceTest, ErrorsWhenFinalPathMissing)
{
  const auto target = tmp_dir_ / "does_not_exist.mcap";
  EXPECT_THROW(
    bagwiz::core::write_bag_inplace(
      target,
      [](const std::filesystem::path & tmp) { write_text_file(tmp, "should-not-be-reached"); }),
    std::runtime_error);
}

namespace
{

// Synthetic bag payload used for the format-preservation tests below.
// The writer treats it as opaque bytes; we only need a value the readers
// will accept on round-trip.
constexpr std::array<std::uint8_t, 4> kInplacePayload{0x11, 0x22, 0x33, 0x44};

bagwiz::io::TopicInfo make_inplace_topic()
{
  bagwiz::io::TopicInfo t;
  t.name = "/probe";
  t.type = "std_msgs/msg/Int32";
  t.serialization_format = "cdr";
  return t;
}

// Materialise a small bag at `path` with the given format/layout. Used
// both to seed the original bag and (via the in-place writer_fn) to
// produce the replacement.
void write_probe_bag(
  const std::filesystem::path & path, bagwiz::io::Format format, bagwiz::io::Layout layout)
{
  bagwiz::io::CreateOptions opts;
  opts.format = format;
  opts.layout = layout;
  opts.mcap_compression = "none";
  auto writer = bagwiz::io::open_write(path, opts);
  writer->declare_topic(make_inplace_topic());
  writer->write(
    "/probe", 1'000'000'000LL,
    std::span<const std::byte>(
      reinterpret_cast<const std::byte *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        kInplacePayload.data()),
      kInplacePayload.size()));
  writer->close();
}

}  // namespace

// Regression: `bagwiz traj join` (and any future in-place rewrite) used
// to pass Format::Auto / Layout::Auto into the writer factory. The tmp
// path produced by write_bag_inplace carries a ".bagwiz-inplace-tmp-..."
// suffix, which is neither .mcap nor .db3, so the auto-resolver fell
// through to the Directory + Mcap default — silently converting db3
// inputs to mcap on swap. These tests pin the realistic combinations
// callers must use to preserve the input's storage identity.

TEST_F(BagInplaceTest, PinnedSqlite3DirectoryPreservedThroughSwap)
{
  const auto target = tmp_dir_ / "rosbag2_db3_dir";
  write_probe_bag(target, bagwiz::io::Format::Sqlite3, bagwiz::io::Layout::Directory);
  ASSERT_EQ(bagwiz::io::detect_format(target), bagwiz::io::Format::Sqlite3);

  bagwiz::core::write_bag_inplace(target, [](const std::filesystem::path & tmp) {
    write_probe_bag(tmp, bagwiz::io::Format::Sqlite3, bagwiz::io::Layout::Directory);
  });

  ASSERT_TRUE(std::filesystem::is_directory(target));
  EXPECT_EQ(bagwiz::io::detect_format(target), bagwiz::io::Format::Sqlite3);
}

TEST_F(BagInplaceTest, PinnedSqlite3SingleFilePreservedThroughSwap)
{
  const auto target = tmp_dir_ / "probe.db3";
  write_probe_bag(target, bagwiz::io::Format::Sqlite3, bagwiz::io::Layout::SingleFile);
  ASSERT_EQ(bagwiz::io::detect_format(target), bagwiz::io::Format::Sqlite3);

  bagwiz::core::write_bag_inplace(target, [](const std::filesystem::path & tmp) {
    write_probe_bag(tmp, bagwiz::io::Format::Sqlite3, bagwiz::io::Layout::SingleFile);
  });

  ASSERT_TRUE(std::filesystem::exists(target));
  EXPECT_FALSE(std::filesystem::is_directory(target));
  EXPECT_EQ(bagwiz::io::detect_format(target), bagwiz::io::Format::Sqlite3);
}

TEST_F(BagInplaceTest, PinnedMcapDirectoryPreservedThroughSwap)
{
  const auto target = tmp_dir_ / "rosbag2_mcap_dir";
  write_probe_bag(target, bagwiz::io::Format::Mcap, bagwiz::io::Layout::Directory);
  ASSERT_EQ(bagwiz::io::detect_format(target), bagwiz::io::Format::Mcap);

  bagwiz::core::write_bag_inplace(target, [](const std::filesystem::path & tmp) {
    write_probe_bag(tmp, bagwiz::io::Format::Mcap, bagwiz::io::Layout::Directory);
  });

  ASSERT_TRUE(std::filesystem::is_directory(target));
  EXPECT_EQ(bagwiz::io::detect_format(target), bagwiz::io::Format::Mcap);
}
