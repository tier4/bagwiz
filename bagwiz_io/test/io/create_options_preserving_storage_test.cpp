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
#include <system_error>

namespace
{

constexpr std::array<std::uint8_t, 4> kPayload{0xDE, 0xAD, 0xBE, 0xEF};

void seed_bag(
  const std::filesystem::path & path, bagwiz::io::Format format, bagwiz::io::Layout layout)
{
  bagwiz::io::CreateOptions options;
  options.format = format;
  options.layout = layout;
  options.mcap_compression = "none";
  auto writer = bagwiz::io::open_write(path, options);
  bagwiz::io::TopicInfo t;
  t.name = "/probe";
  t.type = "std_msgs/msg/Int32";
  t.serialization_format = "cdr";
  writer->declare_topic(t);
  writer->write(
    "/probe", 1'000'000'000LL,
    std::span<const std::byte>(
      reinterpret_cast<const std::byte *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        kPayload.data()),
      kPayload.size()));
  writer->close();
}

class CreateOptionsPreservingStorageTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_create_options_preserving_storage_" +
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

// Contract: format and layout are BOTH inherited from the reference.
// Used by in-place rewrites where the output path is a synthetic tmp
// suffix and must reconstruct the input's exact shape.

TEST_F(CreateOptionsPreservingStorageTest, PreservesSqlite3Directory)
{
  const auto reference = tmp_dir_ / "ref_db3_dir";
  seed_bag(reference, bagwiz::io::Format::Sqlite3, bagwiz::io::Layout::Directory);

  const auto opts = bagwiz::io::create_options_preserving_storage(reference);

  EXPECT_EQ(opts.format, bagwiz::io::Format::Sqlite3);
  EXPECT_EQ(opts.layout, bagwiz::io::Layout::Directory);
}

TEST_F(CreateOptionsPreservingStorageTest, PreservesSqlite3SingleFile)
{
  // The critical case the sibling helper (inheriting_format) cannot
  // serve: a single-file reference must stay single-file. Forcing
  // Layout::Directory here would replace `bag.db3` with a directory
  // of the same name on the in-place swap.
  const auto reference = tmp_dir_ / "ref.db3";
  seed_bag(reference, bagwiz::io::Format::Sqlite3, bagwiz::io::Layout::SingleFile);

  const auto opts = bagwiz::io::create_options_preserving_storage(reference);

  EXPECT_EQ(opts.format, bagwiz::io::Format::Sqlite3);
  EXPECT_EQ(opts.layout, bagwiz::io::Layout::SingleFile);
}

TEST_F(CreateOptionsPreservingStorageTest, PreservesMcapDirectory)
{
  const auto reference = tmp_dir_ / "ref_mcap_dir";
  seed_bag(reference, bagwiz::io::Format::Mcap, bagwiz::io::Layout::Directory);

  const auto opts = bagwiz::io::create_options_preserving_storage(reference);

  EXPECT_EQ(opts.format, bagwiz::io::Format::Mcap);
  EXPECT_EQ(opts.layout, bagwiz::io::Layout::Directory);
}

TEST_F(CreateOptionsPreservingStorageTest, PreservesMcapSingleFile)
{
  const auto reference = tmp_dir_ / "ref.mcap";
  seed_bag(reference, bagwiz::io::Format::Mcap, bagwiz::io::Layout::SingleFile);

  const auto opts = bagwiz::io::create_options_preserving_storage(reference);

  EXPECT_EQ(opts.format, bagwiz::io::Format::Mcap);
  EXPECT_EQ(opts.layout, bagwiz::io::Layout::SingleFile);
}

TEST_F(CreateOptionsPreservingStorageTest, ReturnsAutoWhenReferenceDoesNotExist)
{
  // Defensive: must be noexcept and surface a sentinel rather than
  // throwing, so the caller can produce a user-facing error before
  // opening a writer.
  const auto reference = tmp_dir_ / "does_not_exist";

  const auto opts = bagwiz::io::create_options_preserving_storage(reference);

  EXPECT_EQ(opts.format, bagwiz::io::Format::Auto);
  EXPECT_EQ(opts.layout, bagwiz::io::Layout::Auto);
}

TEST_F(CreateOptionsPreservingStorageTest, ReturnsAutoWhenDirectoryHasNoMetadata)
{
  // A bare directory with no metadata.yaml and no shards fails
  // format detection. Helper should surface that uniformly via
  // Format::Auto.
  const auto reference = tmp_dir_ / "empty_dir";
  std::filesystem::create_directory(reference);

  const auto opts = bagwiz::io::create_options_preserving_storage(reference);

  EXPECT_EQ(opts.format, bagwiz::io::Format::Auto);
  EXPECT_EQ(opts.layout, bagwiz::io::Layout::Auto);
}

TEST_F(CreateOptionsPreservingStorageTest, ReturnsAutoForFileCompressedDirectory)
{
  // FILE-mode `.db3.zstd` envelope: bagwiz writers cannot reproduce the
  // compression, so an in-place rewrite must not pin Sqlite3 (which would
  // silently emit a plain `.db3`). The helper returns Auto/Auto so the
  // caller errors out and asks for an explicit `-o`.
  const auto reference = tmp_dir_ / "file_compressed_dir";
  std::filesystem::create_directory(reference);
  {
    std::ofstream md(reference / "metadata.yaml");
    md << "rosbag2_bagfile_information:\n"
       << "  version: 5\n"
       << "  storage_identifier: sqlite3\n"
       << "  compression_format: zstd\n"
       << "  compression_mode: FILE\n"
       << "  relative_file_paths:\n"
       << "    - shard_0.db3.zstd\n";
  }

  const auto opts = bagwiz::io::create_options_preserving_storage(reference);

  EXPECT_EQ(opts.format, bagwiz::io::Format::Auto);
  EXPECT_EQ(opts.layout, bagwiz::io::Layout::Auto);
}

TEST_F(CreateOptionsPreservingStorageTest, ReturnsAutoForSingleFileZstdEnvelope)
{
  // A bare `.db3.zstd` single-file envelope likewise cannot be preserved.
  const auto reference = tmp_dir_ / "bag.db3.zstd";
  {
    std::ofstream f(reference, std::ios::binary);
    const std::array<char, 4> magic{0x28, '\xB5', 0x2F, '\xFD'};
    f.write(magic.data(), magic.size());
  }

  const auto opts = bagwiz::io::create_options_preserving_storage(reference);

  EXPECT_EQ(opts.format, bagwiz::io::Format::Auto);
  EXPECT_EQ(opts.layout, bagwiz::io::Layout::Auto);
}
