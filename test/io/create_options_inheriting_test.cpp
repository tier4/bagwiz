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

class CreateOptionsInheritingTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_create_options_inheriting_" +
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

TEST_F(CreateOptionsInheritingTest, InheritsSqlite3WhenReferenceIsSqlite3DirectoryAndOutputHasNoExt)
{
  const auto reference = tmp_dir_ / "ref_db3_dir";
  seed_bag(reference, bagwiz::io::Format::Sqlite3, bagwiz::io::Layout::Directory);
  const auto output = tmp_dir_ / "out_no_ext";

  const auto opts = bagwiz::io::create_options_inheriting_directory(reference, output);

  EXPECT_EQ(opts.format, bagwiz::io::Format::Sqlite3);
  EXPECT_EQ(opts.layout, bagwiz::io::Layout::Directory);
}

TEST_F(CreateOptionsInheritingTest, InheritsMcapWhenReferenceIsMcapDirectoryAndOutputHasNoExt)
{
  const auto reference = tmp_dir_ / "ref_mcap_dir";
  seed_bag(reference, bagwiz::io::Format::Mcap, bagwiz::io::Layout::Directory);
  const auto output = tmp_dir_ / "out_no_ext";

  const auto opts = bagwiz::io::create_options_inheriting_directory(reference, output);

  EXPECT_EQ(opts.format, bagwiz::io::Format::Mcap);
  EXPECT_EQ(opts.layout, bagwiz::io::Layout::Directory);
}

TEST_F(CreateOptionsInheritingTest, InheritsWhenOutputIsExistingDirectory)
{
  // The "-o is itself a directory" branch reduces to the "no extension"
  // branch here: an existing directory path with no suffix still has an
  // empty extension(), so infer_format_from_extension returns Auto and
  // inheritance kicks in.
  const auto reference = tmp_dir_ / "ref_db3_dir";
  seed_bag(reference, bagwiz::io::Format::Sqlite3, bagwiz::io::Layout::Directory);
  const auto output = tmp_dir_ / "out_existing_dir";
  std::filesystem::create_directory(output);

  const auto opts = bagwiz::io::create_options_inheriting_directory(reference, output);

  EXPECT_EQ(opts.format, bagwiz::io::Format::Sqlite3);
  EXPECT_EQ(opts.layout, bagwiz::io::Layout::Directory);
}

TEST_F(CreateOptionsInheritingTest, DefersToFactoryWhenOutputHasMcapExtension)
{
  // .mcap is an explicit single-file signal from the user — inheritance
  // must not override it even when the reference bag is sqlite3.
  const auto reference = tmp_dir_ / "ref_db3_dir";
  seed_bag(reference, bagwiz::io::Format::Sqlite3, bagwiz::io::Layout::Directory);
  const auto output = tmp_dir_ / "out.mcap";

  const auto opts = bagwiz::io::create_options_inheriting_directory(reference, output);

  EXPECT_EQ(opts.format, bagwiz::io::Format::Auto);
  EXPECT_EQ(opts.layout, bagwiz::io::Layout::Auto);
}

TEST_F(CreateOptionsInheritingTest, DefersToFactoryWhenOutputHasDb3Extension)
{
  // Symmetric to the .mcap case: an mcap reference plus a .db3 output
  // should defer to the user's explicit choice.
  const auto reference = tmp_dir_ / "ref_mcap_dir";
  seed_bag(reference, bagwiz::io::Format::Mcap, bagwiz::io::Layout::Directory);
  const auto output = tmp_dir_ / "out.db3";

  const auto opts = bagwiz::io::create_options_inheriting_directory(reference, output);

  EXPECT_EQ(opts.format, bagwiz::io::Format::Auto);
  EXPECT_EQ(opts.layout, bagwiz::io::Layout::Auto);
}

TEST_F(CreateOptionsInheritingTest, DefersToFactoryWhenReferenceIsSingleFile)
{
  // The contract is "inherit when reference is a *directory* bag".
  // Single-file references intentionally fall through to factory
  // defaults — the user's spec did not request inheritance for that
  // case and silently directory-ising a single-file <to> would be
  // surprising.
  const auto reference = tmp_dir_ / "ref.db3";
  seed_bag(reference, bagwiz::io::Format::Sqlite3, bagwiz::io::Layout::SingleFile);
  const auto output = tmp_dir_ / "out_no_ext";

  const auto opts = bagwiz::io::create_options_inheriting_directory(reference, output);

  EXPECT_EQ(opts.format, bagwiz::io::Format::Auto);
  EXPECT_EQ(opts.layout, bagwiz::io::Layout::Auto);
}

TEST_F(CreateOptionsInheritingTest, DefersToFactoryWhenReferenceDoesNotExist)
{
  // Defensive: the helper must not throw on a missing reference path
  // (it is marked noexcept). It should return the neutral Auto/Auto.
  const auto reference = tmp_dir_ / "does_not_exist";
  const auto output = tmp_dir_ / "out_no_ext";

  const auto opts = bagwiz::io::create_options_inheriting_directory(reference, output);

  EXPECT_EQ(opts.format, bagwiz::io::Format::Auto);
  EXPECT_EQ(opts.layout, bagwiz::io::Layout::Auto);
}
