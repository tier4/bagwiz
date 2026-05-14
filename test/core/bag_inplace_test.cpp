// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/bag_inplace.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
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
