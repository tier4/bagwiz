// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/base/output_path.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>

namespace
{

class OutputPathTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_output_path_" +
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

void touch(const std::filesystem::path & p)
{
  std::ofstream out(p);
  out << "x";
}

}  // namespace

TEST_F(OutputPathTest, NonExistentPathIsOk)
{
  const auto target = tmp_dir_ / "no_such_file.tum";
  const auto r = bagwiz::core::prepare_output_path(target, /*overwrite=*/false);
  EXPECT_TRUE(r.ok);
  EXPECT_TRUE(r.error.empty());
  EXPECT_FALSE(std::filesystem::exists(target));
}

TEST_F(OutputPathTest, ExistingFileWithoutOverwriteIsError)
{
  const auto target = tmp_dir_ / "existing.tum";
  touch(target);

  const auto r = bagwiz::core::prepare_output_path(target, /*overwrite=*/false);
  EXPECT_FALSE(r.ok);
  // The error must point the user at -w/--overwrite so the resolution path is
  // discoverable from the message alone.
  EXPECT_NE(r.error.find("-w/--overwrite"), std::string::npos);
  EXPECT_NE(r.error.find(target.string()), std::string::npos);
  // The pre-existing file must be left intact when we refuse.
  EXPECT_TRUE(std::filesystem::exists(target));
}

TEST_F(OutputPathTest, ExistingFileWithOverwriteIsRemoved)
{
  const auto target = tmp_dir_ / "existing.tum";
  touch(target);

  const auto r = bagwiz::core::prepare_output_path(target, /*overwrite=*/true);
  EXPECT_TRUE(r.ok) << r.error;
  EXPECT_FALSE(std::filesystem::exists(target));
}

TEST_F(OutputPathTest, ExistingDirectoryWithOverwriteIsRemovedRecursively)
{
  // Mirrors a rosbag2 directory layout: the output "file" is actually a
  // directory containing shards and metadata.yaml.
  const auto target = tmp_dir_ / "existing_bag_dir";
  std::filesystem::create_directories(target / "nested");
  touch(target / "metadata.yaml");
  touch(target / "nested" / "shard_0.mcap");

  const auto r = bagwiz::core::prepare_output_path(target, /*overwrite=*/true);
  EXPECT_TRUE(r.ok) << r.error;
  EXPECT_FALSE(std::filesystem::exists(target));
}

TEST_F(OutputPathTest, ExistingDirectoryWithoutOverwriteIsError)
{
  const auto target = tmp_dir_ / "existing_bag_dir";
  std::filesystem::create_directories(target);

  const auto r = bagwiz::core::prepare_output_path(target, /*overwrite=*/false);
  EXPECT_FALSE(r.ok);
  EXPECT_TRUE(std::filesystem::exists(target));
}
