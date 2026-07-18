// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_save.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <span>
#include <string>
#include <system_error>

namespace
{

using bagwiz::commands::resolve_save_path;
using bagwiz::commands::topic_for_filename;
using bagwiz::commands::write_save_file;

class WalkSaveTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_walk_save_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
       "_" +
       std::to_string(
         reinterpret_cast<std::uintptr_t>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
           this)));
    std::filesystem::create_directories(tmp_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
};

TEST(WalkTopicForFilename, SlashesBecomeDoubleUnderscores)
{
  EXPECT_EQ(topic_for_filename("/a/b/c"), "__a__b__c");
  EXPECT_EQ(topic_for_filename("/"), "__");
  EXPECT_EQ(topic_for_filename("plain"), "plain");
}

TEST_F(WalkSaveTest, ResolveSavePathEmptyLineUsesDefault)
{
  EXPECT_EQ(resolve_save_path("", tmp_dir_, "def.yaml"), tmp_dir_ / "def.yaml");
}

TEST_F(WalkSaveTest, ResolveSavePathTrimsTrailingWhitespace)
{
  EXPECT_EQ(resolve_save_path("name.yaml  \t", tmp_dir_, "def.yaml"), "name.yaml");
  // A line of only whitespace trims to empty and falls back to the default.
  EXPECT_EQ(resolve_save_path("  \t ", tmp_dir_, "def.yaml"), tmp_dir_ / "def.yaml");
}

TEST_F(WalkSaveTest, ResolveSavePathExistingDirectoryGetsDefaultInside)
{
  const auto dir = tmp_dir_ / "sub";
  std::filesystem::create_directories(dir);
  EXPECT_EQ(resolve_save_path(dir.string(), tmp_dir_, "def.yaml"), dir / "def.yaml");
  // A trailing separator forces the directory reading even without stat.
  EXPECT_EQ(resolve_save_path(dir.string() + "/", tmp_dir_, "def.yaml"), dir / "def.yaml");
}

TEST_F(WalkSaveTest, ResolveSavePathPlainFileIsUsedAsIs)
{
  const auto file = tmp_dir_ / "out.yaml";
  EXPECT_EQ(resolve_save_path(file.string(), tmp_dir_, "def.yaml"), file);
}

TEST_F(WalkSaveTest, ResolveSavePathTrailingBackslashGetsDefaultInside)
{
  // Backslash is an ordinary filename character on Linux but still triggers
  // the directory reading, mirroring the original prompt logic.
  EXPECT_EQ(
    resolve_save_path("dir\\", tmp_dir_, "def.yaml"), std::filesystem::path("dir\\") / "def.yaml");
}

TEST_F(WalkSaveTest, WriteSaveFileEmptyLineWritesDefault)
{
  const std::string data = "hello bytes";
  const auto result = write_save_file("", tmp_dir_, "def.bin", std::as_bytes(std::span{data}));
  EXPECT_TRUE(result.error.empty()) << result.error;
  EXPECT_EQ(result.path, tmp_dir_ / "def.bin");

  std::ifstream in(result.path, std::ios::binary);
  const std::string written((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  EXPECT_EQ(written, data);
}

TEST_F(WalkSaveTest, WriteSaveFileCreatesParentDirectories)
{
  const std::string data = "nested";
  const auto target = tmp_dir_ / "a" / "b" / "out.bin";
  const auto result =
    write_save_file(target.string(), tmp_dir_, "def.bin", std::as_bytes(std::span{data}));
  EXPECT_TRUE(result.error.empty()) << result.error;
  EXPECT_EQ(result.path, target);
  EXPECT_TRUE(std::filesystem::exists(target));
}

TEST_F(WalkSaveTest, WriteSaveFileIntoExistingDirectory)
{
  const std::string data = "dir target";
  const auto result =
    write_save_file(tmp_dir_.string(), tmp_dir_, "def.bin", std::as_bytes(std::span{data}));
  EXPECT_TRUE(result.error.empty()) << result.error;
  EXPECT_EQ(result.path, tmp_dir_ / "def.bin");
}

TEST_F(WalkSaveTest, WriteSaveFileReportsMkdirFailure)
{
  // A regular file occupying the parent path makes create_directories fail.
  const auto blocker = tmp_dir_ / "blocked";
  {
    std::ofstream touch(blocker);
  }
  const std::string data = "x";
  const auto result = write_save_file(
    (blocker / "out.bin").string(), tmp_dir_, "def.bin", std::as_bytes(std::span{data}));
  EXPECT_NE(result.error.find("could not create directory"), std::string::npos) << result.error;
}

}  // namespace
