// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/base/atomic_write.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace
{

using bagwiz::core::write_file_atomically;

class AtomicWriteTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_atomic_write_" +
                std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()));
    std::filesystem::remove_all(tmp_dir_);
    std::filesystem::create_directories(tmp_dir_);
  }
  void TearDown() override { std::filesystem::remove_all(tmp_dir_); }

  static std::string read_all(const std::filesystem::path & path)
  {
    std::ifstream in(path, std::ios::binary);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
  }

  std::filesystem::path tmp_dir_;
};

TEST_F(AtomicWriteTest, WritesContentsToANewPath)
{
  const auto path = tmp_dir_ / "out.yaml";
  std::string error;

  ASSERT_TRUE(write_file_atomically(path, "hello: world\n", error)) << error;
  EXPECT_TRUE(error.empty());
  EXPECT_EQ(read_all(path), "hello: world\n");
}

TEST_F(AtomicWriteTest, ReplacesAnExistingFile)
{
  const auto path = tmp_dir_ / "out.yaml";
  {
    std::ofstream seed(path);
    seed << "stale\n";
  }
  std::string error;

  ASSERT_TRUE(write_file_atomically(path, "fresh\n", error)) << error;
  EXPECT_EQ(read_all(path), "fresh\n");
}

// The temporary is a sibling of the destination and must not outlive the call.
TEST_F(AtomicWriteTest, LeavesNoTemporaryBehind)
{
  const auto path = tmp_dir_ / "out.yaml";
  std::string error;
  ASSERT_TRUE(write_file_atomically(path, "x\n", error)) << error;

  int entries = 0;
  for (const auto & e : std::filesystem::directory_iterator(tmp_dir_)) {
    (void)e;
    ++entries;
  }
  EXPECT_EQ(entries, 1);
}

// A missing parent directory cannot be opened for writing, so the call must
// report rather than throw, and must not create the destination.
TEST_F(AtomicWriteTest, ReportsAnUnwritableDestination)
{
  const auto path = tmp_dir_ / "no_such_dir" / "out.yaml";
  std::string error;

  EXPECT_FALSE(write_file_atomically(path, "x\n", error));
  EXPECT_FALSE(error.empty());
  EXPECT_FALSE(std::filesystem::exists(path));
}

}  // namespace
