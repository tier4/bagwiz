// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "traj_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/tf/tf_message_wire.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

namespace
{

constexpr const char * kLogger = "bagwiz.test.traj_common";

bagwiz::io::CreateOptions mcap_dir_opts()
{
  bagwiz::io::CreateOptions opts;
  opts.format = bagwiz::io::Format::Mcap;
  opts.layout = bagwiz::io::Layout::Directory;
  opts.mcap_compression = "none";
  return opts;
}

// Build an MCAP directory bag declaring one tf2_msgs/msg/TFMessage topic
// (schema embedded), carrying no messages: open_topic_decoder only needs the
// topic listing.
std::filesystem::path build_tf_bag(const std::filesystem::path & dir)
{
  const auto path = dir / "input";
  auto writer = bagwiz::io::open_write(path, mcap_dir_opts());
  writer->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf"));
  writer->close();
  return path;
}

class TrajCommonTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_traj_common_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
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

TEST_F(TrajCommonTest, OpenTopicDecoderOpensDeclaredTopic)
{
  const auto bag = build_tf_bag(tmp_dir_);
  auto reader = bagwiz::io::open_read(bag);
  const auto decoder = bagwiz::commands::open_topic_decoder(*reader, "/tf", kLogger);
  EXPECT_NE(decoder, nullptr);
}

TEST_F(TrajCommonTest, OpenTopicDecoderMissingTopicReturnsNull)
{
  const auto bag = build_tf_bag(tmp_dir_);
  auto reader = bagwiz::io::open_read(bag);
  const auto decoder = bagwiz::commands::open_topic_decoder(*reader, "/nope", kLogger);
  EXPECT_EQ(decoder, nullptr);
}

TEST_F(TrajCommonTest, WriteTumFileWritesExactTumBytes)
{
  const std::vector<bagwiz::core::TrajectoryPose> poses{
    {1'784'470'123'456'789'012LL, 1.5, -2.25, 3.125, 0.0, 0.0, 0.0, 1.0},
    {1'700'000'000'000'000'001LL, 0.0, 0.0, 0.0, 0.5, -0.5, 0.5, 0.5},
  };

  const auto out_path = tmp_dir_ / "traj.tum";
  ASSERT_TRUE(bagwiz::commands::write_tum_file(out_path, poses, kLogger));

  std::ostringstream expected;
  bagwiz::core::write_tum(expected, poses);

  std::ifstream in(out_path, std::ios::binary);
  ASSERT_TRUE(in);
  std::ostringstream actual;
  actual << in.rdbuf();
  EXPECT_EQ(actual.str(), expected.str());
  EXPECT_FALSE(actual.str().empty());
}

TEST_F(TrajCommonTest, WriteTumFileFailsOnUnwritablePath)
{
  const std::vector<bagwiz::core::TrajectoryPose> poses{
    {1LL, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
  };
  const auto out_path = tmp_dir_ / "no" / "such" / "dir" / "traj.tum";
  EXPECT_FALSE(bagwiz::commands::write_tum_file(out_path, poses, kLogger));
  EXPECT_FALSE(std::filesystem::exists(out_path));
}

}  // namespace
