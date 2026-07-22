// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "map_slam_mapping.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include "bagwiz/core/slam/point_cloud_io.hpp"
#include "bagwiz/core/tf/trajectory.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
using bagwiz::commands::build_mapper_config;
using bagwiz::commands::MapSlamArgs;
using bagwiz::commands::resolve_scan_progress;
using bagwiz::commands::write_map_outputs;
using bagwiz::core::TrajectoryPose;

constexpr const char * kLogger = "bagwiz.test.map_slam_mapping";

// RAII guard for one environment variable: applies the requested state (a
// value to set, or std::nullopt to unset) for the scope and restores the
// previous state on destruction, so a test neither depends on nor leaks the
// ambient environment.
class EnvVarGuard
{
public:
  EnvVarGuard(const char * name, const std::optional<std::string> & value) : name_(name)
  {
    if (const char * previous = ::getenv(name); previous != nullptr) {
      previous_value_ = previous;
    }
    if (value.has_value()) {
      ::setenv(name, value->c_str(), 1);
    } else {
      ::unsetenv(name);
    }
  }

  EnvVarGuard(const EnvVarGuard &) = delete;
  EnvVarGuard & operator=(const EnvVarGuard &) = delete;
  EnvVarGuard(EnvVarGuard &&) = delete;
  EnvVarGuard & operator=(EnvVarGuard &&) = delete;

  ~EnvVarGuard()
  {
    if (previous_value_.has_value()) {
      ::setenv(name_.c_str(), previous_value_->c_str(), 1);
    } else {
      ::unsetenv(name_.c_str());
    }
  }

private:
  std::string name_;
  std::optional<std::string> previous_value_;
};

MapSlamArgs make_args()
{
  MapSlamArgs args;
  args.cloud_topic = "/points";
  args.imu_topic = "/imu";
  args.gnss_topic = "/fix";
  args.input_resolution = 0.25;
  args.range_min = 2.0;
  args.range_max = 80.0;
  args.fill_min_inlier_fraction = 0.5;
  args.submap_max_keyframes = 7;
  args.fill_start = false;
  args.fill_end = true;
  args.num_threads = 1;
  return args;
}

// Reader stub serving a fixed per-topic count map and recording how often
// compute_topic_counts was called.
class CountsReader : public bagwiz::io::BagReader
{
public:
  explicit CountsReader(std::unordered_map<std::string, std::int64_t> counts)
  : counts_(std::move(counts))
  {
  }

  std::span<const bagwiz::io::TopicInfo> topics() const override { return {}; }
  void set_filter(const bagwiz::io::ReadFilter &) override {}
  bool next(bagwiz::io::RawMessage &) override { return false; }
  Stats compute_stats() override { return {}; }
  TimeExtent compute_time_extent() override { return {}; }
  std::unordered_map<std::string, std::int64_t> compute_topic_counts(
    std::span<const std::string>) override
  {
    ++calls;
    return counts_;
  }

  int calls = 0;

private:
  std::unordered_map<std::string, std::int64_t> counts_;
};

// Reader stub whose compute_topic_counts always throws, exercising the
// indeterminate-bar fallback.
class ThrowingCountReader : public bagwiz::io::BagReader
{
public:
  std::span<const bagwiz::io::TopicInfo> topics() const override { return {}; }
  void set_filter(const bagwiz::io::ReadFilter &) override {}
  bool next(bagwiz::io::RawMessage &) override { return false; }
  Stats compute_stats() override { return {}; }
  TimeExtent compute_time_extent() override { return {}; }
  std::unordered_map<std::string, std::int64_t> compute_topic_counts(
    std::span<const std::string>) override
  {
    throw std::runtime_error("counts unavailable");
  }
};

TEST(BuildMapperConfig, CopiesTheArguments)
{
  const auto args = make_args();
  bagwiz::core::slam::SensorTransform extrinsic;
  extrinsic.translation = {1.0, 2.0, 3.0};
  extrinsic.rotation_xyzw = {0.0, 0.0, 0.0, 1.0};
  const auto config = build_mapper_config(args, extrinsic, true, {4.0, 5.0, 6.0});

  EXPECT_DOUBLE_EQ(config.input_resolution, 0.25);
  EXPECT_DOUBLE_EQ(config.range_min, 2.0);
  EXPECT_DOUBLE_EQ(config.range_max, 80.0);
  EXPECT_DOUBLE_EQ(config.fill_min_inlier_fraction, 0.5);
  EXPECT_EQ(config.submap_max_keyframes, 7);
  ASSERT_TRUE(config.t_lidar_imu.has_value());
  EXPECT_DOUBLE_EQ(config.t_lidar_imu->translation[1], 2.0);
  EXPECT_DOUBLE_EQ(config.t_lidar_imu->rotation_xyzw[3], 1.0);
  EXPECT_EQ(config.num_threads, 1);
  EXPECT_TRUE(config.enable_gnss);
  EXPECT_TRUE(config.use_gpu);
  EXPECT_FALSE(config.fill_start);
  EXPECT_TRUE(config.fill_end);
  EXPECT_DOUBLE_EQ(config.gnss_antenna_offset[0], 4.0);
  EXPECT_DOUBLE_EQ(config.gnss_antenna_offset[1], 5.0);
  EXPECT_DOUBLE_EQ(config.gnss_antenna_offset[2], 6.0);
}

TEST(BuildMapperConfig, LidarOnlyWithoutGnss)
{
  auto args = make_args();
  args.imu_topic.clear();
  args.gnss_topic.clear();
  const auto config = build_mapper_config(args, std::nullopt, false, {0.0, 0.0, 0.0});

  EXPECT_FALSE(config.t_lidar_imu.has_value());
  EXPECT_FALSE(config.enable_gnss);
  EXPECT_FALSE(config.use_gpu);
  EXPECT_DOUBLE_EQ(config.gnss_antenna_offset[0], 0.0);
  EXPECT_DOUBLE_EQ(config.gnss_antenna_offset[1], 0.0);
  EXPECT_DOUBLE_EQ(config.gnss_antenna_offset[2], 0.0);
}

TEST(BuildMapperConfig, CapsThreadsAtTheHardwareLimit)
{
  auto args = make_args();
  args.num_threads = std::numeric_limits<int>::max();
  const auto config = build_mapper_config(args, std::nullopt, false, {0.0, 0.0, 0.0});
  const unsigned int hardware = std::thread::hardware_concurrency();
  if (hardware > 0) {
    EXPECT_EQ(config.num_threads, static_cast<int>(hardware));
  }
}

TEST(BuildMapperConfig, ZeroThreadsResolvesToTheHardwareConcurrency)
{
  auto args = make_args();
  args.num_threads = 0;
  const auto config = build_mapper_config(args, std::nullopt, false, {0.0, 0.0, 0.0});
  const unsigned int hardware = std::thread::hardware_concurrency();
  EXPECT_EQ(config.num_threads, hardware > 0 ? static_cast<int>(hardware) : 1);
}

TEST(ResolveScanProgress, DisabledByTheFlagSkipsTheStatsRead)
{
  auto args = make_args();
  args.no_progress = true;
  CountsReader reader({{"/points", 10}});
  const auto setup = resolve_scan_progress(reader, args, true, kLogger);
  EXPECT_FALSE(setup.enabled);
  EXPECT_EQ(setup.total_msgs, 0);
  EXPECT_EQ(reader.calls, 0);
}

TEST(ResolveScanProgress, DisabledByNoColor)
{
  const EnvVarGuard no_color("NO_COLOR", "1");
  auto args = make_args();
  CountsReader reader({{"/points", 10}});
  const auto setup = resolve_scan_progress(reader, args, true, kLogger);
  EXPECT_FALSE(setup.enabled);
  EXPECT_EQ(setup.total_msgs, 0);
  EXPECT_EQ(reader.calls, 0);
}

TEST(ResolveScanProgress, SumsTheStreamedTopicCounts)
{
  const EnvVarGuard no_color("NO_COLOR", std::nullopt);
  const auto args = make_args();
  CountsReader reader({{"/points", 10}, {"/imu", 90}, {"/fix", 5}, {"/other", 1000}});
  const auto setup = resolve_scan_progress(reader, args, true, kLogger);
  EXPECT_TRUE(setup.enabled);
  EXPECT_EQ(setup.total_msgs, 105);  // /other is not streamed by the read loop
}

TEST(ResolveScanProgress, LidarOnlyCountsOnlyTheCloudTopic)
{
  const EnvVarGuard no_color("NO_COLOR", std::nullopt);
  auto args = make_args();
  args.imu_topic.clear();
  args.gnss_topic.clear();
  CountsReader reader({{"/points", 10}, {"/imu", 90}});
  const auto setup = resolve_scan_progress(reader, args, true, kLogger);
  EXPECT_TRUE(setup.enabled);
  EXPECT_EQ(setup.total_msgs, 10);
}

TEST(ResolveScanProgress, StatsFailureFallsBackToAnIndeterminateBar)
{
  const EnvVarGuard no_color("NO_COLOR", std::nullopt);
  const auto args = make_args();
  ThrowingCountReader reader;
  const auto setup = resolve_scan_progress(reader, args, true, kLogger);
  EXPECT_TRUE(setup.enabled);
  EXPECT_EQ(setup.total_msgs, 0);
}

class WriteMapOutputsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_map_slam_mapping_" +
       std::to_string(::testing::UnitTest::GetInstance()->random_seed()) + "_" +
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

TrajectoryPose make_pose(std::int64_t stamp_ns, double tx)
{
  TrajectoryPose pose;
  pose.timestamp_ns = stamp_ns;
  pose.tx = tx;
  pose.qw = 1.0;
  return pose;
}

TEST_F(WriteMapOutputsTest, WritesTrajectoryAndMap)
{
  const auto traj_path = tmp_dir_ / "traj.tum";
  const auto map_path = tmp_dir_ / "map.pcd";
  const std::vector<TrajectoryPose> poses = {make_pose(0, 1.5), make_pose(1'000'000'000, 2.5)};
  const std::vector<std::array<float, 3>> points = {{1.0F, 2.0F, 3.0F}, {4.0F, 5.0F, 6.0F}};
  const std::vector<float> intensities = {10.0F, 20.0F};
  const std::vector<std::array<std::uint8_t, 3>> colors = {{255, 0, 0}, {0, 255, 0}};

  ASSERT_TRUE(write_map_outputs(traj_path, map_path, poses, points, intensities, colors, kLogger));

  std::ifstream traj_in(traj_path);
  const auto read = bagwiz::core::read_tum(traj_in);
  EXPECT_EQ(read.skipped_lines, 0);
  ASSERT_EQ(read.poses.size(), 2U);
  EXPECT_EQ(read.poses[0].timestamp_ns, 0);
  EXPECT_EQ(read.poses[1].timestamp_ns, 1'000'000'000);
  EXPECT_DOUBLE_EQ(read.poses[0].tx, 1.5);
  EXPECT_DOUBLE_EQ(read.poses[1].qw, 1.0);

  std::ifstream pcd_in(map_path, std::ios::binary);
  const auto pcd = bagwiz::core::slam::read_pcd(pcd_in);
  ASSERT_TRUE(pcd.ok) << pcd.error;
  EXPECT_EQ(pcd.cloud.points.size(), 2U);
  EXPECT_EQ(pcd.cloud.intensities.size(), 2U);
  ASSERT_EQ(pcd.cloud.colors.size(), 2U);
  EXPECT_EQ(pcd.cloud.colors[0], (std::array<std::uint8_t, 3>{255, 0, 0}));
  EXPECT_EQ(pcd.cloud.colors[1], (std::array<std::uint8_t, 3>{0, 255, 0}));
}

TEST_F(WriteMapOutputsTest, MapOpenFailureLeavesTheTrajectoryUntouched)
{
  const auto traj_path = tmp_dir_ / "traj.tum";
  const auto map_path = tmp_dir_ / "missing" / "map.pcd";  // parent does not exist
  const std::vector<TrajectoryPose> poses = {make_pose(0, 1.5)};
  const std::vector<std::array<float, 3>> points = {{1.0F, 2.0F, 3.0F}};

  EXPECT_FALSE(write_map_outputs(traj_path, map_path, poses, points, {}, {}, kLogger));
  EXPECT_FALSE(std::filesystem::exists(traj_path));
}

TEST_F(WriteMapOutputsTest, TrajectoryOpenFailureFails)
{
  const auto traj_path = tmp_dir_ / "missing" / "traj.tum";  // parent does not exist
  const auto map_path = tmp_dir_ / "map.pcd";
  const std::vector<TrajectoryPose> poses = {make_pose(0, 1.5)};
  const std::vector<std::array<float, 3>> points = {{1.0F, 2.0F, 3.0F}};

  EXPECT_FALSE(write_map_outputs(traj_path, map_path, poses, points, {}, {}, kLogger));
  EXPECT_FALSE(std::filesystem::exists(traj_path));
}

}  // namespace
