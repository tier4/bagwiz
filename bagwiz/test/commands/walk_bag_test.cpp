// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_bag.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{

using bagwiz::commands::collect_nonempty_pcd_topics;
using bagwiz::commands::open_bag_and_find_topic;
using bagwiz::commands::resolve_walk_camera_info;

constexpr const char * kLogger = "bagwiz.test.walk_bag";
constexpr std::array<std::uint8_t, 4> kPayload{0x01, 0x02, 0x03, 0x04};

bagwiz::io::TopicInfo make_topic(std::string name, std::string type)
{
  bagwiz::io::TopicInfo t;
  t.name = std::move(name);
  t.type = std::move(type);
  t.serialization_format = "cdr";
  return t;
}

std::span<const std::byte> payload_view()
{
  static_assert(sizeof(std::uint8_t) == sizeof(std::byte));
  return std::span<const std::byte>(
    reinterpret_cast<const std::byte *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
      kPayload.data()),
    kPayload.size());
}

bagwiz::io::CreateOptions mcap_dir_opts()
{
  bagwiz::io::CreateOptions opts;
  opts.format = bagwiz::io::Format::Mcap;
  opts.layout = bagwiz::io::Layout::Directory;
  opts.mcap_compression = "none";
  return opts;
}

// Bag with:
//   /image            sensor_msgs/msg/Image       (1 message)
//   /camera/image_raw sensor_msgs/msg/Image       (1 message)
//   /points_a         sensor_msgs/msg/PointCloud2 (1 message)
//   /points_empty     sensor_msgs/msg/PointCloud2 (declared, 0 messages)
std::filesystem::path build_bag(const std::filesystem::path & dir)
{
  const auto path = dir / "bag";
  auto writer = bagwiz::io::open_write(path, mcap_dir_opts());
  writer->declare_topic(make_topic("/image", "sensor_msgs/msg/Image"));
  writer->declare_topic(make_topic("/camera/image_raw", "sensor_msgs/msg/Image"));
  writer->declare_topic(make_topic("/points_a", "sensor_msgs/msg/PointCloud2"));
  writer->declare_topic(make_topic("/points_empty", "sensor_msgs/msg/PointCloud2"));
  writer->write("/image", 1'000'000'000LL, payload_view());
  writer->write("/camera/image_raw", 1'500'000'000LL, payload_view());
  writer->write("/points_a", 2'000'000'000LL, payload_view());
  writer->close();
  return path;
}

// Reader stub whose compute_topic_counts always throws, exercising the
// fallback that lists every PointCloud2 candidate.
class ThrowingCountReader : public bagwiz::io::BagReader
{
public:
  explicit ThrowingCountReader(std::vector<bagwiz::io::TopicInfo> topics)
  : topics_(std::move(topics))
  {
  }

  std::span<const bagwiz::io::TopicInfo> topics() const override { return topics_; }
  void set_filter(const bagwiz::io::ReadFilter &) override {}
  bool next(bagwiz::io::RawMessage &) override { return false; }
  Stats compute_stats() override { return {}; }
  TimeExtent compute_time_extent() override { return {}; }
  std::unordered_map<std::string, std::int64_t> compute_topic_counts(
    std::span<const std::string>) override
  {
    throw std::runtime_error("counts unavailable");
  }

private:
  std::vector<bagwiz::io::TopicInfo> topics_;
};

class WalkBagTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_walk_bag_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
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

TEST_F(WalkBagTest, OpenBagAndFindTopicFindsDeclaredTopic)
{
  const auto bag = build_bag(tmp_dir_);
  auto opened = open_bag_and_find_topic(bag, "/points_a", kLogger);
  ASSERT_TRUE(opened.has_value());
  ASSERT_NE(opened->reader, nullptr);
  ASSERT_NE(opened->topic_info, nullptr);
  EXPECT_EQ(opened->topic_info->name, "/points_a");
  EXPECT_EQ(opened->topic_info->type, "sensor_msgs/msg/PointCloud2");
}

TEST_F(WalkBagTest, OpenBagAndFindTopicMissingTopicFails)
{
  const auto bag = build_bag(tmp_dir_);
  EXPECT_EQ(open_bag_and_find_topic(bag, "/missing", kLogger), std::nullopt);
}

TEST_F(WalkBagTest, OpenBagAndFindTopicMissingBagFails)
{
  EXPECT_EQ(open_bag_and_find_topic(tmp_dir_ / "nope", "/points_a", kLogger), std::nullopt);
}

TEST_F(WalkBagTest, CollectNonemptyPcdTopicsDropsZeroMessageTopics)
{
  const auto bag = build_bag(tmp_dir_);
  auto reader = bagwiz::io::open_read(bag);
  EXPECT_EQ(collect_nonempty_pcd_topics(*reader, kLogger), std::vector<std::string>{"/points_a"});
}

TEST_F(WalkBagTest, CollectNonemptyPcdTopicsWithoutCandidatesYieldsEmpty)
{
  const auto path = tmp_dir_ / "no_pcd";
  auto writer = bagwiz::io::open_write(path, mcap_dir_opts());
  writer->declare_topic(make_topic("/image", "sensor_msgs/msg/Image"));
  writer->write("/image", 1'000'000'000LL, payload_view());
  writer->close();

  auto reader = bagwiz::io::open_read(path);
  EXPECT_TRUE(collect_nonempty_pcd_topics(*reader, kLogger).empty());
}

TEST(WalkCollectNonemptyPcdTopics, CountFailureFallsBackToAllCandidates)
{
  ThrowingCountReader reader(
    {make_topic("/p1", "sensor_msgs/msg/PointCloud2"), make_topic("/img", "sensor_msgs/msg/Image"),
     make_topic("/p2", "sensor_msgs/msg/PointCloud2")});
  EXPECT_EQ(collect_nonempty_pcd_topics(reader, kLogger), (std::vector<std::string>{"/p1", "/p2"}));
}

TEST(WalkCollectNonemptyPcdTopics, NoCandidatesSkipsCounting)
{
  ThrowingCountReader reader({make_topic("/img", "sensor_msgs/msg/Image")});
  EXPECT_TRUE(collect_nonempty_pcd_topics(reader, kLogger).empty());
}

TEST_F(WalkBagTest, ResolveWalkCameraInfoReportsMissingDerivedTopic)
{
  const auto bag = build_bag(tmp_dir_);
  auto reader = bagwiz::io::open_read(bag);
  // /camera/image_raw derives /camera/camera_info, which the bag lacks.
  const auto result =
    resolve_walk_camera_info(bag, "/camera/image_raw", std::nullopt, reader->topics());
  EXPECT_FALSE(result.info.has_value());
  EXPECT_NE(result.error.find("/camera/camera_info"), std::string::npos) << result.error;
}

TEST_F(WalkBagTest, ResolveWalkCameraInfoReportsUnderivableTopic)
{
  const auto bag = build_bag(tmp_dir_);
  auto reader = bagwiz::io::open_read(bag);
  // "/image" matches no known image-topic suffix, so no candidate exists.
  const auto result = resolve_walk_camera_info(bag, "/image", std::nullopt, reader->topics());
  EXPECT_FALSE(result.info.has_value());
  EXPECT_NE(result.error.find("cannot derive"), std::string::npos) << result.error;
}

TEST_F(WalkBagTest, ResolveWalkCameraInfoExplicitTopicMustExist)
{
  const auto bag = build_bag(tmp_dir_);
  auto reader = bagwiz::io::open_read(bag);
  const auto result =
    resolve_walk_camera_info(bag, "/camera/image_raw", std::string("/missing"), reader->topics());
  EXPECT_FALSE(result.info.has_value());
  EXPECT_NE(result.error.find("/missing"), std::string::npos) << result.error;
}

TEST_F(WalkBagTest, ResolveWalkCameraInfoExplicitTopicMustBeCameraInfo)
{
  const auto bag = build_bag(tmp_dir_);
  auto reader = bagwiz::io::open_read(bag);
  const auto result =
    resolve_walk_camera_info(bag, "/camera/image_raw", std::string("/image"), reader->topics());
  EXPECT_FALSE(result.info.has_value());
  EXPECT_NE(result.error.find("camera_info requires"), std::string::npos) << result.error;
}

}  // namespace
