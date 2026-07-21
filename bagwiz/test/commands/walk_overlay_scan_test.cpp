// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_overlay_scan.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include "bagwiz/core/tf/tf_message_wire.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <tf2/buffer_core.hpp>
#include <tf2/time.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace
{

using bagwiz::commands::OverlayScanResult;
using bagwiz::commands::scan_overlay_inputs;

constexpr std::array<std::uint8_t, 4> kDummyPayload{0x01, 0x02, 0x03, 0x04};

bagwiz::io::TopicInfo make_topic(std::string name, std::string type)
{
  bagwiz::io::TopicInfo t;
  t.name = std::move(name);
  t.type = std::move(type);
  t.serialization_format = "cdr";
  return t;
}

std::span<const std::byte> dummy_payload_view()
{
  static_assert(sizeof(std::uint8_t) == sizeof(std::byte));
  return std::span<const std::byte>(
    reinterpret_cast<const std::byte *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
      kDummyPayload.data()),
    kDummyPayload.size());
}

geometry_msgs::msg::TransformStamped make_edge(
  const std::string & parent, const std::string & child)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = parent;
  ts.child_frame_id = child;
  ts.transform.translation.x = 1.0;
  ts.transform.rotation.w = 1.0;
  return ts;
}

// Bag with:
//   /tf_static     tf2_msgs/msg/TFMessage     (1 message: base_link -> lidar)
//   /points_a      sensor_msgs/msg/PointCloud2 (2 messages, dummy payloads —
//                  the scan never parses point-cloud payloads)
//   /points_empty  sensor_msgs/msg/PointCloud2 (declared, 0 messages)
//   /image         sensor_msgs/msg/Image       (1 message)
std::filesystem::path build_bag(const std::filesystem::path & path)
{
  auto writer = bagwiz::io::open_write(path, [] {
    bagwiz::io::CreateOptions opts;
    opts.format = bagwiz::io::Format::Mcap;
    opts.layout = bagwiz::io::Layout::SingleFile;
    opts.mcap_compression = "none";
    return opts;
  }());
  writer->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));
  writer->declare_topic(make_topic("/points_a", "sensor_msgs/msg/PointCloud2"));
  writer->declare_topic(make_topic("/points_empty", "sensor_msgs/msg/PointCloud2"));
  writer->declare_topic(make_topic("/image", "sensor_msgs/msg/Image"));

  const auto edges =
    std::vector<geometry_msgs::msg::TransformStamped>{make_edge("base_link", "lidar")};
  const auto tf_payload = bagwiz::core::serialize_tf_message(edges);
  writer->write("/tf_static", 1'000'000'000LL, tf_payload);
  writer->write("/points_a", 1'100'000'000LL, dummy_payload_view());
  writer->write("/points_a", 1'200'000'000LL, dummy_payload_view());
  writer->write("/image", 1'150'000'000LL, dummy_payload_view());
  writer->close();
  return path;
}

class WalkOverlayScanTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_walk_overlay_scan_" +
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

TEST_F(WalkOverlayScanTest, CollectsTfBufferAndTopicTimestamps)
{
  const auto bag = build_bag(tmp_dir_ / "bag.mcap");
  std::atomic<bool> cancel{false};
  std::vector<double> fractions;
  OverlayScanResult result;

  scan_overlay_inputs(
    bag, {"/points_a"}, cancel, [&](double f) { fractions.push_back(f); }, result);

  EXPECT_TRUE(result.error.empty()) << result.error;
  ASSERT_EQ(result.entries.size(), 1U);
  ASSERT_EQ(result.entries[0].size(), 2U);
  EXPECT_EQ(result.entries[0][0].record_ns, 1'100'000'000LL);
  EXPECT_EQ(result.entries[0][1].record_ns, 1'200'000'000LL);

  const auto tf = result.tf_buffer.lookupTransform("base_link", "lidar", tf2::TimePointZero);
  EXPECT_NEAR(tf.transform.translation.x, 1.0, 1e-9);

  ASSERT_FALSE(fractions.empty());
  EXPECT_DOUBLE_EQ(fractions.back(), 1.0);
}

TEST_F(WalkOverlayScanTest, EmptySelectedTopicFails)
{
  const auto bag = build_bag(tmp_dir_ / "bag.mcap");
  std::atomic<bool> cancel{false};
  OverlayScanResult result;

  scan_overlay_inputs(bag, {"/points_empty"}, cancel, {}, result);

  EXPECT_NE(result.error.find("'/points_empty' has no messages"), std::string::npos)
    << result.error;
}

TEST_F(WalkOverlayScanTest, BagWithoutTfTopicsFails)
{
  const auto path = tmp_dir_ / "no_tf.mcap";
  auto writer = bagwiz::io::open_write(path, [] {
    bagwiz::io::CreateOptions opts;
    opts.format = bagwiz::io::Format::Mcap;
    opts.layout = bagwiz::io::Layout::SingleFile;
    opts.mcap_compression = "none";
    return opts;
  }());
  writer->declare_topic(make_topic("/points_a", "sensor_msgs/msg/PointCloud2"));
  writer->write("/points_a", 1'000'000'000LL, dummy_payload_view());
  writer->close();

  std::atomic<bool> cancel{false};
  OverlayScanResult result;
  scan_overlay_inputs(path, {"/points_a"}, cancel, {}, result);

  EXPECT_NE(result.error.find("TFMessage"), std::string::npos) << result.error;
}

TEST_F(WalkOverlayScanTest, CancelAbortsScanWithoutError)
{
  const auto bag = build_bag(tmp_dir_ / "bag.mcap");
  std::atomic<bool> cancel{true};  // pre-cancelled: the scan aborts at the first row
  OverlayScanResult result;

  scan_overlay_inputs(bag, {"/points_a"}, cancel, {}, result);

  // Cancellation is not an error: the canceller discards the result.
  EXPECT_TRUE(result.error.empty()) << result.error;
}

}  // namespace
