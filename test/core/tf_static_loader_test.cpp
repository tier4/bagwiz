// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf_static_loader.hpp"

#include "bagwiz/core/tf_message_wire.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <tf2/buffer_core.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <vector>

namespace
{

geometry_msgs::msg::TransformStamped make_tf(
  const std::string & parent, const std::string & child, double tx)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = parent;
  ts.header.stamp.sec = 10;
  ts.header.stamp.nanosec = 0;
  ts.child_frame_id = child;
  ts.transform.translation.x = tx;
  ts.transform.rotation.w = 1.0;
  return ts;
}

bagwiz::io::CreateOptions mcap_options()
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = "none";
  return options;
}

void write_tf_message(
  bagwiz::io::BagWriter & writer, const std::string & topic, std::int64_t stamp_ns,
  const std::vector<geometry_msgs::msg::TransformStamped> & transforms)
{
  const auto cdr = bagwiz::core::serialize_tf_message(
    std::span<const geometry_msgs::msg::TransformStamped>(transforms.data(), transforms.size()));
  writer.write(topic, stamp_ns, std::span<const std::byte>(cdr.data(), cdr.size()));
}

class TfStaticLoaderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_tf_static_loader_" +
                std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()));
    std::filesystem::remove_all(tmp_dir_);
    std::filesystem::create_directories(tmp_dir_);
  }
  void TearDown() override { std::filesystem::remove_all(tmp_dir_); }

  std::filesystem::path tmp_dir_;
};

TEST_F(TfStaticLoaderTest, NoStaticTfReturnsOkWithEmptyBuffer)
{
  const auto path = tmp_dir_ / "no_static.mcap";
  {
    auto writer = bagwiz::io::open_write(path, mcap_options());
    writer->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf"));
    write_tf_message(*writer, "/tf", 1'000'000'000LL, {make_tf("odom", "base_link", 1.0)});
    writer->close();
  }

  const auto result = bagwiz::core::load_static_tf(path);
  ASSERT_TRUE(result.ok);
  ASSERT_NE(result.buffer, nullptr);
  EXPECT_TRUE(result.buffer->getAllFrameNames().empty());
}

TEST_F(TfStaticLoaderTest, LoadsStaticTfAndCanResolve)
{
  const auto path = tmp_dir_ / "static.mcap";
  {
    auto writer = bagwiz::io::open_write(path, mcap_options());
    writer->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));
    write_tf_message(
      *writer, "/tf_static", 1'000'000'000LL,
      {make_tf("map", "odom", 1.0), make_tf("odom", "base_link", 2.0)});
    writer->close();
  }

  const auto result = bagwiz::core::load_static_tf(path);
  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_NE(result.buffer, nullptr);

  const auto tf = result.buffer->lookupTransform("map", "base_link", tf2::TimePoint{});
  EXPECT_DOUBLE_EQ(tf.transform.translation.x, 3.0);

  const auto names = result.buffer->getAllFrameNames();
  EXPECT_EQ(names.size(), 3U);
}

TEST_F(TfStaticLoaderTest, TruncatedTfMessageReturnsError)
{
  const auto path = tmp_dir_ / "truncated.mcap";
  {
    auto writer = bagwiz::io::open_write(path, mcap_options());
    writer->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));

    constexpr std::array<std::byte, 4> kGarbage{
      std::byte{0x00}, std::byte{0x01}, std::byte{0x02}, std::byte{0x03}};
    writer->write("/tf_static", 1'000'000'000LL, std::span<const std::byte>(kGarbage));
    writer->close();
  }

  const auto result = bagwiz::core::load_static_tf(path);
  EXPECT_FALSE(result.ok);
  EXPECT_FALSE(result.error.empty());
}

}  // namespace
