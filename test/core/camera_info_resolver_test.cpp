// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/camera_info_resolver.hpp"

#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstring>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace
{

using bagwiz::core::camera_info::resolve_camera_info_topic;
using bagwiz::core::camera_info::resolve_camera_info_topic_name;
using bagwiz::io::TopicInfo;

TopicInfo make_topic(std::string name, std::string type)
{
  TopicInfo t;
  t.name = std::move(name);
  t.type = std::move(type);
  t.serialization_format = "cdr";
  return t;
}

TEST(CameraInfoResolverTopicName, ImageRaw)
{
  EXPECT_EQ(resolve_camera_info_topic_name("/cam/image_raw"), "/cam/camera_info");
}

TEST(CameraInfoResolverTopicName, ImageRawCompressed)
{
  EXPECT_EQ(resolve_camera_info_topic_name("/cam/image_raw/compressed"), "/cam/camera_info");
}

TEST(CameraInfoResolverTopicName, ImageRectColor)
{
  EXPECT_EQ(resolve_camera_info_topic_name("/cam/image_rect_color"), "/cam/camera_info");
}

TEST(CameraInfoResolverTopicName, ImageRectColorCompressed)
{
  EXPECT_EQ(resolve_camera_info_topic_name("/cam/image_rect_color/compressed"), "/cam/camera_info");
}

TEST(CameraInfoResolverTopicName, UnknownTopic)
{
  EXPECT_FALSE(resolve_camera_info_topic_name("/cam/image").has_value());
}

TEST(CameraInfoResolver, ResolvesWhenCandidateExistsWithCorrectType)
{
  const std::vector<TopicInfo> topics{
    make_topic("/cam/image_raw", "sensor_msgs/msg/Image"),
    make_topic("/cam/camera_info", "sensor_msgs/msg/CameraInfo")};
  const auto result = resolve_camera_info_topic("/cam/image_raw", topics);
  ASSERT_TRUE(result.topic.has_value());
  EXPECT_EQ(*result.topic, "/cam/camera_info");
  EXPECT_FALSE(result.error.has_value());
}

TEST(CameraInfoResolver, ErrorsWhenCandidateMissing)
{
  const std::vector<TopicInfo> topics{make_topic("/cam/image_raw", "sensor_msgs/msg/Image")};
  const auto result = resolve_camera_info_topic("/cam/image_raw", topics);
  EXPECT_FALSE(result.topic.has_value());
  ASSERT_TRUE(result.error.has_value());
  EXPECT_NE(result.error->find("not found"), std::string::npos);
}

TEST(CameraInfoResolver, ErrorsWhenCandidateHasWrongType)
{
  const std::vector<TopicInfo> topics{
    make_topic("/cam/image_raw", "sensor_msgs/msg/Image"),
    make_topic("/cam/camera_info", "sensor_msgs/msg/Image")};
  const auto result = resolve_camera_info_topic("/cam/image_raw", topics);
  EXPECT_FALSE(result.topic.has_value());
  ASSERT_TRUE(result.error.has_value());
  EXPECT_NE(result.error->find("type"), std::string::npos);
}

class CdrBuilder
{
public:
  CdrBuilder()
  {
    for (int b : {0x00, 0x01, 0x00, 0x00}) {
      buf_.push_back(static_cast<std::byte>(b));
    }
  }
  void u32(std::uint32_t v)
  {
    align(4);
    for (int i = 0; i < 4; ++i) {
      buf_.push_back(static_cast<std::byte>((v >> (8 * i)) & 0xFFU));
    }
  }
  void i32(std::int32_t v) { u32(static_cast<std::uint32_t>(v)); }
  void u8(std::uint8_t v) { buf_.push_back(static_cast<std::byte>(v)); }
  void f64(double v)
  {
    align(8);
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(v));
    std::memcpy(&bits, &v, sizeof(v));
    for (std::size_t i = 0; i < 8; ++i) {
      buf_.push_back(static_cast<std::byte>((bits >> (8 * i)) & 0xFFU));
    }
  }
  void str(const std::string & s)
  {
    u32(static_cast<std::uint32_t>(s.size() + 1));
    for (char c : s) {
      buf_.push_back(static_cast<std::byte>(static_cast<unsigned char>(c)));
    }
    buf_.push_back(std::byte{0});
  }
  [[nodiscard]] std::vector<std::byte> take() const { return buf_; }

private:
  void align(std::size_t n)
  {
    while ((buf_.size() - 4) % n != 0) {
      buf_.push_back(std::byte{0});
    }
  }
  std::vector<std::byte> buf_;
};

std::vector<std::byte> make_camera_info_payload(std::uint32_t w, std::uint32_t h)
{
  const std::array<double, 9> k{
    static_cast<double>(w),
    0.0,
    static_cast<double>(w) / 2.0,
    0.0,
    static_cast<double>(h),
    static_cast<double>(h) / 2.0,
    0.0,
    0.0,
    1.0};
  CdrBuilder b;
  b.i32(0);
  b.u32(0);
  b.str("cam");
  b.u32(h);
  b.u32(w);
  b.str("plumb_bob");
  b.u32(0);
  for (std::size_t i = 0; i < 9; ++i) {
    b.f64(k[i]);
  }
  constexpr std::array<double, 9> identity_r{1, 0, 0, 0, 1, 0, 0, 0, 1};
  for (std::size_t i = 0; i < 9; ++i) {
    b.f64(identity_r[i]);
  }
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 4; ++col) {
      double value = 0.0;
      if (col < 3) {
        value = k[row * 3 + col];
      }
      b.f64(value);
    }
  }
  b.u32(0);  // binning_x
  b.u32(0);  // binning_y
  b.u32(0);  // roi.x_offset
  b.u32(0);  // roi.y_offset
  b.u32(0);  // roi.width
  b.u32(0);  // roi.height
  b.u8(0);   // roi.do_rectify
  return b.take();
}

bagwiz::io::CreateOptions mcap_dir_opts()
{
  bagwiz::io::CreateOptions opts;
  opts.format = bagwiz::io::Format::Mcap;
  opts.layout = bagwiz::io::Layout::Directory;
  opts.mcap_compression = "none";
  return opts;
}

std::filesystem::path make_bag_with_camera_info(
  const std::filesystem::path & dir, const std::string & image_topic,
  const std::string & camera_info_topic)
{
  const auto path = dir / "input";
  auto writer = bagwiz::io::open_write(path, mcap_dir_opts());
  writer->declare_topic(make_topic(image_topic, "sensor_msgs/msg/Image"));
  writer->declare_topic(make_topic(camera_info_topic, "sensor_msgs/msg/CameraInfo"));
  const auto payload = make_camera_info_payload(16, 16);
  writer->write(camera_info_topic, 1'000'000'000LL, {payload.data(), payload.size()});
  writer->close();
  return path;
}

class CameraInfoResolverBagTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() / "bagwiz_camera_info_resolver_test";
    std::filesystem::remove_all(tmp_dir_);
    std::filesystem::create_directories(tmp_dir_);
  }
  void TearDown() override { std::filesystem::remove_all(tmp_dir_); }

  std::filesystem::path tmp_dir_;
};

TEST_F(CameraInfoResolverBagTest, ValidateAcceptsExistingCameraInfo)
{
  const auto bag = make_bag_with_camera_info(tmp_dir_, "/cam/image_raw", "/cam/camera_info");
  EXPECT_FALSE(
    bagwiz::core::camera_info::validate_camera_info_topic(bag, "/cam/camera_info").has_value());
}

TEST_F(CameraInfoResolverBagTest, ValidateRejectsMissingTopic)
{
  const auto bag = make_bag_with_camera_info(tmp_dir_, "/cam/image_raw", "/cam/camera_info");
  const auto err =
    bagwiz::core::camera_info::validate_camera_info_topic(bag, "/missing/camera_info");
  ASSERT_TRUE(err.has_value());
  EXPECT_NE(err->find("not found"), std::string::npos);
}

TEST_F(CameraInfoResolverBagTest, ValidateRejectsWrongType)
{
  const auto bag = make_bag_with_camera_info(tmp_dir_, "/cam/image_raw", "/cam/camera_info");
  const auto err = bagwiz::core::camera_info::validate_camera_info_topic(bag, "/cam/image_raw");
  ASSERT_TRUE(err.has_value());
  EXPECT_NE(err->find("type"), std::string::npos);
}

TEST_F(CameraInfoResolverBagTest, LoadReadsFirstCameraInfoMessage)
{
  const auto bag = make_bag_with_camera_info(tmp_dir_, "/cam/image_raw", "/cam/camera_info");
  const auto result = bagwiz::core::camera_info::load_camera_info(bag, "/cam/camera_info");
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.info->width, 16u);
  EXPECT_EQ(result.info->height, 16u);
}

TEST_F(CameraInfoResolverBagTest, LoadErrorsWhenTopicMissing)
{
  const auto bag = make_bag_with_camera_info(tmp_dir_, "/cam/image_raw", "/cam/camera_info");
  const auto result = bagwiz::core::camera_info::load_camera_info(bag, "/missing/camera_info");
  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.error.find("no messages"), std::string::npos);
}

}  // namespace
