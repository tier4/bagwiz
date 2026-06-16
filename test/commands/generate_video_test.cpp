// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/generate_video.hpp"

#include "bagwiz/core/video/video_encoder.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace
{
using bagwiz::commands::check_video_source;
using bagwiz::commands::GenerateVideoArgs;
using bagwiz::commands::run_generate_video;
using bagwiz::commands::VideoSourceStatus;

// Little-endian CDR-1 builder, matching the wire format the production reader
// consumes (see raw_image_test.cpp for the alignment rationale).
class CdrBuilder
{
public:
  CdrBuilder()
  {
    for (int b : {0x00, 0x01, 0x00, 0x00}) {
      buf_.push_back(static_cast<std::byte>(b));
    }
  }
  void u8(std::uint8_t v) { buf_.push_back(static_cast<std::byte>(v)); }
  void u32(std::uint32_t v)
  {
    align(4);
    for (int i = 0; i < 4; ++i) {
      buf_.push_back(static_cast<std::byte>((v >> (8 * i)) & 0xFFU));
    }
  }
  void i32(std::int32_t v) { u32(static_cast<std::uint32_t>(v)); }
  void str(const std::string & s)
  {
    u32(static_cast<std::uint32_t>(s.size() + 1));
    for (char c : s) {
      buf_.push_back(static_cast<std::byte>(static_cast<unsigned char>(c)));
    }
    buf_.push_back(std::byte{0});
  }
  void byte_seq(std::span<const std::byte> b)
  {
    u32(static_cast<std::uint32_t>(b.size()));
    for (auto x : b) {
      buf_.push_back(x);
    }
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

// Serialize a sensor_msgs/msg/Image with tightly-packed (step = width*3) pixels.
std::vector<std::byte> make_image_payload(
  std::uint32_t w, std::uint32_t h, const std::string & encoding, std::uint8_t fill)
{
  std::vector<std::byte> data(static_cast<std::size_t>(w) * h * 3, std::byte{fill});
  CdrBuilder b;
  b.i32(0);  // header.stamp.sec
  b.u32(0);  // header.stamp.nanosec
  b.str("cam");
  b.u32(h);
  b.u32(w);
  b.str(encoding);
  b.u8(0);       // is_bigendian
  b.u32(w * 3);  // step
  b.byte_seq({data.data(), data.size()});
  return b.take();
}

bagwiz::io::TopicInfo make_topic(std::string name, std::string type)
{
  bagwiz::io::TopicInfo t;
  t.name = std::move(name);
  t.type = std::move(type);
  t.serialization_format = "cdr";
  return t;
}

bagwiz::io::CreateOptions mcap_dir_opts()
{
  bagwiz::io::CreateOptions opts;
  opts.format = bagwiz::io::Format::Mcap;
  opts.layout = bagwiz::io::Layout::Directory;
  opts.mcap_compression = "none";
  return opts;
}

constexpr const char * kImageTopic = "/cam/image";
constexpr const char * kImageType = "sensor_msgs/msg/Image";

// Build an MCAP bag with an Image topic (`frames` messages at 100 ms spacing,
// WxH, `encoding`) plus a CompressedImage topic and a lidar topic (declared so
// type-based validation can be exercised).
std::filesystem::path build_bag(
  const std::filesystem::path & dir, int frames, std::uint32_t w, std::uint32_t h,
  const std::string & encoding)
{
  const auto path = dir / "input";
  auto writer = bagwiz::io::open_write(path, mcap_dir_opts());
  writer->declare_topic(make_topic(kImageTopic, kImageType));
  writer->declare_topic(make_topic("/cam/image/compressed", "sensor_msgs/msg/CompressedImage"));
  writer->declare_topic(make_topic("/sensing/lidar", "sensor_msgs/msg/PointCloud2"));
  for (int i = 0; i < frames; ++i) {
    const auto payload = make_image_payload(w, h, encoding, static_cast<std::uint8_t>(i * 20));
    const std::int64_t ts = 1'000'000'000LL + static_cast<std::int64_t>(i) * 100'000'000LL;
    writer->write(kImageTopic, ts, {payload.data(), payload.size()});
  }
  writer->close();
  return path;
}

// True if any entry in `dir` looks like a leftover generate-video temp file.
bool any_partial_left(const std::filesystem::path & dir)
{
  for (const auto & entry : std::filesystem::directory_iterator(dir)) {
    if (entry.path().filename().string().find(".bagwiz-partial") != std::string::npos) {
      return true;
    }
  }
  return false;
}

class GenerateVideoTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_generate_video_" +
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

// ---- check_video_source ---------------------------------------------------

TEST_F(GenerateVideoTest, CheckOkForImage)
{
  const auto in = build_bag(tmp_dir_, 2, 16, 16, "bgr8");
  const auto c = check_video_source(in, kImageTopic);
  EXPECT_EQ(c.status, VideoSourceStatus::kOk);
  EXPECT_EQ(c.topic_type, kImageType);
}

TEST_F(GenerateVideoTest, CheckOkForCompressedImage)
{
  const auto in = build_bag(tmp_dir_, 2, 16, 16, "bgr8");
  const auto c = check_video_source(in, "/cam/image/compressed");
  EXPECT_EQ(c.status, VideoSourceStatus::kOk);
  EXPECT_EQ(c.topic_type, "sensor_msgs/msg/CompressedImage");
}

TEST_F(GenerateVideoTest, CheckTopicNotFound)
{
  const auto in = build_bag(tmp_dir_, 2, 16, 16, "bgr8");
  EXPECT_EQ(check_video_source(in, "/nope").status, VideoSourceStatus::kTopicNotFound);
}

TEST_F(GenerateVideoTest, CheckUnsupportedType)
{
  const auto in = build_bag(tmp_dir_, 2, 16, 16, "bgr8");
  EXPECT_EQ(check_video_source(in, "/sensing/lidar").status, VideoSourceStatus::kUnsupportedType);
}

TEST_F(GenerateVideoTest, CheckInputUnopenable)
{
  EXPECT_EQ(
    check_video_source(tmp_dir_ / "does_not_exist", kImageTopic).status,
    VideoSourceStatus::kInputUnopenable);
}

// ---- run_generate_video: failure paths ------------------------------------

TEST_F(GenerateVideoTest, RunMissingTopicFails)
{
  const auto in = build_bag(tmp_dir_, 2, 16, 16, "bgr8");
  const auto out = tmp_dir_ / "out.avi";
  const GenerateVideoArgs args{in, "/nope", out, false};
  EXPECT_EQ(run_generate_video(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

TEST_F(GenerateVideoTest, RunUnsupportedTypeFails)
{
  const auto in = build_bag(tmp_dir_, 2, 16, 16, "bgr8");
  const auto out = tmp_dir_ / "out.avi";
  const GenerateVideoArgs args{in, "/sensing/lidar", out, false};
  EXPECT_EQ(run_generate_video(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

TEST_F(GenerateVideoTest, RunCompressedImageNotYetSupported)
{
  const auto in = build_bag(tmp_dir_, 2, 16, 16, "bgr8");
  const auto out = tmp_dir_ / "out.avi";
  const GenerateVideoArgs args{in, "/cam/image/compressed", out, false};
  EXPECT_EQ(run_generate_video(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

TEST_F(GenerateVideoTest, RunUnsupportedEncodingFailsAndLeavesNothing)
{
  const auto in = build_bag(tmp_dir_, 3, 16, 16, "mono16");
  const auto out = tmp_dir_ / "out.avi";
  const GenerateVideoArgs args{in, kImageTopic, out, false};
  EXPECT_EQ(run_generate_video(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
  EXPECT_FALSE(any_partial_left(tmp_dir_));
}

TEST_F(GenerateVideoTest, RunOddDimensionsFails)
{
  const auto in = build_bag(tmp_dir_, 3, 15, 16, "bgr8");  // odd width
  const auto out = tmp_dir_ / "out.avi";
  const GenerateVideoArgs args{in, kImageTopic, out, false};
  EXPECT_EQ(run_generate_video(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
  EXPECT_FALSE(any_partial_left(tmp_dir_));
}

// ---- run_generate_video: success path -------------------------------------

TEST_F(GenerateVideoTest, RunEncodesImageTopicToVideo)
{
  constexpr int kFrames = 4;
  const auto in = build_bag(tmp_dir_, kFrames, 16, 16, "bgr8");
  const auto out = tmp_dir_ / "out.avi";  // MJPEG: no libx264 dependency

  const GenerateVideoArgs args{in, kImageTopic, out, false};
  ASSERT_EQ(run_generate_video(args), 0);

  ASSERT_TRUE(std::filesystem::exists(out));
  EXPECT_FALSE(any_partial_left(tmp_dir_));  // no temp left behind

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.width, 16U);
  EXPECT_EQ(probe.height, 16U);
  EXPECT_EQ(probe.frame_count, kFrames);
}

TEST_F(GenerateVideoTest, RunExistingOutputWithoutOverwriteFails)
{
  const auto in = build_bag(tmp_dir_, 2, 16, 16, "bgr8");
  const auto out = tmp_dir_ / "out.avi";
  {
    std::ofstream f(out, std::ios::binary);
    f << "SENTINEL";
  }

  const GenerateVideoArgs args{in, kImageTopic, out, false};
  EXPECT_EQ(run_generate_video(args), 1);

  // The pre-existing file is left untouched.
  std::ifstream f(out, std::ios::binary);
  const std::string content((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  EXPECT_EQ(content, "SENTINEL");
}

TEST_F(GenerateVideoTest, RunOverwriteReplacesExistingOutput)
{
  constexpr int kFrames = 4;
  const auto in = build_bag(tmp_dir_, kFrames, 16, 16, "bgr8");
  const auto out = tmp_dir_ / "out.avi";
  {
    std::ofstream f(out, std::ios::binary);
    f << "SENTINEL";
  }

  const GenerateVideoArgs args{in, kImageTopic, out, true};  // --overwrite
  ASSERT_EQ(run_generate_video(args), 0);

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, kFrames);
}

}  // namespace
