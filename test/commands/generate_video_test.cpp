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
#include "core/image/image_fixture.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
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

// Serialize a sensor_msgs/msg/CompressedImage carrying `format` and the given
// compressed bytes.
std::vector<std::byte> make_compressed_payload(
  const std::string & format, std::span<const std::byte> data)
{
  CdrBuilder b;
  b.i32(0);  // header.stamp.sec
  b.u32(0);  // header.stamp.nanosec
  b.str("cam");
  b.str(format);
  b.byte_seq(data);
  return b.take();
}

// Serialize a sensor_msgs/msg/CameraInfo with the given intrinsics. Distortion
// coefficients are empty and the rectification matrix is identity.
std::vector<std::byte> make_camera_info_payload(
  std::uint32_t w, std::uint32_t h, const std::array<double, 9> & k)
{
  CdrBuilder b;
  b.i32(0);            // header.stamp.sec
  b.u32(0);            // header.stamp.nanosec
  b.str("cam");        // header.frame_id
  b.u32(h);            // height
  b.u32(w);            // width
  b.str("plumb_bob");  // distortion_model
  b.u32(0);            // d.length
  for (std::size_t i = 0; i < 9; ++i) {
    b.f64(k[i]);
  }
  // r = identity
  constexpr std::array<double, 9> identity_r{1, 0, 0, 0, 1, 0, 0, 0, 1};
  for (std::size_t i = 0; i < 9; ++i) {
    b.f64(identity_r[i]);
  }
  // p = [K 0]
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
constexpr const char * kCompressedTopic = "/cam/image/compressed";
constexpr const char * kCompressedType = "sensor_msgs/msg/CompressedImage";
constexpr const char * kCameraInfoType = "sensor_msgs/msg/CameraInfo";

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

// Build an MCAP bag whose CompressedImage topic carries `frames` JPEG messages
// (WxH solid colors at 100 ms spacing). The per-frame fill varies so the encoded
// frames differ.
std::filesystem::path build_compressed_bag(
  const std::filesystem::path & dir, int frames, std::uint32_t w, std::uint32_t h)
{
  const auto path = dir / "input";
  auto writer = bagwiz::io::open_write(path, mcap_dir_opts());
  writer->declare_topic(make_topic(kCompressedTopic, kCompressedType));
  for (int i = 0; i < frames; ++i) {
    const auto jpeg =
      bagwiz::test::encode_still_image("jpeg", w, h, static_cast<std::uint8_t>(i * 20), 100, 50);
    const auto payload = make_compressed_payload("jpeg", {jpeg.data(), jpeg.size()});
    const std::int64_t ts = 1'000'000'000LL + static_cast<std::int64_t>(i) * 100'000'000LL;
    writer->write(kCompressedTopic, ts, {payload.data(), payload.size()});
  }
  writer->close();
  return path;
}

// Build an MCAP bag with an image topic named `image_topic` plus a sibling
// `/camera_info` topic. When `compressed` is true the image topic carries JPEG
// frames; otherwise it carries raw bgr8 frames. The CameraInfo is sized to match
// the image dimensions.
std::filesystem::path build_bag_with_camera_info(
  const std::filesystem::path & dir, const std::string & image_topic, bool compressed, int frames,
  std::uint32_t w, std::uint32_t h)
{
  const auto path = dir / "input";
  auto writer = bagwiz::io::open_write(path, mcap_dir_opts());

  const std::string camera_info_topic = [image_topic]() {
    std::string stem = image_topic;
    for (const auto & suffix :
         {"/image_raw/compressed", "/image_rect_color/compressed", "/image_rect_color"}) {
      if (
        stem.size() > std::string_view{suffix}.size() &&
        std::string_view{stem}.substr(stem.size() - std::string_view{suffix}.size()) == suffix) {
        stem.resize(stem.size() - std::string_view{suffix}.size());
        break;
      }
    }
    return stem + "/camera_info";
  }();

  writer->declare_topic(make_topic(image_topic, compressed ? kCompressedType : kImageType));
  writer->declare_topic(make_topic(camera_info_topic, kCameraInfoType));

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
  const auto camera_info_payload = make_camera_info_payload(w, h, k);
  writer->write(
    camera_info_topic, 1'000'000'000LL, {camera_info_payload.data(), camera_info_payload.size()});

  for (int i = 0; i < frames; ++i) {
    const std::int64_t ts = 1'000'000'000LL + static_cast<std::int64_t>(i) * 100'000'000LL;
    if (compressed) {
      const auto jpeg =
        bagwiz::test::encode_still_image("jpeg", w, h, static_cast<std::uint8_t>(i * 20), 100, 50);
      const auto payload = make_compressed_payload("jpeg", {jpeg.data(), jpeg.size()});
      writer->write(image_topic, ts, {payload.data(), payload.size()});
    } else {
      const auto payload = make_image_payload(w, h, "bgr8", static_cast<std::uint8_t>(i * 20));
      writer->write(image_topic, ts, {payload.data(), payload.size()});
    }
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

// A CompressedImage topic carrying a payload that is neither JPEG nor PNG (by
// its magic bytes) stops the run with no output and no leftover temp.
TEST_F(GenerateVideoTest, RunCompressedImageUnrecognizedFormatFails)
{
  const auto path = tmp_dir_ / "input";
  {
    auto writer = bagwiz::io::open_write(path, mcap_dir_opts());
    writer->declare_topic(make_topic(kCompressedTopic, kCompressedType));
    const std::vector<std::byte> garbage(8, std::byte{0x01});
    const auto payload = make_compressed_payload("weird", {garbage.data(), garbage.size()});
    writer->write(kCompressedTopic, 1'000'000'000LL, {payload.data(), payload.size()});
    writer->write(kCompressedTopic, 1'100'000'000LL, {payload.data(), payload.size()});
    writer->close();
  }
  const auto out = tmp_dir_ / "out.avi";
  const GenerateVideoArgs args{path, kCompressedTopic, out, false};
  EXPECT_EQ(run_generate_video(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
  EXPECT_FALSE(any_partial_left(tmp_dir_));
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

// A CompressedImage (JPEG) topic decodes frame-by-frame and encodes to a video
// with the decoded geometry and frame count — the headline new capability.
TEST_F(GenerateVideoTest, RunEncodesCompressedImageTopicToVideo)
{
  constexpr int kFrames = 4;
  const auto in = build_compressed_bag(tmp_dir_, kFrames, 16, 16);
  const auto out = tmp_dir_ / "out.avi";  // MJPEG: no libx264 dependency

  const GenerateVideoArgs args{in, kCompressedTopic, out, false};
  ASSERT_EQ(run_generate_video(args), 0);

  ASSERT_TRUE(std::filesystem::exists(out));
  EXPECT_FALSE(any_partial_left(tmp_dir_));

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

// ---- camera-info auto-resolution ------------------------------------------

TEST_F(GenerateVideoTest, AutoResolvesCameraInfoForImageRawCompressed)
{
  constexpr int kFrames = 2;
  const auto in =
    build_bag_with_camera_info(tmp_dir_, "/cam/image_raw/compressed", true, kFrames, 16, 16);
  const auto out = tmp_dir_ / "out.avi";

  GenerateVideoArgs args{in, "/cam/image_raw/compressed", out, false};
  args.undistort = true;
  ASSERT_EQ(run_generate_video(args), 0);

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, kFrames);
}

TEST_F(GenerateVideoTest, AutoResolvesCameraInfoForImageRectColor)
{
  constexpr int kFrames = 2;
  const auto in =
    build_bag_with_camera_info(tmp_dir_, "/cam/image_rect_color", false, kFrames, 16, 16);
  const auto out = tmp_dir_ / "out.avi";

  GenerateVideoArgs args{in, "/cam/image_rect_color", out, false};
  args.undistort = true;
  ASSERT_EQ(run_generate_video(args), 0);

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, kFrames);
}

TEST_F(GenerateVideoTest, AutoResolvesCameraInfoForImageRectColorCompressed)
{
  constexpr int kFrames = 2;
  const auto in =
    build_bag_with_camera_info(tmp_dir_, "/cam/image_rect_color/compressed", true, kFrames, 16, 16);
  const auto out = tmp_dir_ / "out.avi";

  GenerateVideoArgs args{in, "/cam/image_rect_color/compressed", out, false};
  args.undistort = true;
  ASSERT_EQ(run_generate_video(args), 0);

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, kFrames);
}

TEST_F(GenerateVideoTest, UndistortFailsWhenAutoResolutionCannotFindCameraInfo)
{
  constexpr int kFrames = 2;
  const auto in = build_bag(tmp_dir_, kFrames, 16, 16, "bgr8");  // /cam/image, no /cam/camera_info
  const auto out = tmp_dir_ / "out.avi";

  GenerateVideoArgs args{in, kImageTopic, out, false};
  args.undistort = true;
  EXPECT_EQ(run_generate_video(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

TEST_F(GenerateVideoTest, ExplicitCameraInfoTopicWorksForUndistort)
{
  constexpr int kFrames = 2;
  const auto in = tmp_dir_ / "input";

  // Build a bag with /cam/image and an unrelated /other/camera_info topic so
  // auto-resolution fails and explicit selection is required.
  {
    auto writer = bagwiz::io::open_write(in, mcap_dir_opts());
    writer->declare_topic(make_topic(kImageTopic, kImageType));
    writer->declare_topic(make_topic("/other/camera_info", kCameraInfoType));
    const std::array<double, 9> k{16, 0, 8, 0, 16, 8, 0, 0, 1};
    const auto ci_payload = make_camera_info_payload(16, 16, k);
    writer->write("/other/camera_info", 1'000'000'000LL, {ci_payload.data(), ci_payload.size()});
    for (int i = 0; i < kFrames; ++i) {
      const auto payload = make_image_payload(16, 16, "bgr8", static_cast<std::uint8_t>(i * 20));
      const std::int64_t ts = 1'000'000'000LL + static_cast<std::int64_t>(i) * 100'000'000LL;
      writer->write(kImageTopic, ts, {payload.data(), payload.size()});
    }
    writer->close();
  }

  const auto out = tmp_dir_ / "out.avi";
  GenerateVideoArgs args{in, kImageTopic, out, false};
  args.camera_info_topic = "/other/camera_info";
  args.undistort = true;
  ASSERT_EQ(run_generate_video(args), 0);

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, kFrames);
}

TEST_F(GenerateVideoTest, ExplicitCameraInfoTopicWithWrongTypeFails)
{
  constexpr int kFrames = 2;
  const auto in = build_bag(tmp_dir_, kFrames, 16, 16, "bgr8");
  const auto out = tmp_dir_ / "out.avi";

  GenerateVideoArgs args{in, kImageTopic, out, false};
  args.camera_info_topic = "/sensing/lidar";  // PointCloud2, not CameraInfo
  args.undistort = true;
  EXPECT_EQ(run_generate_video(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

}  // namespace
