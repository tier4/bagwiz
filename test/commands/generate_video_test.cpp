// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/generate_video.hpp"

#include "bagwiz/core/camera/camera_info.hpp"
#include "bagwiz/core/tf_message_wire.hpp"
#include "bagwiz/core/video/video_encoder.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "core/image/image_fixture.hpp"

#include <gtest/gtest.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

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
  void bool_(bool v) { u8(v ? 1U : 0U); }
  void u32(std::uint32_t v)
  {
    align(4);
    for (int i = 0; i < 4; ++i) {
      buf_.push_back(static_cast<std::byte>((v >> (8 * i)) & 0xFFU));
    }
  }
  void i32(std::int32_t v) { u32(static_cast<std::uint32_t>(v)); }
  void f32(float v)
  {
    align(4);
    std::array<std::byte, 4> bytes{};
    std::memcpy(bytes.data(), &v, sizeof(v));
    for (auto b : bytes) {
      buf_.push_back(b);
    }
  }
  void f64(double v)
  {
    align(8);
    std::array<std::byte, 8> bytes{};
    std::memcpy(bytes.data(), &v, sizeof(v));
    for (auto b : bytes) {
      buf_.push_back(b);
    }
  }
  void fixed_f64_array(std::span<const double> values)
  {
    for (double v : values) {
      f64(v);
    }
  }
  void f64_seq(std::span<const double> values)
  {
    u32(static_cast<std::uint32_t>(values.size()));
    for (double v : values) {
      f64(v);
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

// Serialize a sensor_msgs/msg/CameraInfo. K/R/P are the standard fixed arrays;
// D is empty for simplicity.
std::vector<std::byte> make_camera_info_payload(
  std::uint32_t width, std::uint32_t height, const std::array<double, 9> & K,
  const std::string & frame_id)
{
  const std::array<double, 9> R{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  const std::array<double, 12> P{
    K[0], 0.0, K[2], 0.0, 0.0, K[4], K[5], 0.0, 0.0, 0.0, 1.0, 0.0};
  CdrBuilder b;
  b.i32(0);                // header.stamp.sec
  b.u32(0);                // header.stamp.nanosec
  b.str(frame_id);         // header.frame_id
  b.u32(height);           // height
  b.u32(width);            // width
  b.str("plumb_bob");      // distortion_model
  b.fixed_f64_array({K.data(), K.size()});
  b.f64_seq({});           // D
  b.fixed_f64_array({R.data(), R.size()});
  b.fixed_f64_array({P.data(), P.size()});
  b.u32(0);                // binning_x
  b.u32(0);                // binning_y
  b.u32(0);                // roi.x_offset
  b.u32(0);                // roi.y_offset
  b.u32(0);                // roi.height
  b.u32(0);                // roi.width
  b.bool_(false);          // roi.do_rectify
  return b.take();
}

// Serialize a sensor_msgs/msg/PointCloud2 carrying `points` as FLOAT32 x/y/z
// fields. The cloud has frame_id `frame_id` and a single row.
std::vector<std::byte> make_point_cloud_payload(
  const std::string & frame_id, const std::vector<std::array<float, 3>> & points)
{
  constexpr std::uint32_t kPointStep = 12U;
  CdrBuilder b;
  b.i32(0);                         // header.stamp.sec
  b.u32(0);                         // header.stamp.nanosec
  b.str(frame_id);                  // header.frame_id
  b.u32(1);                         // height
  b.u32(static_cast<std::uint32_t>(points.size()));  // width
  b.u32(3);                         // field count
  b.str("x");
  b.u32(0);
  b.u8(7);  // FLOAT32
  b.u32(1);
  b.str("y");
  b.u32(4);
  b.u8(7);
  b.u32(1);
  b.str("z");
  b.u32(8);
  b.u8(7);
  b.u32(1);
  b.u8(0);                          // is_bigendian
  b.u32(kPointStep);                // point_step
  b.u32(kPointStep * static_cast<std::uint32_t>(points.size()));  // row_step
  std::vector<std::byte> data;
  data.reserve(points.size() * kPointStep);
  for (const auto & p : points) {
    std::array<std::byte, 4> bytes{};
    std::memcpy(bytes.data(), &p[0], sizeof(float));
    data.insert(data.end(), bytes.begin(), bytes.end());
    std::memcpy(bytes.data(), &p[1], sizeof(float));
    data.insert(data.end(), bytes.begin(), bytes.end());
    std::memcpy(bytes.data(), &p[2], sizeof(float));
    data.insert(data.end(), bytes.begin(), bytes.end());
  }
  b.byte_seq({data.data(), data.size()});
  b.bool_(true);  // is_dense
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

// Build an MCAP bag with an Image topic (`frames` messages at 100 ms spacing,
// WxH, `encoding`) plus a CompressedImage topic, a lidar topic, and a
// CameraInfo topic (declared so overlay validation can be exercised).
std::filesystem::path build_bag(
  const std::filesystem::path & dir, int frames, std::uint32_t w, std::uint32_t h,
  const std::string & encoding)
{
  const auto path = dir / "input";
  auto writer = bagwiz::io::open_write(path, mcap_dir_opts());
  writer->declare_topic(make_topic(kImageTopic, kImageType));
  writer->declare_topic(make_topic("/cam/image/compressed", "sensor_msgs/msg/CompressedImage"));
  writer->declare_topic(make_topic("/sensing/lidar", "sensor_msgs/msg/PointCloud2"));
  writer->declare_topic(make_topic("/cam/camera_info", "sensor_msgs/msg/CameraInfo"));
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

// Build an MCAP bag with an image topic, a CameraInfo topic, a PointCloud2
// topic, and a /tf_static topic. The camera is aligned with the image so that
// a lidar point at (0,0,z) projects to the image center. The static TF relates
// the lidar frame to the camera frame with an identity transform.
std::filesystem::path build_overlay_bag(
  const std::filesystem::path & dir, int frames, std::uint32_t w, std::uint32_t h)
{
  const std::string image_topic = "/cam/image";
  const std::string camera_info_topic = "/cam/camera_info";
  const std::string pcd_topic = "/lidar/points";
  const std::string tf_static_topic = "/tf_static";
  const std::string camera_frame = "cam";
  const std::string lidar_frame = "lidar";

  const auto path = dir / "input";
  auto writer = bagwiz::io::open_write(path, mcap_dir_opts());
  writer->declare_topic(make_topic(image_topic, "sensor_msgs/msg/Image"));
  writer->declare_topic(make_topic(camera_info_topic, "sensor_msgs/msg/CameraInfo"));
  writer->declare_topic(make_topic(pcd_topic, "sensor_msgs/msg/PointCloud2"));
  writer->declare_topic(bagwiz::core::make_tf_message_topic_info(tf_static_topic));

  // fx and fy chosen so the center of the image maps to (0,0,z) in camera frame.
  const std::array<double, 9> K{
    static_cast<double>(w) / 2.0, 0.0, static_cast<double>(w) / 2.0,
    0.0, static_cast<double>(h) / 2.0, static_cast<double>(h) / 2.0,
    0.0, 0.0, 1.0};
  const auto camera_info_payload = make_camera_info_payload(w, h, K, camera_frame);
  writer->write(
    camera_info_topic, 1'000'000'000LL,
    std::span<const std::byte>{camera_info_payload.data(), camera_info_payload.size()});

  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = camera_frame;
  tf.header.stamp.sec = 1;
  tf.header.stamp.nanosec = 0;
  tf.child_frame_id = lidar_frame;
  tf.transform.rotation.w = 1.0;
  const std::vector<geometry_msgs::msg::TransformStamped> transforms{tf};
  const auto tf_payload = bagwiz::core::serialize_tf_message(transforms);
  writer->write(
    tf_static_topic, 1'000'000'000LL,
    std::span<const std::byte>{tf_payload.data(), tf_payload.size()});

  for (int i = 0; i < frames; ++i) {
    const std::int64_t ts = 1'000'000'000LL + static_cast<std::int64_t>(i) * 100'000'000LL;
    const auto image_payload = make_image_payload(w, h, "bgr8", 0);  // black frame
    writer->write(
      image_topic, ts, std::span<const std::byte>{image_payload.data(), image_payload.size()});
    const auto pcd_payload = make_point_cloud_payload(lidar_frame, {{0.0f, 0.0f, 10.0f}});
    writer->write(
      pcd_topic, ts, std::span<const std::byte>{pcd_payload.data(), pcd_payload.size()});
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

TEST_F(GenerateVideoTest, CheckInfersCameraInfoFromImageRaw)
{
  const auto in = build_bag(tmp_dir_, 2, 16, 16, "bgr8");
  GenerateVideoArgs args{in, kImageTopic, tmp_dir_ / "out.avi", false};
  args.pcd_topics = {"/sensing/lidar"};
  const auto c = check_video_source(in, args);
  EXPECT_EQ(c.status, VideoSourceStatus::kOk);
  ASSERT_TRUE(c.camera_info_topic.has_value());
  EXPECT_EQ(*c.camera_info_topic, "/cam/camera_info");
}

TEST_F(GenerateVideoTest, CheckInfersCameraInfoFromCompressedImage)
{
  const auto in = build_bag(tmp_dir_, 2, 16, 16, "bgr8");
  GenerateVideoArgs args{in, "/cam/image/compressed", tmp_dir_ / "out.avi", false};
  args.pcd_topics = {"/sensing/lidar"};
  const auto c = check_video_source(in, args);
  EXPECT_EQ(c.status, VideoSourceStatus::kOk);
  ASSERT_TRUE(c.camera_info_topic.has_value());
  EXPECT_EQ(*c.camera_info_topic, "/cam/camera_info");
}

TEST_F(GenerateVideoTest, CheckExplicitCameraInfoTopic)
{
  const auto in = build_bag(tmp_dir_, 2, 16, 16, "bgr8");
  GenerateVideoArgs args{in, kImageTopic, tmp_dir_ / "out.avi", false};
  args.pcd_topics = {"/sensing/lidar"};
  args.camera_info_topic = "/cam/camera_info";
  const auto c = check_video_source(in, args);
  EXPECT_EQ(c.status, VideoSourceStatus::kOk);
  ASSERT_TRUE(c.camera_info_topic.has_value());
  EXPECT_EQ(*c.camera_info_topic, "/cam/camera_info");
}

std::filesystem::path build_bag_with_image_topic(
  const std::filesystem::path & dir, const std::string & image_topic)
{
  const auto path = dir / "input";
  auto writer = bagwiz::io::open_write(path, mcap_dir_opts());
  writer->declare_topic(make_topic(image_topic, kImageType));
  writer->declare_topic(make_topic("/sensing/lidar", "sensor_msgs/msg/PointCloud2"));
  writer->close();
  return path;
}

TEST_F(GenerateVideoTest, CheckRejectsMissingCameraInfoInference)
{
  // Image topic name does not match any CameraInfo inference suffix and no
  // explicit --camera-info is given, so overlay validation must fail.
  const auto in = build_bag_with_image_topic(tmp_dir_, "/other/image");
  GenerateVideoArgs args{in, "/other/image", tmp_dir_ / "out.avi", false};
  args.pcd_topics = {"/sensing/lidar"};
  EXPECT_EQ(check_video_source(in, args).status, VideoSourceStatus::kCameraInfoTopicInvalid);
}

TEST_F(GenerateVideoTest, CheckRejectsInvalidPcdTopicType)
{
  const auto in = build_bag(tmp_dir_, 2, 16, 16, "bgr8");
  GenerateVideoArgs args{in, kImageTopic, tmp_dir_ / "out.avi", false};
  args.pcd_topics = {kImageTopic};  // not a PointCloud2
  EXPECT_EQ(check_video_source(in, args).status, VideoSourceStatus::kPcdTopicInvalid);
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

TEST_F(GenerateVideoTest, RunEncodesPointcloudOverlayToVideo)
{
  constexpr int kFrames = 4;
  const auto in = build_overlay_bag(tmp_dir_, kFrames, 16, 16);
  const auto out = tmp_dir_ / "out.avi";

  GenerateVideoArgs args{in, "/cam/image", out, false};
  args.pcd_topics = {"/lidar/points"};
  ASSERT_EQ(run_generate_video(args), 0);

  ASSERT_TRUE(std::filesystem::exists(out));
  EXPECT_FALSE(any_partial_left(tmp_dir_));

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.width, 16U);
  EXPECT_EQ(probe.height, 16U);
  EXPECT_EQ(probe.frame_count, kFrames);
}

}  // namespace
