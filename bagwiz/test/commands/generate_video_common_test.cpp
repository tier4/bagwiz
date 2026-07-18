// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "generate_video_common.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

// Unit tests for the extracted generate-video internals: tmp-path handling, the
// RAII partial-file guard, finalize (rename/clobber), input validation, the
// pass-1 scan, the threading decision, and the frame normalizer's decode /
// resize. Exercises generate_video_common.hpp directly without driving the CLI.

namespace
{

using bagwiz::commands::finalize_video_output;
using bagwiz::commands::finish_video_encode;
using bagwiz::commands::FrameBuffer;
using bagwiz::commands::FrameNormalizer;
using bagwiz::commands::load_video_geometry;
using bagwiz::commands::open_encode_reader;
using bagwiz::commands::partial_tmp_path_for;
using bagwiz::commands::PartialFileGuard;
using bagwiz::commands::scan_video_inputs;
using bagwiz::commands::should_use_threaded_projection;
using bagwiz::commands::validate_video_inputs;
using bagwiz::commands::validate_video_output_path;
using bagwiz::commands::VideoSourceStatus;

constexpr const char * kImageType = "sensor_msgs/msg/Image";
constexpr const char * kCameraInfoType = "sensor_msgs/msg/CameraInfo";
constexpr const char * kPointCloudType = "sensor_msgs/msg/PointCloud2";

// ---- tmp dir fixture --------------------------------------------------------

class GenerateVideoCommonTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_gvc_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
                "_" + ::testing::UnitTest::GetInstance()->current_test_info()->name());
    std::filesystem::create_directories(tmp_dir_);
  }
  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
};

// ---- bag fixture helpers ----------------------------------------------------

bagwiz::io::CreateOptions mcap_options()
{
  bagwiz::io::CreateOptions o;
  o.format = bagwiz::io::Format::Mcap;
  o.layout = bagwiz::io::Layout::SingleFile;
  o.mcap_compression = "none";
  return o;
}

void declare_topic(bagwiz::io::BagWriter & w, const std::string & name, const std::string & type)
{
  bagwiz::io::TopicInfo info;
  info.name = name;
  info.type = type;
  info.serialization_format = "cdr";
  w.declare_topic(info);
}

// A bag with a single raw-image topic and `frames` garbage-payload messages at
// 100 ms spacing starting at 1 s (scan reads timestamps only, never payloads).
std::filesystem::path write_image_bag(
  const std::filesystem::path & dir, const std::string & name, int frames)
{
  const auto path = dir / name;
  auto w = bagwiz::io::open_write(path, mcap_options());
  declare_topic(*w, "/cam/image", kImageType);
  const std::array<std::byte, 4> garbage{
    std::byte{0x00}, std::byte{0x01}, std::byte{0x02}, std::byte{0x03}};
  for (int i = 0; i < frames; ++i) {
    w->write("/cam/image", 1'000'000'000LL + i * 100'000'000LL, garbage);
  }
  w->close();
  return path;
}

// ---- minimal CDR builder for the image decode fixture -----------------------

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
  std::vector<std::byte> take() { return std::move(buf_); }

private:
  void align(std::size_t n)
  {
    while (buf_.size() % n != 0) {
      buf_.push_back(std::byte{0});
    }
  }
  std::vector<std::byte> buf_;
};

std::vector<std::byte> make_bgr8_image_payload(std::uint32_t w, std::uint32_t h, std::uint8_t fill)
{
  std::vector<std::byte> data(static_cast<std::size_t>(w) * h * 3, std::byte{fill});
  CdrBuilder b;
  b.i32(0);  // header.stamp.sec
  b.u32(0);  // header.stamp.nanosec
  b.str("cam");
  b.u32(h);
  b.u32(w);
  b.str("bgr8");
  b.u8(0);       // is_bigendian
  b.u32(w * 3);  // step
  b.byte_seq({data.data(), data.size()});
  return b.take();
}

void write_file(const std::filesystem::path & path, const std::string & content)
{
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out << content;
}

std::string read_file(const std::filesystem::path & path)
{
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

// ---- partial_tmp_path_for ---------------------------------------------------

TEST(PartialTmpPath, KeepsExtensionAfterMarker)
{
  EXPECT_EQ(
    partial_tmp_path_for("/dir/out.avi"), std::filesystem::path("/dir") / "out.bagwiz-partial.avi");
}

TEST(PartialTmpPath, MultiDotNameKeepsOnlyFinalExtension)
{
  EXPECT_EQ(partial_tmp_path_for("a.b.mp4"), std::filesystem::path("a.b.bagwiz-partial.mp4"));
}

TEST(PartialTmpPath, NoExtensionAppendsMarker)
{
  EXPECT_EQ(partial_tmp_path_for("/dir/out"), std::filesystem::path("/dir") / "out.bagwiz-partial");
}

// ---- should_use_threaded_projection -----------------------------------------

TEST(ShouldUseThreadedProjection, RequiresPointClouds)
{
  EXPECT_FALSE(should_use_threaded_projection(false, true, 100, 8));
}

TEST(ShouldUseThreadedProjection, RespectsDisableFlag)
{
  EXPECT_FALSE(should_use_threaded_projection(true, false, 100, 8));
}

TEST(ShouldUseThreadedProjection, RequiresEnoughFrames)
{
  EXPECT_FALSE(should_use_threaded_projection(true, true, 3, 8));
  EXPECT_TRUE(should_use_threaded_projection(true, true, 4, 8));
}

TEST(ShouldUseThreadedProjection, RequiresParallelHardware)
{
  EXPECT_FALSE(should_use_threaded_projection(true, true, 100, 1));
  EXPECT_FALSE(should_use_threaded_projection(true, true, 100, 0));
}

// ---- PartialFileGuard -------------------------------------------------------

TEST_F(GenerateVideoCommonTest, PartialFileGuardCtorRemovesStaleFile)
{
  const auto tmp = tmp_dir_ / "out.bagwiz-partial.avi";
  write_file(tmp, "stale");
  {
    PartialFileGuard guard(tmp);
  }
  EXPECT_FALSE(std::filesystem::exists(tmp));
}

TEST_F(GenerateVideoCommonTest, PartialFileGuardDtorRemovesLeftover)
{
  const auto tmp = tmp_dir_ / "out.bagwiz-partial.avi";
  {
    PartialFileGuard guard(tmp);
    write_file(tmp, "partial");  // simulate an aborted encode
  }
  EXPECT_FALSE(std::filesystem::exists(tmp));
}

TEST_F(GenerateVideoCommonTest, PartialFileGuardDtorLeavesRenamedAwayOutput)
{
  const auto tmp = tmp_dir_ / "out.bagwiz-partial.avi";
  const auto out = tmp_dir_ / "out.avi";
  {
    PartialFileGuard guard(tmp);
    write_file(tmp, "video");
    std::filesystem::rename(tmp, out);
  }
  ASSERT_TRUE(std::filesystem::exists(out));
  EXPECT_EQ(read_file(out), "video");
}

// ---- finalize_video_output --------------------------------------------------

TEST_F(GenerateVideoCommonTest, FinalizeRenamesTmpIntoPlace)
{
  const auto tmp = tmp_dir_ / "out.bagwiz-partial.avi";
  const auto out = tmp_dir_ / "out.avi";
  write_file(tmp, "video");
  EXPECT_EQ(finalize_video_output(tmp, out, false), "");
  EXPECT_FALSE(std::filesystem::exists(tmp));
  ASSERT_TRUE(std::filesystem::exists(out));
  EXPECT_EQ(read_file(out), "video");
}

TEST_F(GenerateVideoCommonTest, FinalizeRejectsExistingOutputWithoutOverwrite)
{
  const auto tmp = tmp_dir_ / "out.bagwiz-partial.avi";
  const auto out = tmp_dir_ / "out.avi";
  write_file(tmp, "video");
  write_file(out, "old");
  const auto err = finalize_video_output(tmp, out, false);
  EXPECT_EQ(
    err, "output path '" + out.string() + "' already exists; pass -w/--overwrite to replace it");
  // The tmp is left for the caller's PartialFileGuard to remove.
  EXPECT_TRUE(std::filesystem::exists(tmp));
  EXPECT_EQ(read_file(out), "old");
}

TEST_F(GenerateVideoCommonTest, FinalizeOverwriteReplacesExistingOutput)
{
  const auto tmp = tmp_dir_ / "out.bagwiz-partial.avi";
  const auto out = tmp_dir_ / "out.avi";
  write_file(tmp, "video");
  write_file(out, "old");
  EXPECT_EQ(finalize_video_output(tmp, out, true), "");
  EXPECT_FALSE(std::filesystem::exists(tmp));
  EXPECT_EQ(read_file(out), "video");
}

// ---- validate_video_output_path ---------------------------------------------

TEST_F(GenerateVideoCommonTest, ValidateOutputPathRejectsCollisionWithoutOverwrite)
{
  const auto out = tmp_dir_ / "out.avi";
  write_file(out, "old");
  EXPECT_EQ(
    validate_video_output_path(out, false),
    "output '" + out.string() + "' already exists; pass -w/--overwrite to replace it.");
}

TEST_F(GenerateVideoCommonTest, ValidateOutputPathAcceptsCollisionWithOverwrite)
{
  const auto out = tmp_dir_ / "out.avi";
  write_file(out, "old");
  EXPECT_EQ(validate_video_output_path(out, true), "");
}

TEST_F(GenerateVideoCommonTest, ValidateOutputPathCreatesMissingParentDirectories)
{
  const auto out = tmp_dir_ / "a" / "b" / "out.avi";
  EXPECT_EQ(validate_video_output_path(out, false), "");
  EXPECT_TRUE(std::filesystem::is_directory(tmp_dir_ / "a" / "b"));
}

// ---- validate_video_inputs --------------------------------------------------

TEST_F(GenerateVideoCommonTest, ValidateInputsUnopenableInput)
{
  bagwiz::commands::GenerateVideoArgs args(
    tmp_dir_ / "does_not_exist.mcap", "/cam/image", tmp_dir_ / "out.avi", false);
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_EQ(v.check.status, VideoSourceStatus::kInputUnopenable);
  EXPECT_EQ(v.error, v.check.message);
  EXPECT_NE(v.error.find("failed to open"), std::string::npos);
}

TEST_F(GenerateVideoCommonTest, ValidateInputsTopicNotFound)
{
  const auto bag = write_image_bag(tmp_dir_, "in.mcap", 1);
  bagwiz::commands::GenerateVideoArgs args(bag, "/nope", tmp_dir_ / "out.avi", false);
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_EQ(v.check.status, VideoSourceStatus::kTopicNotFound);
  EXPECT_EQ(v.error, "topic '/nope' not found in " + bag.string());
}

TEST_F(GenerateVideoCommonTest, ValidateInputsPlainImageTopicOk)
{
  const auto bag = write_image_bag(tmp_dir_, "in.mcap", 1);
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/image", tmp_dir_ / "out.avi", false);
  const auto v = validate_video_inputs(args);
  ASSERT_TRUE(v.ok()) << v.error;
  EXPECT_EQ(v.check.topic_type, kImageType);
  EXPECT_FALSE(v.camera_info_topic.has_value());
}

TEST_F(GenerateVideoCommonTest, ValidateInputsUndistortWithoutCamInfoFails)
{
  const auto bag = write_image_bag(tmp_dir_, "in.mcap", 1);
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/image", tmp_dir_ / "out.avi", false);
  args.undistort = true;
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_EQ(
    v.error,
    "A camera-info topic is required for --undistort or --pcd, but none could be derived from "
    "'/cam/image'. Pass it explicitly with --cam-info.");
}

TEST_F(GenerateVideoCommonTest, ValidateInputsDerivesCamInfoAndAcceptsPcd)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, mcap_options());
    declare_topic(*w, "/cam/image_raw", kImageType);
    declare_topic(*w, "/cam/camera_info", kCameraInfoType);
    declare_topic(*w, "/points", kPointCloudType);
    w->close();
  }
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/image_raw", tmp_dir_ / "out.avi", false);
  args.pointcloud_topics = {"/points"};
  const auto v = validate_video_inputs(args);
  ASSERT_TRUE(v.ok()) << v.error;
  EXPECT_EQ(v.camera_info_topic, "/cam/camera_info");
}

TEST_F(GenerateVideoCommonTest, ValidateInputsPcdTopicNotFoundFails)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, mcap_options());
    declare_topic(*w, "/cam/image_raw", kImageType);
    declare_topic(*w, "/cam/camera_info", kCameraInfoType);
    w->close();
  }
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/image_raw", tmp_dir_ / "out.avi", false);
  args.pointcloud_topics = {"/points"};
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_EQ(v.error, "pcd topic '/points' not found in " + bag.string());
}

TEST_F(GenerateVideoCommonTest, ValidateInputsPcdTopicWrongTypeFails)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, mcap_options());
    declare_topic(*w, "/cam/image_raw", kImageType);
    declare_topic(*w, "/cam/camera_info", kCameraInfoType);
    declare_topic(*w, "/points", kImageType);  // wrong type
    w->close();
  }
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/image_raw", tmp_dir_ / "out.avi", false);
  args.pointcloud_topics = {"/points"};
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_EQ(
    v.error,
    "pcd topic '/points' has type 'sensor_msgs/msg/Image', expected sensor_msgs/msg/PointCloud2");
}

TEST_F(GenerateVideoCommonTest, ValidateInputsExplicitCamInfoMissingFails)
{
  const auto bag = write_image_bag(tmp_dir_, "in.mcap", 1);
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/image", tmp_dir_ / "out.avi", false);
  args.camera_info_topic = "/cam/camera_info";
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_EQ(v.error, "camera_info topic '/cam/camera_info' not found in " + bag.string());
}

// ---- scan_video_inputs ------------------------------------------------------

TEST_F(GenerateVideoCommonTest, ScanEmptyTopicFails)
{
  const auto bag = write_image_bag(tmp_dir_, "in.mcap", 0);
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/image", tmp_dir_ / "out.avi", false);
  const auto s = scan_video_inputs(args);
  EXPECT_FALSE(s.ok());
  EXPECT_EQ(s.error, "topic '/cam/image' has no messages to render.");
}

TEST_F(GenerateVideoCommonTest, ScanDerivesSpanAndFps)
{
  const auto bag = write_image_bag(tmp_dir_, "in.mcap", 3);
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/image", tmp_dir_ / "out.avi", false);
  const auto s = scan_video_inputs(args);
  ASSERT_TRUE(s.ok()) << s.error;
  EXPECT_EQ(s.span.count, 3);
  EXPECT_EQ(s.span.first_ns, 1'000'000'000LL);
  EXPECT_EQ(s.span.last_ns, 1'200'000'000LL);
  EXPECT_EQ(s.fps.num, 10);
  EXPECT_EQ(s.fps.den, 1);
  EXPECT_TRUE(s.pcd_spans.empty());
  EXPECT_TRUE(s.pcd_topic_has_stamps.empty());
  EXPECT_EQ(s.global_property_min, 0.0);
  EXPECT_EQ(s.global_property_max, 0.0);
}

// ---- load_video_geometry ----------------------------------------------------

TEST_F(GenerateVideoCommonTest, LoadVideoGeometryDefaultsToEmpty)
{
  const auto bag = write_image_bag(tmp_dir_, "in.mcap", 1);
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/image", tmp_dir_ / "out.avi", false);
  bagwiz::commands::VideoGeometry g;
  EXPECT_EQ(load_video_geometry(args, std::nullopt, g), "");
  EXPECT_FALSE(g.camera_info.has_value());
  EXPECT_FALSE(g.tf_buffer.has_value());
}

TEST_F(GenerateVideoCommonTest, LoadVideoGeometryFailsWhenCamInfoUnreadable)
{
  // The cam-info topic is declared but carries no message to load.
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, mcap_options());
    declare_topic(*w, "/cam/image_raw", kImageType);
    declare_topic(*w, "/cam/camera_info", kCameraInfoType);
    w->close();
  }
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/image_raw", tmp_dir_ / "out.avi", false);
  bagwiz::commands::VideoGeometry g;
  EXPECT_FALSE(load_video_geometry(args, "/cam/camera_info", g).empty());
}

// ---- open_encode_reader -----------------------------------------------------

TEST_F(GenerateVideoCommonTest, OpenEncodeReaderMissingBagReturnsNull)
{
  bagwiz::commands::GenerateVideoArgs args(
    tmp_dir_ / "does_not_exist.mcap", "/cam/image", tmp_dir_ / "out.avi", false);
  EXPECT_EQ(open_encode_reader(args), nullptr);
}

TEST_F(GenerateVideoCommonTest, OpenEncodeReaderFiltersToTheImageTopic)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, mcap_options());
    declare_topic(*w, "/cam/image", kImageType);
    declare_topic(*w, "/other", kImageType);
    const std::array<std::byte, 4> garbage{
      std::byte{0x00}, std::byte{0x01}, std::byte{0x02}, std::byte{0x03}};
    w->write("/cam/image", 1'000'000'000LL, garbage);
    w->write("/other", 1'000'000'000LL, garbage);
    w->close();
  }
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/image", tmp_dir_ / "out.avi", false);
  auto reader = open_encode_reader(args);
  ASSERT_NE(reader, nullptr);
  bagwiz::io::RawMessage raw;
  ASSERT_TRUE(reader->next(raw));
  EXPECT_EQ(raw.topic->name, "/cam/image");
  EXPECT_FALSE(reader->next(raw));  // /other is filtered out
}

// ---- finish_video_encode ----------------------------------------------------

TEST_F(GenerateVideoCommonTest, FinishEncodeRequiresAStartedEncoder)
{
  bagwiz::commands::GenerateVideoArgs args(
    tmp_dir_ / "in.mcap", "/cam/image", tmp_dir_ / "out.avi", false);
  bagwiz::commands::VideoFrameEncoder encoder(
    partial_tmp_path_for(args.output_path), bagwiz::core::video::FrameRate{10, 1}, args, nullptr,
    0.0, 0.0);
  EXPECT_EQ(
    finish_video_encode(
      encoder, args.topic, partial_tmp_path_for(args.output_path), args.output_path, false),
    "topic '/cam/image' yielded no frames in the encode pass.");
  EXPECT_FALSE(std::filesystem::exists(args.output_path));
  EXPECT_FALSE(std::filesystem::exists(partial_tmp_path_for(args.output_path)));
}

// ---- FrameNormalizer::decode ------------------------------------------------

TEST(FrameNormalizerDecode, Bgr8ImageBecomesCanonicalFrame)
{
  FrameNormalizer normalizer(kImageType, 1.0);
  const auto payload = make_bgr8_image_payload(2, 1, 0x2A);
  const auto frame = normalizer.decode(42, payload, 0);
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->timestamp_ns, 42);
  EXPECT_EQ(frame->header_stamp_ns, 0);
  EXPECT_EQ(frame->width, 2u);
  EXPECT_EQ(frame->height, 1u);
  EXPECT_EQ(frame->step, 6u);
  EXPECT_EQ(frame->encoding, "bgr8");
  ASSERT_EQ(frame->data.size(), 6u);
  EXPECT_EQ(frame->data[0], std::byte{0x2A});
}

TEST(FrameNormalizerDecode, GarbagePayloadRejected)
{
  FrameNormalizer normalizer(kImageType, 1.0);
  const std::array<std::byte, 4> garbage{
    std::byte{0x00}, std::byte{0x01}, std::byte{0x02}, std::byte{0x03}};
  EXPECT_FALSE(normalizer.decode(42, garbage, 0).has_value());
}

// ---- FrameNormalizer::resize ------------------------------------------------

TEST(FrameNormalizerResize, ScaleOneLeavesFrameUntouched)
{
  FrameNormalizer normalizer(kImageType, 1.0);
  FrameBuffer frame;
  frame.width = 2;
  frame.height = 1;
  frame.step = 6;
  frame.data = {std::byte{1}, std::byte{2}, std::byte{3}, std::byte{4}, std::byte{5}, std::byte{6}};
  const auto original = frame.data;
  EXPECT_TRUE(normalizer.resize(frame));
  EXPECT_EQ(frame.width, 2u);
  EXPECT_EQ(frame.height, 1u);
  EXPECT_EQ(frame.data, original);
}

TEST(FrameNormalizerResize, DownscaleHalvesDimensions)
{
  FrameNormalizer normalizer(kImageType, 0.5);
  FrameBuffer frame;
  frame.width = 4;
  frame.height = 2;
  frame.step = 12;
  frame.data.resize(24, std::byte{0x7F});
  EXPECT_TRUE(normalizer.resize(frame));
  EXPECT_EQ(frame.width, 2u);
  EXPECT_EQ(frame.height, 1u);
  EXPECT_EQ(frame.step, 6u);
  EXPECT_EQ(frame.data.size(), 6u);
}

TEST(FrameNormalizerResize, RejectsZeroSizeResult)
{
  FrameNormalizer normalizer(kImageType, 0.1);
  FrameBuffer frame;
  frame.width = 1;
  frame.height = 1;
  frame.step = 3;
  frame.data.resize(3, std::byte{0x00});
  EXPECT_FALSE(normalizer.resize(frame));
}

}  // namespace
