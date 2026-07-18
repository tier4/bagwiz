// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/cam_info_dump.hpp"

#include "bagwiz/core/image/camera_calibration_yaml.hpp"
#include "bagwiz/core/introspection/introspection_loader.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <sensor_msgs/msg/camera_info.hpp>

#include <gtest/gtest.h>
#include <rcutils/allocator.h>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <rmw/types.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <span>
#include <string>
#include <vector>

namespace
{

using bagwiz::commands::CamInfoDumpArgs;
using bagwiz::commands::run_cam_info_dump;

constexpr const char * kCameraInfoType = "sensor_msgs/msg/CameraInfo";

constexpr std::array<double, 9> kRealK{854.298157, 0.000000, 964.290283, 0.000000, 907.693054,
                                       646.557800, 0.000000, 0.000000,   1.000000};
const std::vector<double> kRealD{
  -0.143768742681, 0.031336024404, -0.001296524890, -0.001500067534, -0.003719333094};
constexpr std::uint32_t kRealWidth = 1920;
constexpr std::uint32_t kRealHeight = 1280;

// The bag's p: identity, which is NOT the alpha=0 solution for the k/d above.
// Dumping must reproduce exactly this -- if a recomputation ever creeps back in,
// p[0] becomes a focal length in the hundreds and these assertions fail loudly.
constexpr std::array<double, 12> kBagP{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};

bagwiz::io::CreateOptions mcap_options()
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = "none";
  return options;
}

bagwiz::io::TopicInfo camera_info_topic_info(const std::string & name)
{
  bagwiz::io::TopicInfo t;
  t.name = name;
  t.type = kCameraInfoType;
  t.serialization_format = "cdr";
  return t;
}

std::vector<std::byte> serialize_camera_info(const sensor_msgs::msg::CameraInfo & msg)
{
  auto intro = bagwiz::core::load_introspection(kCameraInfoType);
  EXPECT_TRUE(intro.ok()) << intro.error;

  rmw_serialized_message_t serialized = rmw_get_zero_initialized_serialized_message();
  rcutils_allocator_t alloc = rcutils_get_default_allocator();
  EXPECT_EQ(rmw_serialized_message_init(&serialized, 0, &alloc), RMW_RET_OK);
  EXPECT_EQ(rmw_serialize(&msg, intro.typesupport, &serialized), RMW_RET_OK);
  std::vector<std::byte> out(serialized.buffer_length);
  if (serialized.buffer_length > 0) {
    std::memcpy(out.data(), serialized.buffer, serialized.buffer_length);
  }
  rmw_serialized_message_fini(&serialized);
  return out;
}

sensor_msgs::msg::CameraInfo deserialize_camera_info(std::span<const std::byte> bytes)
{
  auto intro = bagwiz::core::load_introspection(kCameraInfoType);
  EXPECT_TRUE(intro.ok()) << intro.error;

  rmw_serialized_message_t view = rmw_get_zero_initialized_serialized_message();
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
  view.buffer = const_cast<std::uint8_t *>(reinterpret_cast<const std::uint8_t *>(bytes.data()));
  view.buffer_length = bytes.size();
  view.buffer_capacity = bytes.size();
  view.allocator = rcutils_get_default_allocator();

  sensor_msgs::msg::CameraInfo msg;
  EXPECT_EQ(rmw_deserialize(&view, intro.typesupport, &msg), RMW_RET_OK);
  return msg;
}

sensor_msgs::msg::CameraInfo make_original(std::int32_t sec, const std::string & frame_id)
{
  sensor_msgs::msg::CameraInfo msg;
  msg.header.stamp.sec = sec;
  msg.header.stamp.nanosec = 250U;
  msg.header.frame_id = frame_id;
  msg.height = kRealHeight;
  msg.width = kRealWidth;
  msg.distortion_model = "plumb_bob";
  msg.d = kRealD;
  std::copy(kRealK.begin(), kRealK.end(), msg.k.begin());
  msg.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  std::copy(kBagP.begin(), kBagP.end(), msg.p.begin());
  msg.binning_x = 2;
  msg.binning_y = 3;
  msg.roi.x_offset = 10;
  msg.roi.y_offset = 20;
  msg.roi.width = 30;
  msg.roi.height = 40;
  msg.roi.do_rectify = true;
  return msg;
}

constexpr std::array<std::byte, 4> kOtherPayload{
  std::byte{0xDE}, std::byte{0xAD}, std::byte{0xBE}, std::byte{0xEF}};

// One CameraInfo topic with two messages, one declared-but-empty CameraInfo
// topic, and one unrelated topic -- enough to exercise every refusal.
void write_input_bag(const std::filesystem::path & path)
{
  auto writer = bagwiz::io::open_write(path, mcap_options());
  writer->declare_topic(camera_info_topic_info("/camera/camera_info"));
  writer->declare_topic(camera_info_topic_info("/empty/camera_info"));

  bagwiz::io::TopicInfo other;
  other.name = "/other";
  other.type = "std_msgs/msg/String";
  other.serialization_format = "cdr";
  writer->declare_topic(other);

  const auto m0 = serialize_camera_info(make_original(100, "camera_optical_frame"));
  const auto m1 = serialize_camera_info(make_original(101, "camera_optical_frame"));
  writer->write(
    "/camera/camera_info", 1'000'000'000LL, std::span<const std::byte>(m0.data(), m0.size()));
  writer->write(
    "/camera/camera_info", 2'000'000'000LL, std::span<const std::byte>(m1.data(), m1.size()));
  writer->write(
    "/other", 1'500'000'000LL,
    std::span<const std::byte>(kOtherPayload.data(), kOtherPayload.size()));
  writer->close();
}

// Same, but the second message carries a different k, so the stream's
// calibration is not constant.
void write_varying_bag(const std::filesystem::path & path)
{
  auto writer = bagwiz::io::open_write(path, mcap_options());
  writer->declare_topic(camera_info_topic_info("/camera/camera_info"));

  auto second = make_original(101, "camera_optical_frame");
  second.k[0] = 999.0;

  const auto m0 = serialize_camera_info(make_original(100, "camera_optical_frame"));
  const auto m1 = serialize_camera_info(second);
  writer->write(
    "/camera/camera_info", 1'000'000'000LL, std::span<const std::byte>(m0.data(), m0.size()));
  writer->write(
    "/camera/camera_info", 2'000'000'000LL, std::span<const std::byte>(m1.data(), m1.size()));
  writer->close();
}

class CamInfoDumpTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_cam_info_dump_" +
                std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()));
    std::filesystem::remove_all(tmp_dir_);
    std::filesystem::create_directories(tmp_dir_);
    input_ = tmp_dir_ / "in.mcap";
  }
  void TearDown() override { std::filesystem::remove_all(tmp_dir_); }

  static std::string read_all(const std::filesystem::path & path)
  {
    std::ifstream in(path, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  }

  std::vector<sensor_msgs::msg::CameraInfo> read_topic(
    const std::filesystem::path & bag, const std::string & topic)
  {
    auto reader = bagwiz::io::open_read(bag);
    reader->populate_schemas();
    bagwiz::io::ReadFilter filter;
    filter.topics = {topic};
    reader->set_filter(filter);

    std::vector<sensor_msgs::msg::CameraInfo> out;
    bagwiz::io::RawMessage raw;
    while (reader->next(raw)) {
      if (raw.topic->name != topic) {
        continue;
      }
      out.push_back(deserialize_camera_info(raw.payload));
    }
    return out;
  }

  std::filesystem::path tmp_dir_;
  std::filesystem::path input_;
};

// The whole point of the split: dump exports, it does not recompute.
TEST_F(CamInfoDumpTest, WritesPVerbatimWithoutRecomputingIt)
{
  write_input_bag(input_);
  const auto out = tmp_dir_ / "calib.yaml";

  CamInfoDumpArgs args;
  args.input_path = input_;
  args.topic = "/camera/camera_info";
  args.output_path = out;
  ASSERT_EQ(run_cam_info_dump(args), 0);

  const auto parsed = bagwiz::core::image::parse_camera_calibration_yaml(out);
  ASSERT_TRUE(parsed.ok()) << parsed.error;
  for (std::size_t i = 0; i < 12; ++i) {
    EXPECT_DOUBLE_EQ(parsed.calibration->p[i], kBagP[i]) << "p[" << i << "]";
  }
}

TEST_F(CamInfoDumpTest, RoundTripsEveryOtherCalibrationField)
{
  write_input_bag(input_);
  const auto out = tmp_dir_ / "calib.yaml";

  CamInfoDumpArgs args;
  args.input_path = input_;
  args.topic = "/camera/camera_info";
  args.output_path = out;
  ASSERT_EQ(run_cam_info_dump(args), 0);

  const auto parsed = bagwiz::core::image::parse_camera_calibration_yaml(out);
  ASSERT_TRUE(parsed.ok()) << parsed.error;
  EXPECT_EQ(parsed.calibration->width, kRealWidth);
  EXPECT_EQ(parsed.calibration->height, kRealHeight);
  EXPECT_EQ(parsed.calibration->distortion_model, "plumb_bob");
  EXPECT_EQ(parsed.calibration->d, kRealD);
  for (std::size_t i = 0; i < 9; ++i) {
    EXPECT_DOUBLE_EQ(parsed.calibration->k[i], kRealK[i]) << "k[" << i << "]";
  }
  EXPECT_DOUBLE_EQ(parsed.calibration->r[0], 1.0);
}

// camera_name is not a CameraInfo field, so the bag cannot supply one.
TEST_F(CamInfoDumpTest, OutputHasNoCameraName)
{
  write_input_bag(input_);
  const auto out = tmp_dir_ / "calib.yaml";

  CamInfoDumpArgs args;
  args.input_path = input_;
  args.topic = "/camera/camera_info";
  args.output_path = out;
  ASSERT_EQ(run_cam_info_dump(args), 0);

  const auto parsed = bagwiz::core::image::parse_camera_calibration_yaml(out);
  ASSERT_TRUE(parsed.ok()) << parsed.error;
  EXPECT_FALSE(parsed.calibration->camera_name.has_value());
}

TEST_F(CamInfoDumpTest, PrintsTheSameYamlToStdoutWhenNoOutputGiven)
{
  write_input_bag(input_);
  const auto out = tmp_dir_ / "calib.yaml";

  CamInfoDumpArgs file_args;
  file_args.input_path = input_;
  file_args.topic = "/camera/camera_info";
  file_args.output_path = out;
  ASSERT_EQ(run_cam_info_dump(file_args), 0);
  const std::string from_file = read_all(out);

  CamInfoDumpArgs stdout_args;
  stdout_args.input_path = input_;
  stdout_args.topic = "/camera/camera_info";
  testing::internal::CaptureStdout();
  ASSERT_EQ(run_cam_info_dump(stdout_args), 0);
  const std::string from_stdout = testing::internal::GetCapturedStdout();

  EXPECT_FALSE(from_stdout.empty());
  EXPECT_EQ(from_stdout, from_file);
}

TEST_F(CamInfoDumpTest, LeavesTheBagUnmodified)
{
  write_input_bag(input_);
  const auto before = std::filesystem::file_size(input_);

  CamInfoDumpArgs args;
  args.input_path = input_;
  args.topic = "/camera/camera_info";
  args.output_path = tmp_dir_ / "calib.yaml";
  ASSERT_EQ(run_cam_info_dump(args), 0);

  EXPECT_EQ(std::filesystem::file_size(input_), before);

  // File size alone is a coarse proxy -- a rewrite that happened to preserve
  // size would still pass it. Read the calibration back too and confirm p is
  // still the bag's original identity, not recomputed (that's recompute-p's
  // job, not dump's).
  const auto msgs = read_topic(input_, "/camera/camera_info");
  ASSERT_EQ(msgs.size(), 2U);
  for (std::size_t i = 0; i < 12; ++i) {
    EXPECT_DOUBLE_EQ(msgs[0].p[i], kBagP[i]) << "p[" << i << "]";
  }
}

TEST_F(CamInfoDumpTest, RejectsMissingTopic)
{
  write_input_bag(input_);

  CamInfoDumpArgs args;
  args.input_path = input_;
  args.topic = "/nope";
  args.output_path = tmp_dir_ / "calib.yaml";
  EXPECT_NE(run_cam_info_dump(args), 0);
  EXPECT_FALSE(std::filesystem::exists(tmp_dir_ / "calib.yaml"));
}

TEST_F(CamInfoDumpTest, RejectsWrongTypedTopic)
{
  write_input_bag(input_);

  CamInfoDumpArgs args;
  args.input_path = input_;
  args.topic = "/other";
  args.output_path = tmp_dir_ / "calib.yaml";
  EXPECT_NE(run_cam_info_dump(args), 0);
  EXPECT_FALSE(std::filesystem::exists(tmp_dir_ / "calib.yaml"));
}

TEST_F(CamInfoDumpTest, RejectsTopicWithNoMessages)
{
  write_input_bag(input_);

  CamInfoDumpArgs args;
  args.input_path = input_;
  args.topic = "/empty/camera_info";
  args.output_path = tmp_dir_ / "calib.yaml";
  EXPECT_NE(run_cam_info_dump(args), 0);
  EXPECT_FALSE(std::filesystem::exists(tmp_dir_ / "calib.yaml"));
}

TEST_F(CamInfoDumpTest, HonoursOverwrite)
{
  write_input_bag(input_);
  const auto out = tmp_dir_ / "calib.yaml";
  {
    std::ofstream seed(out);
    seed << "pre-existing\n";
  }

  CamInfoDumpArgs args;
  args.input_path = input_;
  args.topic = "/camera/camera_info";
  args.output_path = out;
  EXPECT_NE(run_cam_info_dump(args), 0);

  args.overwrite = true;
  EXPECT_EQ(run_cam_info_dump(args), 0);
  EXPECT_TRUE(bagwiz::core::image::parse_camera_calibration_yaml(out).ok());
}

// A typo'd topic must not cost the user their -o file, even with -w. The old
// export path prepared the output before validating the topic and would have.
TEST_F(CamInfoDumpTest, DoesNotClobberOutputWhenTopicIsInvalid)
{
  write_input_bag(input_);
  const auto out = tmp_dir_ / "calib.yaml";
  {
    std::ofstream seed(out);
    seed << "precious\n";
  }

  CamInfoDumpArgs args;
  args.input_path = input_;
  args.topic = "/nope";
  args.output_path = out;
  args.overwrite = true;
  EXPECT_NE(run_cam_info_dump(args), 0);

  EXPECT_EQ(read_all(out), "precious\n");
}

// The same protection for a topic that exists and is a CameraInfo topic but
// carries no messages: only reading can discover that, so the output must not be
// claimed until the read has produced a calibration.
TEST_F(CamInfoDumpTest, DoesNotClobberOutputWhenTopicIsEmpty)
{
  write_input_bag(input_);
  const auto out = tmp_dir_ / "calib.yaml";
  {
    std::ofstream seed(out);
    seed << "precious\n";
  }

  CamInfoDumpArgs args;
  args.input_path = input_;
  args.topic = "/empty/camera_info";
  args.output_path = out;
  args.overwrite = true;
  EXPECT_NE(run_cam_info_dump(args), 0);

  EXPECT_EQ(read_all(out), "precious\n");
}

TEST_F(CamInfoDumpTest, UsesFirstMessageWhenCalibrationVaries)
{
  const auto varying = tmp_dir_ / "varying.mcap";
  write_varying_bag(varying);
  const auto out = tmp_dir_ / "calib.yaml";

  CamInfoDumpArgs args;
  args.input_path = varying;
  args.topic = "/camera/camera_info";
  args.output_path = out;
  ASSERT_EQ(run_cam_info_dump(args), 0);

  const auto parsed = bagwiz::core::image::parse_camera_calibration_yaml(out);
  ASSERT_TRUE(parsed.ok()) << parsed.error;
  EXPECT_DOUBLE_EQ(parsed.calibration->k[0], kRealK[0]);  // the first message's, not 999.0
}

}  // namespace
