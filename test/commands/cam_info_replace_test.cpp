// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/cam_info_replace.hpp"

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
#include <span>
#include <string>
#include <vector>

namespace
{

constexpr const char * kCameraInfoType = "sensor_msgs/msg/CameraInfo";

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

// A CameraInfo carrying a deliberately "wrong" calibration plus distinctive
// header / binning / roi values that must survive the replace untouched.
sensor_msgs::msg::CameraInfo make_original(std::int32_t sec, const std::string & frame_id)
{
  sensor_msgs::msg::CameraInfo msg;
  msg.header.stamp.sec = sec;
  msg.header.stamp.nanosec = 250U;
  msg.header.frame_id = frame_id;
  msg.height = 1;
  msg.width = 1;
  msg.distortion_model = "none";
  msg.d = {9.9};
  msg.k = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  msg.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  msg.p = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
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

// Writes an input bag with two CameraInfo topics (/camera/camera_info and
// /camera2/camera_info), each carrying two messages with a deliberately wrong
// calibration, plus one unrelated /other message. The second camera topic lets
// the multi-topic tests confirm a single YAML is applied to every listed topic
// while a non-listed CameraInfo topic is still copied verbatim.
void write_input_bag(const std::filesystem::path & path)
{
  auto writer = bagwiz::io::open_write(path, mcap_options());
  writer->declare_topic(camera_info_topic_info("/camera/camera_info"));
  writer->declare_topic(camera_info_topic_info("/camera2/camera_info"));

  bagwiz::io::TopicInfo other;
  other.name = "/other";
  other.type = "std_msgs/msg/String";
  other.serialization_format = "cdr";
  writer->declare_topic(other);

  const auto m0 = serialize_camera_info(make_original(100, "camera_optical_frame"));
  const auto m1 = serialize_camera_info(make_original(101, "camera_optical_frame"));
  const auto n0 = serialize_camera_info(make_original(200, "camera2_optical_frame"));
  const auto n1 = serialize_camera_info(make_original(201, "camera2_optical_frame"));
  writer->write(
    "/camera/camera_info", 1'000'000'000LL, std::span<const std::byte>(m0.data(), m0.size()));
  writer->write(
    "/camera/camera_info", 2'000'000'000LL, std::span<const std::byte>(m1.data(), m1.size()));
  writer->write(
    "/camera2/camera_info", 1'100'000'000LL, std::span<const std::byte>(n0.data(), n0.size()));
  writer->write(
    "/camera2/camera_info", 2'100'000'000LL, std::span<const std::byte>(n1.data(), n1.size()));
  writer->write(
    "/other", 1'500'000'000LL,
    std::span<const std::byte>(kOtherPayload.data(), kOtherPayload.size()));
  writer->close();
}

constexpr const char * kCalibYaml = R"(image_width: 640
image_height: 480
camera_name: narrow_stereo
camera_matrix:
  rows: 3
  cols: 3
  data: [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.01, -0.02, 0.003, 0.004, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
)";

const bagwiz::io::TopicInfo * find_topic(
  const bagwiz::io::BagReader & reader, const std::string & name)
{
  for (const auto & t : reader.topics()) {
    if (t.name == name) {
      return &t;
    }
  }
  return nullptr;
}

struct CameraInfoReadback
{
  int count = 0;
  std::vector<sensor_msgs::msg::CameraInfo> messages;
};

CameraInfoReadback read_camera_info(
  const std::filesystem::path & path, const std::string & topic = "/camera/camera_info")
{
  CameraInfoReadback result;
  auto reader = bagwiz::io::open_read(path);
  reader->populate_schemas();
  bagwiz::io::ReadFilter filter;
  filter.topics = {topic};
  reader->set_filter(filter);
  bagwiz::io::RawMessage raw;
  while (reader->next(raw)) {
    if (raw.topic->name != topic) {
      continue;
    }
    ++result.count;
    result.messages.push_back(deserialize_camera_info(raw.payload));
  }
  return result;
}

std::vector<std::byte> read_other_payload(
  const std::filesystem::path & path, std::string & type_out)
{
  std::vector<std::byte> out;
  auto reader = bagwiz::io::open_read(path);
  const auto * info = find_topic(*reader, "/other");
  if (info == nullptr) {
    return out;
  }
  type_out = info->type;
  bagwiz::io::ReadFilter filter;
  filter.topics = {"/other"};
  reader->set_filter(filter);
  bagwiz::io::RawMessage raw;
  while (reader->next(raw)) {
    if (raw.topic->name != "/other") {
      continue;
    }
    out.assign(raw.payload.begin(), raw.payload.end());
  }
  return out;
}

class CamInfoReplaceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_cam_info_replace_" +
                std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()));
    std::filesystem::remove_all(tmp_dir_);
    std::filesystem::create_directories(tmp_dir_);
    calib_path_ = tmp_dir_ / "calib.yaml";
    std::ofstream out(calib_path_);
    out << kCalibYaml;
    out.close();
  }
  void TearDown() override { std::filesystem::remove_all(tmp_dir_); }

  // Assert that `msg` carries the YAML calibration and the preserved fields.
  static void expect_replaced(const sensor_msgs::msg::CameraInfo & msg, std::int32_t expect_sec)
  {
    EXPECT_EQ(msg.width, 640U);
    EXPECT_EQ(msg.height, 480U);
    EXPECT_EQ(msg.distortion_model, "plumb_bob");
    ASSERT_EQ(msg.d.size(), 5U);
    EXPECT_DOUBLE_EQ(msg.d[0], 0.01);
    EXPECT_DOUBLE_EQ(msg.k[0], 500.0);
    EXPECT_DOUBLE_EQ(msg.k[2], 320.0);
    EXPECT_DOUBLE_EQ(msg.p[0], 500.0);
    // Preserved fields.
    EXPECT_EQ(msg.header.stamp.sec, expect_sec);
    EXPECT_EQ(msg.header.stamp.nanosec, 250U);
    EXPECT_EQ(msg.binning_x, 2U);
    EXPECT_EQ(msg.binning_y, 3U);
    EXPECT_EQ(msg.roi.x_offset, 10U);
    EXPECT_EQ(msg.roi.height, 40U);
    EXPECT_TRUE(msg.roi.do_rectify);
  }

  std::filesystem::path tmp_dir_;
  std::filesystem::path calib_path_;
};

TEST_F(CamInfoReplaceTest, ReplacesToOutputAndCopiesOthers)
{
  const auto in = tmp_dir_ / "in.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_input_bag(in);

  bagwiz::commands::CamInfoReplaceArgs args;
  args.input_path = in;
  args.yaml_path = calib_path_;
  args.topics = {"/camera/camera_info"};
  args.output_path = out;
  ASSERT_EQ(bagwiz::commands::run_cam_info_replace(args), 0);

  const auto readback = read_camera_info(out);
  ASSERT_EQ(readback.count, 2);
  expect_replaced(readback.messages[0], 100);
  expect_replaced(readback.messages[1], 101);
  // frame_id is preserved when --frame-id is not given.
  EXPECT_EQ(readback.messages[0].header.frame_id, "camera_optical_frame");

  // The unrelated topic is copied verbatim.
  std::string other_type;
  const auto payload = read_other_payload(out, other_type);
  EXPECT_EQ(other_type, "std_msgs/msg/String");
  ASSERT_EQ(payload.size(), kOtherPayload.size());
  EXPECT_TRUE(std::equal(payload.begin(), payload.end(), kOtherPayload.begin()));

  // Input bag is untouched in -o mode.
  const auto in_readback = read_camera_info(in);
  ASSERT_EQ(in_readback.count, 2);
  EXPECT_EQ(in_readback.messages[0].width, 1U);
}

TEST_F(CamInfoReplaceTest, ReplacesInPlace)
{
  const auto in = tmp_dir_ / "in.mcap";
  write_input_bag(in);

  bagwiz::commands::CamInfoReplaceArgs args;
  args.input_path = in;
  args.yaml_path = calib_path_;
  args.topics = {"/camera/camera_info"};
  ASSERT_EQ(bagwiz::commands::run_cam_info_replace(args), 0);

  const auto readback = read_camera_info(in);
  ASSERT_EQ(readback.count, 2);
  expect_replaced(readback.messages[0], 100);
}

TEST_F(CamInfoReplaceTest, FrameIdOverride)
{
  const auto in = tmp_dir_ / "in.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_input_bag(in);

  bagwiz::commands::CamInfoReplaceArgs args;
  args.input_path = in;
  args.yaml_path = calib_path_;
  args.topics = {"/camera/camera_info"};
  args.frame_id = "new_frame";
  args.output_path = out;
  ASSERT_EQ(bagwiz::commands::run_cam_info_replace(args), 0);

  const auto readback = read_camera_info(out);
  ASSERT_EQ(readback.count, 2);
  EXPECT_EQ(readback.messages[0].header.frame_id, "new_frame");
  EXPECT_EQ(readback.messages[1].header.frame_id, "new_frame");
}

TEST_F(CamInfoReplaceTest, RejectsMissingTopic)
{
  const auto in = tmp_dir_ / "in.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_input_bag(in);

  bagwiz::commands::CamInfoReplaceArgs args;
  args.input_path = in;
  args.yaml_path = calib_path_;
  args.topics = {"/does/not/exist"};
  args.output_path = out;
  EXPECT_EQ(bagwiz::commands::run_cam_info_replace(args), 1);
}

TEST_F(CamInfoReplaceTest, RejectsWrongType)
{
  const auto in = tmp_dir_ / "in.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_input_bag(in);

  bagwiz::commands::CamInfoReplaceArgs args;
  args.input_path = in;
  args.yaml_path = calib_path_;
  args.topics = {"/other"};  // std_msgs/msg/String, not CameraInfo
  args.output_path = out;
  EXPECT_EQ(bagwiz::commands::run_cam_info_replace(args), 1);
}

TEST_F(CamInfoReplaceTest, ReplacesMultipleTopicsWithOneYaml)
{
  const auto in = tmp_dir_ / "in.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_input_bag(in);

  bagwiz::commands::CamInfoReplaceArgs args;
  args.input_path = in;
  args.yaml_path = calib_path_;
  args.topics = {"/camera/camera_info", "/camera2/camera_info"};
  args.output_path = out;
  ASSERT_EQ(bagwiz::commands::run_cam_info_replace(args), 0);

  // Both listed topics carry the same YAML calibration, each preserving its own
  // per-message header timestamp and frame_id.
  const auto cam1 = read_camera_info(out, "/camera/camera_info");
  ASSERT_EQ(cam1.count, 2);
  expect_replaced(cam1.messages[0], 100);
  expect_replaced(cam1.messages[1], 101);
  EXPECT_EQ(cam1.messages[0].header.frame_id, "camera_optical_frame");

  const auto cam2 = read_camera_info(out, "/camera2/camera_info");
  ASSERT_EQ(cam2.count, 2);
  expect_replaced(cam2.messages[0], 200);
  expect_replaced(cam2.messages[1], 201);
  EXPECT_EQ(cam2.messages[0].header.frame_id, "camera2_optical_frame");

  // The input is untouched in -o mode.
  const auto in_cam2 = read_camera_info(in, "/camera2/camera_info");
  ASSERT_EQ(in_cam2.count, 2);
  EXPECT_EQ(in_cam2.messages[0].width, 1U);
}

TEST_F(CamInfoReplaceTest, LeavesUnlistedCameraInfoTopicVerbatim)
{
  const auto in = tmp_dir_ / "in.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_input_bag(in);

  bagwiz::commands::CamInfoReplaceArgs args;
  args.input_path = in;
  args.yaml_path = calib_path_;
  args.topics = {"/camera/camera_info"};  // /camera2/camera_info intentionally omitted
  args.output_path = out;
  ASSERT_EQ(bagwiz::commands::run_cam_info_replace(args), 0);

  // The listed topic is rewritten...
  const auto cam1 = read_camera_info(out, "/camera/camera_info");
  ASSERT_EQ(cam1.count, 2);
  expect_replaced(cam1.messages[0], 100);

  // ...while the unlisted CameraInfo topic keeps its original (wrong) calibration.
  const auto cam2 = read_camera_info(out, "/camera2/camera_info");
  ASSERT_EQ(cam2.count, 2);
  EXPECT_EQ(cam2.messages[0].width, 1U);
  EXPECT_EQ(cam2.messages[0].distortion_model, "none");
}

TEST_F(CamInfoReplaceTest, RejectsWhenAnyListedTopicIsInvalid)
{
  const auto in = tmp_dir_ / "in.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_input_bag(in);

  bagwiz::commands::CamInfoReplaceArgs args;
  args.input_path = in;
  args.yaml_path = calib_path_;
  // One valid topic plus one that does not exist: the whole run must fail before
  // any output is produced (validation precedes the rewrite passes).
  args.topics = {"/camera/camera_info", "/does/not/exist"};
  args.output_path = out;
  EXPECT_EQ(bagwiz::commands::run_cam_info_replace(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

TEST_F(CamInfoReplaceTest, DeduplicatesRepeatedTopic)
{
  const auto in = tmp_dir_ / "in.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_input_bag(in);

  bagwiz::commands::CamInfoReplaceArgs args;
  args.input_path = in;
  args.yaml_path = calib_path_;
  args.topics = {"/camera/camera_info", "/camera/camera_info"};  // duplicate is harmless
  args.output_path = out;
  ASSERT_EQ(bagwiz::commands::run_cam_info_replace(args), 0);

  const auto cam1 = read_camera_info(out, "/camera/camera_info");
  ASSERT_EQ(cam1.count, 2);  // two messages, each rewritten once (not duplicated)
  expect_replaced(cam1.messages[0], 100);
}

std::vector<std::vector<std::byte>> read_raw_payloads(
  const std::filesystem::path & path, const std::string & topic)
{
  std::vector<std::vector<std::byte>> out;
  auto reader = bagwiz::io::open_read(path);
  bagwiz::io::ReadFilter filter;
  filter.topics = {topic};
  reader->set_filter(filter);
  bagwiz::io::RawMessage raw;
  while (reader->next(raw)) {
    out.emplace_back(raw.payload.begin(), raw.payload.end());
  }
  return out;
}

// The rewritten CameraInfo payloads must be byte-identical to the equivalent
// messages serialized through rmw_serialize.
TEST_F(CamInfoReplaceTest, RewrittenPayloadMatchesRmwSerialize)
{
  const auto in = tmp_dir_ / "in.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_input_bag(in);

  bagwiz::commands::CamInfoReplaceArgs args;
  args.input_path = in;
  args.yaml_path = calib_path_;
  args.topics = {"/camera/camera_info"};
  args.output_path = out;
  ASSERT_EQ(bagwiz::commands::run_cam_info_replace(args), 0);

  const auto readback = read_camera_info(out);
  const auto payloads = read_raw_payloads(out, "/camera/camera_info");
  ASSERT_EQ(readback.count, static_cast<int>(payloads.size()));
  for (std::size_t i = 0; i < payloads.size(); ++i) {
    const auto expected = serialize_camera_info(readback.messages[i]);
    EXPECT_EQ(payloads[i].size(), expected.size());
    EXPECT_TRUE(std::equal(payloads[i].begin(), payloads[i].end(), expected.begin()));
  }
}

}  // namespace
