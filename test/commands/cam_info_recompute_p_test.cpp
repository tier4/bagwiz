// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/cam_info_recompute_p.hpp"

#include "bagwiz/core/image/camera_calibration_yaml.hpp"
#include "bagwiz/core/image/projection_matrix.hpp"
#include "bagwiz/core/introspection_loader.hpp"
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

using bagwiz::commands::CamInfoRecomputePArgs;
using bagwiz::commands::run_cam_info_recompute_p;

constexpr const char * kCameraInfoType = "sensor_msgs/msg/CameraInfo";

// A real plumb_bob calibration (the same one projection_matrix_test.cpp anchors
// on), so the p expected below is whatever the linked OpenCV produces for it --
// computed rather than hardcoded, since the value is OpenCV-version-dependent.
constexpr std::array<double, 9> kRealK{854.298157, 0.000000, 964.290283, 0.000000, 907.693054,
                                       646.557800, 0.000000, 0.000000,   1.000000};
const std::vector<double> kRealD{
  -0.143768742681, 0.031336024404, -0.001296524890, -0.001500067534, -0.003719333094};
constexpr std::uint32_t kRealWidth = 1920;
constexpr std::uint32_t kRealHeight = 1280;

// The p the command must produce for the fixture above, from this build's
// OpenCV. Pinning a literal instead would fail on the other ROS distros, whose
// OpenCV versions give a slightly different (equally correct) answer.
std::array<double, 12> expected_p(double alpha)
{
  bagwiz::core::image::ProjectionMatrixInput in;
  in.k = kRealK;
  in.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  in.d = kRealD;
  in.distortion_model = "plumb_bob";
  in.width = kRealWidth;
  in.height = kRealHeight;
  const auto result = bagwiz::core::image::compute_projection_matrix(in, alpha);
  EXPECT_TRUE(result.ok()) << result.error;
  return *result.p;
}

bagwiz::io::CreateOptions mcap_options()
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = "none";
  return options;
}

bagwiz::io::CreateOptions sqlite3_options()
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Sqlite3;
  options.layout = bagwiz::io::Layout::SingleFile;
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

// A CameraInfo with a correct k/d but a deliberately wrong p (identity), plus
// distinctive header / binning / roi values that must survive untouched.
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
  msg.p = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};  // wrong on purpose
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

// Two CameraInfo topics, each with two messages, plus one unrelated /other
// message. The second camera topic proves a non-listed CameraInfo topic is
// copied verbatim rather than swept up.
void write_input_bag_with(
  const std::filesystem::path & path, const bagwiz::io::CreateOptions & options)
{
  auto writer = bagwiz::io::open_write(path, options);
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

void write_input_bag(const std::filesystem::path & path)
{
  write_input_bag_with(path, mcap_options());
}

constexpr const char * kRealCalibYaml = R"(image_width: 1920
image_height: 1280
camera_name: camera
camera_matrix:
  rows: 3
  cols: 3
  data: [854.298157, 0.0, 964.290283, 0.0, 907.693054, 646.5578, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.143768742681, 0.031336024404, -0.00129652489, -0.001500067534, -0.003719333094]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
)";

class CamInfoRecomputePTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_cam_info_recompute_p_" +
                std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()));
    std::filesystem::remove_all(tmp_dir_);
    std::filesystem::create_directories(tmp_dir_);
    input_ = tmp_dir_ / "in.mcap";
  }
  void TearDown() override { std::filesystem::remove_all(tmp_dir_); }

  std::filesystem::path write_yaml(const std::string & contents, const std::string & name)
  {
    const auto path = tmp_dir_ / name;
    std::ofstream out(path);
    out << contents;
    out.close();
    return path;
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

// --- YAML mode -------------------------------------------------------------

TEST_F(CamInfoRecomputePTest, YamlModeRecomputesProjectionMatrix)
{
  const auto path = write_yaml(kRealCalibYaml, "calib.yaml");
  CamInfoRecomputePArgs args;
  args.input_path = path;

  ASSERT_EQ(run_cam_info_recompute_p(args), 0);

  const auto reparsed = bagwiz::core::image::parse_camera_calibration_yaml(path);
  ASSERT_TRUE(reparsed.ok()) << reparsed.error;
  const auto want = expected_p(0.0);
  for (std::size_t i = 0; i < 12; ++i) {
    EXPECT_NEAR(reparsed.calibration->p[i], want[i], 1e-6) << "p[" << i << "]";
  }
}

TEST_F(CamInfoRecomputePTest, YamlModePreservesEverythingElse)
{
  const auto path = write_yaml(kRealCalibYaml, "calib.yaml");
  const auto before = bagwiz::core::image::parse_camera_calibration_yaml(path);
  ASSERT_TRUE(before.ok()) << before.error;

  CamInfoRecomputePArgs args;
  args.input_path = path;
  ASSERT_EQ(run_cam_info_recompute_p(args), 0);

  const auto after = bagwiz::core::image::parse_camera_calibration_yaml(path);
  ASSERT_TRUE(after.ok()) << after.error;
  EXPECT_EQ(after.calibration->width, before.calibration->width);
  EXPECT_EQ(after.calibration->height, before.calibration->height);
  EXPECT_EQ(after.calibration->distortion_model, before.calibration->distortion_model);
  EXPECT_EQ(after.calibration->d, before.calibration->d);
  EXPECT_EQ(after.calibration->k, before.calibration->k);
  EXPECT_EQ(after.calibration->r, before.calibration->r);
  // The reason camera_name is carried through the parser at all.
  EXPECT_EQ(after.calibration->camera_name, before.calibration->camera_name);
}

TEST_F(CamInfoRecomputePTest, YamlModeWritesToOutputAndLeavesInputAlone)
{
  const auto path = write_yaml(kRealCalibYaml, "calib.yaml");
  const auto out = tmp_dir_ / "out.yaml";

  CamInfoRecomputePArgs args;
  args.input_path = path;
  args.output_path = out;
  ASSERT_EQ(run_cam_info_recompute_p(args), 0);

  ASSERT_TRUE(std::filesystem::exists(out));
  // Input keeps its original (identity) p.
  const auto original = bagwiz::core::image::parse_camera_calibration_yaml(path);
  ASSERT_TRUE(original.ok()) << original.error;
  EXPECT_DOUBLE_EQ(original.calibration->p[0], 1.0);
}

TEST_F(CamInfoRecomputePTest, YamlModeRefusesExistingOutputWithoutOverwrite)
{
  const auto path = write_yaml(kRealCalibYaml, "calib.yaml");
  const auto out = write_yaml("pre-existing\n", "out.yaml");

  CamInfoRecomputePArgs args;
  args.input_path = path;
  args.output_path = out;
  EXPECT_NE(run_cam_info_recompute_p(args), 0);

  args.overwrite = true;
  EXPECT_EQ(run_cam_info_recompute_p(args), 0);
}

// A YAML carries no topics, so --topics against one is a mistake worth naming
// rather than quietly ignoring.
TEST_F(CamInfoRecomputePTest, YamlModeRejectsTopicsFlag)
{
  CamInfoRecomputePArgs args;
  args.input_path = write_yaml(kRealCalibYaml, "calib.yaml");
  args.topics = {"/camera/camera_info"};

  EXPECT_NE(run_cam_info_recompute_p(args), 0);
}

TEST_F(CamInfoRecomputePTest, YamlModeRefusesFisheyeWithoutWritingAnything)
{
  std::string yaml = kRealCalibYaml;
  const auto at = yaml.find("plumb_bob");
  ASSERT_NE(at, std::string::npos);
  yaml.replace(at, std::string("plumb_bob").size(), "equidistant");
  const auto path = write_yaml(yaml, "calib.yaml");

  CamInfoRecomputePArgs args;
  args.input_path = path;
  EXPECT_NE(run_cam_info_recompute_p(args), 0);

  // The file must be untouched: p is still the identity it started as.
  const auto after = bagwiz::core::image::parse_camera_calibration_yaml(path);
  ASSERT_TRUE(after.ok()) << after.error;
  EXPECT_DOUBLE_EQ(after.calibration->p[0], 1.0);
}

// -o names where the result goes, not what it is: a YAML input writes a YAML,
// whatever the output is called. (This used to be an error.)
TEST_F(CamInfoRecomputePTest, YamlInputWritesYamlToAnyOutputPath)
{
  const auto out = tmp_dir_ / "out.mcap";
  CamInfoRecomputePArgs args;
  args.input_path = write_yaml(kRealCalibYaml, "calib.yaml");
  args.output_path = out;
  ASSERT_EQ(run_cam_info_recompute_p(args), 0);

  // Named .mcap, but it is a calibration YAML.
  const auto parsed = bagwiz::core::image::parse_camera_calibration_yaml(out);
  ASSERT_TRUE(parsed.ok()) << parsed.error;
  EXPECT_NEAR(parsed.calibration->p[0], expected_p(0.0)[0], 1e-6);
}

// --- bag mode --------------------------------------------------------------

TEST_F(CamInfoRecomputePTest, BagModeRecomputesPOnTargetTopicOnly)
{
  write_input_bag(input_);
  const auto out = tmp_dir_ / "out.mcap";

  CamInfoRecomputePArgs args;
  args.input_path = input_;
  args.topics = {"/camera/camera_info"};
  args.output_path = out;
  ASSERT_EQ(run_cam_info_recompute_p(args), 0);

  const auto want = expected_p(0.0);
  const auto target = read_topic(out, "/camera/camera_info");
  ASSERT_EQ(target.size(), 2U);
  for (const auto & msg : target) {
    for (std::size_t i = 0; i < 12; ++i) {
      EXPECT_NEAR(msg.p[i], want[i], 1e-9) << "p[" << i << "]";
    }
  }

  // The CameraInfo topic that was not listed keeps its original identity p.
  const auto untouched = read_topic(out, "/camera2/camera_info");
  ASSERT_EQ(untouched.size(), 2U);
  EXPECT_DOUBLE_EQ(untouched[0].p[0], 1.0);
}

TEST_F(CamInfoRecomputePTest, BagModePreservesHeaderBinningRoiAndIntrinsics)
{
  write_input_bag(input_);
  const auto out = tmp_dir_ / "out.mcap";

  CamInfoRecomputePArgs args;
  args.input_path = input_;
  args.topics = {"/camera/camera_info"};
  args.output_path = out;
  ASSERT_EQ(run_cam_info_recompute_p(args), 0);

  const auto msgs = read_topic(out, "/camera/camera_info");
  ASSERT_EQ(msgs.size(), 2U);
  const auto & m = msgs[0];
  EXPECT_EQ(m.header.stamp.sec, 100);
  EXPECT_EQ(m.header.stamp.nanosec, 250U);
  EXPECT_EQ(m.header.frame_id, "camera_optical_frame");
  EXPECT_EQ(m.binning_x, 2U);
  EXPECT_EQ(m.binning_y, 3U);
  EXPECT_EQ(m.roi.x_offset, 10U);
  EXPECT_EQ(m.roi.y_offset, 20U);
  EXPECT_EQ(m.roi.width, 30U);
  EXPECT_EQ(m.roi.height, 40U);
  EXPECT_TRUE(m.roi.do_rectify);
  // Only p changes: k/d/r and the size are the inputs, not outputs.
  EXPECT_EQ(m.width, kRealWidth);
  EXPECT_EQ(m.height, kRealHeight);
  EXPECT_EQ(m.distortion_model, "plumb_bob");
  EXPECT_EQ(m.d, kRealD);
  for (std::size_t i = 0; i < 9; ++i) {
    EXPECT_DOUBLE_EQ(m.k[i], kRealK[i]) << "k[" << i << "]";
  }
}

TEST_F(CamInfoRecomputePTest, BagModeAppliesAlpha)
{
  write_input_bag(input_);
  const auto out = tmp_dir_ / "out.mcap";

  CamInfoRecomputePArgs args;
  args.input_path = input_;
  args.topics = {"/camera/camera_info"};
  args.output_path = out;
  args.alpha = 1.0;
  ASSERT_EQ(run_cam_info_recompute_p(args), 0);

  const auto want = expected_p(1.0);
  const auto msgs = read_topic(out, "/camera/camera_info");
  ASSERT_EQ(msgs.size(), 2U);
  EXPECT_NEAR(msgs[0].p[0], want[0], 1e-9);
  // alpha=1 retains all source pixels, so it must differ from the alpha=0 crop.
  EXPECT_NE(msgs[0].p[0], expected_p(0.0)[0]);
}

TEST_F(CamInfoRecomputePTest, BagModeRewritesEveryListedTopic)
{
  write_input_bag(input_);
  const auto out = tmp_dir_ / "out.mcap";

  CamInfoRecomputePArgs args;
  args.input_path = input_;
  args.topics = {"/camera/camera_info", "/camera2/camera_info"};
  args.output_path = out;
  ASSERT_EQ(run_cam_info_recompute_p(args), 0);

  const auto want = expected_p(0.0);
  for (const char * topic : {"/camera/camera_info", "/camera2/camera_info"}) {
    const auto msgs = read_topic(out, topic);
    ASSERT_EQ(msgs.size(), 2U) << topic;
    EXPECT_NEAR(msgs[0].p[0], want[0], 1e-9) << topic;
  }
}

TEST_F(CamInfoRecomputePTest, BagModeRewritesInPlaceWhenNoOutputGiven)
{
  write_input_bag(input_);

  CamInfoRecomputePArgs args;
  args.input_path = input_;
  args.topics = {"/camera/camera_info"};
  ASSERT_EQ(run_cam_info_recompute_p(args), 0);

  const auto want = expected_p(0.0);
  const auto msgs = read_topic(input_, "/camera/camera_info");
  ASSERT_EQ(msgs.size(), 2U);
  EXPECT_NEAR(msgs[0].p[0], want[0], 1e-9);
}

// --topics is required for a bag: without it there is no way to know which
// CameraInfo topic to rewrite, and silently rewriting all of them would be a
// guess. CLI11 cannot express "required only for a bag", so this is validated
// once the mode is known.
TEST_F(CamInfoRecomputePTest, BagModeRequiresTopicsFlag)
{
  write_input_bag(input_);

  CamInfoRecomputePArgs args;
  args.input_path = input_;
  EXPECT_NE(run_cam_info_recompute_p(args), 0);
}

TEST_F(CamInfoRecomputePTest, BagModeRejectsMissingTopic)
{
  write_input_bag(input_);

  CamInfoRecomputePArgs args;
  args.input_path = input_;
  args.topics = {"/nope"};
  args.output_path = tmp_dir_ / "out.mcap";
  EXPECT_NE(run_cam_info_recompute_p(args), 0);
}

TEST_F(CamInfoRecomputePTest, BagModeRejectsWrongTypedTopic)
{
  write_input_bag(input_);

  CamInfoRecomputePArgs args;
  args.input_path = input_;
  args.topics = {"/other"};
  args.output_path = tmp_dir_ / "out.mcap";
  EXPECT_NE(run_cam_info_recompute_p(args), 0);
}

// An extension-less -o inherits <input>'s storage format rather than defaulting
// to MCAP, matching topic drop/keep/rename and pcd concat/undistort.
TEST_F(CamInfoRecomputePTest, BagModeInheritsInputFormatForExtensionlessOutput)
{
  const auto db3_input = tmp_dir_ / "in.db3";
  write_input_bag_with(db3_input, sqlite3_options());
  const auto out = tmp_dir_ / "out_dir";

  CamInfoRecomputePArgs args;
  args.input_path = db3_input;
  args.topics = {"/camera/camera_info"};
  args.output_path = out;
  ASSERT_EQ(run_cam_info_recompute_p(args), 0);

  EXPECT_EQ(bagwiz::io::detect_format(out), bagwiz::io::Format::Sqlite3);
  const auto msgs = read_topic(out, "/camera/camera_info");
  ASSERT_EQ(msgs.size(), 2U);
  EXPECT_NEAR(msgs[0].p[0], expected_p(0.0)[0], 1e-6);
}

// A .mcap -o still wins over a sqlite3 input: the extension picks the format.
TEST_F(CamInfoRecomputePTest, BagModeConvertsWhenOutputExtensionNamesAFormat)
{
  const auto db3_input = tmp_dir_ / "in.db3";
  write_input_bag_with(db3_input, sqlite3_options());
  const auto out = tmp_dir_ / "out.mcap";

  CamInfoRecomputePArgs args;
  args.input_path = db3_input;
  args.topics = {"/camera/camera_info"};
  args.output_path = out;
  ASSERT_EQ(run_cam_info_recompute_p(args), 0);

  EXPECT_EQ(bagwiz::io::detect_format(out), bagwiz::io::Format::Mcap);
  const auto msgs = read_topic(out, "/camera/camera_info");
  ASSERT_EQ(msgs.size(), 2U);
}

// -o names where the result goes, not what it is: a bag input still gets a bag
// output even when -o is named "calib.yaml" -- this is the documented breaking
// change from the old export-by-extension behavior. See "Migration from the
// old -o behavior" in docs/commands/cam-info.md; use `cam-info dump` to
// actually export a calibration YAML from a bag.
TEST_F(CamInfoRecomputePTest, BagModeWritesABagEvenWhenOutputIsNamedYaml)
{
  write_input_bag(input_);
  const auto out = tmp_dir_ / "calib.yaml";

  CamInfoRecomputePArgs args;
  args.input_path = input_;
  args.topics = {"/camera/camera_info"};
  args.output_path = out;
  ASSERT_EQ(run_cam_info_recompute_p(args), 0);

  // A directory bag named "calib.yaml", not a calibration YAML.
  ASSERT_TRUE(std::filesystem::is_directory(out));
  EXPECT_EQ(bagwiz::io::detect_format(out), bagwiz::io::Format::Mcap);
  const auto want = expected_p(0.0);
  const auto msgs = read_topic(out, "/camera/camera_info");
  ASSERT_EQ(msgs.size(), 2U);
  for (std::size_t i = 0; i < 12; ++i) {
    EXPECT_NEAR(msgs[0].p[i], want[i], 1e-9) << "p[" << i << "]";
  }
}

TEST_F(CamInfoRecomputePTest, BagModeLeavesInputIntactWhenValidationFails)
{
  write_input_bag(input_);

  CamInfoRecomputePArgs args;
  args.input_path = input_;
  args.topics = {"/nope"};
  ASSERT_NE(run_cam_info_recompute_p(args), 0);

  // In-place mode must not have touched the bag on a validation failure.
  const auto msgs = read_topic(input_, "/camera/camera_info");
  ASSERT_EQ(msgs.size(), 2U);
  EXPECT_DOUBLE_EQ(msgs[0].p[0], 1.0);
}

}  // namespace
