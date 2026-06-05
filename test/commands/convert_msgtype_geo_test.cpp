// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/convert_msgtype_geo.hpp"

#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/introspection_loader.hpp"
#include "bagwiz/core/msgtype_convert/geo_pose_convert.hpp"
#include "bagwiz/core/tf_value_extract.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <gtest/gtest.h>
#include <rcutils/allocator.h>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace
{

namespace mtc = bagwiz::core::msgtype_convert;

constexpr const char * kNavSatFixType = "sensor_msgs/msg/NavSatFix";

bagwiz::io::CreateOptions mcap_options()
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = "none";
  return options;
}

// Serialize a NavSatFix to CDR via the introspection typesupport (the same path
// the converter uses to read it back).
std::vector<std::byte> serialize_nav_sat_fix(
  double lat, double lon, double alt, std::int32_t sec, std::uint32_t nsec)
{
  sensor_msgs::msg::NavSatFix msg;
  msg.header.stamp.sec = sec;
  msg.header.stamp.nanosec = nsec;
  msg.header.frame_id = "gnss_link";
  msg.latitude = lat;
  msg.longitude = lon;
  msg.altitude = alt;
  for (std::size_t i = 0; i < 9; ++i) {
    msg.position_covariance[i] = static_cast<double>(i);
  }

  auto intro = bagwiz::core::load_introspection(kNavSatFixType);
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

bagwiz::io::TopicInfo nav_sat_fix_topic_info(const std::string & name)
{
  bagwiz::io::TopicInfo t;
  t.name = name;
  t.type = kNavSatFixType;
  t.serialization_format = "cdr";
  return t;
}

// Input bag: a NavSatFix topic `/fix` (two samples) plus an unrelated `/other`
// topic carrying raw bytes that must survive the conversion verbatim.
constexpr std::array<std::byte, 4> kOtherPayload{
  std::byte{0xDE}, std::byte{0xAD}, std::byte{0xBE}, std::byte{0xEF}};

void write_input_bag(const std::filesystem::path & path)
{
  auto writer = bagwiz::io::open_write(path, mcap_options());
  writer->declare_topic(nav_sat_fix_topic_info("/fix"));

  bagwiz::io::TopicInfo other;
  other.name = "/other";
  other.type = "std_msgs/msg/String";
  other.serialization_format = "cdr";
  writer->declare_topic(other);

  const auto m0 = serialize_nav_sat_fix(35.0, 139.0, 10.0, 100, 0U);
  const auto m1 = serialize_nav_sat_fix(35.001, 139.0, 10.0, 101, 0U);
  writer->write("/fix", 1'000'000'000LL, std::span<const std::byte>(m0.data(), m0.size()));
  writer->write("/fix", 2'000'000'000LL, std::span<const std::byte>(m1.data(), m1.size()));
  writer->write(
    "/other", 1'500'000'000LL,
    std::span<const std::byte>(kOtherPayload.data(), kOtherPayload.size()));
  writer->close();
}

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

struct PoseReadback
{
  int count = 0;
  std::string type;
  std::vector<geometry_msgs::msg::PoseStamped> poses;
};

// Read `/fix` back as PoseStamped (works for both pose targets since we only
// inspect position/header here).
PoseReadback read_fix_as_pose(const std::filesystem::path & path)
{
  PoseReadback result;
  auto reader = bagwiz::io::open_read(path);
  reader->populate_schemas();
  const auto * info = find_topic(*reader, "/fix");
  if (info == nullptr) {
    return result;
  }
  result.type = info->type;
  auto open = bagwiz::core::decoder::open_decoder(*info);
  EXPECT_TRUE(open.ok()) << open.error;

  bagwiz::io::ReadFilter filter;
  filter.topics = {"/fix"};
  reader->set_filter(filter);
  const bool is_covariance = result.type == std::string(mtc::kPoseWithCovarianceStampedType);
  bagwiz::io::RawMessage raw;
  while (reader->next(raw)) {
    if (raw.topic->name != "/fix") {
      continue;
    }
    ++result.count;
    const auto decoded = open.decoder->decode(raw.payload);
    EXPECT_TRUE(decoded.ok()) << decoded.error;
    // Normalise both pose targets to a PoseStamped (header + position) so the
    // assertions stay uniform; the covariance type nests pose under pose.pose.
    if (is_covariance) {
      if (
        const auto pwc =
          bagwiz::core::extract_pose_with_covariance_stamped_message(*decoded.value)) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = pwc->header;
        ps.pose = pwc->pose.pose;
        result.poses.push_back(ps);
      }
    } else if (const auto pose = bagwiz::core::extract_pose_stamped_message(*decoded.value)) {
      result.poses.push_back(*pose);
    }
  }
  return result;
}

// Read back the raw payload of the (single-message) `/other` topic.
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

class ConvertMsgtypeGeoTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_convert_msgtype_geo_" +
                std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()));
    std::filesystem::remove_all(tmp_dir_);
    std::filesystem::create_directories(tmp_dir_);
  }
  void TearDown() override { std::filesystem::remove_all(tmp_dir_); }

  std::filesystem::path tmp_dir_;
};

// -o mode, by --src type: /fix is re-typed to pose_with_covariance_stamped and
// converted; /other is copied verbatim.
TEST_F(ConvertMsgtypeGeoTest, ByTypeToOutputConvertsAndCopies)
{
  const auto in = tmp_dir_ / "in.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_input_bag(in);

  bagwiz::commands::ConvertMsgtypeGeoArgs args;
  args.input_path = in;
  args.src = "nav_sat_fix";
  args.dst = "pose_with_covariance_stamped";
  args.crs = "enu";
  args.origin = "35.0,139.0,10.0";
  args.output_path = out;
  ASSERT_EQ(bagwiz::commands::run_convert_msgtype_geo(args), 0);

  const auto fix = read_fix_as_pose(out);
  EXPECT_EQ(fix.type, std::string(mtc::kPoseWithCovarianceStampedType));
  ASSERT_EQ(fix.count, 2);
  ASSERT_EQ(fix.poses.size(), 2U);
  // First sample sits at the origin -> ~0; default frame_id 'map'; stamp kept.
  EXPECT_NEAR(fix.poses[0].pose.position.x, 0.0, 1e-6);
  EXPECT_NEAR(fix.poses[0].pose.position.y, 0.0, 1e-6);
  EXPECT_EQ(fix.poses[0].header.frame_id, "map");
  EXPECT_EQ(fix.poses[0].header.stamp.sec, 100);
  // Second sample is ~111 m north.
  EXPECT_GT(fix.poses[1].pose.position.y, 100.0);

  std::string other_type;
  const auto payload = read_other_payload(out, other_type);
  EXPECT_EQ(other_type, "std_msgs/msg/String");
  ASSERT_EQ(payload.size(), kOtherPayload.size());
  EXPECT_TRUE(std::equal(payload.begin(), payload.end(), kOtherPayload.begin()));

  // Input bag is untouched in -o mode.
  const auto in_fix = read_fix_as_pose(in);
  EXPECT_EQ(in_fix.type, std::string(kNavSatFixType));
}

// --topic selection with UTM target; --src ignored.
TEST_F(ConvertMsgtypeGeoTest, ByTopicToOutputUsesToAndIgnoresFrom)
{
  const auto in = tmp_dir_ / "in.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_input_bag(in);

  bagwiz::commands::ConvertMsgtypeGeoArgs args;
  args.input_path = in;
  args.topics = {"/fix"};
  args.dst = "pose_stamped";
  args.crs = "utm";
  args.origin = "35.0,139.0,10.0";
  args.frame_id = "utm_local";
  args.output_path = out;
  ASSERT_EQ(bagwiz::commands::run_convert_msgtype_geo(args), 0);

  const auto fix = read_fix_as_pose(out);
  EXPECT_EQ(fix.type, std::string(mtc::kPoseStampedType));
  ASSERT_EQ(fix.poses.size(), 2U);
  EXPECT_NEAR(fix.poses[0].pose.position.x, 0.0, 1e-3);
  EXPECT_NEAR(fix.poses[0].pose.position.y, 0.0, 1e-3);
  EXPECT_EQ(fix.poses[0].header.frame_id, "utm_local");
}

// In-place rewrite (no -o) replaces the input bag.
TEST_F(ConvertMsgtypeGeoTest, InPlaceRewritesInput)
{
  const auto in = tmp_dir_ / "in.mcap";
  write_input_bag(in);

  bagwiz::commands::ConvertMsgtypeGeoArgs args;
  args.input_path = in;
  args.src = "nav_sat_fix";
  args.dst = "pose_stamped";
  args.crs = "enu";
  args.origin = "35.0,139.0,10.0";
  ASSERT_EQ(bagwiz::commands::run_convert_msgtype_geo(args), 0);

  const auto fix = read_fix_as_pose(in);
  EXPECT_EQ(fix.type, std::string(mtc::kPoseStampedType));
  EXPECT_EQ(fix.count, 2);
}

// ENU origin auto-derived from the first NavSatFix when --origin is omitted.
TEST_F(ConvertMsgtypeGeoTest, EnuDerivesOriginFromFirstSample)
{
  const auto in = tmp_dir_ / "in.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_input_bag(in);

  bagwiz::commands::ConvertMsgtypeGeoArgs args;
  args.input_path = in;
  args.src = "nav_sat_fix";
  args.dst = "pose_stamped";
  args.crs = "enu";  // no origin -> first sample (35.0,139.0,10.0)
  args.output_path = out;
  ASSERT_EQ(bagwiz::commands::run_convert_msgtype_geo(args), 0);

  const auto fix = read_fix_as_pose(out);
  ASSERT_EQ(fix.poses.size(), 2U);
  EXPECT_NEAR(fix.poses[0].pose.position.x, 0.0, 1e-6);
  EXPECT_NEAR(fix.poses[0].pose.position.y, 0.0, 1e-6);
}

// --crs is optional and defaults to ENU: omitting it projects to a local
// East-North-Up frame just as `--crs enu` would.
TEST_F(ConvertMsgtypeGeoTest, CrsDefaultsToEnu)
{
  const auto in = tmp_dir_ / "in.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_input_bag(in);

  bagwiz::commands::ConvertMsgtypeGeoArgs args;
  args.input_path = in;
  args.src = "nav_sat_fix";
  args.dst = "pose_stamped";
  // args.crs intentionally left at its default ("enu").
  args.origin = "35.0,139.0,10.0";
  args.output_path = out;
  ASSERT_EQ(bagwiz::commands::run_convert_msgtype_geo(args), 0);

  const auto fix = read_fix_as_pose(out);
  ASSERT_EQ(fix.poses.size(), 2U);
  // First sample == origin -> ENU (0,0,0); default frame_id 'map'.
  EXPECT_NEAR(fix.poses[0].pose.position.x, 0.0, 1e-6);
  EXPECT_NEAR(fix.poses[0].pose.position.y, 0.0, 1e-6);
  EXPECT_EQ(fix.poses[0].header.frame_id, "map");
}

// --overwrite governs clobbering an existing -o path.
TEST_F(ConvertMsgtypeGeoTest, OverwriteGuardsExistingOutput)
{
  const auto in = tmp_dir_ / "in.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_input_bag(in);
  write_input_bag(out);  // pre-existing output

  bagwiz::commands::ConvertMsgtypeGeoArgs args;
  args.input_path = in;
  args.src = "nav_sat_fix";
  args.dst = "pose_stamped";
  args.crs = "enu";
  args.origin = "35.0,139.0,10.0";
  args.output_path = out;

  // Without --overwrite the run stops.
  EXPECT_EQ(bagwiz::commands::run_convert_msgtype_geo(args), 1);
  // With --overwrite it proceeds.
  args.overwrite = true;
  EXPECT_EQ(bagwiz::commands::run_convert_msgtype_geo(args), 0);
}

// --- error paths ---

TEST_F(ConvertMsgtypeGeoTest, MissingFromWithoutTopicFails)
{
  const auto in = tmp_dir_ / "in.mcap";
  write_input_bag(in);

  bagwiz::commands::ConvertMsgtypeGeoArgs args;
  args.input_path = in;
  args.dst = "pose_stamped";
  args.crs = "enu";
  args.origin = "35.0,139.0,10.0";
  EXPECT_EQ(bagwiz::commands::run_convert_msgtype_geo(args), 1);
}

TEST_F(ConvertMsgtypeGeoTest, MixedTopicTypesFails)
{
  const auto in = tmp_dir_ / "in.mcap";
  write_input_bag(in);

  bagwiz::commands::ConvertMsgtypeGeoArgs args;
  args.input_path = in;
  args.topics = {"/fix", "/other"};  // NavSatFix + String -> mixed
  args.dst = "pose_stamped";
  args.crs = "enu";
  args.origin = "35.0,139.0,10.0";
  EXPECT_EQ(bagwiz::commands::run_convert_msgtype_geo(args), 1);
}

TEST_F(ConvertMsgtypeGeoTest, UnsupportedRouteFails)
{
  const auto in = tmp_dir_ / "in.mcap";
  write_input_bag(in);

  // /other is std_msgs/msg/String -> no whitelisted route to a pose type.
  bagwiz::commands::ConvertMsgtypeGeoArgs args;
  args.input_path = in;
  args.topics = {"/other"};
  args.dst = "pose_stamped";
  args.crs = "enu";
  args.origin = "35.0,139.0,10.0";
  EXPECT_EQ(bagwiz::commands::run_convert_msgtype_geo(args), 1);
}

TEST_F(ConvertMsgtypeGeoTest, NoMatchingTopicForFromFails)
{
  const auto in = tmp_dir_ / "in.mcap";
  // A bag with no NavSatFix topic at all.
  {
    auto writer = bagwiz::io::open_write(in, mcap_options());
    bagwiz::io::TopicInfo other;
    other.name = "/other";
    other.type = "std_msgs/msg/String";
    other.serialization_format = "cdr";
    writer->declare_topic(other);
    writer->write(
      "/other", 1'000'000'000LL,
      std::span<const std::byte>(kOtherPayload.data(), kOtherPayload.size()));
    writer->close();
  }

  bagwiz::commands::ConvertMsgtypeGeoArgs args;
  args.input_path = in;
  args.src = "nav_sat_fix";
  args.dst = "pose_stamped";
  args.crs = "enu";
  args.origin = "35.0,139.0,10.0";
  EXPECT_EQ(bagwiz::commands::run_convert_msgtype_geo(args), 1);
}

TEST_F(ConvertMsgtypeGeoTest, MalformedOriginFails)
{
  const auto in = tmp_dir_ / "in.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_input_bag(in);

  bagwiz::commands::ConvertMsgtypeGeoArgs args;
  args.input_path = in;
  args.src = "nav_sat_fix";
  args.dst = "pose_stamped";
  args.crs = "enu";
  args.origin = "not,a,number";
  args.output_path = out;
  EXPECT_EQ(bagwiz::commands::run_convert_msgtype_geo(args), 1);
}

}  // namespace
