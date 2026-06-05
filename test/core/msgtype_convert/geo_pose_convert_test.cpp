// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/msgtype_convert/geo_pose_convert.hpp"

#include "bagwiz/core/cdr_walker/value.hpp"
#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/tf_value_extract.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace
{

namespace mtc = bagwiz::core::msgtype_convert;
namespace cdr = bagwiz::core::cdr_walker;

// --- helpers to build a decoded NavSatFix Value tree ---

cdr::Value make_nav_sat_fix_value(
  double lat, double lon, double alt, std::int32_t sec = 0, std::uint32_t nsec = 0,
  const std::array<double, 9> & cov = {})
{
  cdr::Object stamp;
  stamp.fields.emplace_back("sec", cdr::Value{sec});
  stamp.fields.emplace_back("nanosec", cdr::Value{nsec});

  cdr::Object header;
  header.fields.emplace_back("stamp", cdr::Value{stamp});
  header.fields.emplace_back("frame_id", cdr::Value{std::string("gnss_link")});

  cdr::Sequence cov_seq;
  for (const double c : cov) {
    cov_seq.elements.emplace_back(c);
  }

  cdr::Object root;
  root.fields.emplace_back("header", cdr::Value{header});
  root.fields.emplace_back("latitude", cdr::Value{lat});
  root.fields.emplace_back("longitude", cdr::Value{lon});
  root.fields.emplace_back("altitude", cdr::Value{alt});
  root.fields.emplace_back("position_covariance", cdr::Value{cov_seq});
  return cdr::Value{root};
}

// Decode a serialized payload of `type` via the introspection backend (empty
// schema_text routes the factory there, dlopening the type's typesupport).
std::optional<cdr::Value> decode_payload(
  std::span<const std::byte> payload, const std::string & type)
{
  bagwiz::io::TopicInfo info;
  info.name = "/x";
  info.type = type;
  info.serialization_format = "cdr";
  auto open = bagwiz::core::decoder::open_decoder(info);
  if (!open.ok()) {
    return std::nullopt;
  }
  auto decoded = open.decoder->decode(payload);
  if (!decoded.ok()) {
    return std::nullopt;
  }
  return std::move(decoded.value);
}

// Pull pose.covariance (36 doubles) out of a decoded PoseWithCovarianceStamped.
std::optional<std::array<double, 36>> read_pose_covariance(const cdr::Value & v)
{
  const auto * root = std::get_if<cdr::Object>(&v.v);
  if (root == nullptr) {
    return std::nullopt;
  }
  const cdr::Object * pose = nullptr;
  for (const auto & f : root->fields) {
    if (f.first == "pose") {
      pose = std::get_if<cdr::Object>(&f.second.v);
    }
  }
  if (pose == nullptr) {
    return std::nullopt;
  }
  const cdr::Sequence * cov = nullptr;
  for (const auto & f : pose->fields) {
    if (f.first == "covariance") {
      cov = std::get_if<cdr::Sequence>(&f.second.v);
    }
  }
  if (cov == nullptr || cov->elements.size() != 36) {
    return std::nullopt;
  }
  std::array<double, 36> out{};
  for (std::size_t i = 0; i < 36; ++i) {
    if (const auto * d = std::get_if<double>(&cov->elements[i].v)) {
      out[i] = *d;
    }
  }
  return out;
}

// --- whitelist / mapping ---

TEST(GeoRoute, FromChoicesIsNavSatFixOnly)
{
  const auto & from = mtc::from_snake_choices();
  ASSERT_EQ(from.size(), 1U);
  EXPECT_EQ(from[0], "nav_sat_fix");
}

TEST(GeoRoute, ToChoicesAreThePoseFamily)
{
  const auto & to = mtc::to_snake_choices();
  ASSERT_EQ(to.size(), 2U);
  EXPECT_EQ(to[0], "pose_with_covariance_stamped");
  EXPECT_EQ(to[1], "pose_stamped");
}

TEST(GeoRoute, SnakeMapsToRosType)
{
  EXPECT_EQ(mtc::from_snake_to_ros_type("nav_sat_fix"), std::string(mtc::kNavSatFixType));
  EXPECT_EQ(
    mtc::to_snake_to_ros_type("pose_with_covariance_stamped"),
    std::string(mtc::kPoseWithCovarianceStampedType));
  EXPECT_EQ(mtc::to_snake_to_ros_type("pose_stamped"), std::string(mtc::kPoseStampedType));
  EXPECT_FALSE(mtc::from_snake_to_ros_type("nope").has_value());
}

TEST(GeoRoute, RosTypeMapsBackToSnake)
{
  EXPECT_EQ(mtc::ros_type_to_snake(mtc::kNavSatFixType), "nav_sat_fix");
  EXPECT_EQ(mtc::ros_type_to_snake(mtc::kPoseStampedType), "pose_stamped");
  EXPECT_FALSE(mtc::ros_type_to_snake("sensor_msgs/msg/Imu").has_value());
}

TEST(GeoRoute, WhitelistAllowsNavSatFixToPoseFamily)
{
  const auto a = mtc::resolve_route(mtc::kNavSatFixType, mtc::kPoseWithCovarianceStampedType);
  ASSERT_TRUE(a.has_value());
  EXPECT_TRUE(a->target_has_covariance);

  const auto b = mtc::resolve_route(mtc::kNavSatFixType, mtc::kPoseStampedType);
  ASSERT_TRUE(b.has_value());
  EXPECT_FALSE(b->target_has_covariance);
}

TEST(GeoRoute, WhitelistRejectsReverseAndUnknown)
{
  EXPECT_FALSE(
    mtc::resolve_route(mtc::kPoseWithCovarianceStampedType, mtc::kNavSatFixType).has_value());
  EXPECT_FALSE(mtc::resolve_route("sensor_msgs/msg/Imu", mtc::kPoseStampedType).has_value());
}

// --- extract_nav_sat_fix ---

TEST(ExtractNavSatFix, ReadsAllFields)
{
  std::array<double, 9> cov{};
  for (std::size_t i = 0; i < 9; ++i) {
    cov[i] = static_cast<double>(i) + 0.5;
  }
  const auto v = make_nav_sat_fix_value(35.5, 139.5, 12.0, 42, 7U, cov);
  const auto s = mtc::extract_nav_sat_fix(v);
  ASSERT_TRUE(s.has_value());
  EXPECT_DOUBLE_EQ(s->latitude, 35.5);
  EXPECT_DOUBLE_EQ(s->longitude, 139.5);
  EXPECT_DOUBLE_EQ(s->altitude, 12.0);
  EXPECT_EQ(s->stamp_sec, 42);
  EXPECT_EQ(s->stamp_nanosec, 7U);
  for (std::size_t i = 0; i < 9; ++i) {
    EXPECT_DOUBLE_EQ(s->position_covariance[i], static_cast<double>(i) + 0.5);
  }
}

TEST(ExtractNavSatFix, RejectsTreeMissingLatitude)
{
  cdr::Object root;
  root.fields.emplace_back("longitude", cdr::Value{1.0});
  root.fields.emplace_back("altitude", cdr::Value{2.0});
  EXPECT_FALSE(mtc::extract_nav_sat_fix(cdr::Value{root}).has_value());
}

// --- conversion + round-trip ---

TEST(GeoPoseConverter, EnuOriginAtSampleProjectsToZero)
{
  mtc::GeoConvertConfig cfg;
  cfg.crs = mtc::GeoCrs::kEnu;
  cfg.origin = mtc::GeoOrigin{35.0, 139.0, 10.0};
  cfg.frame_id = "map";
  cfg.target_ros_type = mtc::kPoseWithCovarianceStampedType;
  cfg.target_has_covariance = true;
  mtc::GeoPoseConverter converter(cfg);

  const auto v = make_nav_sat_fix_value(35.0, 139.0, 10.0, 5, 0U);
  const auto sample = mtc::extract_nav_sat_fix(v);
  ASSERT_TRUE(sample.has_value());
  const auto payload = converter.convert(*sample);

  const auto decoded = decode_payload(payload, mtc::kPoseWithCovarianceStampedType);
  ASSERT_TRUE(decoded.has_value());
  const auto pose = bagwiz::core::extract_pose_with_covariance_stamped_message(*decoded);
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->pose.pose.position.x, 0.0, 1e-6);
  EXPECT_NEAR(pose->pose.pose.position.y, 0.0, 1e-6);
  EXPECT_NEAR(pose->pose.pose.position.z, 0.0, 1e-6);
  EXPECT_DOUBLE_EQ(pose->pose.pose.orientation.w, 1.0);
  EXPECT_EQ(pose->header.frame_id, "map");
  EXPECT_EQ(pose->header.stamp.sec, 5);
}

TEST(GeoPoseConverter, EnuNorthwardOffsetMapsToPositiveY)
{
  mtc::GeoConvertConfig cfg;
  cfg.crs = mtc::GeoCrs::kEnu;
  cfg.origin = mtc::GeoOrigin{35.0, 139.0, 0.0};
  cfg.frame_id = "map";
  cfg.target_ros_type = mtc::kPoseStampedType;
  mtc::GeoPoseConverter converter(cfg);

  // ~0.001 deg north ~= 111 m; east ~= 0.
  const auto v = make_nav_sat_fix_value(35.001, 139.0, 0.0);
  const auto sample = mtc::extract_nav_sat_fix(v);
  ASSERT_TRUE(sample.has_value());
  const auto payload = converter.convert(*sample);

  const auto decoded = decode_payload(payload, mtc::kPoseStampedType);
  ASSERT_TRUE(decoded.has_value());
  const auto pose = bagwiz::core::extract_pose_stamped_message(*decoded);
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->pose.position.x, 0.0, 1.0);
  EXPECT_GT(pose->pose.position.y, 100.0);
  EXPECT_LT(pose->pose.position.y, 120.0);
}

TEST(GeoPoseConverter, CovarianceMapsIntoUpperLeftBlock)
{
  std::array<double, 9> cov{};
  for (std::size_t i = 0; i < 9; ++i) {
    cov[i] = static_cast<double>(i) + 1.0;  // 1..9, non-zero so mapping is visible
  }
  mtc::GeoConvertConfig cfg;
  cfg.crs = mtc::GeoCrs::kEnu;
  cfg.origin = mtc::GeoOrigin{0.0, 0.0, 0.0};
  cfg.frame_id = "map";
  cfg.target_ros_type = mtc::kPoseWithCovarianceStampedType;
  cfg.target_has_covariance = true;
  mtc::GeoPoseConverter converter(cfg);

  const auto v = make_nav_sat_fix_value(0.0, 0.0, 0.0, 0, 0U, cov);
  const auto sample = mtc::extract_nav_sat_fix(v);
  ASSERT_TRUE(sample.has_value());
  const auto payload = converter.convert(*sample);

  const auto decoded = decode_payload(payload, mtc::kPoseWithCovarianceStampedType);
  ASSERT_TRUE(decoded.has_value());
  const auto out = read_pose_covariance(*decoded);
  ASSERT_TRUE(out.has_value());
  // 3x3 in[i*3+j] -> 6x6 out[i*6+j]; rotation block stays zero.
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      EXPECT_DOUBLE_EQ((*out)[(i * 6) + j], cov[(i * 3) + j]);
    }
  }
  EXPECT_DOUBLE_EQ((*out)[21], 0.0);  // rotation diagonal (index 3,3)
  EXPECT_DOUBLE_EQ((*out)[35], 0.0);  // rotation diagonal (index 5,5)
}

TEST(GeoPoseConverter, UtmOriginAtSampleProjectsToZero)
{
  mtc::GeoConvertConfig cfg;
  cfg.crs = mtc::GeoCrs::kUtm;
  cfg.origin = mtc::GeoOrigin{35.0, 139.0, 50.0};
  cfg.frame_id = "utm";
  cfg.target_ros_type = mtc::kPoseStampedType;
  mtc::GeoPoseConverter converter(cfg);

  const auto v = make_nav_sat_fix_value(35.0, 139.0, 50.0);
  const auto sample = mtc::extract_nav_sat_fix(v);
  ASSERT_TRUE(sample.has_value());
  const auto payload = converter.convert(*sample);

  const auto decoded = decode_payload(payload, mtc::kPoseStampedType);
  ASSERT_TRUE(decoded.has_value());
  const auto pose = bagwiz::core::extract_pose_stamped_message(*decoded);
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->pose.position.x, 0.0, 1e-3);
  EXPECT_NEAR(pose->pose.position.y, 0.0, 1e-3);
  EXPECT_NEAR(pose->pose.position.z, 0.0, 1e-6);
}

TEST(GeoPoseConverter, EnuWithoutOriginThrows)
{
  mtc::GeoConvertConfig cfg;
  cfg.crs = mtc::GeoCrs::kEnu;
  cfg.frame_id = "map";
  cfg.target_ros_type = mtc::kPoseStampedType;
  EXPECT_THROW({ mtc::GeoPoseConverter converter(cfg); }, std::invalid_argument);
}

}  // namespace
