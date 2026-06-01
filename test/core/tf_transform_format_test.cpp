// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf_transform_format.hpp"

#include <nlohmann/json.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <string>

namespace
{

constexpr double kPi = std::numbers::pi;
constexpr double kSqrtHalf = 0.70710678118654752440;  // cos/sin(pi/4)

geometry_msgs::msg::TransformStamped make_identity_tf(
  const std::string & parent, const std::string & child, double x, double y, double z)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = z;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = 0.0;
  t.transform.rotation.w = 1.0;
  return t;
}

// ---------------------------------------------------------------------------
// quaternion_to_rpy
// ---------------------------------------------------------------------------

TEST(QuaternionToRpy, IdentityIsZero)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = 0.0;
  q.w = 1.0;

  const auto rpy = bagwiz::core::quaternion_to_rpy(q);

  EXPECT_NEAR(rpy.roll, 0.0, 1e-9);
  EXPECT_NEAR(rpy.pitch, 0.0, 1e-9);
  EXPECT_NEAR(rpy.yaw, 0.0, 1e-9);
}

TEST(QuaternionToRpy, NinetyDegreeYaw)
{
  // q for RPY(0, 0, +pi/2) is (0, 0, sin(pi/4), cos(pi/4)).
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = kSqrtHalf;
  q.w = kSqrtHalf;

  const auto rpy = bagwiz::core::quaternion_to_rpy(q);

  EXPECT_NEAR(rpy.roll, 0.0, 1e-9);
  EXPECT_NEAR(rpy.pitch, 0.0, 1e-9);
  EXPECT_NEAR(rpy.yaw, kPi / 2.0, 1e-9);
}

TEST(QuaternionToRpy, ZeroQuaternionTreatedAsIdentity)
{
  // A default-initialised geometry_msgs Quaternion is all zeros, which is
  // not a unit quaternion. quaternion_to_rpy must not return NaN for it.
  geometry_msgs::msg::Quaternion q;

  const auto rpy = bagwiz::core::quaternion_to_rpy(q);

  EXPECT_FALSE(std::isnan(rpy.roll));
  EXPECT_FALSE(std::isnan(rpy.pitch));
  EXPECT_FALSE(std::isnan(rpy.yaw));
  EXPECT_NEAR(rpy.roll, 0.0, 1e-9);
  EXPECT_NEAR(rpy.pitch, 0.0, 1e-9);
  EXPECT_NEAR(rpy.yaw, 0.0, 1e-9);
}

TEST(QuaternionToRpy, NonUnitQuaternionIsNormalized)
{
  // A scalar multiple of a unit quaternion encodes the same rotation. Since
  // tf2::Matrix3x3 does not normalise, quaternion_to_rpy must do so to avoid
  // a skewed matrix. Here 2 * (0,0,sin(pi/4),cos(pi/4)) still means yaw +90.
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = 2.0 * kSqrtHalf;
  q.w = 2.0 * kSqrtHalf;

  const auto rpy = bagwiz::core::quaternion_to_rpy(q);

  EXPECT_NEAR(rpy.roll, 0.0, 1e-9);
  EXPECT_NEAR(rpy.pitch, 0.0, 1e-9);
  EXPECT_NEAR(rpy.yaw, kPi / 2.0, 1e-9);
}

// ---------------------------------------------------------------------------
// format_transform_human
// ---------------------------------------------------------------------------

TEST(FormatTransformHuman, MirrorsJsonKeyHierarchy)
{
  const auto tf = make_identity_tf("base_link", "velodyne", 1.0, 2.0, 3.0);

  const std::string out = bagwiz::core::format_transform_human(tf, "base_link", "velodyne");

  // Direction label is "<from> -> <to>".
  EXPECT_NE(out.find("base_link -> velodyne"), std::string::npos);
  // Body uses the --json key/hierarchy: t {x,y,z}, r {quat, rpy_rad, rpy_deg}.
  EXPECT_NE(out.find("  t:"), std::string::npos);
  EXPECT_NE(out.find("  r:"), std::string::npos);
  EXPECT_NE(out.find("    quat:"), std::string::npos);
  EXPECT_NE(out.find("    rpy_rad:"), std::string::npos);
  EXPECT_NE(out.find("    rpy_deg:"), std::string::npos);
  // Translation values (x, y, z) appear under t.
  EXPECT_NE(out.find("1.000000"), std::string::npos);
  EXPECT_NE(out.find("2.000000"), std::string::npos);
  EXPECT_NE(out.find("3.000000"), std::string::npos);
  // Trailing newline is part of the contract.
  ASSERT_FALSE(out.empty());
  EXPECT_EQ(out.back(), '\n');
}

TEST(FormatTransformHuman, IdentitySelfTransform)
{
  // from == to resolves to the identity transform; the formatter must still
  // render a well-formed block labelled "<frame> -> <frame>".
  const auto tf = make_identity_tf("lidar", "lidar", 0.0, 0.0, 0.0);

  const std::string out = bagwiz::core::format_transform_human(tf, "lidar", "lidar");

  EXPECT_NE(out.find("lidar -> lidar"), std::string::npos);
  EXPECT_NE(out.find("0.000000"), std::string::npos);
  ASSERT_FALSE(out.empty());
  EXPECT_EQ(out.back(), '\n');
}

// `tf static` passes an annotation so the direction line is tagged "(static)";
// the annotation appears right after the "<from> -> <to>" label.
TEST(FormatTransformHuman, AppendsAnnotationToDirectionLine)
{
  const auto tf = make_identity_tf("base_link", "lidar", 1.0, 0.0, 0.0);

  const std::string out =
    bagwiz::core::format_transform_human(tf, "base_link", "lidar", "  (static)");

  EXPECT_NE(out.find("base_link -> lidar  (static)"), std::string::npos);
}

// `tf walk` does not classify transforms, so the default (no annotation) must
// not tag the block with a "(static)" suffix.
TEST(FormatTransformHuman, OmitsAnnotationByDefault)
{
  const auto tf = make_identity_tf("base_link", "velodyne", 1.0, 2.0, 3.0);

  const std::string out = bagwiz::core::format_transform_human(tf, "base_link", "velodyne");

  EXPECT_EQ(out.find("(static)"), std::string::npos);
}

// ---------------------------------------------------------------------------
// format_transform_json (parsed back with nlohmann to verify values)
// ---------------------------------------------------------------------------

TEST(FormatTransformJson, RoundTripsExpectedSchemaAndValues)
{
  const auto tf = make_identity_tf("map", "odom", 1.5, -2.5, 0.25);

  const std::string out = bagwiz::core::format_transform_json(tf, "map", "odom");
  const auto j = nlohmann::json::parse(out);

  EXPECT_EQ(j.at("from").get<std::string>(), "map");
  EXPECT_EQ(j.at("to").get<std::string>(), "odom");

  EXPECT_DOUBLE_EQ(j.at("t").at("x").get<double>(), 1.5);
  EXPECT_DOUBLE_EQ(j.at("t").at("y").get<double>(), -2.5);
  EXPECT_DOUBLE_EQ(j.at("t").at("z").get<double>(), 0.25);

  EXPECT_DOUBLE_EQ(j.at("r").at("quat").at("x").get<double>(), 0.0);
  EXPECT_DOUBLE_EQ(j.at("r").at("quat").at("y").get<double>(), 0.0);
  EXPECT_DOUBLE_EQ(j.at("r").at("quat").at("z").get<double>(), 0.0);
  EXPECT_DOUBLE_EQ(j.at("r").at("quat").at("w").get<double>(), 1.0);

  EXPECT_NEAR(j.at("r").at("rpy_rad").at("yaw").get<double>(), 0.0, 1e-9);
  EXPECT_NEAR(j.at("r").at("rpy_deg").at("yaw").get<double>(), 0.0, 1e-9);
}

TEST(FormatTransformJson, DegreesAreRadiansScaled)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.transform.rotation.x = 0.0;
  tf.transform.rotation.y = 0.0;
  tf.transform.rotation.z = kSqrtHalf;  // +pi/2 yaw
  tf.transform.rotation.w = kSqrtHalf;

  const std::string out = bagwiz::core::format_transform_json(tf, "a", "b");
  const auto j = nlohmann::json::parse(out);

  EXPECT_NEAR(j.at("r").at("rpy_rad").at("yaw").get<double>(), kPi / 2.0, 1e-9);
  EXPECT_NEAR(j.at("r").at("rpy_deg").at("yaw").get<double>(), 90.0, 1e-6);
}

}  // namespace
