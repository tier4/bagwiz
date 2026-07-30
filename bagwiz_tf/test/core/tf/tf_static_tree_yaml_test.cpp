// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf/tf_static_tree_yaml.hpp"

#include <tf2/LinearMath/Quaternion.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <span>
#include <string>
#include <vector>

namespace
{

using bagwiz::core::emit_static_tf_tree_yaml;

geometry_msgs::msg::TransformStamped make_edge(
  const std::string & parent, const std::string & child)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = parent;
  // A stamp the schema has nowhere to put; it must simply not appear.
  ts.header.stamp.sec = 42;
  ts.header.stamp.nanosec = 7;
  ts.child_frame_id = child;
  ts.transform.rotation.w = 1.0;
  return ts;
}

geometry_msgs::msg::TransformStamped make_edge_rpy(
  const std::string & parent, const std::string & child, double roll, double pitch, double yaw)
{
  auto ts = make_edge(parent, child);
  // The exact path a recorded bag's static TF takes: a config's RPY becomes a
  // quaternion in the broadcaster, and the dump has to recover the RPY from it.
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  ts.transform.rotation.x = q.x();
  ts.transform.rotation.y = q.y();
  ts.transform.rotation.z = q.z();
  ts.transform.rotation.w = q.w();
  return ts;
}

std::string emit(const std::vector<geometry_msgs::msg::TransformStamped> & transforms)
{
  return emit_static_tf_tree_yaml(
    std::span<const geometry_msgs::msg::TransformStamped>(transforms.data(), transforms.size()),
    "test.mcap");
}

// Byte offset of `needle` in `haystack`, so tests can assert relative ordering of
// the emitted parent groups.
std::size_t offset_of(const std::string & haystack, const std::string & needle)
{
  const auto pos = haystack.find(needle);
  EXPECT_NE(pos, std::string::npos) << "missing from output: " << needle;
  return pos;
}

TEST(TfStaticTreeYamlTest, EmitsSixKeysPerChildAndNoStamp)
{
  const std::string yaml = emit({make_edge("base_link", "lidar")});

  const YAML::Node doc = YAML::Load(yaml);
  ASSERT_TRUE(doc["base_link"]);
  const YAML::Node child = doc["base_link"]["lidar"];
  ASSERT_TRUE(child);
  EXPECT_EQ(child.size(), 6U);
  for (const char * key : {"x", "y", "z", "roll", "pitch", "yaw"}) {
    EXPECT_TRUE(child[key]) << key;
  }
  // header.stamp has no place in this schema.
  EXPECT_EQ(yaml.find("stamp"), std::string::npos);
  EXPECT_EQ(yaml.find("42"), std::string::npos);
}

// A bare `0` loads as an integer, which a strictly-typed consumer rejects where
// `0.0` is accepted, so every value has to carry a decimal point.
TEST(TfStaticTreeYamlTest, WritesZeroAsAnExplicitFloat)
{
  const std::string yaml = emit({make_edge("base_link", "lidar")});

  EXPECT_NE(yaml.find("    x: 0.0\n"), std::string::npos) << yaml;
  EXPECT_NE(yaml.find("    roll: 0.0\n"), std::string::npos) << yaml;
  EXPECT_EQ(yaml.find(": 0\n"), std::string::npos) << yaml;
}

// getRPY hands back -0.0 for an identity rotation's pitch (it is -asin(0.0)).
// That loads as the same number but reads as a suspicious value in a config.
TEST(TfStaticTreeYamlTest, NormalisesNegativeZero)
{
  auto edge = make_edge("base_link", "lidar");
  edge.transform.translation.z = -0.0;
  const std::string yaml = emit({edge});

  EXPECT_EQ(yaml.find("-0.0"), std::string::npos) << yaml;
  EXPECT_NE(yaml.find("    pitch: 0.0\n"), std::string::npos) << yaml;
  EXPECT_NE(yaml.find("    z: 0.0\n"), std::string::npos) << yaml;
}

// The fixed camera_link -> camera_optical_link rotation that ROS camera stacks
// use. Pinning its RPY pins the convention: tf2 fixed-axis, radians.
TEST(TfStaticTreeYamlTest, ConvertsTheCameraOpticalQuaternionToRightAngles)
{
  auto edge = make_edge("camera0/camera_link", "camera0/camera_optical_link");
  edge.transform.rotation.x = 0.5;
  edge.transform.rotation.y = -0.5;
  edge.transform.rotation.z = 0.5;
  edge.transform.rotation.w = -0.5;

  const YAML::Node doc = YAML::Load(emit({edge}));
  const YAML::Node child = doc["camera0/camera_link"]["camera0/camera_optical_link"];
  ASSERT_TRUE(child);
  constexpr double kHalfPi = std::numbers::pi / 2.0;
  EXPECT_NEAR(child["roll"].as<double>(), -kHalfPi, 1e-12);
  EXPECT_NEAR(child["pitch"].as<double>(), 0.0, 1e-12);
  EXPECT_NEAR(child["yaw"].as<double>(), -kHalfPi, 1e-12);
}

// Round-tripping RPY through a quaternion costs a few ULP, which the shortest
// exact rendering would expose as -0.0027009999999999795. The emitted precision
// has to fold that back to the value the calibration was written with.
TEST(TfStaticTreeYamlTest, FoldsQuaternionRoundTripNoise)
{
  const std::string yaml = emit({
    make_edge_rpy("a", "b", -0.002701, 0.408891, -0.489522),
    make_edge_rpy("b", "c", 0.005816, 0.018911, 1.574117),
  });

  for (const char * expected :
       {"    roll: -0.002701\n", "    pitch: 0.408891\n", "    yaw: -0.489522\n",
        "    roll: 0.005816\n", "    pitch: 0.018911\n", "    yaw: 1.574117\n"}) {
    EXPECT_NE(yaml.find(expected), std::string::npos) << expected << " not in:\n" << yaml;
  }
}

// Translations do not pass through a quaternion, so they must survive verbatim
// rather than be rounded to the same 6 decimals the rotations land on.
TEST(TfStaticTreeYamlTest, KeepsTranslationPrecision)
{
  auto edge = make_edge("base_link", "lidar");
  edge.transform.translation.x = std::numbers::pi;
  edge.transform.translation.y = -0.650337;
  edge.transform.translation.z = 1234.56789012;

  const YAML::Node doc = YAML::Load(emit({edge}));
  const YAML::Node child = doc["base_link"]["lidar"];
  EXPECT_NEAR(child["x"].as<double>(), std::numbers::pi, 1e-13);
  EXPECT_EQ(child["y"].as<double>(), -0.650337);
  EXPECT_EQ(child["z"].as<double>(), 1234.56789012);
}

// Feeding a config back to a publisher goes through setRPY, so the emitted RPY
// must reconstruct the rotation the bag carried.
TEST(TfStaticTreeYamlTest, RoundTripsThroughSetRpy)
{
  const std::vector<std::vector<double>> rpys{
    {0.0, 0.0, 0.0},
    {0.1, -0.2, 0.3},
    {-0.019885, 0.450969, 0.477995},
    {std::numbers::pi, 0.0, -std::numbers::pi / 4.0},
    {0.7, 1.2, -2.9},
  };
  for (const auto & rpy : rpys) {
    const auto edge = make_edge_rpy("p", "c", rpy[0], rpy[1], rpy[2]);
    const YAML::Node child = YAML::Load(emit({edge}))["p"]["c"];
    ASSERT_TRUE(child);

    tf2::Quaternion reloaded;
    reloaded.setRPY(
      child["roll"].as<double>(), child["pitch"].as<double>(), child["yaw"].as<double>());
    const tf2::Quaternion original(
      edge.transform.rotation.x, edge.transform.rotation.y, edge.transform.rotation.z,
      edge.transform.rotation.w);
    // q and -q are the same rotation, so compare the rotations, not the
    // components: |dot| == 1 exactly when they agree.
    EXPECT_NEAR(std::abs(reloaded.dot(original)), 1.0, 1e-12)
      << "rpy " << rpy[0] << ", " << rpy[1] << ", " << rpy[2];
  }
}

TEST(TfStaticTreeYamlTest, OrdersParentGroupsBreadthFirstFromTheRoot)
{
  // Deliberately scrambled: the deepest edge first, the root's edge last.
  const std::string yaml = emit({
    make_edge("lidar_front", "camera0/camera_link"),
    make_edge("drs_base_link", "lidar_front"),
    make_edge("base_link", "drs_base_link"),
  });

  // base_link is the only frame that is never a child, so it heads the file, and
  // each group precedes the groups it introduces.
  EXPECT_LT(offset_of(yaml, "\nbase_link:\n"), offset_of(yaml, "\ndrs_base_link:\n"));
  EXPECT_LT(offset_of(yaml, "\ndrs_base_link:\n"), offset_of(yaml, "\nlidar_front:\n"));
}

TEST(TfStaticTreeYamlTest, EmitsChildrenInFirstSeenOrderWithinAParent)
{
  const std::string yaml = emit({
    make_edge("drs", "lidar_left"),
    make_edge("drs", "lidar_right"),
    make_edge("drs", "lidar_rear"),
  });

  EXPECT_LT(offset_of(yaml, "lidar_left:"), offset_of(yaml, "lidar_right:"));
  EXPECT_LT(offset_of(yaml, "lidar_right:"), offset_of(yaml, "lidar_rear:"));
}

TEST(TfStaticTreeYamlTest, EmitsEveryRootOfAForest)
{
  const YAML::Node doc = YAML::Load(emit({
    make_edge("map_a", "a"),
    make_edge("map_b", "b"),
  }));

  EXPECT_TRUE(doc["map_a"]["a"]);
  EXPECT_TRUE(doc["map_b"]["b"]);
}

// A cycle has no root, so the breadth-first walk reaches nothing. Dropping the
// unreachable groups would silently lose transforms, which a dump must not do.
TEST(TfStaticTreeYamlTest, EmitsEveryGroupWhenTheTreeIsCyclic)
{
  const YAML::Node doc = YAML::Load(emit({
    make_edge("a", "b"),
    make_edge("b", "a"),
  }));

  EXPECT_TRUE(doc["a"]["b"]);
  EXPECT_TRUE(doc["b"]["a"]);
}

// Frame ids come from the bag, so they are untrusted. Each of these would either
// break the document or load as something other than the original string if it
// were emitted as a plain scalar.
TEST(TfStaticTreeYamlTest, QuotesFrameIdsThatWouldNotSurviveAsPlainScalars)
{
  for (const std::string & name :
       {"no", "yes", "true", "NULL", "y", "1.0", "42", "a: b", "#x", "", " leading", "with'quote",
        "with\"quote", "tab\there"}) {
    const std::string yaml = emit({make_edge("base_link", name)});
    YAML::Node doc;
    ASSERT_NO_THROW(doc = YAML::Load(yaml)) << "unparseable for " << name << ":\n" << yaml;
    ASSERT_TRUE(doc["base_link"]) << yaml;
    // The key must come back as the exact original string.
    EXPECT_TRUE(doc["base_link"][name]) << "key lost for '" << name << "':\n" << yaml;
  }
}

TEST(TfStaticTreeYamlTest, LeavesOrdinaryFrameIdsUnquoted)
{
  const std::string yaml = emit({make_edge("base_link", "camera0/camera_optical_link")});

  EXPECT_NE(yaml.find("\nbase_link:\n"), std::string::npos) << yaml;
  EXPECT_NE(yaml.find("  camera0/camera_optical_link:\n"), std::string::npos) << yaml;
  EXPECT_EQ(yaml.find('"'), std::string::npos) << yaml;
}

// A label carrying a newline would otherwise end the `#` comment and let the
// rest of the label be parsed as YAML.
TEST(TfStaticTreeYamlTest, KeepsTheSourceLabelInsideItsComment)
{
  const std::vector<geometry_msgs::msg::TransformStamped> transforms{
    make_edge("base_link", "lidar")};
  const std::string yaml = emit_static_tf_tree_yaml(
    std::span<const geometry_msgs::msg::TransformStamped>(transforms.data(), transforms.size()),
    "bag.mcap\ninjected: 1");

  YAML::Node doc;
  ASSERT_NO_THROW(doc = YAML::Load(yaml)) << yaml;
  EXPECT_FALSE(doc["injected"]) << yaml;
  EXPECT_NE(yaml.find("# Source bag: bag.mcap injected: 1\n"), std::string::npos) << yaml;
}

// Garbage in a bag should still produce a loadable file rather than one whose
// values silently come back as the strings "nan"/"inf".
TEST(TfStaticTreeYamlTest, EmitsNonFiniteValuesAsTypedYamlFloats)
{
  auto edge = make_edge("base_link", "lidar");
  edge.transform.translation.x = std::nan("");
  edge.transform.translation.y = std::numeric_limits<double>::infinity();
  edge.transform.translation.z = -std::numeric_limits<double>::infinity();

  const std::string yaml = emit({edge});
  EXPECT_NE(yaml.find("    x: .nan\n"), std::string::npos) << yaml;
  EXPECT_NE(yaml.find("    y: .inf\n"), std::string::npos) << yaml;
  EXPECT_NE(yaml.find("    z: -.inf\n"), std::string::npos) << yaml;

  const YAML::Node child = YAML::Load(yaml)["base_link"]["lidar"];
  EXPECT_TRUE(std::isnan(child["x"].as<double>()));
  EXPECT_TRUE(std::isinf(child["y"].as<double>()));
}

TEST(TfStaticTreeYamlTest, EmitsAParseableDocumentForNoTransforms)
{
  const std::string yaml = emit({});

  EXPECT_NO_THROW(YAML::Load(yaml));
  EXPECT_NE(yaml.find("# Source bag: test.mcap\n"), std::string::npos) << yaml;
}

}  // namespace
