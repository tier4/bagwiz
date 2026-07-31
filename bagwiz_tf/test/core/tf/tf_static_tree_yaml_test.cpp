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
#include <filesystem>
#include <fstream>
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
  const std::vector<std::vector<double>> rpy_cases{
    {0.0, 0.0, 0.0},
    {0.1, -0.2, 0.3},
    {-0.019885, 0.450969, 0.477995},
    {std::numbers::pi, 0.0, -std::numbers::pi / 4.0},
    {0.7, 1.2, -2.9},
  };
  for (const auto & rpy : rpy_cases) {
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
  for (const char * raw :
       {"no", "yes", "true", "NULL", "y", "1.0", "42", "a: b", "#x", "", " leading", "with'quote",
        "with\"quote", "tab\there"}) {
    const std::string name(raw);
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

// ---------------------------------------------------------------------------
// parse_static_tf_tree_yaml
// ---------------------------------------------------------------------------

using bagwiz::core::parse_static_tf_tree_yaml;

class StaticTfTreeParseTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dir_ = std::filesystem::temp_directory_path() /
           ("bagwiz_tf_tree_parse_" +
            std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()));
    std::filesystem::remove_all(dir_);
    std::filesystem::create_directories(dir_);
  }
  void TearDown() override { std::filesystem::remove_all(dir_); }

  std::filesystem::path write(const std::string & contents) const
  {
    const auto path = dir_ / "tree.yaml";
    std::ofstream(path) << contents;
    return path;
  }

  // Parse `contents` and return the error, asserting that it failed.
  std::string error_for(const std::string & contents) const
  {
    const auto result = parse_static_tf_tree_yaml(write(contents));
    EXPECT_FALSE(result.ok()) << "expected a rejection for:\n" << contents;
    EXPECT_FALSE(result.transforms.has_value());
    return result.error;
  }

  std::filesystem::path dir_;
};

TEST_F(StaticTfTreeParseTest, ReadsTheEmittedSchema)
{
  const auto result = parse_static_tf_tree_yaml(write(
    "base_link:\n"
    "  drs_base_link:\n"
    "    x: 0.796\n"
    "    y: 0.0\n"
    "    z: 1.826\n"
    "    roll: 0.0\n"
    "    pitch: 0.0\n"
    "    yaw: 1.5707963267948966\n"));

  ASSERT_TRUE(result.ok()) << result.error;
  ASSERT_EQ(result.transforms->size(), 1U);
  const auto & t = result.transforms->front();
  EXPECT_EQ(t.header.frame_id, "base_link");
  EXPECT_EQ(t.child_frame_id, "drs_base_link");
  EXPECT_DOUBLE_EQ(t.transform.translation.x, 0.796);
  EXPECT_DOUBLE_EQ(t.transform.translation.z, 1.826);
  // The schema has no stamp; the caller supplies one for the bag it writes into.
  EXPECT_EQ(t.header.stamp.sec, 0);
  EXPECT_EQ(t.header.stamp.nanosec, 0U);
  // yaw = pi/2 about z.
  EXPECT_NEAR(t.transform.rotation.z, std::sin(std::numbers::pi / 4.0), 1e-12);
  EXPECT_NEAR(t.transform.rotation.w, std::cos(std::numbers::pi / 4.0), 1e-12);
}

// Recovering RPY from a quaternion cannot hold an exact zero next to a right
// angle: the camera_optical rotation (roll = -pi/2, yaw = -pi/2) returns
// pitch = -5.55e-17. Without the angle floor, a config written by `dump`, joined
// into a bag, and dumped again would not match itself.
TEST_F(StaticTfTreeParseTest, FoldsSubResolutionAnglesToZeroSoRightAnglesRoundTrip)
{
  auto optical = make_edge("camera0/camera_link", "camera0/camera_optical_link");
  optical.transform.rotation.x = 0.5;
  optical.transform.rotation.y = -0.5;
  optical.transform.rotation.z = 0.5;
  optical.transform.rotation.w = -0.5;

  const std::string first = emit({optical});
  EXPECT_NE(first.find("    pitch: 0.0\n"), std::string::npos) << first;

  const auto parsed = parse_static_tf_tree_yaml(write(first));
  ASSERT_TRUE(parsed.ok()) << parsed.error;
  EXPECT_EQ(emit(*parsed.transforms), first);

  // Folding that component must not have moved the rotation itself.
  const auto & q = parsed.transforms->front().transform.rotation;
  const tf2::Quaternion reloaded(q.x, q.y, q.z, q.w);
  const tf2::Quaternion original(0.5, -0.5, 0.5, -0.5);
  EXPECT_NEAR(std::abs(reloaded.dot(original)), 1.0, 1e-12);
}

// The reason the two functions live together: a config written by `dump` must
// read back to the same transforms, and re-emit to the same bytes.
TEST_F(StaticTfTreeParseTest, EmitParseEmitIsAFixedPoint)
{
  const std::vector<geometry_msgs::msg::TransformStamped> original{
    make_edge_rpy("base_link", "drs_base_link", 0.0, 0.0, 0.0),
    make_edge_rpy("drs_base_link", "lidar_left", -0.019885, 0.450969, 0.477995),
    make_edge_rpy("lidar_left", "camera6/camera_link", 0.1, -0.2, 0.3),
  };
  const std::string first = emit(original);

  const auto parsed = parse_static_tf_tree_yaml(write(first));
  ASSERT_TRUE(parsed.ok()) << parsed.error;
  ASSERT_EQ(parsed.transforms->size(), original.size());

  const std::string second = emit(*parsed.transforms);
  EXPECT_EQ(second, first);

  // And the rotations survive as rotations, not just as text.
  for (std::size_t i = 0; i < original.size(); ++i) {
    const auto & a = original[i].transform.rotation;
    const auto & b = (*parsed.transforms)[i].transform.rotation;
    const tf2::Quaternion qa(a.x, a.y, a.z, a.w);
    const tf2::Quaternion qb(b.x, b.y, b.z, b.w);
    EXPECT_NEAR(std::abs(qa.dot(qb)), 1.0, 1e-12) << "edge " << i;
  }
}

TEST_F(StaticTfTreeParseTest, ReadsAMultiParentTree)
{
  const auto result = parse_static_tf_tree_yaml(write(
    "base_link:\n"
    "  a:\n"
    "    x: 1.0\n    y: 0.0\n    z: 0.0\n    roll: 0.0\n    pitch: 0.0\n    yaw: 0.0\n"
    "  b:\n"
    "    x: 2.0\n    y: 0.0\n    z: 0.0\n    roll: 0.0\n    pitch: 0.0\n    yaw: 0.0\n"
    "a:\n"
    "  c:\n"
    "    x: 3.0\n    y: 0.0\n    z: 0.0\n    roll: 0.0\n    pitch: 0.0\n    yaw: 0.0\n"));

  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(result.transforms->size(), 3U);
}

TEST_F(StaticTfTreeParseTest, RejectsAMissingFile)
{
  const auto result = parse_static_tf_tree_yaml(dir_ / "absent.yaml");
  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.error.find("failed to parse"), std::string::npos) << result.error;
}

TEST_F(StaticTfTreeParseTest, RejectsMalformedYaml)
{
  EXPECT_NE(
    error_for("base_link:\n  - not: a mapping\n   bad indent\n").find("failed to parse"),
    std::string::npos);
}

TEST_F(StaticTfTreeParseTest, RejectsANonMappingDocument)
{
  EXPECT_NE(error_for("- a\n- b\n").find("not a top-level mapping"), std::string::npos);
  EXPECT_NE(error_for("").find("empty or not a top-level mapping"), std::string::npos);
}

TEST_F(StaticTfTreeParseTest, RejectsAMissingKey)
{
  // Every one of the six is required: a pose missing one is underspecified, and
  // defaulting it to 0 would invent a transform the author did not write.
  for (const char * omit : {"x", "y", "z", "roll", "pitch", "yaw"}) {
    std::string body;
    for (const char * key : {"x", "y", "z", "roll", "pitch", "yaw"}) {
      if (std::string(key) != omit) {
        body += std::string("    ") + key + ": 0.0\n";
      }
    }
    const std::string err = error_for("base_link:\n  lidar:\n" + body);
    EXPECT_NE(err.find(std::string("missing key '") + omit + "'"), std::string::npos) << err;
  }
}

// A typo would otherwise leave the mistyped axis silently at 0. The fixture needs a
// real misspelling, so the spell checker is told to expect this one.
// cspell:ignore pich
TEST_F(StaticTfTreeParseTest, RejectsAnUnknownKey)
{
  const std::string err = error_for(
    "base_link:\n  lidar:\n"
    "    x: 0.0\n    y: 0.0\n    z: 0.0\n    roll: 0.0\n    pitch: 0.0\n    yaw: 0.0\n"
    "    pich: 0.5\n");
  EXPECT_NE(err.find("unknown key 'pich'"), std::string::npos) << err;
}

// `.nan` and `.inf` are valid YAML floats, so they parse fine — but tf2 drops
// such a transform, leaving a well-formed /tf_static whose tree is empty. The
// tree-buildable check is what catches it. (The emitter writes these spellings for
// a corrupt bag, so this is also the path a dump -> join of a corrupt bag takes.)
TEST_F(StaticTfTreeParseTest, RejectsValuesTf2WouldDrop)
{
  for (const char * bad : {".nan", ".inf", "-.inf"}) {
    const std::string err = error_for(
      std::string("base_link:\n  lidar:\n    x: ") + bad +
      "\n    y: 0.0\n    z: 0.0\n    roll: 0.0\n    pitch: 0.0\n    yaw: 0.0\n");
    EXPECT_NE(err.find("non-finite translation.x"), std::string::npos) << bad << ": " << err;
  }
  // A non-finite angle reaches the rotation through rpy_to_quaternion.
  const std::string rot = error_for(
    "base_link:\n  lidar:\n"
    "    x: 0.0\n    y: 0.0\n    z: 0.0\n    roll: .nan\n    pitch: 0.0\n    yaw: 0.0\n");
  EXPECT_NE(rot.find("non-finite rotation"), std::string::npos) << rot;
}

TEST_F(StaticTfTreeParseTest, RejectsANonNumericValue)
{
  const std::string err = error_for(
    "base_link:\n  lidar:\n"
    "    x: forward\n    y: 0.0\n    z: 0.0\n    roll: 0.0\n    pitch: 0.0\n    yaw: 0.0\n");
  EXPECT_NE(err.find("key 'x' must be a number"), std::string::npos) << err;
}

// Nesting deeper than two levels is a grouping heading, matching the reference
// publisher: only the level immediately above a transform names its parent. This
// is what lets a large rig config be split into sections.
TEST_F(StaticTfTreeParseTest, AcceptsGroupingLevelsAtAnyDepth)
{
  const auto result = parse_static_tf_tree_yaml(write(
    "sensors:\n"
    "  base_link:\n"
    "    drs_base_link:\n"
    "      x: 1.0\n      y: 0.0\n      z: 0.0\n      roll: 0.0\n      pitch: 0.0\n      yaw: 0.0\n"
    "  drs_base_link:\n"
    "    lidar_front:\n"
    "      x: 2.0\n      y: 0.0\n      z: 0.0\n      roll: 0.0\n      pitch: 0.0\n      yaw: "
    "0.0\n"));

  ASSERT_TRUE(result.ok()) << result.error;
  ASSERT_EQ(result.transforms->size(), 2U);
  // `sensors` is a heading, so it parents nothing and the edges come from the
  // level directly above each transform.
  EXPECT_EQ((*result.transforms)[0].header.frame_id, "base_link");
  EXPECT_EQ((*result.transforms)[0].child_frame_id, "drs_base_link");
  EXPECT_EQ((*result.transforms)[1].header.frame_id, "drs_base_link");
  EXPECT_EQ((*result.transforms)[1].child_frame_id, "lidar_front");
  EXPECT_EQ(result.grouping_frames, std::vector<std::string>{"sensors"});
}

// Arbitrary depth, and the headings are reported innermost-first as the walk
// unwinds.
TEST_F(StaticTfTreeParseTest, AcceptsFourLevelsAndReportsEveryHeading)
{
  const auto result = parse_static_tf_tree_yaml(write(
    "a:\n  b:\n    c:\n      d:\n"
    "        x: 0.0\n        y: 0.0\n        z: 0.0\n"
    "        roll: 0.0\n        pitch: 0.0\n        yaw: 0.0\n"));

  ASSERT_TRUE(result.ok()) << result.error;
  ASSERT_EQ(result.transforms->size(), 1U);
  // Only `c` -> `d` is a transform; `a` and `b` are headings.
  EXPECT_EQ(result.transforms->front().header.frame_id, "c");
  EXPECT_EQ(result.transforms->front().child_frame_id, "d");
  EXPECT_EQ(result.grouping_frames, (std::vector<std::string>{"b", "a"}));
}

// The two-level form `dump` writes has no headings, so nothing is reported and a
// caller stays quiet.
TEST_F(StaticTfTreeParseTest, ReportsNoGroupingFramesForTheTwoLevelForm)
{
  const auto result = parse_static_tf_tree_yaml(write(
    "base_link:\n  lidar:\n"
    "    x: 0.0\n    y: 0.0\n    z: 0.0\n    roll: 0.0\n    pitch: 0.0\n    yaw: 0.0\n"));

  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_TRUE(result.grouping_frames.empty());
}

// A transform mapping that ALSO nests a child is depth 3 in disguise. The six
// keys are present, so the nesting check passes and the unknown-key check catches
// it instead.
TEST_F(StaticTfTreeParseTest, RejectsANestedChildBesideTheTransformKeys)
{
  const std::string err = error_for(
    "base_link:\n  lidar:\n"
    "    x: 0.0\n    y: 0.0\n    z: 0.0\n    roll: 0.0\n    pitch: 0.0\n    yaw: 0.0\n"
    "    camera:\n      x: 1.0\n");
  EXPECT_NE(err.find("unknown key 'camera'"), std::string::npos) << err;
}

// One level too shallow: forgetting the parent frame. Reporting the first of the
// six keys as a malformed child frame would point at the wrong thing. (The
// reference publisher instead broadcasts this with an empty parent frame id.)
TEST_F(StaticTfTreeParseTest, RejectsATransformWithNoParentFrame)
{
  const std::string err =
    error_for("lidar:\n  x: 0.0\n  y: 0.0\n  z: 0.0\n  roll: 0.0\n  pitch: 0.0\n  yaw: 0.0\n");
  EXPECT_NE(err.find("declares a transform at the top level"), std::string::npos) << err;
  EXPECT_NE(err.find("needs a parent frame above it"), std::string::npos) << err;
}

TEST_F(StaticTfTreeParseTest, RejectsAScalarWhereAMappingBelongs)
{
  EXPECT_NE(error_for("base_link: 5\n").find("must be a mapping"), std::string::npos);
  EXPECT_NE(error_for("base_link:\n  lidar: 5\n").find("must be a mapping"), std::string::npos);
}

TEST_F(StaticTfTreeParseTest, RejectsAnEmptyMapping)
{
  EXPECT_NE(
    error_for("base_link: {}\n").find("declares neither a transform nor any child frames"),
    std::string::npos);
}

TEST_F(StaticTfTreeParseTest, RejectsNestingBeyondTheDepthCap)
{
  // 40 levels of grouping, then a transform. Legal in shape but past the guard.
  std::string doc;
  for (int i = 0; i < 40; ++i) {
    doc += std::string(static_cast<std::size_t>(i) * 2, ' ') + "f" + std::to_string(i) + ":\n";
  }
  EXPECT_NE(error_for(doc).find("nesting is deeper than"), std::string::npos);
}

TEST_F(StaticTfTreeParseTest, RejectsASelfEdge)
{
  const std::string err = error_for(
    "base_link:\n  base_link:\n"
    "    x: 0.0\n    y: 0.0\n    z: 0.0\n    roll: 0.0\n    pitch: 0.0\n    yaw: 0.0\n");
  EXPECT_NE(err.find("is its own parent"), std::string::npos) << err;
}

// The structural checks come from validate_tf_forest, the same validation
// `bagwiz tf tree` applies to a bag's merged tree.
TEST_F(StaticTfTreeParseTest, RejectsAChildWithTwoParents)
{
  const std::string err = error_for(
    "a:\n  shared:\n"
    "    x: 0.0\n    y: 0.0\n    z: 0.0\n    roll: 0.0\n    pitch: 0.0\n    yaw: 0.0\n"
    "b:\n  shared:\n"
    "    x: 1.0\n    y: 0.0\n    z: 0.0\n    roll: 0.0\n    pitch: 0.0\n    yaw: 0.0\n");
  EXPECT_NE(err.find("has parent"), std::string::npos) << err;
}

TEST_F(StaticTfTreeParseTest, RejectsOppositeEdges)
{
  const std::string err = error_for(
    "a:\n  b:\n"
    "    x: 0.0\n    y: 0.0\n    z: 0.0\n    roll: 0.0\n    pitch: 0.0\n    yaw: 0.0\n"
    "b:\n  a:\n"
    "    x: 0.0\n    y: 0.0\n    z: 0.0\n    roll: 0.0\n    pitch: 0.0\n    yaw: 0.0\n");
  EXPECT_NE(err.find("opposite edges"), std::string::npos) << err;
}

TEST_F(StaticTfTreeParseTest, RejectsACycle)
{
  const std::string err = error_for(
    "a:\n  b:\n"
    "    x: 0.0\n    y: 0.0\n    z: 0.0\n    roll: 0.0\n    pitch: 0.0\n    yaw: 0.0\n"
    "b:\n  c:\n"
    "    x: 0.0\n    y: 0.0\n    z: 0.0\n    roll: 0.0\n    pitch: 0.0\n    yaw: 0.0\n"
    "c:\n  a:\n"
    "    x: 0.0\n    y: 0.0\n    z: 0.0\n    roll: 0.0\n    pitch: 0.0\n    yaw: 0.0\n");
  EXPECT_NE(err.find("directed cycle"), std::string::npos) << err;
}

// The emitter quotes awkward frame ids; the parser has to give the same string
// back, or a round trip would rename frames.
TEST_F(StaticTfTreeParseTest, ReadsQuotedFrameIds)
{
  const std::vector<geometry_msgs::msg::TransformStamped> original{
    make_edge("base_link", "no"), make_edge("no", "1.0")};
  const auto parsed = parse_static_tf_tree_yaml(write(emit(original)));

  ASSERT_TRUE(parsed.ok()) << parsed.error;
  ASSERT_EQ(parsed.transforms->size(), 2U);
  EXPECT_EQ((*parsed.transforms)[0].child_frame_id, "no");
  EXPECT_EQ((*parsed.transforms)[1].header.frame_id, "no");
  EXPECT_EQ((*parsed.transforms)[1].child_frame_id, "1.0");
}

}  // namespace
