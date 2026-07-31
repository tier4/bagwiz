// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/tf_static_join.hpp"

#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/tf/tf_message_wire.hpp"
#include "bagwiz/core/tf/tf_transform_format.hpp"
#include "bagwiz/core/tf/tf_value_extract.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace
{

using bagwiz::commands::run_tf_static_join;

constexpr const char * kTfMessageType = "tf2_msgs/msg/TFMessage";

bagwiz::io::CreateOptions mcap_options()
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = "none";
  return options;
}

bagwiz::io::TopicInfo tf_topic_info(const std::string & name)
{
  bagwiz::io::TopicInfo t;
  t.name = name;
  t.type = kTfMessageType;
  t.serialization_format = "cdr";
  t.schema_encoding = "ros2msg";
  t.schema_text = bagwiz::core::kTfMessageWireSchema;
  return t;
}

geometry_msgs::msg::TransformStamped make_edge(
  const std::string & parent, const std::string & child, double tx)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = parent;
  ts.child_frame_id = child;
  ts.transform.translation.x = tx;
  ts.transform.rotation.w = 1.0;
  return ts;
}

void write_tf_message(
  bagwiz::io::BagWriter & writer, const std::string & topic, std::int64_t stamp_ns,
  const std::vector<geometry_msgs::msg::TransformStamped> & transforms)
{
  const auto cdr = bagwiz::core::serialize_tf_message(
    std::span<const geometry_msgs::msg::TransformStamped>(transforms.data(), transforms.size()));
  writer.write(topic, stamp_ns, std::span<const std::byte>(cdr.data(), cdr.size()));
}

// A bag with a single non-TF topic whose earliest message fixes the bag's start
// time at `start_ns`.
void write_plain_bag(const std::filesystem::path & path, std::int64_t start_ns)
{
  bagwiz::io::TopicInfo clock;
  clock.name = "/clock";
  clock.type = "std_msgs/msg/String";
  clock.serialization_format = "cdr";

  constexpr std::array<std::byte, 4> kPayload{
    std::byte{0xDE}, std::byte{0xAD}, std::byte{0xBE}, std::byte{0xEF}};
  const auto bytes = std::span<const std::byte>(kPayload.data(), kPayload.size());

  auto writer = bagwiz::io::open_write(path, mcap_options());
  writer->declare_topic(clock);
  writer->write("/clock", start_ns, bytes);
  writer->write("/clock", start_ns + 1'000'000'000LL, bytes);
  writer->close();
}

struct ReadTfResult
{
  bool present = false;
  std::int64_t stamp_ns = 0;
  int message_count = 0;
  // Storage position of the topic's first message among all messages, in the
  // order the reader hands them back.
  int first_index = -1;
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
};

ReadTfResult read_tf_topic(const std::filesystem::path & path, const std::string & topic)
{
  ReadTfResult result;
  auto reader = bagwiz::io::open_read(path);
  reader->populate_schemas();

  const bagwiz::io::TopicInfo * info = nullptr;
  for (const auto & t : reader->topics()) {
    if (t.name == topic) {
      info = &t;
      break;
    }
  }
  if (info == nullptr) {
    return result;
  }
  result.present = true;

  auto open = bagwiz::core::decoder::open_decoder(*info);
  EXPECT_TRUE(open.ok()) << open.error;

  bagwiz::io::RawMessage raw;
  int index = 0;
  while (reader->next(raw)) {
    if (raw.topic->name != topic) {
      ++index;
      continue;
    }
    if (result.first_index < 0) {
      result.first_index = index;
    }
    ++index;
    ++result.message_count;
    result.stamp_ns = raw.timestamp_ns;
    const auto decoded = open.decoder->decode(raw.payload);
    EXPECT_TRUE(decoded.ok()) << decoded.error;
    result.transforms = bagwiz::core::extract_tf_message(*decoded.value);
  }
  return result;
}

bool topic_present(const std::filesystem::path & path, const std::string & topic)
{
  auto reader = bagwiz::io::open_read(path);
  for (const auto & t : reader->topics()) {
    if (t.name == topic) {
      return true;
    }
  }
  return false;
}

class TfStaticJoinTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_tf_static_join_" +
                std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()));
    std::filesystem::remove_all(tmp_dir_);
    std::filesystem::create_directories(tmp_dir_);
  }
  void TearDown() override { std::filesystem::remove_all(tmp_dir_); }

  std::filesystem::path write_yaml(const std::string & contents) const
  {
    const auto path = tmp_dir_ / "tf_static.yaml";
    std::ofstream(path) << contents;
    return path;
  }

  static std::string read_all(const std::filesystem::path & path)
  {
    std::ifstream in(path, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  }

  // A two-edge tree: base_link -> drs_base_link, then a lidar rotated 90 deg
  // about z so the RPY -> quaternion conversion is observable.
  static std::string sample_yaml()
  {
    return "base_link:\n"
           "  drs_base_link:\n"
           "    x: 0.796\n"
           "    y: 0.0\n"
           "    z: 1.826\n"
           "    roll: 0.0\n"
           "    pitch: 0.0\n"
           "    yaw: 0.0\n"
           "\n"
           "drs_base_link:\n"
           "  lidar_front:\n"
           "    x: 1.0\n"
           "    y: 2.0\n"
           "    z: 3.0\n"
           "    roll: 0.0\n"
           "    pitch: 0.0\n"
           "    yaw: 1.5707963267948966\n";
  }

  std::filesystem::path tmp_dir_;
};

TEST_F(TfStaticJoinTest, EmbedsTheConfigStampedAtTheBagStart)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  constexpr std::int64_t kStart = 5'000'000'000LL;
  write_plain_bag(bag, kStart);
  const auto yaml = write_yaml(sample_yaml());

  ASSERT_EQ(
    run_tf_static_join(bag, yaml, "/tf_static", out, /*force=*/false, /*overwrite=*/false), 0);

  const auto joined = read_tf_topic(out, "/tf_static");
  ASSERT_TRUE(joined.present);
  EXPECT_EQ(joined.message_count, 1);
  EXPECT_EQ(joined.stamp_ns, kStart);
  ASSERT_EQ(joined.transforms.size(), 2U);
  // Both the receive time and every header.stamp carry the bag's start time.
  for (const auto & t : joined.transforms) {
    EXPECT_EQ(t.header.stamp.sec, 5);
    EXPECT_EQ(t.header.stamp.nanosec, 0U);
  }
  EXPECT_EQ(joined.transforms[0].header.frame_id, "base_link");
  EXPECT_EQ(joined.transforms[0].child_frame_id, "drs_base_link");
  EXPECT_DOUBLE_EQ(joined.transforms[0].transform.translation.x, 0.796);
  EXPECT_DOUBLE_EQ(joined.transforms[0].transform.translation.z, 1.826);
  EXPECT_EQ(joined.transforms[1].child_frame_id, "lidar_front");

  // The bag's own topic survives and the input is untouched in -o mode.
  EXPECT_TRUE(topic_present(out, "/clock"));
  EXPECT_FALSE(topic_present(bag, "/tf_static"));
}

// The YAML stores rotations as RPY; the bag needs a quaternion. A 90 deg yaw is
// the clearest check that the conversion runs and uses the right axis.
TEST_F(TfStaticJoinTest, ConvertsRpyToAQuaternion)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_plain_bag(bag, 1'000'000'000LL);
  const auto yaml = write_yaml(sample_yaml());

  ASSERT_EQ(
    run_tf_static_join(bag, yaml, "/tf_static", out, /*force=*/false, /*overwrite=*/false), 0);

  const auto joined = read_tf_topic(out, "/tf_static");
  ASSERT_EQ(joined.transforms.size(), 2U);
  // Identity for the zero-RPY edge.
  const auto & identity = joined.transforms[0].transform.rotation;
  EXPECT_NEAR(identity.w, 1.0, 1e-12);
  EXPECT_NEAR(identity.x, 0.0, 1e-12);
  // yaw = pi/2 about z: (0, 0, sin(pi/4), cos(pi/4)).
  const auto & yawed = joined.transforms[1].transform.rotation;
  EXPECT_NEAR(yawed.x, 0.0, 1e-12);
  EXPECT_NEAR(yawed.y, 0.0, 1e-12);
  EXPECT_NEAR(yawed.z, std::sin(std::numbers::pi / 4.0), 1e-12);
  EXPECT_NEAR(yawed.w, std::cos(std::numbers::pi / 4.0), 1e-12);
  // And it survives the trip back through the reader's RPY view.
  const auto rpy = bagwiz::core::quaternion_to_rpy(yawed);
  EXPECT_NEAR(rpy.yaw, std::numbers::pi / 2.0, 1e-12);
}

// The injected message carries the bag's lowest timestamp, so it must also hold
// the lowest storage position. Appending it instead would leave the one row whose
// physical order disagrees with its time, which a consumer reading a .db3 in
// rowid order (Foxglove) would deliver last.
TEST_F(TfStaticJoinTest, InjectedStaticTfIsEmittedInTimestampOrder)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_plain_bag(bag, 5'000'000'000LL);
  const auto yaml = write_yaml(sample_yaml());

  ASSERT_EQ(
    run_tf_static_join(bag, yaml, "/tf_static", out, /*force=*/false, /*overwrite=*/false), 0);

  const auto joined = read_tf_topic(out, "/tf_static");
  ASSERT_TRUE(joined.present);
  EXPECT_EQ(joined.first_index, 0) << "static TF must be the first message in storage order";
}

TEST_F(TfStaticJoinTest, RewritesTheBagInPlaceWithoutOutput)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  write_plain_bag(bag, 1'000'000'000LL);
  const auto yaml = write_yaml(sample_yaml());

  ASSERT_EQ(
    run_tf_static_join(bag, yaml, "/tf_static", std::nullopt, /*force=*/false, /*overwrite=*/false),
    0);

  const auto joined = read_tf_topic(bag, "/tf_static");
  ASSERT_TRUE(joined.present);
  EXPECT_EQ(joined.transforms.size(), 2U);
  EXPECT_TRUE(topic_present(bag, "/clock"));
}

TEST_F(TfStaticJoinTest, HonoursACustomTopic)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_plain_bag(bag, 1'000'000'000LL);
  const auto yaml = write_yaml(sample_yaml());

  ASSERT_EQ(
    run_tf_static_join(bag, yaml, "/sensing/tf_static", out, /*force=*/false, /*overwrite=*/false),
    0);

  EXPECT_TRUE(read_tf_topic(out, "/sensing/tf_static").present);
  EXPECT_FALSE(topic_present(out, "/tf_static"));
}

// A populated destination topic is a conflict: silently replacing a bag's real
// static TF with a config would be unrecoverable.
TEST_F(TfStaticJoinTest, RefusesAPopulatedTopicWithoutForce)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  {
    auto writer = bagwiz::io::open_write(bag, mcap_options());
    writer->declare_topic(tf_topic_info("/tf_static"));
    write_tf_message(*writer, "/tf_static", 1'000'000'000LL, {make_edge("map", "odom", 7.0)});
    writer->close();
  }
  const auto yaml = write_yaml(sample_yaml());

  EXPECT_EQ(
    run_tf_static_join(bag, yaml, "/tf_static", out, /*force=*/false, /*overwrite=*/false), 1);
  // The original transform is still the one in the bag.
  const auto original = read_tf_topic(bag, "/tf_static");
  ASSERT_EQ(original.transforms.size(), 1U);
  EXPECT_EQ(original.transforms[0].child_frame_id, "odom");
}

TEST_F(TfStaticJoinTest, ReplacesAPopulatedTopicWithForce)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  {
    auto writer = bagwiz::io::open_write(bag, mcap_options());
    writer->declare_topic(tf_topic_info("/tf_static"));
    write_tf_message(*writer, "/tf_static", 1'000'000'000LL, {make_edge("map", "odom", 7.0)});
    writer->close();
  }
  const auto yaml = write_yaml(sample_yaml());

  ASSERT_EQ(
    run_tf_static_join(bag, yaml, "/tf_static", out, /*force=*/true, /*overwrite=*/false), 0);

  const auto joined = read_tf_topic(out, "/tf_static");
  EXPECT_EQ(joined.message_count, 1);
  ASSERT_EQ(joined.transforms.size(), 2U);
  EXPECT_EQ(joined.transforms[0].child_frame_id, "drs_base_link");
}

TEST_F(TfStaticJoinTest, RefusesAnExistingOutputWithoutOverwrite)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_plain_bag(bag, 1'000'000'000LL);
  const auto yaml = write_yaml(sample_yaml());
  std::ofstream(out) << "precious\n";

  EXPECT_EQ(
    run_tf_static_join(bag, yaml, "/tf_static", out, /*force=*/false, /*overwrite=*/false), 1);
  EXPECT_EQ(read_all(out), "precious\n");
}

TEST_F(TfStaticJoinTest, HonoursOverwriteForTheOutputPath)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_plain_bag(bag, 1'000'000'000LL);
  const auto yaml = write_yaml(sample_yaml());
  std::ofstream(out) << "stale\n";

  ASSERT_EQ(
    run_tf_static_join(bag, yaml, "/tf_static", out, /*force=*/false, /*overwrite=*/true), 0);
  EXPECT_TRUE(read_tf_topic(out, "/tf_static").present);
}

// --force and -w/--overwrite are separate permissions (matching `traj join`), so
// --force alone must not clobber an existing -o path.
TEST_F(TfStaticJoinTest, ForceDoesNotPermitClobberingTheOutputPath)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_plain_bag(bag, 1'000'000'000LL);
  const auto yaml = write_yaml(sample_yaml());
  std::ofstream(out) << "precious\n";

  EXPECT_EQ(
    run_tf_static_join(bag, yaml, "/tf_static", out, /*force=*/true, /*overwrite=*/false), 1);
  EXPECT_EQ(read_all(out), "precious\n");
}

// `.nan` is a valid YAML float, so it parses — but tf2 drops such a transform,
// which would leave the written /tf_static well-formed and its tree empty. The
// tree-buildable check refuses it before anything is written.
TEST_F(TfStaticJoinTest, RejectsAConfigTf2CouldNotBuildATreeFrom)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_plain_bag(bag, 1'000'000'000LL);
  const auto yaml = write_yaml(
    "base_link:\n  lidar:\n"
    "    x: .nan\n    y: 0.0\n    z: 0.0\n    roll: 0.0\n    pitch: 0.0\n    yaw: 0.0\n");

  EXPECT_EQ(
    run_tf_static_join(bag, yaml, "/tf_static", out, /*force=*/false, /*overwrite=*/false), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

// A forest is legal, as it is for `tf tree` and for ROS: a partial config can be
// completed by TF the bag already carries, so disconnected trees are embedded as
// given.
TEST_F(TfStaticJoinTest, EmbedsDisconnectedTrees)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_plain_bag(bag, 1'000'000'000LL);
  const auto yaml = write_yaml(
    "base_link:\n  drs_base_link:\n"
    "    x: 0.0\n    y: 0.0\n    z: 0.0\n    roll: 0.0\n    pitch: 0.0\n    yaw: 0.0\n"
    "drs_baselink:\n  lidar_front:\n"
    "    x: 1.0\n    y: 0.0\n    z: 0.0\n    roll: 0.0\n    pitch: 0.0\n    yaw: 0.0\n");

  ASSERT_EQ(
    run_tf_static_join(bag, yaml, "/tf_static", out, /*force=*/false, /*overwrite=*/false), 0);
  EXPECT_EQ(read_tf_topic(out, "/tf_static").transforms.size(), 2U);
}

TEST_F(TfStaticJoinTest, RejectsAnInvalidConfigAndLeavesTheBagAlone)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  write_plain_bag(bag, 1'000'000'000LL);
  const std::string before = read_all(bag);

  // pitch is missing, so the pose is underspecified.
  const auto yaml = write_yaml(
    "base_link:\n"
    "  lidar:\n"
    "    x: 0.0\n"
    "    y: 0.0\n"
    "    z: 0.0\n"
    "    roll: 0.0\n"
    "    yaw: 0.0\n");

  EXPECT_EQ(
    run_tf_static_join(bag, yaml, "/tf_static", std::nullopt, /*force=*/false, /*overwrite=*/false),
    1);
  EXPECT_EQ(read_all(bag), before);
  EXPECT_FALSE(topic_present(bag, "/tf_static"));
}

TEST_F(TfStaticJoinTest, RejectsAnEmptyTopic)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  write_plain_bag(bag, 1'000'000'000LL);
  const auto yaml = write_yaml(sample_yaml());

  EXPECT_EQ(
    run_tf_static_join(bag, yaml, "", std::nullopt, /*force=*/false, /*overwrite=*/false), 1);
}

// A topic whose type is not TFMessage cannot hold transforms, and --force must
// not relax that.
TEST_F(TfStaticJoinTest, RejectsATopicOfAnotherType)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_plain_bag(bag, 1'000'000'000LL);
  const auto yaml = write_yaml(sample_yaml());

  EXPECT_EQ(run_tf_static_join(bag, yaml, "/clock", out, /*force=*/true, /*overwrite=*/false), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

}  // namespace
