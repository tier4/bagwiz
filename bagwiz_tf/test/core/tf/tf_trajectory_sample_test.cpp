// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf/tf_trajectory_sample.hpp"

#include "bagwiz/core/tf/tf_message_wire.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <tf2/buffer_core.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace
{

using bagwiz::core::TfInputEdge;

constexpr std::int64_t kSec = 1'000'000'000LL;
constexpr double kTol = 1e-12;

geometry_msgs::msg::TransformStamped make_tf(
  const std::string & parent, const std::string & child, std::int64_t stamp_ns, double tx,
  double ty = 0.0, double tz = 0.0)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = parent;
  t.header.stamp.sec = static_cast<std::int32_t>(stamp_ns / kSec);
  t.header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % kSec);
  t.child_frame_id = child;
  t.transform.translation.x = tx;
  t.transform.translation.y = ty;
  t.transform.translation.z = tz;
  t.transform.rotation.w = 1.0;
  return t;
}

// 90-degree rotation about z: (x, y, z) -> (-y, x, z).
geometry_msgs::msg::TransformStamped make_tf_rot_z90(
  const std::string & parent, const std::string & child, std::int64_t stamp_ns, double tx,
  double ty = 0.0, double tz = 0.0)
{
  auto t = make_tf(parent, child, stamp_ns, tx, ty, tz);
  t.transform.rotation.z = std::sin(M_PI / 4.0);
  t.transform.rotation.w = std::cos(M_PI / 4.0);
  return t;
}

geometry_msgs::msg::Pose make_pose(double px, double py, double pz)
{
  geometry_msgs::msg::Pose p;
  p.position.x = px;
  p.position.y = py;
  p.position.z = pz;
  p.orientation.w = 1.0;
  return p;
}

tf2::TimePoint tp(std::int64_t ns)
{
  return tf2::TimePoint{std::chrono::nanoseconds(ns)};
}

// ---------------------------------------------------------------------------
// collect_path_sample_stamps
// ---------------------------------------------------------------------------

TEST(CollectPathSampleStamps, EmptyInputsYieldEmpty)
{
  const std::vector<std::pair<std::string, std::string>> path{{"map", "odom"}};
  const std::vector<TfInputEdge> no_edges;
  EXPECT_TRUE(bagwiz::core::collect_path_sample_stamps(no_edges, path).empty());

  const std::vector<TfInputEdge> edges{{"map", "odom", 10}};
  const std::vector<std::pair<std::string, std::string>> no_path;
  EXPECT_TRUE(bagwiz::core::collect_path_sample_stamps(edges, no_path).empty());
}

TEST(CollectPathSampleStamps, NoMatchingEdgesYieldEmpty)
{
  const std::vector<TfInputEdge> edges{{"a", "b", 10}, {"c", "d", 20}};
  const std::vector<std::pair<std::string, std::string>> path{{"map", "odom"}};
  EXPECT_TRUE(bagwiz::core::collect_path_sample_stamps(edges, path).empty());
}

TEST(CollectPathSampleStamps, FiltersSortsAndDedups)
{
  const std::vector<TfInputEdge> edges{
    {"odom", "base_link", 30},  {"map", "odom", 10},
    {"odom", "base_link", 20},  {"odom", "base_link", 20},  // duplicate stamp: emitted once
    {"lidar", "base_link", 40},                             // not on the path
    {"odom", "map", 50},  // swapped orientation of a path edge: no match
  };
  const std::vector<std::pair<std::string, std::string>> path{
    {"map", "odom"}, {"odom", "base_link"}};
  const auto stamps = bagwiz::core::collect_path_sample_stamps(edges, path);
  EXPECT_EQ(stamps, (std::vector<std::int64_t>{10, 20, 30}));
}

// ---------------------------------------------------------------------------
// lookup_trajectory_at_stamps
// ---------------------------------------------------------------------------

// Fill `buffer` (BufferCore is non-copyable): static map -> odom (tx=1),
// dynamic odom -> base_link (tx=2 @10s, tx=4 @20s).
void fill_lookup_buffer(tf2::BufferCore & buffer)
{
  buffer.setTransform(make_tf("map", "odom", 0, 1.0), "test", true);
  buffer.setTransform(make_tf("odom", "base_link", 10 * kSec, 2.0), "test", false);
  buffer.setTransform(make_tf("odom", "base_link", 20 * kSec, 4.0), "test", false);
}

TEST(LookupTrajectoryAtStamps, ResolvesPoseOfOfExpressedInRef)
{
  tf2::BufferCore buffer{std::chrono::seconds(60)};
  fill_lookup_buffer(buffer);
  const std::vector<std::int64_t> stamps{10 * kSec, 20 * kSec};
  const auto result = bagwiz::core::lookup_trajectory_at_stamps(buffer, "map", "base_link", stamps);

  ASSERT_EQ(result.poses.size(), 2u);
  EXPECT_EQ(result.skipped, 0);
  EXPECT_TRUE(result.last_skip_reason.empty());

  // Pose of base_link expressed in map: 1 (static) + 2 = 3 at 10s.
  EXPECT_EQ(result.poses[0].timestamp_ns, 10 * kSec);
  EXPECT_DOUBLE_EQ(result.poses[0].tx, 3.0);
  EXPECT_DOUBLE_EQ(result.poses[0].ty, 0.0);
  EXPECT_DOUBLE_EQ(result.poses[0].tz, 0.0);
  EXPECT_DOUBLE_EQ(result.poses[0].qx, 0.0);
  EXPECT_DOUBLE_EQ(result.poses[0].qy, 0.0);
  EXPECT_DOUBLE_EQ(result.poses[0].qz, 0.0);
  EXPECT_DOUBLE_EQ(result.poses[0].qw, 1.0);

  EXPECT_EQ(result.poses[1].timestamp_ns, 20 * kSec);
  EXPECT_DOUBLE_EQ(result.poses[1].tx, 5.0);
}

TEST(LookupTrajectoryAtStamps, PreservesInputOrder)
{
  tf2::BufferCore buffer{std::chrono::seconds(60)};
  fill_lookup_buffer(buffer);
  const std::vector<std::int64_t> stamps{20 * kSec, 10 * kSec};
  const auto result = bagwiz::core::lookup_trajectory_at_stamps(buffer, "map", "base_link", stamps);

  ASSERT_EQ(result.poses.size(), 2u);
  EXPECT_EQ(result.poses[0].timestamp_ns, 20 * kSec);
  EXPECT_EQ(result.poses[1].timestamp_ns, 10 * kSec);
}

TEST(LookupTrajectoryAtStamps, EmptyStampsYieldEmpty)
{
  tf2::BufferCore buffer{std::chrono::seconds(60)};
  fill_lookup_buffer(buffer);
  const std::vector<std::int64_t> stamps;
  const auto result = bagwiz::core::lookup_trajectory_at_stamps(buffer, "map", "base_link", stamps);
  EXPECT_TRUE(result.poses.empty());
  EXPECT_EQ(result.skipped, 0);
  EXPECT_TRUE(result.last_skip_reason.empty());
}

TEST(LookupTrajectoryAtStamps, SkipsUnresolvableStamps)
{
  tf2::BufferCore buffer{std::chrono::seconds(60)};
  fill_lookup_buffer(buffer);
  // 5s predates the earliest dynamic odom -> base_link sample (10s), so its
  // lookup throws tf2::ExtrapolationException; 10s resolves.
  const std::vector<std::int64_t> stamps{5 * kSec, 10 * kSec};
  const auto result = bagwiz::core::lookup_trajectory_at_stamps(buffer, "map", "base_link", stamps);

  ASSERT_EQ(result.poses.size(), 1u);
  EXPECT_EQ(result.poses[0].timestamp_ns, 10 * kSec);
  EXPECT_EQ(result.skipped, 1);

  // The recorded reason is the exception's what(), verbatim.
  std::string expected_reason;
  try {
    buffer.lookupTransform("map", "base_link", tp(5 * kSec));
  } catch (const tf2::TransformException & e) {
    expected_reason = e.what();
  }
  ASSERT_FALSE(expected_reason.empty());
  EXPECT_EQ(result.last_skip_reason, expected_reason);
}

TEST(LookupTrajectoryAtStamps, LastReasonReflectsLastFailure)
{
  tf2::BufferCore buffer{std::chrono::seconds(60)};
  fill_lookup_buffer(buffer);
  // Both 5s and 6s fail; the reason must be the one from 6s (the last).
  const std::vector<std::int64_t> stamps{5 * kSec, 6 * kSec, 10 * kSec};
  const auto result = bagwiz::core::lookup_trajectory_at_stamps(buffer, "map", "base_link", stamps);

  ASSERT_EQ(result.poses.size(), 1u);
  EXPECT_EQ(result.skipped, 2);

  std::string expected_reason;
  try {
    buffer.lookupTransform("map", "base_link", tp(6 * kSec));
  } catch (const tf2::TransformException & e) {
    expected_reason = e.what();
  }
  EXPECT_EQ(result.last_skip_reason, expected_reason);
}

TEST(LookupTrajectoryAtStamps, AllFailedYieldsEmptyPoses)
{
  tf2::BufferCore buffer{std::chrono::seconds(60)};
  fill_lookup_buffer(buffer);
  const std::vector<std::int64_t> stamps{10 * kSec};
  const auto result = bagwiz::core::lookup_trajectory_at_stamps(buffer, "map", "ghost", stamps);
  EXPECT_TRUE(result.poses.empty());
  EXPECT_EQ(result.skipped, 1);
  EXPECT_FALSE(result.last_skip_reason.empty());
}

// ---------------------------------------------------------------------------
// compose_tf_bridged_sample
// ---------------------------------------------------------------------------

TEST(ComposeTfBridgedSample, NoBridgesReturnsBodyVerbatim)
{
  // Empty buffer: any accidental lookup would throw, so passing proves no
  // lookup happened. The pose is returned verbatim (no renormalisation).
  tf2::BufferCore buffer{std::chrono::seconds(60)};
  auto body = make_pose(1.0, 2.0, 3.0);
  body.orientation.w = 2.0;  // deliberately unnormalised
  const auto out = bagwiz::core::compose_tf_bridged_sample(
    buffer, "odom", "odom", std::nullopt, "", body, 123'456'789);

  EXPECT_EQ(out.timestamp_ns, 123'456'789);
  EXPECT_DOUBLE_EQ(out.tx, 1.0);
  EXPECT_DOUBLE_EQ(out.ty, 2.0);
  EXPECT_DOUBLE_EQ(out.tz, 3.0);
  EXPECT_DOUBLE_EQ(out.qx, 0.0);
  EXPECT_DOUBLE_EQ(out.qy, 0.0);
  EXPECT_DOUBLE_EQ(out.qz, 0.0);
  EXPECT_DOUBLE_EQ(out.qw, 2.0);
}

TEST(ComposeTfBridgedSample, OfEqualToChildDisablesBodyBridge)
{
  // Empty buffer: with --of equal to the body frame no tracked-side lookup
  // may happen, so the sample passes through untransformed.
  tf2::BufferCore buffer{std::chrono::seconds(60)};
  const std::optional<std::string> of{"base_link"};
  const auto out = bagwiz::core::compose_tf_bridged_sample(
    buffer, "odom", "odom", of, "base_link", make_pose(4.0, 5.0, 6.0), 7 * kSec);

  EXPECT_EQ(out.timestamp_ns, 7 * kSec);
  EXPECT_DOUBLE_EQ(out.tx, 4.0);
  EXPECT_DOUBLE_EQ(out.ty, 5.0);
  EXPECT_DOUBLE_EQ(out.tz, 6.0);
}

TEST(ComposeTfBridgedSample, ReferenceBridgeReExpressesIntoRef)
{
  tf2::BufferCore buffer{std::chrono::seconds(60)};
  buffer.setTransform(make_tf_rot_z90("map", "odom", 0, 10.0), "test", true);

  const auto out = bagwiz::core::compose_tf_bridged_sample(
    buffer, "map", "odom", std::nullopt, "base_link", make_pose(1.0, 0.0, 0.0), 0);

  // T_map_odom * T_odom_body: rotate (1,0,0) by +90 deg about z -> (0,1,0),
  // then translate by (10,0,0). A swapped lookupTransform(target=odom,
  // source=map) would invert the bridge and yield (-10,-1,0) instead.
  EXPECT_NEAR(out.tx, 10.0, kTol);
  EXPECT_NEAR(out.ty, 1.0, kTol);
  EXPECT_NEAR(out.tz, 0.0, kTol);
  EXPECT_NEAR(out.qx, 0.0, kTol);
  EXPECT_NEAR(out.qy, 0.0, kTol);
  EXPECT_NEAR(out.qz, std::sin(M_PI / 4.0), kTol);
  EXPECT_NEAR(out.qw, std::cos(M_PI / 4.0), kTol);
}

TEST(ComposeTfBridgedSample, BodyBridgeWalksChildToOf)
{
  tf2::BufferCore buffer{std::chrono::seconds(60)};
  buffer.setTransform(make_tf("base_link", "sensor", 0, 0.0, 0.0, 1.0), "test", true);

  const std::optional<std::string> of{"sensor"};
  auto body = make_pose(1.0, 2.0, 3.0);
  body.orientation.z = std::sin(M_PI / 4.0);
  body.orientation.w = std::cos(M_PI / 4.0);
  const auto out =
    bagwiz::core::compose_tf_bridged_sample(buffer, "odom", "odom", of, "base_link", body, 0);

  // T_odom_body * T_base_sensor: the z-offset is invariant under the body's
  // z rotation. A swapped lookupTransform(target=sensor, source=base_link)
  // would produce tz=2 instead.
  EXPECT_NEAR(out.tx, 1.0, kTol);
  EXPECT_NEAR(out.ty, 2.0, kTol);
  EXPECT_NEAR(out.tz, 4.0, kTol);
  EXPECT_NEAR(out.qz, std::sin(M_PI / 4.0), kTol);
  EXPECT_NEAR(out.qw, std::cos(M_PI / 4.0), kTol);
}

TEST(ComposeTfBridgedSample, BothBridgesCompose)
{
  tf2::BufferCore buffer{std::chrono::seconds(60)};
  buffer.setTransform(make_tf_rot_z90("map", "odom", 0, 10.0), "test", true);
  buffer.setTransform(make_tf("base_link", "sensor", 0, 0.0, 0.0, 1.0), "test", true);

  const std::optional<std::string> of{"sensor"};
  const auto out = bagwiz::core::compose_tf_bridged_sample(
    buffer, "map", "odom", of, "base_link", make_pose(1.0, 2.0, 3.0), 0);

  // Body bridge first: (1,2,3) + (0,0,1) = (1,2,4). Reference bridge:
  // rotate by +90 deg about z -> (-2,1,4), translate -> (8,1,4).
  EXPECT_NEAR(out.tx, 8.0, kTol);
  EXPECT_NEAR(out.ty, 1.0, kTol);
  EXPECT_NEAR(out.tz, 4.0, kTol);
  EXPECT_NEAR(out.qz, std::sin(M_PI / 4.0), kTol);
  EXPECT_NEAR(out.qw, std::cos(M_PI / 4.0), kTol);
}

TEST(ComposeTfBridgedSample, LookupUsesTheSampleStamp)
{
  tf2::BufferCore buffer{std::chrono::seconds(60)};
  buffer.setTransform(make_tf("map", "odom", 10 * kSec, 1.0), "test", false);
  buffer.setTransform(make_tf("map", "odom", 20 * kSec, 2.0), "test", false);

  const auto out = bagwiz::core::compose_tf_bridged_sample(
    buffer, "map", "odom", std::nullopt, "", make_pose(0.0, 0.0, 0.0), 10 * kSec);

  // At 10s the bridge is tx=1, not the latest tx=2: the lookup honors the
  // per-sample stamp rather than TimePointZero / latest.
  EXPECT_EQ(out.timestamp_ns, 10 * kSec);
  EXPECT_DOUBLE_EQ(out.tx, 1.0);
}

TEST(ComposeTfBridgedSample, UnresolvableBridgeThrows)
{
  tf2::BufferCore buffer{std::chrono::seconds(60)};
  buffer.setTransform(make_tf("map", "odom", 0, 1.0), "test", true);

  EXPECT_THROW(
    static_cast<void>(bagwiz::core::compose_tf_bridged_sample(
      buffer, "map", "ghost", std::nullopt, "", make_pose(0.0, 0.0, 0.0), 0)),
    tf2::TransformException);

  const std::optional<std::string> of{"ghost"};
  EXPECT_THROW(
    static_cast<void>(bagwiz::core::compose_tf_bridged_sample(
      buffer, "odom", "odom", of, "base_link", make_pose(0.0, 0.0, 0.0), 0)),
    tf2::TransformException);
}

// ---------------------------------------------------------------------------
// sample_tf_message_trajectory
// ---------------------------------------------------------------------------

using Failure = bagwiz::core::TfMessageTrajectoryResult::Failure;

bagwiz::io::CreateOptions mcap_file_options()
{
  bagwiz::io::CreateOptions o;
  o.format = bagwiz::io::Format::Mcap;
  o.layout = bagwiz::io::Layout::SingleFile;
  o.mcap_compression = "none";
  return o;
}

void write_tf_payload(
  bagwiz::io::BagWriter & w, const std::string & topic, std::int64_t record_ns,
  std::span<const geometry_msgs::msg::TransformStamped> edges)
{
  const auto payload = bagwiz::core::serialize_tf_message(edges);
  w.write(topic, record_ns, std::span<const std::byte>(payload.data(), payload.size()));
}

// /tf_static: map -> odom (tx=1, static); /pose_tf: odom -> base_link at 10s
// (tx=2) and 20s (tx=4). The caller-side contract pre-loads the static side
// via load_static_tf_buffer.
std::filesystem::path write_sample_bag(const std::filesystem::path & path)
{
  std::filesystem::remove(path);
  auto w = bagwiz::io::open_write(path, mcap_file_options());
  w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));
  w->declare_topic(bagwiz::core::make_tf_message_topic_info("/pose_tf"));
  write_tf_payload(*w, "/tf_static", 0, std::vector{make_tf("map", "odom", 0, 1.0)});
  write_tf_payload(
    *w, "/pose_tf", 10 * kSec, std::vector{make_tf("odom", "base_link", 10 * kSec, 2.0)});
  write_tf_payload(
    *w, "/pose_tf", 20 * kSec, std::vector{make_tf("odom", "base_link", 20 * kSec, 4.0)});
  w->close();
  return path;
}

TEST(SampleTfMessageTrajectory, ResolvesSamplesThroughStaticAndDynamicEdges)
{
  const auto bag =
    write_sample_bag(std::filesystem::temp_directory_path() / "bagwiz_tf_sample_traj_ok.mcap");
  tf2::BufferCore buffer{std::chrono::hours(24 * 365)};
  ASSERT_FALSE(bagwiz::core::load_static_tf_buffer(bag, buffer).has_value());

  const auto topic = bagwiz::core::make_tf_message_topic_info("/pose_tf");
  const auto r = bagwiz::core::sample_tf_message_trajectory(bag, topic, "map", "base_link", buffer);

  EXPECT_EQ(r.failure, Failure::kNone) << r.failure_detail;
  EXPECT_EQ(r.sample_stamps, 2u);
  EXPECT_EQ(r.skipped, 0);
  ASSERT_EQ(r.poses.size(), 2u);
  // Pose of base_link expressed in map: 1 (static) + 2 = 3 at 10s, 1 + 4 = 5 at 20s.
  EXPECT_EQ(r.poses[0].timestamp_ns, 10 * kSec);
  EXPECT_DOUBLE_EQ(r.poses[0].tx, 3.0);
  EXPECT_EQ(r.poses[1].timestamp_ns, 20 * kSec);
  EXPECT_DOUBLE_EQ(r.poses[1].tx, 5.0);

  std::filesystem::remove(bag);
}

TEST(SampleTfMessageTrajectory, OpenBagFailureIsStructured)
{
  tf2::BufferCore buffer{std::chrono::seconds(60)};
  const auto topic = bagwiz::core::make_tf_message_topic_info("/pose_tf");
  const auto missing =
    std::filesystem::temp_directory_path() / "bagwiz_tf_sample_traj_missing.mcap";
  std::filesystem::remove(missing);

  const auto r =
    bagwiz::core::sample_tf_message_trajectory(missing, topic, "map", "base_link", buffer);

  EXPECT_EQ(r.failure, Failure::kOpenBag);
  EXPECT_FALSE(r.failure_detail.empty());
}

TEST(SampleTfMessageTrajectory, DecodeFailureIsStructured)
{
  const auto bag = std::filesystem::temp_directory_path() / "bagwiz_tf_sample_traj_decode.mcap";
  std::filesystem::remove(bag);
  {
    auto w = bagwiz::io::open_write(bag, mcap_file_options());
    w->declare_topic(bagwiz::core::make_tf_message_topic_info("/pose_tf"));
    const std::array<std::byte, 4> garbage{
      std::byte{0xDE}, std::byte{0xAD}, std::byte{0xBE}, std::byte{0xEF}};
    w->write("/pose_tf", 10 * kSec, std::span<const std::byte>(garbage.data(), garbage.size()));
    w->close();
  }

  tf2::BufferCore buffer{std::chrono::seconds(60)};
  const auto topic = bagwiz::core::make_tf_message_topic_info("/pose_tf");
  const auto r = bagwiz::core::sample_tf_message_trajectory(bag, topic, "map", "base_link", buffer);

  EXPECT_EQ(r.failure, Failure::kDecode);
  EXPECT_FALSE(r.failure_detail.empty());

  std::filesystem::remove(bag);
}

TEST(SampleTfMessageTrajectory, TopicWithoutTransformsIsStructured)
{
  const auto bag = std::filesystem::temp_directory_path() / "bagwiz_tf_sample_traj_empty.mcap";
  std::filesystem::remove(bag);
  {
    auto w = bagwiz::io::open_write(bag, mcap_file_options());
    w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));
    w->declare_topic(bagwiz::core::make_tf_message_topic_info("/pose_tf"));
    // One message whose TFMessage carries zero TransformStamped entries.
    write_tf_payload(*w, "/tf_static", 0, std::vector{make_tf("map", "odom", 0, 1.0)});
    write_tf_payload(
      *w, "/pose_tf", 10 * kSec, std::span<const geometry_msgs::msg::TransformStamped>{});
    w->close();
  }

  tf2::BufferCore buffer{std::chrono::hours(24 * 365)};
  ASSERT_FALSE(bagwiz::core::load_static_tf_buffer(bag, buffer).has_value());
  const auto topic = bagwiz::core::make_tf_message_topic_info("/pose_tf");
  const auto r = bagwiz::core::sample_tf_message_trajectory(bag, topic, "map", "base_link", buffer);

  EXPECT_EQ(r.failure, Failure::kNoTransforms);

  std::filesystem::remove(bag);
}

TEST(SampleTfMessageTrajectory, MissingPathIsStructured)
{
  const auto bag =
    write_sample_bag(std::filesystem::temp_directory_path() / "bagwiz_tf_sample_traj_nopath.mcap");
  tf2::BufferCore buffer{std::chrono::hours(24 * 365)};
  ASSERT_FALSE(bagwiz::core::load_static_tf_buffer(bag, buffer).has_value());

  const auto topic = bagwiz::core::make_tf_message_topic_info("/pose_tf");
  const auto r = bagwiz::core::sample_tf_message_trajectory(bag, topic, "map", "ghost", buffer);

  EXPECT_EQ(r.failure, Failure::kNoPath);

  std::filesystem::remove(bag);
}

TEST(SampleTfMessageTrajectory, StaticOnlyPathIsStructured)
{
  // The of -> ref path resolves purely through /tf_static (map -> base_link);
  // /pose_tf publishes an unrelated dynamic edge, so no chain edge carries a
  // stamp on the topic.
  const auto bag = std::filesystem::temp_directory_path() / "bagwiz_tf_sample_traj_staticonly.mcap";
  std::filesystem::remove(bag);
  {
    auto w = bagwiz::io::open_write(bag, mcap_file_options());
    w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));
    w->declare_topic(bagwiz::core::make_tf_message_topic_info("/pose_tf"));
    write_tf_payload(*w, "/tf_static", 0, std::vector{make_tf("map", "base_link", 0, 1.0)});
    write_tf_payload(
      *w, "/pose_tf", 10 * kSec, std::vector{make_tf("odom", "wheel", 10 * kSec, 2.0)});
    w->close();
  }

  tf2::BufferCore buffer{std::chrono::hours(24 * 365)};
  ASSERT_FALSE(bagwiz::core::load_static_tf_buffer(bag, buffer).has_value());
  const auto topic = bagwiz::core::make_tf_message_topic_info("/pose_tf");
  const auto r = bagwiz::core::sample_tf_message_trajectory(bag, topic, "map", "base_link", buffer);

  EXPECT_EQ(r.failure, Failure::kNoPathStamps);

  std::filesystem::remove(bag);
}

}  // namespace
