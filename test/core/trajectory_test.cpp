// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/trajectory.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace
{

using bagwiz::core::compose_trajectory_pose;
using bagwiz::core::parse_upsample_spec;
using bagwiz::core::pose_to_transform_stamped;
using bagwiz::core::read_tum;
using bagwiz::core::TrajectoryPose;
using bagwiz::core::upsample_trajectory;
using bagwiz::core::UpsampleMode;
using bagwiz::core::UpsampleSpec;
using bagwiz::core::write_tum;

// 90-degree rotation about +Z as a unit quaternion (qz, qw); the rest zero.
constexpr double kSinPiOver4 = 0.7071067811865476;

geometry_msgs::msg::Transform make_transform(
  double x, double y, double z, double qx, double qy, double qz, double qw)
{
  geometry_msgs::msg::Transform t;
  t.translation.x = x;
  t.translation.y = y;
  t.translation.z = z;
  t.rotation.x = qx;
  t.rotation.y = qy;
  t.rotation.z = qz;
  t.rotation.w = qw;
  return t;
}

geometry_msgs::msg::Pose make_pose(
  double x, double y, double z, double qx, double qy, double qz, double qw)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation.x = qx;
  p.orientation.y = qy;
  p.orientation.z = qz;
  p.orientation.w = qw;
  return p;
}

TEST(WriteTum, EmitsExpectedLineLayout)
{
  std::vector<TrajectoryPose> poses;
  // 1.5 s, identity rotation at (1, 2, 3).
  poses.push_back({1'500'000'000LL, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0});
  // 2.75 s, a quarter turn about Z at (0, 0, 0).
  poses.push_back(
    {2'750'000'000LL, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7071067811865475, 0.7071067811865475});

  std::ostringstream os;
  write_tum(os, poses);

  const std::string text = os.str();
  ASSERT_FALSE(text.empty());

  // Nanosecond-precision timestamp + 7 whitespace-separated values per line.
  EXPECT_NE(
    text.find(
      "1.500000000 1.000000000 2.000000000 3.000000000 0.000000000 0.000000000 "
      "0.000000000 1.000000000\n"),
    std::string::npos)
    << "got:\n"
    << text;
  EXPECT_NE(
    text.find(
      "2.750000000 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 "
      "0.707106781 0.707106781\n"),
    std::string::npos)
    << "got:\n"
    << text;
}

TEST(WriteTum, EmitsBitExactNanosecondsAtModernEpochs)
{
  // The double ULP near 1.77e18 (year-2026 magnitudes in ns) is ~256,
  // so a `static_cast<double>(ns) / 1e9` round trip silently drifts
  // the last few decimal digits. The formatter must format sec /
  // nanosec from the integer directly so the TUM timestamp is
  // bit-exact with the source header.stamp.
  const std::int64_t sec = 1773211197LL;
  const std::int64_t nsec = 937418279LL;
  const std::int64_t ts_ns = sec * 1'000'000'000LL + nsec;

  std::vector<TrajectoryPose> poses;
  poses.push_back({ts_ns, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});

  std::ostringstream os;
  write_tum(os, poses);
  const std::string text = os.str();

  EXPECT_NE(text.find("1773211197.937418279 "), std::string::npos) << "got:\n" << text;
}

TEST(WriteTum, EmitsNothingForEmptyTrajectory)
{
  std::ostringstream os;
  std::vector<TrajectoryPose> empty;
  write_tum(os, empty);
  EXPECT_EQ(os.str(), "");
}

TEST(WriteTum, RestoresStreamFormatting)
{
  std::ostringstream os;
  os << 0.1;  // default precision
  const std::string before = os.str();

  os.str({});
  os.clear();
  write_tum(os, std::vector<TrajectoryPose>{{1'000'000'000LL, 0, 0, 0, 0, 0, 0, 1}});

  // After writing, the default precision should be restored for the
  // caller so they do not silently inherit fixed/9-digit formatting.
  os.str({});
  os.clear();
  os << 0.1;
  EXPECT_EQ(os.str(), before);
}

TEST(ReadTum, ParsesEightFieldLines)
{
  std::istringstream is(
    "1.500000000 1.0 2.0 3.0 0.0 0.0 0.0 1.0\n"
    "2.750000000 0.0 0.0 0.0 0.0 0.0 0.707106781 0.707106781\n");
  const auto r = read_tum(is);
  ASSERT_EQ(r.poses.size(), 2U);
  EXPECT_EQ(r.skipped_lines, 0);
  EXPECT_EQ(r.poses[0].timestamp_ns, 1'500'000'000LL);
  EXPECT_DOUBLE_EQ(r.poses[0].tx, 1.0);
  EXPECT_DOUBLE_EQ(r.poses[0].ty, 2.0);
  EXPECT_DOUBLE_EQ(r.poses[0].tz, 3.0);
  EXPECT_DOUBLE_EQ(r.poses[0].qw, 1.0);
  EXPECT_EQ(r.poses[1].timestamp_ns, 2'750'000'000LL);
  EXPECT_DOUBLE_EQ(r.poses[1].qz, 0.707106781);
  EXPECT_DOUBLE_EQ(r.poses[1].qw, 0.707106781);
}

TEST(ReadTum, RestoresNanosecondsBitExactAtModernEpochs)
{
  // Round-trip a year-2026-magnitude timestamp through write_tum -> read_tum
  // and verify the integer nanoseconds survive without ULP drift.
  const std::int64_t sec = 1773211197LL;
  const std::int64_t nsec = 937418279LL;
  const std::int64_t ts_ns = sec * 1'000'000'000LL + nsec;
  const std::vector<TrajectoryPose> in = {{ts_ns, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};

  std::ostringstream os;
  write_tum(os, in);

  std::istringstream is(os.str());
  const auto r = read_tum(is);
  ASSERT_EQ(r.poses.size(), 1U);
  EXPECT_EQ(r.poses[0].timestamp_ns, ts_ns);
}

TEST(ReadTum, SkipsEmptyAndCommentAndMalformedLines)
{
  std::istringstream is(
    "\n"
    "   \n"
    "# this is a comment\n"
    "  # indented comment\n"
    "1.0 only-one-field\n"
    "2.0 1 2 3 0 0 0 1\n"           // good
    "not-a-number 1 2 3 0 0 0 1\n"  // bad timestamp
    "3.0 1 2 3 0 0 0 abc\n"         // bad qw
    "4.0 1 2 3 0 0 0 1\n");         // good
  const auto r = read_tum(is);
  ASSERT_EQ(r.poses.size(), 2U);
  EXPECT_EQ(r.skipped_lines, 3);
  EXPECT_EQ(r.poses[0].timestamp_ns, 2'000'000'000LL);
  EXPECT_EQ(r.poses[1].timestamp_ns, 4'000'000'000LL);
}

TEST(ReadTum, AcceptsTimestampWithoutFraction)
{
  std::istringstream is("42 1 2 3 0 0 0 1\n");
  const auto r = read_tum(is);
  ASSERT_EQ(r.poses.size(), 1U);
  EXPECT_EQ(r.poses[0].timestamp_ns, 42LL * 1'000'000'000LL);
}

TEST(ReadTum, RightPadsShortFractionalPartTo9Digits)
{
  // "1.5" -> 1.500000000 s -> 1'500'000'000 ns.
  std::istringstream is("1.5 0 0 0 0 0 0 1\n");
  const auto r = read_tum(is);
  ASSERT_EQ(r.poses.size(), 1U);
  EXPECT_EQ(r.poses[0].timestamp_ns, 1'500'000'000LL);
}

TEST(ReadTum, RejectsScientificNotationAndOver9DigitFraction)
{
  std::istringstream is(
    "1e9 0 0 0 0 0 0 1\n"
    "1.1234567890 0 0 0 0 0 0 1\n");
  const auto r = read_tum(is);
  EXPECT_TRUE(r.poses.empty());
  EXPECT_EQ(r.skipped_lines, 2);
}

TEST(ReadTum, EmitsEmptyResultOnEmptyStream)
{
  std::istringstream is("");
  const auto r = read_tum(is);
  EXPECT_TRUE(r.poses.empty());
  EXPECT_EQ(r.skipped_lines, 0);
}

TEST(PoseToTransformStamped, CopiesAllFieldsAndAssignsFrames)
{
  TrajectoryPose p;
  p.timestamp_ns = 1'500'000'250LL;  // 1.500000250 s
  p.tx = 10.0;
  p.ty = 20.0;
  p.tz = 30.0;
  p.qx = 0.1;
  p.qy = 0.2;
  p.qz = 0.3;
  p.qw = 0.9273618495495704;  // unit quaternion when combined with (0.1, 0.2, 0.3)

  const auto ts = pose_to_transform_stamped(p, "map", "base_link");
  EXPECT_EQ(ts.header.stamp.sec, 1);
  EXPECT_EQ(ts.header.stamp.nanosec, 500'000'250U);
  EXPECT_EQ(ts.header.frame_id, "map");
  EXPECT_EQ(ts.child_frame_id, "base_link");
  EXPECT_DOUBLE_EQ(ts.transform.translation.x, 10.0);
  EXPECT_DOUBLE_EQ(ts.transform.translation.y, 20.0);
  EXPECT_DOUBLE_EQ(ts.transform.translation.z, 30.0);
  EXPECT_DOUBLE_EQ(ts.transform.rotation.x, 0.1);
  EXPECT_DOUBLE_EQ(ts.transform.rotation.y, 0.2);
  EXPECT_DOUBLE_EQ(ts.transform.rotation.z, 0.3);
  EXPECT_DOUBLE_EQ(ts.transform.rotation.w, 0.9273618495495704);
}

TEST(PoseToTransformStamped, HandlesYear2026EpochWithoutDrift)
{
  TrajectoryPose p;
  p.timestamp_ns = 1773211197LL * 1'000'000'000LL + 937418279LL;
  p.qw = 1.0;
  const auto ts = pose_to_transform_stamped(p, "a", "b");
  EXPECT_EQ(ts.header.stamp.sec, 1773211197);
  EXPECT_EQ(ts.header.stamp.nanosec, 937418279U);
}

TEST(ComposeTrajectoryPose, NoBridgesReturnsPoseVerbatim)
{
  // A deliberately non-unit quaternion: the verbatim path must NOT renormalise.
  const auto body = make_pose(1.25, -2.5, 3.75, 0.0, 0.0, 0.3, 0.8);
  const auto out = compose_trajectory_pose(std::nullopt, body, std::nullopt);
  EXPECT_DOUBLE_EQ(out.position.x, 1.25);
  EXPECT_DOUBLE_EQ(out.position.y, -2.5);
  EXPECT_DOUBLE_EQ(out.position.z, 3.75);
  EXPECT_DOUBLE_EQ(out.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(out.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(out.orientation.z, 0.3);
  EXPECT_DOUBLE_EQ(out.orientation.w, 0.8);
}

TEST(ComposeTrajectoryPose, ReferenceBridgeRotatesIntoFromFrame)
{
  // from_header = 90 deg about Z; body pose at (1, 0, 0) with identity rotation.
  // T_from_to = R_z(90) * pose -> position rotates to (0, 1, 0).
  const auto from_header = make_transform(0.0, 0.0, 0.0, 0.0, 0.0, kSinPiOver4, kSinPiOver4);
  const auto body = make_pose(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  const auto out = compose_trajectory_pose(from_header, body, std::nullopt);
  EXPECT_NEAR(out.position.x, 0.0, 1e-9);
  EXPECT_NEAR(out.position.y, 1.0, 1e-9);
  EXPECT_NEAR(out.position.z, 0.0, 1e-9);
  EXPECT_NEAR(out.orientation.z, kSinPiOver4, 1e-9);
  EXPECT_NEAR(out.orientation.w, kSinPiOver4, 1e-9);
}

TEST(ComposeTrajectoryPose, TrackedBridgeAppliesInBodyFrame)
{
  // body pose = pure 90 deg about Z at the origin; body_to = +1 along X in the
  // body frame. The tracked offset must be applied in the body frame (right
  // multiply): R_z(90) * (1, 0, 0) -> (0, 1, 0).
  const auto body = make_pose(0.0, 0.0, 0.0, 0.0, 0.0, kSinPiOver4, kSinPiOver4);
  const auto body_to = make_transform(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  const auto out = compose_trajectory_pose(std::nullopt, body, body_to);
  EXPECT_NEAR(out.position.x, 0.0, 1e-9);
  EXPECT_NEAR(out.position.y, 1.0, 1e-9);
  EXPECT_NEAR(out.position.z, 0.0, 1e-9);
  EXPECT_NEAR(out.orientation.z, kSinPiOver4, 1e-9);
  EXPECT_NEAR(out.orientation.w, kSinPiOver4, 1e-9);
}

TEST(ComposeTrajectoryPose, BothBridgesComposeLeftAndRight)
{
  // from_header translates +10 in Y; body at (2, 0, 0) identity; body_to
  // translates +3 in Z (identity rotations throughout, so positions add).
  const auto from_header = make_transform(0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  const auto body = make_pose(2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  const auto body_to = make_transform(0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 1.0);
  const auto out = compose_trajectory_pose(from_header, body, body_to);
  EXPECT_NEAR(out.position.x, 2.0, 1e-9);
  EXPECT_NEAR(out.position.y, 10.0, 1e-9);
  EXPECT_NEAR(out.position.z, 3.0, 1e-9);
  EXPECT_NEAR(out.orientation.w, 1.0, 1e-9);
}

// sin/cos of 22.5 deg — the half-angle quaternion of a 45 deg rotation about Z,
// i.e. the SLERP midpoint between identity and a 90 deg rotation about Z.
constexpr double kSin22p5 = 0.3826834323650898;
constexpr double kCos22p5 = 0.9238795325112867;

TEST(ParseUpsampleSpec, BareNumberIsFrequencyHz)
{
  const auto s = parse_upsample_spec("10");
  ASSERT_TRUE(s.has_value());
  EXPECT_EQ(s->mode, UpsampleMode::kFrequencyHz);
  EXPECT_DOUBLE_EQ(s->value, 10.0);
}

TEST(ParseUpsampleSpec, HzSuffixIsFrequencyCaseInsensitive)
{
  for (const char * text : {"10hz", "10HZ", "10Hz", "10.5hz"}) {
    const auto s = parse_upsample_spec(text);
    ASSERT_TRUE(s.has_value()) << text;
    EXPECT_EQ(s->mode, UpsampleMode::kFrequencyHz) << text;
  }
  EXPECT_DOUBLE_EQ(parse_upsample_spec("10.5hz")->value, 10.5);
}

TEST(ParseUpsampleSpec, XSuffixIsMultiplierCaseInsensitive)
{
  const auto lower = parse_upsample_spec("2x");
  ASSERT_TRUE(lower.has_value());
  EXPECT_EQ(lower->mode, UpsampleMode::kMultiplier);
  EXPECT_DOUBLE_EQ(lower->value, 2.0);

  const auto upper = parse_upsample_spec("2.0X");
  ASSERT_TRUE(upper.has_value());
  EXPECT_EQ(upper->mode, UpsampleMode::kMultiplier);
  EXPECT_DOUBLE_EQ(upper->value, 2.0);

  EXPECT_DOUBLE_EQ(parse_upsample_spec("0.5x")->value, 0.5);
}

TEST(ParseUpsampleSpec, RejectsMalformedOrNonPositive)
{
  for (const char * text : {"", "x", "X", "hz", "Hz", "abc", "-2", "0", "0x", "10hzz", "2..0"}) {
    EXPECT_FALSE(parse_upsample_spec(text).has_value()) << "expected reject: " << text;
  }
}

TEST(UpsampleTrajectory, PassesThroughFewerThanTwoPoses)
{
  const std::vector<TrajectoryPose> one = {{1'000'000'000LL, 1, 2, 3, 0, 0, 0, 1}};
  const auto r = upsample_trajectory(one, {UpsampleMode::kFrequencyHz, 100.0});
  EXPECT_FALSE(r.resampled);
  ASSERT_EQ(r.poses.size(), 1U);
  EXPECT_EQ(r.poses[0].timestamp_ns, 1'000'000'000LL);
}

TEST(UpsampleTrajectory, PassesThroughWhenTargetAtOrBelowNativeRate)
{
  // Native average rate = (3 - 1) / 2 s = 1 Hz.
  const std::vector<TrajectoryPose> in = {
    {0LL, 0, 0, 0, 0, 0, 0, 1},
    {1'000'000'000LL, 1, 0, 0, 0, 0, 0, 1},
    {2'000'000'000LL, 2, 0, 0, 0, 0, 0, 1}};

  const auto below = upsample_trajectory(in, {UpsampleMode::kFrequencyHz, 0.5});
  EXPECT_FALSE(below.resampled);
  EXPECT_EQ(below.poses.size(), 3U);
  EXPECT_NEAR(below.native_rate_hz, 1.0, 1e-12);

  // A multiplier below 1.0 is likewise a down-sample request -> unchanged.
  const auto half = upsample_trajectory(in, {UpsampleMode::kMultiplier, 0.5});
  EXPECT_FALSE(half.resampled);
  EXPECT_EQ(half.poses.size(), 3U);
}

TEST(UpsampleTrajectory, UniformGridStaysWithinSpanWithLinearPosition)
{
  // Native 1 Hz; request 4 Hz -> 0.25 s step over [0, 2] s -> 9 samples.
  const std::vector<TrajectoryPose> in = {
    {0LL, 0, 0, 0, 0, 0, 0, 1},
    {1'000'000'000LL, 1, 0, 0, 0, 0, 0, 1},
    {2'000'000'000LL, 2, 0, 0, 0, 0, 0, 1}};

  const auto r = upsample_trajectory(in, {UpsampleMode::kFrequencyHz, 4.0});
  ASSERT_TRUE(r.resampled);
  ASSERT_EQ(r.poses.size(), 9U);

  // First sample at the start, last at or before the end, all within the span.
  EXPECT_EQ(r.poses.front().timestamp_ns, 0LL);
  EXPECT_EQ(r.poses.back().timestamp_ns, 2'000'000'000LL);
  for (const auto & p : r.poses) {
    EXPECT_GE(p.timestamp_ns, 0LL);
    EXPECT_LE(p.timestamp_ns, 2'000'000'000LL);
  }
  // Strictly-uniform 0.25 s spacing.
  for (std::size_t i = 1; i < r.poses.size(); ++i) {
    EXPECT_EQ(r.poses[i].timestamp_ns - r.poses[i - 1].timestamp_ns, 250'000'000LL);
  }
  // Linear position: tx == seconds along the straight x = t line.
  EXPECT_NEAR(r.poses[1].tx, 0.25, 1e-12);  // t = 0.25 s
  EXPECT_NEAR(r.poses[2].tx, 0.50, 1e-12);  // t = 0.50 s
}

TEST(UpsampleTrajectory, SlerpOrientationAtMidpoint)
{
  // Identity -> 90 deg about Z; the 2 Hz grid midpoint is the 45 deg rotation.
  const std::vector<TrajectoryPose> in = {
    {0LL, 0, 0, 0, 0, 0, 0, 1}, {1'000'000'000LL, 0, 0, 0, 0, 0, kSinPiOver4, kSinPiOver4}};

  const auto r = upsample_trajectory(in, {UpsampleMode::kFrequencyHz, 2.0});
  ASSERT_TRUE(r.resampled);
  ASSERT_EQ(r.poses.size(), 3U);
  EXPECT_EQ(r.poses[1].timestamp_ns, 500'000'000LL);
  EXPECT_NEAR(r.poses[1].qz, kSin22p5, 1e-9);
  EXPECT_NEAR(r.poses[1].qw, kCos22p5, 1e-9);
}

TEST(UpsampleTrajectory, SlerpTakesShortestPathAcrossDoubleCover)
{
  // The second quaternion is the negation of a 90 deg-about-Z rotation: the same
  // orientation, but a naive SLERP would spin the long way (> 180 deg). The
  // shortest-path midpoint must still be the +45 deg rotation.
  const std::vector<TrajectoryPose> in = {
    {0LL, 0, 0, 0, 0, 0, 0, 1}, {1'000'000'000LL, 0, 0, 0, 0, 0, -kSinPiOver4, -kSinPiOver4}};

  const auto r = upsample_trajectory(in, {UpsampleMode::kFrequencyHz, 2.0});
  ASSERT_EQ(r.poses.size(), 3U);
  EXPECT_GT(r.poses[1].qz, 0.0);
  EXPECT_NEAR(r.poses[1].qz, kSin22p5, 1e-9);
  EXPECT_NEAR(r.poses[1].qw, kCos22p5, 1e-9);
}

TEST(UpsampleTrajectory, MultiplierResolvesAgainstNativeRate)
{
  // Native 1 Hz; 2x -> 2 Hz -> 0.5 s step -> 3 samples.
  const std::vector<TrajectoryPose> in = {
    {0LL, 0, 0, 0, 0, 0, 0, 1},
    {1'000'000'000LL, 1, 0, 0, 0, 0, 0, 1},
    {2'000'000'000LL, 2, 0, 0, 0, 0, 0, 1}};

  const auto r = upsample_trajectory(in, {UpsampleMode::kMultiplier, 2.0});
  ASSERT_TRUE(r.resampled);
  EXPECT_NEAR(r.target_rate_hz, 2.0, 1e-12);
  // 2 Hz over [0, 2] s -> 0.5 s step -> 5 samples (0, 0.5, 1.0, 1.5, 2.0).
  EXPECT_EQ(r.poses.size(), 5U);
}

TEST(UpsampleTrajectory, LeavesHoleAcrossLargeGap)
{
  // Three tight 0.1 s samples, a 4.8 s dropout, then two more tight samples.
  const std::vector<TrajectoryPose> in = {
    {0LL, 0, 0, 0, 0, 0, 0, 1},
    {100'000'000LL, 1, 0, 0, 0, 0, 0, 1},
    {200'000'000LL, 2, 0, 0, 0, 0, 0, 1},
    {5'000'000'000LL, 3, 0, 0, 0, 0, 0, 1},
    {5'100'000'000LL, 4, 0, 0, 0, 0, 0, 1}};

  // 10 Hz grid (0.1 s step) with an explicit 0.3 s gap threshold for determinism.
  const auto r = upsample_trajectory(in, {UpsampleMode::kFrequencyHz, 10.0}, 0.3);
  ASSERT_TRUE(r.resampled);
  EXPECT_EQ(r.skipped_gap_count, 1);
  EXPECT_EQ(r.skipped_point_count, 47);
  EXPECT_DOUBLE_EQ(r.gap_threshold_s, 0.3);

  // No fabricated sample lands strictly inside the dropout; the real endpoints
  // (0.2 s and 5.0 s) survive and the span is still respected.
  EXPECT_EQ(r.poses.front().timestamp_ns, 0LL);
  EXPECT_EQ(r.poses.back().timestamp_ns, 5'100'000'000LL);
  for (const auto & p : r.poses) {
    const bool inside_gap = p.timestamp_ns > 200'000'000LL && p.timestamp_ns < 5'000'000'000LL;
    EXPECT_FALSE(inside_gap) << "stamp " << p.timestamp_ns << " is inside the dropout";
  }
}

TEST(UpsampleTrajectory, PreservesOriginalTimestampsAndPosesOnNonUniformInput)
{
  // Non-uniform spacing (0.07 s then 0.13 s) with deliberately un-round pose
  // values. Native rate = (3 - 1) / 0.2 s = 10 Hz; request 20 Hz.
  const TrajectoryPose a{0LL, 0.5, -1.25, 3.0, 0, 0, 0, 1};
  const TrajectoryPose b{70'000'000LL, 7.5, 2.0, -1.0, 0, 0, kSinPiOver4, kSinPiOver4};
  const TrajectoryPose c{200'000'000LL, 20.25, -4.5, 6.0, 0, 0, 0, 1};
  const std::vector<TrajectoryPose> in = {a, b, c};

  const auto r = upsample_trajectory(in, {UpsampleMode::kFrequencyHz, 20.0});
  ASSERT_TRUE(r.resampled);

  // Every original timestamp is retained, even 70 ms, which a uniform 20 Hz grid
  // laid from t_start (0, 50, 100, 150, 200 ms) would never land on.
  auto contains_stamp = [&](std::int64_t ts) {
    return std::any_of(r.poses.begin(), r.poses.end(), [ts](const TrajectoryPose & p) {
      return p.timestamp_ns == ts;
    });
  };
  EXPECT_TRUE(contains_stamp(0LL));
  EXPECT_TRUE(contains_stamp(70'000'000LL));
  EXPECT_TRUE(contains_stamp(200'000'000LL));

  // The retained originals carry their input values verbatim (endpoints are
  // copied, never re-interpolated).
  auto expect_same_pose = [](const TrajectoryPose & got, const TrajectoryPose & want) {
    EXPECT_EQ(got.timestamp_ns, want.timestamp_ns);
    EXPECT_DOUBLE_EQ(got.tx, want.tx);
    EXPECT_DOUBLE_EQ(got.ty, want.ty);
    EXPECT_DOUBLE_EQ(got.tz, want.tz);
    EXPECT_DOUBLE_EQ(got.qx, want.qx);
    EXPECT_DOUBLE_EQ(got.qy, want.qy);
    EXPECT_DOUBLE_EQ(got.qz, want.qz);
    EXPECT_DOUBLE_EQ(got.qw, want.qw);
  };
  expect_same_pose(r.poses.front(), a);
  expect_same_pose(r.poses.back(), c);
  ASSERT_GE(r.poses.size(), 2U);
  // The second emitted pose is the first original (no interior point fits the
  // 0.07 s segment at 20 Hz: round(0.07 * 20) == 1 sub-interval).
  expect_same_pose(r.poses[1], b);

  // Strictly time-increasing, within the original span, no extrapolation.
  for (std::size_t i = 1; i < r.poses.size(); ++i) {
    EXPECT_GT(r.poses[i].timestamp_ns, r.poses[i - 1].timestamp_ns);
    EXPECT_GE(r.poses[i].timestamp_ns, 0LL);
    EXPECT_LE(r.poses[i].timestamp_ns, 200'000'000LL);
  }
  // The 0.13 s segment is subdivided (round(0.13 * 20) == 3 -> 2 interior
  // points), so densification did happen beyond the three originals.
  EXPECT_GT(r.poses.size(), in.size());
}

}  // namespace
