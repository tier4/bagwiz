// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/trajectory.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

namespace
{

using bagwiz::core::pose_to_transform_stamped;
using bagwiz::core::read_tum;
using bagwiz::core::TrajectoryPose;
using bagwiz::core::write_tum;

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

}  // namespace
