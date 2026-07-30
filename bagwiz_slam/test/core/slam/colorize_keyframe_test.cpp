// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/colorize_keyframe.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <stdexcept>
#include <utility>
#include <vector>

namespace
{
namespace slam = bagwiz::core::slam;
using bagwiz::core::TrajectoryPose;

constexpr std::int64_t kSecondNs = 1'000'000'000;

TrajectoryPose make_pose(std::int64_t stamp_ns, double tx, double yaw_deg = 0.0)
{
  TrajectoryPose pose;
  pose.timestamp_ns = stamp_ns;
  pose.tx = tx;
  const double half = yaw_deg * std::numbers::pi / 180.0 / 2.0;
  pose.qz = std::sin(half);
  pose.qw = std::cos(half);
  return pose;
}

// Straight-line trajectory along +x at 1 m/s, one pose per second over
// [0, span_s]: the pose at stamp t is (t seconds, t meters).
std::vector<TrajectoryPose> make_line_trajectory(int span_s)
{
  std::vector<TrajectoryPose> poses;
  for (int s = 0; s <= span_s; ++s) {
    poses.push_back(make_pose(s * kSecondNs, static_cast<double>(s)));
  }
  return poses;
}

// Stationary trajectory that yaws at 6 deg/s, one pose per second. 6 deg/s
// keeps every gate comparison comfortably away from the 10-degree threshold
// (5 deg/s would test the >= boundary exactly, which floating-point
// quaternion round-trips cannot hit reliably).
std::vector<TrajectoryPose> make_turn_trajectory(int span_s)
{
  std::vector<TrajectoryPose> poses;
  for (int s = 0; s <= span_s; ++s) {
    poses.push_back(make_pose(s * kSecondNs, 0.0, 6.0 * s));
  }
  return poses;
}

std::vector<std::byte> make_flat_raster(
  std::uint32_t width, std::uint32_t height, std::uint8_t level)
{
  return std::vector<std::byte>(
    static_cast<std::size_t>(width) * 3U * height, static_cast<std::byte>(level));
}

// Packed BGR24 raster of alternating black/white columns with the given
// period: the smaller the period, the denser the edges and the higher the
// mean Sobel magnitude.
std::vector<std::byte> make_stripe_raster(
  std::uint32_t width, std::uint32_t height, std::uint32_t period)
{
  std::vector<std::byte> raster(static_cast<std::size_t>(width) * 3U * height, std::byte{0});
  for (std::uint32_t v = 0; v < height; ++v) {
    for (std::uint32_t u = 0; u < width; ++u) {
      if ((u / period) % 2 == 0) {
        continue;
      }
      const std::size_t base = (static_cast<std::size_t>(v) * width + u) * 3U;
      raster[base + 0] = raster[base + 1] = raster[base + 2] = static_cast<std::byte>(255);
    }
  }
  return raster;
}

slam::ColorizeKeyframePicker::Frame make_frame(
  std::int64_t stamp_ns, std::vector<std::byte> bgr, std::uint32_t width, std::uint32_t height)
{
  slam::ColorizeKeyframePicker::Frame frame;
  frame.stamp_ns = stamp_ns;
  frame.bgr = std::move(bgr);
  frame.width = width;
  frame.height = height;
  return frame;
}

TEST(ImageSharpnessScore, UniformImageScoresZero)
{
  const auto raster = make_flat_raster(16, 16, 128);
  EXPECT_DOUBLE_EQ(slam::image_sharpness_score(raster, 16, 16), 0.0);
}

TEST(ImageSharpnessScore, DenserEdgesScoreHigher)
{
  // Period 2, not 1: the Sobel x kernel samples u - 1 and u + 1, so a
  // 1-pixel-period stripe aliases to a zero gradient everywhere.
  const auto sharp = make_stripe_raster(32, 32, 2);
  const auto soft = make_stripe_raster(32, 32, 8);
  const double sharp_score = slam::image_sharpness_score(sharp, 32, 32);
  const double soft_score = slam::image_sharpness_score(soft, 32, 32);
  EXPECT_GT(sharp_score, soft_score);
  EXPECT_GT(soft_score, 0.0);
}

TEST(ImageSharpnessScore, DegenerateInputsScoreZero)
{
  const auto raster = make_stripe_raster(2, 2, 1);
  // No interior pixels.
  EXPECT_DOUBLE_EQ(slam::image_sharpness_score(raster, 2, 2), 0.0);
  // Size mismatch.
  EXPECT_DOUBLE_EQ(slam::image_sharpness_score(raster, 16, 16), 0.0);
  // Empty raster.
  EXPECT_DOUBLE_EQ(slam::image_sharpness_score({}, 0, 0), 0.0);
}

TEST(ColorizeKeyframePicker, DisabledGateAcceptsEveryFrame)
{
  const auto trajectory = make_line_trajectory(10);
  slam::ColorizeKeyframePicker picker({.min_dist = 0.0}, trajectory);
  for (int s = 0; s <= 10; ++s) {
    EXPECT_TRUE(picker.accept(s * kSecondNs));
  }
  EXPECT_EQ(picker.kept(), 0U);
  EXPECT_EQ(picker.skipped(), 0U);
}

TEST(ColorizeKeyframePicker, DistanceGateThinsCloseFrames)
{
  const auto trajectory = make_line_trajectory(10);
  slam::ColorizeKeyframePicker picker({.min_dist = 2.0}, trajectory);
  EXPECT_TRUE(picker.accept(0));                               // first frame anchors at x = 0
  EXPECT_FALSE(picker.accept(1 * kSecondNs));                  // 1 m from the anchor
  EXPECT_FALSE(picker.accept(1 * kSecondNs + kSecondNs / 2));  // 1.5 m
  EXPECT_TRUE(picker.accept(2 * kSecondNs));                   // 2 m: new bucket at x = 2
  EXPECT_FALSE(picker.accept(3 * kSecondNs));                  // 1 m from the NEW anchor
  EXPECT_TRUE(picker.accept(4 * kSecondNs));
  EXPECT_EQ(picker.kept(), 3U);
  EXPECT_EQ(picker.skipped(), 3U);
}

TEST(ColorizeKeyframePicker, RotationOpensTheGateWithoutDistance)
{
  const auto trajectory = make_turn_trajectory(10);  // 6 deg/s in place
  slam::ColorizeKeyframePicker picker({.min_dist = 2.0, .min_rot_deg = 10.0}, trajectory);
  EXPECT_TRUE(picker.accept(0));               // anchors at yaw 0
  EXPECT_FALSE(picker.accept(1 * kSecondNs));  // 6 deg
  EXPECT_TRUE(picker.accept(2 * kSecondNs));   // 12 deg: rotation opens the gate
  EXPECT_FALSE(picker.accept(3 * kSecondNs));  // 6 deg from the new anchor
  EXPECT_EQ(picker.kept(), 2U);
  EXPECT_EQ(picker.skipped(), 2U);
}

TEST(ColorizeKeyframePicker, OutOfSpanStampsBypassTheGate)
{
  const auto trajectory = make_line_trajectory(10);
  slam::ColorizeKeyframePicker picker({.min_dist = 2.0}, trajectory);
  EXPECT_TRUE(picker.accept(-kSecondNs));      // before the span
  EXPECT_TRUE(picker.accept(11 * kSecondNs));  // after the span
  EXPECT_EQ(picker.kept(), 0U);                // bypasses count in neither
  EXPECT_EQ(picker.skipped(), 0U);
  EXPECT_TRUE(picker.accept(0));  // the gate itself is untouched
  EXPECT_FALSE(picker.accept(1 * kSecondNs));
  EXPECT_EQ(picker.kept(), 1U);
  EXPECT_EQ(picker.skipped(), 1U);
}

TEST(ColorizeKeyframePicker, BlurPathDispatchesTheSharpestBucketMember)
{
  const auto trajectory = make_line_trajectory(10);
  slam::ColorizeKeyframePicker picker({.min_dist = 2.0, .blur = true}, trajectory);

  // Bucket 1 (x in [0, 2)): a soft frame, a sharp frame, a flat frame.
  EXPECT_FALSE(picker.offer(make_frame(0, make_stripe_raster(32, 32, 8), 32, 32)).has_value());
  EXPECT_FALSE(
    picker.offer(make_frame(kSecondNs / 2, make_stripe_raster(32, 32, 2), 32, 32)).has_value());
  EXPECT_FALSE(
    picker.offer(make_frame(kSecondNs, make_flat_raster(32, 32, 128), 32, 32)).has_value());

  // The frame at x = 2 opens bucket 2 and dispatches bucket 1's sharpest.
  auto dispatched = picker.offer(make_frame(2 * kSecondNs, make_flat_raster(32, 32, 0), 32, 32));
  ASSERT_TRUE(dispatched.has_value());
  EXPECT_EQ(dispatched->stamp_ns, kSecondNs / 2);

  // flush() emits the final bucket's only member.
  auto last = picker.flush();
  ASSERT_TRUE(last.has_value());
  EXPECT_EQ(last->stamp_ns, 2 * kSecondNs);
  EXPECT_FALSE(picker.flush().has_value());  // idempotent

  EXPECT_EQ(picker.kept(), 2U);
  EXPECT_EQ(picker.skipped(), 2U);
}

TEST(ColorizeKeyframePicker, BlurPathKeepsTheEarlierFrameOnATie)
{
  const auto trajectory = make_line_trajectory(10);
  slam::ColorizeKeyframePicker picker({.min_dist = 2.0, .blur = true}, trajectory);

  // Two identical-sharpness frames in one bucket: the earlier one wins.
  EXPECT_FALSE(picker.offer(make_frame(0, make_stripe_raster(32, 32, 2), 32, 32)).has_value());
  EXPECT_FALSE(
    picker.offer(make_frame(kSecondNs, make_stripe_raster(32, 32, 2), 32, 32)).has_value());
  auto dispatched = picker.flush();
  ASSERT_TRUE(dispatched.has_value());
  EXPECT_EQ(dispatched->stamp_ns, 0);
}

TEST(ColorizeKeyframePicker, BlurPathPassesOutOfSpanFramesThrough)
{
  const auto trajectory = make_line_trajectory(10);
  slam::ColorizeKeyframePicker picker({.min_dist = 2.0, .blur = true}, trajectory);

  // Open a bucket, then offer an out-of-span frame: it comes straight back
  // without disturbing the buffered candidate.
  EXPECT_FALSE(picker.offer(make_frame(0, make_stripe_raster(32, 32, 1), 32, 32)).has_value());
  auto bypass = picker.offer(make_frame(11 * kSecondNs, make_flat_raster(32, 32, 0), 32, 32));
  ASSERT_TRUE(bypass.has_value());
  EXPECT_EQ(bypass->stamp_ns, 11 * kSecondNs);
  auto last = picker.flush();
  ASSERT_TRUE(last.has_value());
  EXPECT_EQ(last->stamp_ns, 0);
}

TEST(ColorizeKeyframePicker, BlurPathCarriesFramePayloadThrough)
{
  const auto trajectory = make_line_trajectory(10);
  slam::ColorizeKeyframePicker picker({.min_dist = 2.0, .blur = true}, trajectory);

  auto frame = make_frame(0, make_stripe_raster(32, 32, 1), 32, 32);
  frame.dynamic_points = {{1.0F, 2.0F, 3.0F}};
  EXPECT_FALSE(picker.offer(std::move(frame)).has_value());
  auto dispatched = picker.flush();
  ASSERT_TRUE(dispatched.has_value());
  EXPECT_EQ(dispatched->width, 32U);
  EXPECT_EQ(dispatched->height, 32U);
  EXPECT_EQ(dispatched->bgr.size(), 32U * 3U * 32U);
  ASSERT_EQ(dispatched->dynamic_points.size(), 1U);
  EXPECT_FLOAT_EQ(dispatched->dynamic_points[0][2], 3.0F);
}

TEST(ColorizeKeyframePicker, EmptyTrajectoryBypassesTheGate)
{
  slam::ColorizeKeyframePicker picker({.min_dist = 2.0}, {});
  EXPECT_TRUE(picker.accept(0));
  EXPECT_TRUE(picker.accept(kSecondNs));
  EXPECT_EQ(picker.kept(), 0U);
  EXPECT_EQ(picker.skipped(), 0U);
}

TEST(ColorizeKeyframePicker, WrongModeEntryPointThrows)
{
  const auto trajectory = make_line_trajectory(10);

  // accept() is the blur-off entry point: a blur-configured picker rejects it
  // (mixing the modes would drop frames silently, see the class comment).
  slam::ColorizeKeyframePicker blur_picker({.min_dist = 2.0, .blur = true}, trajectory);
  EXPECT_THROW((void)blur_picker.accept(0), std::logic_error);

  // offer()/flush() are the blur entry points: a non-blur picker rejects them.
  slam::ColorizeKeyframePicker gate_picker({.min_dist = 2.0}, trajectory);
  EXPECT_THROW(
    (void)gate_picker.offer(make_frame(0, make_flat_raster(32, 32, 0), 32, 32)), std::logic_error);
  EXPECT_THROW((void)gate_picker.flush(), std::logic_error);
}

}  // namespace
