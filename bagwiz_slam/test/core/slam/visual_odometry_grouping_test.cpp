// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "visual_odometry_grouping.hpp"  // NOLINT(build/include_subdir) src-local header

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace
{
namespace slam = bagwiz::core::slam;

slam::VisualObservation obs(std::int32_t cam, std::uint64_t track, std::int64_t stamp_ns)
{
  slam::VisualObservation o;
  o.camera_id = cam;
  o.track_id = track;
  o.stamp_ns = stamp_ns;
  return o;
}

constexpr std::int64_t kPeriod = 100'000'000;  // 100 ms

slam::GroupingBuffer::Config three_cameras()
{
  slam::GroupingBuffer::Config cfg;
  cfg.anchor_camera_id = 0;
  cfg.period_ns = kPeriod;
  cfg.camera_count = 3;
  return cfg;
}

TEST(GroupingBuffer, StaggeredObservationsJoinTheirAnchorWindow)
{
  slam::GroupingBuffer buf(three_cameras());
  // Anchor frames at 0 and 100 ms; camera 1 staggered +30 ms, camera 2 +79 ms
  // (the validation rigs' worst spread) — all must join the window their
  // stamp falls in, not the nearest cluster.
  buf.insert(std::vector<slam::VisualObservation>{obs(0, 1, 0), obs(0, 2, 0)});
  buf.insert(std::vector<slam::VisualObservation>{obs(1, 1, 30'000'000)});
  buf.insert(std::vector<slam::VisualObservation>{obs(2, 1, 79'000'000)});
  buf.insert(std::vector<slam::VisualObservation>{obs(0, 1, kPeriod)});
  buf.insert(std::vector<slam::VisualObservation>{obs(1, 1, kPeriod + 30'000'000)});
  buf.insert(std::vector<slam::VisualObservation>{obs(2, 1, kPeriod + 79'000'000)});

  // Every camera has advanced past window [0, 100ms) only after the second
  // round of frames above.
  const auto ready = buf.pop_ready();
  ASSERT_EQ(ready.size(), 1U);
  EXPECT_EQ(ready[0].anchor_stamp_ns, 0);
  EXPECT_EQ(ready[0].observations.size(), 4U);  // 2 anchor obs + cam1 + cam2

  const auto rest = buf.finish();
  ASSERT_EQ(rest.size(), 1U);
  EXPECT_EQ(rest[0].anchor_stamp_ns, kPeriod);
  EXPECT_EQ(rest[0].observations.size(), 3U);
  EXPECT_EQ(buf.dropped_count(), 0);
}

TEST(GroupingBuffer, SimultaneousTriggersAreTheDegenerateCase)
{
  slam::GroupingBuffer buf(three_cameras());
  for (std::int64_t k = 0; k < 3; ++k) {
    buf.insert(
      std::vector<slam::VisualObservation>{
        obs(0, 1, k * kPeriod), obs(1, 1, k * kPeriod), obs(2, 1, k * kPeriod)});
  }
  auto groups = buf.pop_ready();
  const auto rest = buf.finish();
  groups.insert(groups.end(), rest.begin(), rest.end());
  ASSERT_EQ(groups.size(), 3U);
  for (std::size_t k = 0; k < 3; ++k) {
    EXPECT_EQ(groups[k].anchor_stamp_ns, static_cast<std::int64_t>(k) * kPeriod);
    EXPECT_EQ(groups[k].observations.size(), 3U);
  }
  EXPECT_EQ(buf.dropped_count(), 0);
}

TEST(GroupingBuffer, ObservationBeforeItsAnchorArrivesIsHeldNotDropped)
{
  slam::GroupingBuffer buf(three_cameras());
  // Camera 1's frame for window [100ms, 200ms) arrives BEFORE the anchor
  // frame that opens the window (cross-camera arrival order is arbitrary).
  buf.insert(std::vector<slam::VisualObservation>{obs(0, 1, 0)});
  buf.insert(std::vector<slam::VisualObservation>{obs(1, 1, 130'000'000)});
  buf.insert(std::vector<slam::VisualObservation>{obs(0, 1, kPeriod)});
  const auto groups = buf.finish();
  ASSERT_EQ(groups.size(), 2U);
  EXPECT_EQ(groups[1].anchor_stamp_ns, kPeriod);
  EXPECT_EQ(groups[1].observations.size(), 2U);  // anchor obs + held camera-1 obs
  EXPECT_EQ(buf.dropped_count(), 0);
}

TEST(GroupingBuffer, AnchorFrameDropDropsUncoveredObservations)
{
  slam::GroupingBuffer buf(three_cameras());
  // Anchor frames at 0 and 300 ms (frames at 100/200 ms dropped). Camera 1
  // observations in the uncovered gap must be dropped and counted, and one
  // in a covered window must survive.
  buf.insert(std::vector<slam::VisualObservation>{obs(0, 1, 0)});
  buf.insert(std::vector<slam::VisualObservation>{obs(1, 1, 150'000'000)});
  buf.insert(std::vector<slam::VisualObservation>{obs(1, 2, 330'000'000)});
  buf.insert(std::vector<slam::VisualObservation>{obs(0, 1, 3 * kPeriod)});
  const auto groups = buf.finish();
  ASSERT_EQ(groups.size(), 2U);
  EXPECT_EQ(groups[0].observations.size(), 1U);
  EXPECT_EQ(groups[1].observations.size(), 2U);
  EXPECT_EQ(buf.dropped_count(), 1);
}

TEST(GroupingBuffer, SilentCameraReleasesAfterMaxLag)
{
  auto cfg = three_cameras();
  cfg.max_lag_periods = 2;
  slam::GroupingBuffer buf(cfg);
  // Camera 2 never delivers. Groups must still release once any stream head
  // is 2 periods past the window end.
  for (std::int64_t k = 0; k < 4; ++k) {
    buf.insert(std::vector<slam::VisualObservation>{obs(0, 1, k * kPeriod)});
    buf.insert(std::vector<slam::VisualObservation>{obs(1, 1, k * kPeriod + 30'000'000)});
  }
  const auto ready = buf.pop_ready();
  ASSERT_GE(ready.size(), 1U);
  EXPECT_EQ(ready[0].anchor_stamp_ns, 0);
}

TEST(GroupingBuffer, LateObservationForPoppedGroupIsDroppedAndCounted)
{
  auto cfg = three_cameras();
  cfg.max_lag_periods = 2;
  slam::GroupingBuffer buf(cfg);
  for (std::int64_t k = 0; k < 4; ++k) {
    buf.insert(std::vector<slam::VisualObservation>{obs(0, 1, k * kPeriod)});
    buf.insert(std::vector<slam::VisualObservation>{obs(1, 1, k * kPeriod + 30'000'000)});
  }
  const auto ready = buf.pop_ready();
  ASSERT_GE(ready.size(), 1U);
  // Camera 2 wakes up with an observation for the already-popped window 0.
  buf.insert(std::vector<slam::VisualObservation>{obs(2, 1, 50'000'000)});
  EXPECT_EQ(buf.dropped_count(), 1);
}

TEST(GroupingBuffer, RejectsAnchorOutsideCameraCount)
{
  auto cfg = three_cameras();
  cfg.anchor_camera_id = 3;  // cameras are 0..2
  EXPECT_THROW(slam::GroupingBuffer{cfg}, std::invalid_argument);
  cfg.anchor_camera_id = 0;
  cfg.camera_count = 0;
  EXPECT_THROW(slam::GroupingBuffer{cfg}, std::invalid_argument);
}

}  // namespace
