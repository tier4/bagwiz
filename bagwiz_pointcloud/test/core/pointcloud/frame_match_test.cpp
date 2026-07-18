// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/fetcher.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace
{

using bagwiz::core::pointcloud::choose_frame_match;
using bagwiz::core::pointcloud::PointCloudMatchKey;

constexpr std::int64_t kFrameStampNs = 1'700'000'000'123'456'789;
constexpr std::int64_t kFrameRecordNs = 1'700'000'001'000'000'000;

TEST(FrameMatch, CaptureTimeOnlyWhenBothSidesCarryStamps)
{
  const auto match = choose_frame_match(kFrameStampNs, kFrameRecordNs, true);
  EXPECT_EQ(match.target_ns, kFrameStampNs);
  EXPECT_EQ(match.key, PointCloudMatchKey::kHeaderStamp);
}

TEST(FrameMatch, RecordTimeWhenCloudTopicLacksStamps)
{
  // The frame has a header.stamp but the topic does not, so the compare must
  // fall back to bag record time on both sides (one clock).
  const auto match = choose_frame_match(kFrameStampNs, kFrameRecordNs, false);
  EXPECT_EQ(match.target_ns, kFrameRecordNs);
  EXPECT_EQ(match.key, PointCloudMatchKey::kRecordTime);
}

TEST(FrameMatch, RecordTimeWhenFrameLacksStamp)
{
  const auto match = choose_frame_match(0, kFrameRecordNs, true);
  EXPECT_EQ(match.target_ns, kFrameRecordNs);
  EXPECT_EQ(match.key, PointCloudMatchKey::kRecordTime);
}

TEST(FrameMatch, RecordTimeWhenNeitherSideHasStamps)
{
  const auto match = choose_frame_match(0, kFrameRecordNs, false);
  EXPECT_EQ(match.target_ns, kFrameRecordNs);
  EXPECT_EQ(match.key, PointCloudMatchKey::kRecordTime);
}

}  // namespace
