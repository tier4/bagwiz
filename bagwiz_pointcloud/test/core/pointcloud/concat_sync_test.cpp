// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/concat_sync.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace
{
using bagwiz::core::pointcloud::plan_sync;
using bagwiz::core::pointcloud::SyncGroup;
using bagwiz::core::pointcloud::SyncTopic;

constexpr std::int64_t kMs = 1'000'000;
}  // namespace

TEST(ConcatSync, ExactStampsPairOneToOne)
{
  std::vector<SyncTopic> topics{
    SyncTopic{{1000 * kMs, 1100 * kMs}, 0},
    SyncTopic{{1000 * kMs, 1100 * kMs}, 0},
  };
  const auto groups = plan_sync(topics, /*reference=*/0, /*tolerance=*/kMs);
  ASSERT_EQ(groups.size(), 2u);
  EXPECT_EQ(groups[0].output_stamp_ns, 1000 * kMs);
  ASSERT_TRUE(groups[0].picks[0].has_value());
  ASSERT_TRUE(groups[0].picks[1].has_value());
  EXPECT_EQ(*groups[0].picks[0], 0u);
  EXPECT_EQ(*groups[0].picks[1], 0u);
  EXPECT_EQ(*groups[1].picks[1], 1u);
}

TEST(ConcatSync, NearestWithinToleranceElseMissing)
{
  std::vector<SyncTopic> topics{
    SyncTopic{{1000 * kMs}, 0},
    SyncTopic{{1030 * kMs}, 0},  // 30 ms away
  };
  // tolerance 50 ms -> matched
  auto g = plan_sync(topics, 0, 50 * kMs);
  ASSERT_EQ(g.size(), 1u);
  ASSERT_TRUE(g[0].picks[1].has_value());
  EXPECT_EQ(*g[0].picks[1], 0u);

  // tolerance 20 ms -> partial (missing)
  g = plan_sync(topics, 0, 20 * kMs);
  ASSERT_EQ(g.size(), 1u);
  EXPECT_FALSE(g[0].picks[1].has_value());
  EXPECT_TRUE(g[0].picks[0].has_value());  // reference always present
}

// The stamp offset rescues a match that trigger skew would otherwise push out
// of tolerance: topic 1 fires ~60 ms early, so with no offset both its messages
// miss; +60 ms aligns its early message to the reference.
TEST(ConcatSync, StampOffsetCorrectsSkew)
{
  const std::int64_t ref = 1000 * kMs;
  std::vector<SyncTopic> no_offset{
    SyncTopic{{ref}, 0},
    SyncTopic{{940 * kMs, 1060 * kMs}, 0},
  };
  auto g = plan_sync(no_offset, 0, 52 * kMs);
  ASSERT_EQ(g.size(), 1u);
  EXPECT_FALSE(g[0].picks[1].has_value());  // 60 ms away both sides -> missing

  std::vector<SyncTopic> with_offset{
    SyncTopic{{ref}, 0},
    SyncTopic{{940 * kMs, 1060 * kMs}, 60 * kMs},  // +60 ms
  };
  g = plan_sync(with_offset, 0, 52 * kMs);
  ASSERT_EQ(g.size(), 1u);
  ASSERT_TRUE(g[0].picks[1].has_value());
  EXPECT_EQ(*g[0].picks[1], 0u);  // the early message now aligns
}

TEST(ConcatSync, TieResolvesToEarliest)
{
  std::vector<SyncTopic> topics{
    SyncTopic{{1000 * kMs}, 0},
    SyncTopic{{950 * kMs, 1050 * kMs}, 0},  // both 50 ms away
  };
  const auto g = plan_sync(topics, 0, 60 * kMs);
  ASSERT_EQ(g.size(), 1u);
  ASSERT_TRUE(g[0].picks[1].has_value());
  EXPECT_EQ(*g[0].picks[1], 0u);  // earliest stamp wins the tie
}

// A message exactly `tolerance_ns` away still matches (inclusive boundary).
TEST(ConcatSync, ToleranceBoundaryIsInclusive)
{
  std::vector<SyncTopic> topics{
    SyncTopic{{1000 * kMs}, 0},
    SyncTopic{{1050 * kMs}, 0},  // exactly 50 ms away
  };
  const auto g = plan_sync(topics, 0, 50 * kMs);
  ASSERT_EQ(g.size(), 1u);
  EXPECT_TRUE(g[0].picks[1].has_value());
}

// The reference topic's own --stamp-offset shifts the match target for the other
// topics (match_target = ref.stamp + ref.offset).
TEST(ConcatSync, ReferenceOffsetShiftsMatchTarget)
{
  std::vector<SyncTopic> topics{
    SyncTopic{{1000 * kMs}, 50 * kMs},  // reference: target = 1050 ms
    SyncTopic{{1000 * kMs, 1050 * kMs}, 0},
  };
  const auto g = plan_sync(topics, 0, 10 * kMs);  // tight tolerance
  ASSERT_EQ(g.size(), 1u);
  ASSERT_TRUE(g[0].picks[1].has_value());
  EXPECT_EQ(*g[0].picks[1], 1u);                // the 1050 ms message aligns to the shifted target
  EXPECT_EQ(g[0].output_stamp_ns, 1000 * kMs);  // output stamp is the raw ref stamp
}

TEST(ConcatSync, EmptyOtherTopicIsMissing)
{
  std::vector<SyncTopic> topics{
    SyncTopic{{1000 * kMs}, 0},
    SyncTopic{{}, 0},
  };
  const auto g = plan_sync(topics, 0, 50 * kMs);
  ASSERT_EQ(g.size(), 1u);
  EXPECT_FALSE(g[0].picks[1].has_value());
}
