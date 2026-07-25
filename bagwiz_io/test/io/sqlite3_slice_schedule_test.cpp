// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "io/sqlite3_slice_schedule.hpp"  // NOLINT(build/include_subdir) src-local header under test

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

// build_slice_schedule() splits a bag's time extent into disjoint half-open
// timestamp ranges whose union is exactly the row set the serial scan would
// emit. It returns an empty schedule whenever a parallel scan is not worth it
// (or not representable), and the caller then stays on the serial path.
namespace
{

using bagwiz::io::detail::build_slice_schedule;
using bagwiz::io::detail::SliceScheduleParams;

// A bag whose extent is [1000, 1999] and whose size asks for 4 slices.
SliceScheduleParams base_params()
{
  SliceScheduleParams p;
  p.extent_start_ns = 1000;
  p.extent_end_ns = 1999;
  p.file_size_bytes = 4096;
  p.target_slice_bytes = 1024;  // 4096 / 1024 = 4 slices
  p.max_slices = 4096;
  return p;
}

}  // namespace

TEST(Sqlite3SliceScheduleTest, SplitsExtentIntoContiguousHalfOpenRanges)
{
  const auto slices = build_slice_schedule(base_params());
  ASSERT_EQ(slices.size(), 4u);

  // Outer bounds are unset (no caller filter), so the union covers every row.
  EXPECT_FALSE(slices.front().start_ns.has_value());
  EXPECT_FALSE(slices.back().end_ns.has_value());

  // step = span / n = 1000 / 4 = 250.
  EXPECT_EQ(slices[0].end_ns, std::optional<std::int64_t>(1250));
  EXPECT_EQ(slices[1].start_ns, std::optional<std::int64_t>(1250));
  EXPECT_EQ(slices[1].end_ns, std::optional<std::int64_t>(1500));
  EXPECT_EQ(slices[2].start_ns, std::optional<std::int64_t>(1500));
  EXPECT_EQ(slices[2].end_ns, std::optional<std::int64_t>(1750));
  EXPECT_EQ(slices[3].start_ns, std::optional<std::int64_t>(1750));
}

TEST(Sqlite3SliceScheduleTest, AdjacentSlicesShareTheirBoundaryExactly)
{
  const auto slices = build_slice_schedule(base_params());
  ASSERT_GE(slices.size(), 2u);
  for (std::size_t i = 1; i < slices.size(); ++i) {
    ASSERT_TRUE(slices[i].start_ns.has_value());
    ASSERT_TRUE(slices[i - 1].end_ns.has_value());
    // Half-open [start, end): the previous slice's exclusive end is the next
    // slice's inclusive start, so no row is dropped or emitted twice.
    EXPECT_EQ(*slices[i].start_ns, *slices[i - 1].end_ns);
  }
}

TEST(Sqlite3SliceScheduleTest, CarriesTheCallerFilterOnTheOuterBounds)
{
  auto p = base_params();
  p.filter_start_ns = 1100;
  p.filter_end_ns = 1900;  // exclusive
  const auto slices = build_slice_schedule(p);
  ASSERT_FALSE(slices.empty());
  EXPECT_EQ(slices.front().start_ns, std::optional<std::int64_t>(1100));
  EXPECT_EQ(slices.back().end_ns, std::optional<std::int64_t>(1900));
  // Interior boundaries are derived from the clamped extent [1100, 1899].
  ASSERT_TRUE(slices.front().end_ns.has_value());
  ASSERT_TRUE(slices.back().start_ns.has_value());
  EXPECT_GT(*slices.front().end_ns, 1100);
  EXPECT_LT(*slices.back().start_ns, 1900);
}

TEST(Sqlite3SliceScheduleTest, ReturnsEmptyWhenFileIsSmallerThanOneSlice)
{
  auto p = base_params();
  p.file_size_bytes = 512;  // < target_slice_bytes => 1 slice => not worth it
  EXPECT_TRUE(build_slice_schedule(p).empty());
}

TEST(Sqlite3SliceScheduleTest, ReturnsEmptyWhenTheFilterExcludesEverything)
{
  auto p = base_params();
  p.filter_end_ns = 1000;  // exclusive, so nothing at or after extent_start
  EXPECT_TRUE(build_slice_schedule(p).empty());
}

TEST(Sqlite3SliceScheduleTest, ReturnsEmptyForAZeroWidthExtent)
{
  auto p = base_params();
  p.extent_start_ns = 1000;
  p.extent_end_ns = 1000;  // one distinct timestamp: nothing to split
  EXPECT_TRUE(build_slice_schedule(p).empty());
}

TEST(Sqlite3SliceScheduleTest, ClampsSliceCountToMaxSlices)
{
  auto p = base_params();
  p.file_size_bytes = 1024ULL * 1024ULL;  // would ask for 1024 slices
  p.max_slices = 8;
  EXPECT_EQ(build_slice_schedule(p).size(), 8u);
}

TEST(Sqlite3SliceScheduleTest, NeverProducesMoreSlicesThanDistinctTimestamps)
{
  auto p = base_params();
  p.extent_start_ns = 1000;
  p.extent_end_ns = 1002;                 // span = 3
  p.file_size_bytes = 1024ULL * 1024ULL;  // would ask for 1024 slices
  const auto slices = build_slice_schedule(p);
  ASSERT_EQ(slices.size(), 3u);
  EXPECT_EQ(slices[0].end_ns, std::optional<std::int64_t>(1001));
  EXPECT_EQ(slices[1].end_ns, std::optional<std::int64_t>(1002));
}

TEST(Sqlite3SliceScheduleTest, ReturnsEmptyWhenTargetSliceBytesIsZero)
{
  auto p = base_params();
  p.target_slice_bytes = 0;  // guards the division
  EXPECT_TRUE(build_slice_schedule(p).empty());
}
