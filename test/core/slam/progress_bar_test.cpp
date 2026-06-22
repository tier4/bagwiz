// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/progress_bar.hpp"

#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

// Unit tests for the `slam run` progress helpers. The pure decision functions
// (progress_enabled / progress_total) get a truth-table / edge sweep; the RAII
// reporters are exercised only on the DISABLED path, which must be a complete
// no-op (no thread, no terminal output) and is the behavior the rest of the
// codebase relies on when stderr is not a TTY.
namespace
{
namespace slam = bagwiz::core::slam;
using bagwiz::io::BagReader;

TEST(ProgressEnabled, ShownOnlyOnInteractiveStderrWithoutOptOut)
{
  // tty, no NO_COLOR, no --no-progress -> shown.
  EXPECT_TRUE(slam::progress_enabled(true, false, false));
}

TEST(ProgressEnabled, SuppressedOffATty)
{
  EXPECT_FALSE(slam::progress_enabled(false, false, false));
}

TEST(ProgressEnabled, SuppressedUnderNoColor)
{
  EXPECT_FALSE(slam::progress_enabled(true, true, false));
}

TEST(ProgressEnabled, SuppressedByNoProgressFlag)
{
  EXPECT_FALSE(slam::progress_enabled(true, false, true));
}

TEST(ProgressTotal, SumsCountsForRequestedTopicsOnly)
{
  BagReader::Stats stats;
  stats.per_topic["/lidar"] = 100;
  stats.per_topic["/imu"] = 5000;
  stats.per_topic["/gnss"] = 30;
  stats.per_topic["/camera"] = 99999;  // not requested -> excluded

  const std::vector<std::string> topics{"/lidar", "/imu", "/gnss"};
  EXPECT_EQ(slam::progress_total(stats, topics), 5130);
}

TEST(ProgressTotal, SkipsEmptyTopicNames)
{
  BagReader::Stats stats;
  stats.per_topic["/lidar"] = 42;

  // The IMU / GNSS slots are empty (those features off) and must be ignored.
  const std::vector<std::string> topics{"/lidar", "", ""};
  EXPECT_EQ(slam::progress_total(stats, topics), 42);
}

TEST(ProgressTotal, ZeroWhenNoCountsKnown)
{
  // e.g. an MCAP without summary statistics: per_topic is empty.
  BagReader::Stats stats;
  const std::vector<std::string> topics{"/lidar", "/imu"};
  EXPECT_EQ(slam::progress_total(stats, topics), 0);
}

TEST(ProgressTotal, IgnoresNonPositiveCounts)
{
  BagReader::Stats stats;
  stats.per_topic["/lidar"] = 0;
  stats.per_topic["/imu"] = -1;  // defensive: never negative in practice
  const std::vector<std::string> topics{"/lidar", "/imu"};
  EXPECT_EQ(slam::progress_total(stats, topics), 0);
}

TEST(ScanProgressDisabled, DeterminateIsNoOp)
{
  slam::ScanProgress progress(1000, /*enabled=*/false);
  progress.update(0, 0);
  progress.update(500, 17);
  progress.update(1000, 42);
  progress.done();
  progress.done();  // idempotent
  SUCCEED();
}

TEST(ScanProgressDisabled, IndeterminateIsNoOp)
{
  // total <= 0 would select the indeterminate bar when enabled; disabled it
  // must still be inert.
  slam::ScanProgress progress(0, /*enabled=*/false);
  progress.update(123, 4);
  progress.done();
  SUCCEED();
}

TEST(FinalizeSpinnerDisabled, StartsNoThreadAndStopsCleanly)
{
  slam::FinalizeSpinner spinner("Optimizing global map", /*enabled=*/false);
  spinner.stop();
  spinner.stop();  // idempotent
  SUCCEED();
}

}  // namespace
