// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pipeline/stage_profiler.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <string>

namespace bagwiz::core::pipeline
{
namespace
{

using std::chrono_literals::operator""ns;

// --- profile_value_enabled: parsing of the BAGWIZ_PROFILE env value ---------

TEST(ProfileValueEnabled, NullAndEmptyAndFalseyAreDisabled)
{
  EXPECT_FALSE(profile_value_enabled(nullptr));
  EXPECT_FALSE(profile_value_enabled(""));
  EXPECT_FALSE(profile_value_enabled("0"));
  EXPECT_FALSE(profile_value_enabled("false"));
  EXPECT_FALSE(profile_value_enabled("no"));
  EXPECT_FALSE(profile_value_enabled("off"));
}

TEST(ProfileValueEnabled, NonEmptyTruthyValuesAreEnabled)
{
  EXPECT_TRUE(profile_value_enabled("1"));
  EXPECT_TRUE(profile_value_enabled("yes"));
  EXPECT_TRUE(profile_value_enabled("true"));
  EXPECT_TRUE(profile_value_enabled("on"));
}

// --- accumulation ------------------------------------------------------------

TEST(StageProfiler, AddAccumulatesPerStageAndMessageCounters)
{
  StageProfiler prof(true);
  prof.add(Stage::kRead, 100ns);
  prof.add(Stage::kRead, 150ns);
  prof.add(Stage::kProcess, 40ns);
  prof.add(Stage::kWrite, 200ns);
  prof.add_message(/*in_bytes=*/10, /*out_bytes=*/10);
  prof.add_message(/*in_bytes=*/30, /*out_bytes=*/0);

  const auto & t = prof.totals();
  EXPECT_EQ(t.read_ns, 250);
  EXPECT_EQ(t.process_ns, 40);
  EXPECT_EQ(t.write_ns, 200);
  EXPECT_EQ(t.messages, 2u);
  EXPECT_EQ(t.in_bytes, 40u);
  EXPECT_EQ(t.out_bytes, 10u);
}

TEST(StageProfiler, DisabledScopeIsNoOp)
{
  StageProfiler prof(false);
  EXPECT_FALSE(prof.enabled());
  {
    auto s = prof.time(Stage::kRead);  // disabled -> must not accumulate
    (void)s;
  }
  EXPECT_EQ(prof.totals().read_ns, 0);
}

TEST(StageProfiler, EnabledScopeAccumulatesNonZero)
{
  StageProfiler prof(true);
  {
    auto s = prof.time(Stage::kWrite);
    volatile int sink = 0;
    for (int i = 0; i < 100000; ++i) {
      sink += i;
    }
    (void)sink;
  }
  EXPECT_GT(prof.totals().write_ns, 0);
}

// --- pure formatter ----------------------------------------------------------

TEST(FormatStageReport, IncludesCommandStagesAndBalancedPercents)
{
  StageTotals t;
  t.read_ns = 1'000'000'000;  // 1.0 s
  t.process_ns = 0;
  t.write_ns = 1'000'000'000;  // 1.0 s
  t.messages = 42;
  t.in_bytes = 2u * 1024 * 1024 * 1024;
  t.out_bytes = 2u * 1024 * 1024 * 1024;

  const std::string r = format_stage_report("topic drop", t);

  EXPECT_NE(r.find("topic drop"), std::string::npos);
  EXPECT_NE(r.find("read"), std::string::npos);
  EXPECT_NE(r.find("write"), std::string::npos);
  EXPECT_NE(r.find("42"), std::string::npos);
  // read and write each ~50% of the 2.0 s wall.
  EXPECT_NE(r.find("50"), std::string::npos);
}

TEST(FormatStageReport, HandlesZeroTotalsWithoutDivByZero)
{
  StageTotals t;  // all zero
  const std::string r = format_stage_report("empty", t);
  EXPECT_NE(r.find("empty"), std::string::npos);  // must not crash / must mention command
}

// The threaded backends accumulate read and write on concurrent threads, so
// their stage times sum to more than the run actually took. Reporting that sum
// as "wall" overstates the cost of exactly the backend whose overlap is the
// point, so the header must carry the measured elapsed time when there is one.
TEST(FormatStageReport, ReportsMeasuredElapsedAsWall)
{
  StageTotals t;
  t.read_ns = 1'000'000'000;   // 1.0 s
  t.write_ns = 1'000'000'000;  // 1.0 s
  t.elapsed_ns = 3'000'000'000;
  t.messages = 1;

  const std::string r = format_stage_report("topic drop", t);

  EXPECT_NE(r.find("wall 3.000 s"), std::string::npos) << r;
  EXPECT_EQ(r.find("wall 2.000 s"), std::string::npos) << r;
}

TEST(FormatStageReport, FlagsStagesThatOverlapInsteadOfInflatingWall)
{
  StageTotals t;
  t.read_ns = 1'000'000'000;  // 1.0 s
  t.write_ns = 500'000'000;   // 0.5 s, concurrent with the read
  t.elapsed_ns = 1'000'000'000;
  t.messages = 1;

  const std::string r = format_stage_report("topic drop", t);

  // The run took 1.0 s, not the 1.5 s the stages sum to.
  EXPECT_NE(r.find("wall 1.000 s"), std::string::npos) << r;
  EXPECT_EQ(r.find("wall 1.500 s"), std::string::npos) << r;
  // And the overlap is stated rather than left for the reader to infer.
  EXPECT_NE(r.find("overlap"), std::string::npos) << r;
  EXPECT_NE(r.find("1.500 s"), std::string::npos) << r;
}

TEST(FormatStageReport, DoesNotClaimOverlapWhenStagesAreSequential)
{
  StageTotals t;
  t.read_ns = 1'000'000'000;
  t.write_ns = 1'000'000'000;
  t.elapsed_ns = 2'000'000'000;  // stages ran back to back
  t.messages = 1;

  const std::string r = format_stage_report("topic drop", t);

  EXPECT_NE(r.find("wall 2.000 s"), std::string::npos) << r;
  EXPECT_EQ(r.find("overlap"), std::string::npos) << r;
}

TEST(FormatStageReport, FallsBackToTheStageSumWhenElapsedWasNotMeasured)
{
  StageTotals t;
  t.read_ns = 1'000'000'000;
  t.write_ns = 1'000'000'000;
  t.elapsed_ns = 0;  // caller did not measure
  t.messages = 1;

  const std::string r = format_stage_report("topic drop", t);

  EXPECT_NE(r.find("wall 2.000 s"), std::string::npos) << r;
  EXPECT_EQ(r.find("overlap"), std::string::npos) << r;
}

TEST(StageProfilerElapsed, SetElapsedLandsInTotals)
{
  StageProfiler prof(true);
  prof.set_elapsed(std::chrono::nanoseconds(1'500'000'000));
  EXPECT_EQ(prof.totals().elapsed_ns, 1'500'000'000);
}

}  // namespace
}  // namespace bagwiz::core::pipeline
