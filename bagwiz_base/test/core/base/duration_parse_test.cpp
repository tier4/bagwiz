// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/base/duration_parse.hpp"

#include <gtest/gtest.h>

#include <optional>

namespace
{
using bagwiz::core::parse_duration_ns;
}

TEST(ParseDurationNs, UnitSuffixes)
{
  EXPECT_EQ(parse_duration_ns("50ms"), 50'000'000);
  EXPECT_EQ(parse_duration_ns("500ns"), 500);
  EXPECT_EQ(parse_duration_ns("2us"), 2'000);
  EXPECT_EQ(parse_duration_ns("µs"), std::nullopt);  // unit alone, no number
  EXPECT_EQ(parse_duration_ns("3µs"), 3'000);        // micro sign
  EXPECT_EQ(parse_duration_ns("1s"), 1'000'000'000);
  EXPECT_EQ(parse_duration_ns("0.05s"), 50'000'000);
}

TEST(ParseDurationNs, NoUnitDefaultsToMilliseconds)
{
  EXPECT_EQ(parse_duration_ns("50"), 50'000'000);
  EXPECT_EQ(parse_duration_ns("1.5"), 1'500'000);
}

TEST(ParseDurationNs, SignedAndFractional)
{
  EXPECT_EQ(parse_duration_ns("-50ms"), -50'000'000);
  EXPECT_EQ(parse_duration_ns("+50ms"), 50'000'000);
  EXPECT_EQ(parse_duration_ns("1.5ms"), 1'500'000);
  EXPECT_EQ(parse_duration_ns("-500ns"), -500);
}

TEST(ParseDurationNs, WhitespaceTolerated)
{
  EXPECT_EQ(parse_duration_ns("  50ms  "), 50'000'000);
  EXPECT_EQ(parse_duration_ns("50 ms"), 50'000'000);
}

TEST(ParseDurationNs, RejectsGarbage)
{
  EXPECT_EQ(parse_duration_ns(""), std::nullopt);
  EXPECT_EQ(parse_duration_ns("abc"), std::nullopt);
  EXPECT_EQ(parse_duration_ns("50min"), std::nullopt);  // unknown unit
  EXPECT_EQ(parse_duration_ns("5x"), std::nullopt);
  EXPECT_EQ(parse_duration_ns("ms"), std::nullopt);  // no number
}

TEST(ParseDurationNs, RejectsNonFinite)
{
  EXPECT_EQ(parse_duration_ns("nan"), std::nullopt);
  EXPECT_EQ(parse_duration_ns("inf"), std::nullopt);
  EXPECT_EQ(parse_duration_ns("infinity"), std::nullopt);
  EXPECT_EQ(parse_duration_ns("-inf"), std::nullopt);
  EXPECT_EQ(parse_duration_ns("1e400ms"), std::nullopt);  // strtod overflows to +inf
  EXPECT_EQ(parse_duration_ns("1e30s"), std::nullopt);    // finite but out of int64 ns range
}
