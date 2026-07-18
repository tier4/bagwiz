// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/base/str_utils.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace
{

using bagwiz::core::format_timestamp;
using bagwiz::core::join_csv;
using bagwiz::core::split_lines;

TEST(FormatTimestamp, EpochZero)
{
  EXPECT_EQ(format_timestamp(0), "1970-01-01 00:00:00.000000000 UTC (0.000000000)");
}

TEST(FormatTimestamp, NanosecondPrecision)
{
  // 2023-11-14 22:13:20 UTC, plus 123456789 ns.
  EXPECT_EQ(
    format_timestamp(1'700'000'000'123'456'789),
    "2023-11-14 22:13:20.123456789 UTC (1700000000.123456789)");
}

TEST(FormatTimestamp, SubSecondOnly)
{
  EXPECT_EQ(format_timestamp(999'999'999), "1970-01-01 00:00:00.999999999 UTC (0.999999999)");
}

TEST(SplitLines, EmptyStringYieldsNoLines)
{
  EXPECT_TRUE(split_lines("").empty());
}

TEST(SplitLines, SingleLineWithoutTrailingNewline)
{
  EXPECT_EQ(split_lines("a"), (std::vector<std::string>{"a"}));
}

TEST(SplitLines, TrailingNewlineDoesNotAddEmptyLine)
{
  EXPECT_EQ(split_lines("a\n"), (std::vector<std::string>{"a"}));
  EXPECT_EQ(split_lines("a\nb\n"), (std::vector<std::string>{"a", "b"}));
}

TEST(SplitLines, MultipleLines)
{
  EXPECT_EQ(split_lines("a\nb"), (std::vector<std::string>{"a", "b"}));
}

TEST(SplitLines, EmptyLinesAreKept)
{
  EXPECT_EQ(split_lines("\n"), (std::vector<std::string>{""}));
  EXPECT_EQ(split_lines("\n\n"), (std::vector<std::string>{"", ""}));
  EXPECT_EQ(split_lines("a\n\nb"), (std::vector<std::string>{"a", "", "b"}));
}

TEST(JoinCsv, EmptyListIsNone)
{
  EXPECT_EQ(join_csv({}), "(none)");
}

TEST(JoinCsv, SingleElement)
{
  EXPECT_EQ(join_csv({"a"}), "a");
}

TEST(JoinCsv, MultipleElements)
{
  EXPECT_EQ(join_csv({"a", "b", "c"}), "a, b, c");
}

}  // namespace
