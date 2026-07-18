// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_frame.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace
{

using bagwiz::commands::resolve_scroll_hint;
using bagwiz::commands::ScrollHintResolution;

constexpr bagwiz::core::tui::Size kTerm{24, 80, 0, 0};

// Footer template mirroring walk's YAML view: blank separator, a placeholder
// index row (overwritten by the resolver), the key legend, and the status row.
std::vector<std::string> footer_template()
{
  return {"", "", "legend", ""};
}

TEST(WalkResolveScrollHint, BodyFitsWithoutHint)
{
  const auto resolved = resolve_scroll_hint(kTerm, 3, "IDX", footer_template(), 5, 0);
  EXPECT_TRUE(resolved.scroll_hint.empty());
  // body_rows = 24 rows - 3 header - 4 footer = 17.
  EXPECT_EQ(resolved.body_rows, 17);
  ASSERT_EQ(resolved.footer.size(), 4U);
  EXPECT_EQ(resolved.footer[1], "IDX");
}

TEST(WalkResolveScrollHint, OverflowAppendsHintToIndexRow)
{
  const auto resolved = resolve_scroll_hint(kTerm, 3, "IDX", footer_template(), 30, 0);
  EXPECT_EQ(resolved.scroll_hint, "    lines 1-17 of 30");
  EXPECT_EQ(resolved.body_rows, 17);
  ASSERT_EQ(resolved.footer.size(), 4U);
  EXPECT_EQ(resolved.footer[1], "IDX    lines 1-17 of 30");
}

TEST(WalkResolveScrollHint, ScrollOffsetShiftsWindow)
{
  const auto resolved = resolve_scroll_hint(kTerm, 3, "IDX", footer_template(), 30, 5);
  EXPECT_EQ(resolved.scroll_hint, "    lines 6-22 of 30");
}

TEST(WalkResolveScrollHint, WindowEndClampsAtTotal)
{
  const auto resolved = resolve_scroll_hint(kTerm, 3, "IDX", footer_template(), 30, 25);
  EXPECT_EQ(resolved.scroll_hint, "    lines 26-30 of 30");
}

TEST(WalkResolveScrollHint, HintRewrapShrinksBodyWindowOnce)
{
  // A 19-column index row fits alone at 22 columns, but appending the hint
  // wraps it to two lines, which eats one body row and changes the hint.
  const std::string index_no_hint = "  [0 / 0]  /t  type";  // 19 columns
  ASSERT_EQ(index_no_hint.size(), 19U);
  constexpr bagwiz::core::tui::Size tight{10, 22, 0, 0};

  const auto resolved = resolve_scroll_hint(tight, 3, index_no_hint, footer_template(), 100, 0);
  // First pass: body_rows = 10 - 3 - 4 = 3, hint "lines 1-3 of 100". The
  // 39-column index row then wraps to two lines, so the second pass lands on
  // body_rows = 2 and the re-derived hint.
  EXPECT_EQ(resolved.body_rows, 2);
  EXPECT_EQ(resolved.scroll_hint, "    lines 1-2 of 100");
  EXPECT_EQ(resolved.footer.size(), 5U);  // wrapped index row adds one line
}

TEST(WalkResolveScrollHint, ZeroBodyRowsYieldsNoHint)
{
  constexpr bagwiz::core::tui::Size tiny{6, 80, 0, 0};
  const auto resolved = resolve_scroll_hint(tiny, 3, "IDX", footer_template(), 100, 0);
  EXPECT_EQ(resolved.body_rows, 0);
  EXPECT_TRUE(resolved.scroll_hint.empty());
  EXPECT_EQ(resolved.footer.size(), 4U);
}

TEST(WalkResolveScrollHint, FooterContextLinesArePreserved)
{
  std::vector<std::string> footer = {"", "", "legend", "  status!"};
  const auto resolved = resolve_scroll_hint(kTerm, 3, "IDX", footer, 5, 0);
  ASSERT_EQ(resolved.footer.size(), 4U);
  EXPECT_TRUE(resolved.footer[0].empty());
  EXPECT_EQ(resolved.footer[2], "legend");
  EXPECT_EQ(resolved.footer[3], "  status!");
}

}  // namespace
