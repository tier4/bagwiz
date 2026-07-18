// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tui/layout.hpp"

#include <gtest/gtest.h>

namespace
{

using bagwiz::core::tui::FixedLayout;
using bagwiz::core::tui::Size;

TEST(FixedLayout, HeaderStartIsRowOne)
{
  FixedLayout l{Size{24, 80}, 3, 2};
  EXPECT_EQ(l.header_start_row(), 1);
  EXPECT_EQ(l.header_end_row(), 3);
}

TEST(FixedLayout, BodyStartFollowsHeader)
{
  FixedLayout l{Size{24, 80}, 3, 2};
  EXPECT_EQ(l.body_start_row(), 4);
}

TEST(FixedLayout, FooterEndIsTerminalLast)
{
  FixedLayout l{Size{24, 80}, 3, 2};
  EXPECT_EQ(l.footer_end_row(), 24);
}

TEST(FixedLayout, FooterStartRow)
{
  FixedLayout l{Size{24, 80}, 3, 2};
  EXPECT_EQ(l.footer_start_row(), 23);
}

TEST(FixedLayout, BodyRowsComputed)
{
  FixedLayout l{Size{24, 80}, 3, 2};
  // body = rows 4..22 inclusive = 19 rows
  EXPECT_EQ(l.body_end_row(), 22);
  EXPECT_EQ(l.body_rows(), 19);
}

TEST(FixedLayout, BodyRowsWithWalkLayout)
{
  // walk uses header=4 (3 lines + blank), footer=4 (blank + 3 lines)
  FixedLayout l{Size{24, 80}, 4, 4};
  EXPECT_EQ(l.body_start_row(), 5);
  EXPECT_EQ(l.body_end_row(), 20);
  EXPECT_EQ(l.body_rows(), 16);
  EXPECT_EQ(l.footer_start_row(), 21);
  EXPECT_EQ(l.footer_end_row(), 24);
}

TEST(FixedLayout, ZeroHeaderAndFooterFillsBody)
{
  FixedLayout l{Size{24, 80}, 0, 0};
  EXPECT_EQ(l.body_start_row(), 1);
  EXPECT_EQ(l.body_end_row(), 24);
  EXPECT_EQ(l.body_rows(), 24);
}

TEST(FixedLayout, CollapsesGracefullyWhenOversized)
{
  // header + footer >= rows: body collapses to 1 row (documented).
  FixedLayout l{Size{5, 80}, 4, 4};
  EXPECT_EQ(l.body_rows(), 1);
}

TEST(FixedLayout, ExactFitNoBody)
{
  // header + footer == rows: body still gets >=1 row (collapse behavior).
  FixedLayout l{Size{6, 80}, 3, 3};
  EXPECT_EQ(l.body_rows(), 1);
}

}  // namespace
