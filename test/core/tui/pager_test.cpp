// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tui/pager.hpp"

#include <gtest/gtest.h>

namespace
{

using bagwiz::core::KeyEvent;
using bagwiz::core::tui::NavKey;
using bagwiz::core::tui::to_nav_key;

TEST(ToNavKey, NavBindings)
{
  EXPECT_EQ(to_nav_key(KeyEvent::kNext), NavKey::kNext);
  EXPECT_EQ(to_nav_key(KeyEvent::kPrev), NavKey::kPrev);
  EXPECT_EQ(to_nav_key(KeyEvent::kFirst), NavKey::kFirst);
  EXPECT_EQ(to_nav_key(KeyEvent::kLast), NavKey::kLast);
  EXPECT_EQ(to_nav_key(KeyEvent::kScrollUp), NavKey::kScrollUp);
  EXPECT_EQ(to_nav_key(KeyEvent::kScrollDown), NavKey::kScrollDown);
  EXPECT_EQ(to_nav_key(KeyEvent::kScrollHead), NavKey::kScrollHead);
  EXPECT_EQ(to_nav_key(KeyEvent::kScrollTail), NavKey::kScrollTail);
}

TEST(ToNavKey, QuitAndResize)
{
  EXPECT_EQ(to_nav_key(KeyEvent::kQuit), NavKey::kQuit);
  EXPECT_EQ(to_nav_key(KeyEvent::kResize), NavKey::kResize);
}

TEST(ToNavKey, AppBindingsArePassthrough)
{
  EXPECT_EQ(to_nav_key(KeyEvent::kSaveYaml), NavKey::kNone);
  EXPECT_EQ(to_nav_key(KeyEvent::kToggleArrayExpand), NavKey::kNone);
  EXPECT_EQ(to_nav_key(KeyEvent::kToggleRotation), NavKey::kNone);
}

TEST(ToNavKey, UnknownIsNone)
{
  EXPECT_EQ(to_nav_key(KeyEvent::kUnknown), NavKey::kNone);
}

}  // namespace
