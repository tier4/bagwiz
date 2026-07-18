// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_cursor.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace
{

using bagwiz::commands::MessageCursor;
using bagwiz::commands::MsgNav;
using bagwiz::commands::OwnedMessage;

constexpr std::int64_t kSecondNs = 1'000'000'000;

// Feed the cursor from a fixed list of timestamps, one message per pull.
struct Script
{
  std::vector<std::int64_t> timestamps;
  std::size_t next = 0;
};

MessageCursor::Source scripted(Script & script)
{
  return [&script](OwnedMessage & msg) {
    if (script.next >= script.timestamps.size()) {
      return false;
    }
    msg.timestamp_ns = script.timestamps[script.next++];
    msg.payload.assign(1, std::byte{0xAB});
    return true;
  };
}

// Load every scripted message up front (cursor ends exhausted at index 0).
void load_all(MessageCursor & cursor)
{
  while (cursor.load_next()) {
  }
}

TEST(WalkMessageCursor, LoadNextReadsUntilExhausted)
{
  Script script{{100, 200}};
  std::string status;
  MessageCursor cursor(scripted(script), status);

  EXPECT_TRUE(cursor.load_next());
  EXPECT_TRUE(cursor.load_next());
  EXPECT_FALSE(cursor.exhausted());
  EXPECT_FALSE(cursor.load_next());
  EXPECT_TRUE(cursor.exhausted());
  // Once exhausted the source is never consulted again.
  EXPECT_FALSE(cursor.load_next());

  ASSERT_EQ(cursor.cache().size(), 2U);
  EXPECT_EQ(cursor.cache()[0].timestamp_ns, 100);
  EXPECT_EQ(cursor.cache()[1].timestamp_ns, 200);
  // The cache owns a copy of each payload.
  EXPECT_EQ(cursor.cache()[0].payload, (std::vector<std::byte>{std::byte{0xAB}}));
}

TEST(WalkMessageCursor, NextPullsFromSource)
{
  Script script{{100, 200}};
  std::string status = "dirty";
  MessageCursor cursor(scripted(script), status);
  ASSERT_TRUE(cursor.load_next());

  EXPECT_TRUE(cursor.navigate(MsgNav::kNext));
  EXPECT_EQ(cursor.index(), 1U);
  EXPECT_EQ(cursor.cache().size(), 2U);
  EXPECT_TRUE(status.empty());  // cleared on entry, no boundary notice
}

TEST(WalkMessageCursor, NextWithinCacheMovesWithoutLoading)
{
  Script script{{100, 200, 300}};
  std::string status;
  MessageCursor cursor(scripted(script), status);
  load_all(cursor);

  EXPECT_TRUE(cursor.navigate(MsgNav::kNext));
  EXPECT_EQ(cursor.index(), 1U);
  EXPECT_EQ(script.next, 3U);  // nothing pulled beyond the initial scan
  EXPECT_TRUE(status.empty());
}

TEST(WalkMessageCursor, NextAtExhaustedEndWrapsToFirst)
{
  Script script{{100, 200}};
  std::string status;
  MessageCursor cursor(scripted(script), status);
  ASSERT_TRUE(cursor.load_next());
  ASSERT_TRUE(cursor.navigate(MsgNav::kNext));  // loads 200, lands on index 1

  EXPECT_TRUE(cursor.navigate(MsgNav::kNext));
  EXPECT_EQ(cursor.index(), 0U);
  EXPECT_EQ(status, "(wrapped to first)");
  EXPECT_TRUE(cursor.exhausted());
}

TEST(WalkMessageCursor, PrevMovesBack)
{
  Script script{{100, 200}};
  std::string status;
  MessageCursor cursor(scripted(script), status);
  ASSERT_TRUE(cursor.load_next());
  ASSERT_TRUE(cursor.navigate(MsgNav::kNext));

  EXPECT_TRUE(cursor.navigate(MsgNav::kPrev));
  EXPECT_EQ(cursor.index(), 0U);
  EXPECT_TRUE(status.empty());
}

TEST(WalkMessageCursor, PrevAtFirstMessageKeepsIndex)
{
  Script script{{100, 200}};
  std::string status;
  MessageCursor cursor(scripted(script), status);
  ASSERT_TRUE(cursor.load_next());

  EXPECT_FALSE(cursor.navigate(MsgNav::kPrev));
  EXPECT_EQ(cursor.index(), 0U);
  EXPECT_EQ(status, "(at first message)");
}

TEST(WalkMessageCursor, FirstReportsOnlyRealMoves)
{
  Script script{{100, 200}};
  std::string status;
  MessageCursor cursor(scripted(script), status);
  ASSERT_TRUE(cursor.load_next());

  EXPECT_FALSE(cursor.navigate(MsgNav::kFirst));  // already at first
  ASSERT_TRUE(cursor.navigate(MsgNav::kNext));
  EXPECT_TRUE(cursor.navigate(MsgNav::kFirst));
  EXPECT_EQ(cursor.index(), 0U);
  EXPECT_TRUE(status.empty());
}

TEST(WalkMessageCursor, LastScansToEnd)
{
  Script script{{100, 200, 300}};
  std::string status;
  MessageCursor cursor(scripted(script), status);
  ASSERT_TRUE(cursor.load_next());

  EXPECT_TRUE(cursor.navigate(MsgNav::kLast));
  EXPECT_EQ(cursor.index(), 2U);
  EXPECT_TRUE(cursor.exhausted());
  EXPECT_TRUE(status.empty());
}

TEST(WalkMessageCursor, LastWhenAlreadyScannedReportsBoundary)
{
  Script script{{100, 200, 300}};
  std::string status;
  MessageCursor cursor(scripted(script), status);
  load_all(cursor);
  ASSERT_TRUE(cursor.navigate(MsgNav::kLast));

  EXPECT_FALSE(cursor.navigate(MsgNav::kLast));
  EXPECT_EQ(cursor.index(), 2U);
  EXPECT_EQ(status, "(already at last message)");
}

TEST(WalkMessageCursor, StepForward1sLandsOnFirstMessageAtOrAfterTarget)
{
  Script script{{0, kSecondNs / 2, kSecondNs + kSecondNs / 2, 2 * kSecondNs}};
  std::string status;
  MessageCursor cursor(scripted(script), status);
  load_all(cursor);

  EXPECT_TRUE(cursor.navigate(MsgNav::kStepForward1s));
  EXPECT_EQ(cursor.index(), 2U);  // first message at or after t=1s
  EXPECT_TRUE(status.empty());
}

TEST(WalkMessageCursor, StepForward1sPullsUntilTarget)
{
  Script script{{0, kSecondNs / 2, kSecondNs + kSecondNs / 2, 2 * kSecondNs}};
  std::string status;
  MessageCursor cursor(scripted(script), status);
  ASSERT_TRUE(cursor.load_next());  // only t=0 cached

  EXPECT_TRUE(cursor.navigate(MsgNav::kStepForward1s));
  EXPECT_EQ(cursor.index(), 2U);
  EXPECT_EQ(cursor.cache().size(), 3U);  // pulled t=0.5s and t=1.5s
  EXPECT_FALSE(cursor.exhausted());
}

TEST(WalkMessageCursor, StepForward1sBeyondEndClampsWithStatus)
{
  Script script{{0, kSecondNs / 2, kSecondNs + kSecondNs / 2, 2 * kSecondNs}};
  std::string status;
  MessageCursor cursor(scripted(script), status);
  load_all(cursor);
  ASSERT_TRUE(cursor.navigate(MsgNav::kStepForward1s));  // -> index 2 (t=1.5s)

  EXPECT_TRUE(cursor.navigate(MsgNav::kStepForward1s));  // target 2.5s > last 2s
  EXPECT_EQ(cursor.index(), 3U);
  EXPECT_EQ(status, "(reached end)");

  EXPECT_FALSE(cursor.navigate(MsgNav::kStepForward1s));  // already at last
  EXPECT_EQ(cursor.index(), 3U);
  EXPECT_EQ(status, "(already at last message)");
}

TEST(WalkMessageCursor, StepForward10sUsesTenSecondDelta)
{
  Script script{{0, 5 * kSecondNs, 15 * kSecondNs}};
  std::string status;
  MessageCursor cursor(scripted(script), status);
  load_all(cursor);

  EXPECT_TRUE(cursor.navigate(MsgNav::kStepForward10s));
  EXPECT_EQ(cursor.index(), 2U);  // first message at or after t=10s
}

TEST(WalkMessageCursor, StepBackward1sLandsOnLastMessageAtOrBeforeTarget)
{
  Script script{{0, kSecondNs / 2, kSecondNs + kSecondNs / 2, 2 * kSecondNs}};
  std::string status;
  MessageCursor cursor(scripted(script), status);
  load_all(cursor);
  ASSERT_TRUE(cursor.navigate(MsgNav::kStepForward1s));  // -> index 2 (t=1.5s)

  EXPECT_TRUE(cursor.navigate(MsgNav::kStepBackward1s));  // target 0.5s
  EXPECT_EQ(cursor.index(), 1U);                          // last message at or before t=0.5s
  EXPECT_TRUE(status.empty());
}

TEST(WalkMessageCursor, StepBackward1sBeforeFirstClampsToZero)
{
  Script script{{0, kSecondNs / 2, kSecondNs + kSecondNs / 2, 2 * kSecondNs}};
  std::string status;
  MessageCursor cursor(scripted(script), status);
  load_all(cursor);
  ASSERT_TRUE(cursor.navigate(MsgNav::kNext));  // -> index 1 (t=0.5s)

  EXPECT_TRUE(cursor.navigate(MsgNav::kStepBackward1s));  // target -0.5s < first
  EXPECT_EQ(cursor.index(), 0U);
  EXPECT_TRUE(status.empty());

  EXPECT_FALSE(cursor.navigate(MsgNav::kStepBackward1s));  // already at first
  EXPECT_EQ(cursor.index(), 0U);
  EXPECT_EQ(status, "(at first message)");
}

TEST(WalkMessageCursor, StepBackward10sUsesTenSecondDelta)
{
  Script script{{0, 5 * kSecondNs, 15 * kSecondNs}};
  std::string status;
  MessageCursor cursor(scripted(script), status);
  load_all(cursor);
  ASSERT_TRUE(cursor.navigate(MsgNav::kLast));  // -> index 2 (t=15s)

  EXPECT_TRUE(cursor.navigate(MsgNav::kStepBackward10s));  // target 5s
  EXPECT_EQ(cursor.index(), 1U);
}

TEST(WalkMessageCursor, NavigateClearsStatusOnEntry)
{
  Script script{{100, 200}};
  std::string status = "dirty";
  MessageCursor cursor(scripted(script), status);
  load_all(cursor);
  ASSERT_TRUE(cursor.navigate(MsgNav::kLast));

  EXPECT_TRUE(cursor.navigate(MsgNav::kPrev));
  EXPECT_TRUE(status.empty());
}

}  // namespace
