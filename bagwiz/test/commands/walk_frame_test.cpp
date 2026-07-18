// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_frame.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include "bagwiz/core/base/str_utils.hpp"
#include "bagwiz/core/cdr_walker/value.hpp"
#include "bagwiz/core/decoder/decoder.hpp"
#include "walk_cursor.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace
{

using bagwiz::commands::build_yaml_frame;
using bagwiz::commands::MessageCursor;
using bagwiz::commands::OwnedMessage;

// Decoder stub returning a fixed message: one string field, or `field_count`
// integer fields when a long body is needed. `fail` forces a decode error.
class StubDecoder : public bagwiz::core::decoder::Decoder
{
public:
  bool fail = false;
  int field_count = 0;  // 0 -> single { data: 'hello' }

  bagwiz::core::decoder::DecodeResult decode(std::span<const std::byte>) const override
  {
    bagwiz::core::decoder::DecodeResult result;
    if (fail) {
      result.error = "boom";
      return result;
    }
    bagwiz::core::cdr_walker::Object obj;
    if (field_count == 0) {
      obj.fields.emplace_back("data", bagwiz::core::cdr_walker::Value{std::string{"hello"}});
    } else {
      for (int i = 0; i < field_count; ++i) {
        obj.fields.emplace_back(
          "f" + std::to_string(i), bagwiz::core::cdr_walker::Value{std::int32_t{0}});
      }
    }
    result.value = bagwiz::core::cdr_walker::Value{std::move(obj)};
    return result;
  }

  std::string_view backend() const noexcept override { return "stub"; }
};

// Cursor over `count` scripted messages (42-byte payloads, 1s apart). The
// caller decides whether to drain the source (the extra failing pull that
// flips `exhausted`, and with it the "+" suffix in the index row).
MessageCursor make_cursor(std::size_t count, std::string & status)
{
  return MessageCursor(
    [count, loaded = std::size_t{0}](OwnedMessage & msg) mutable {
      if (loaded >= count) {
        return false;
      }
      msg.timestamp_ns = static_cast<std::int64_t>(loaded) * 1'000'000'000;
      msg.payload.assign(42, std::byte{0});
      ++loaded;
      return true;
    },
    status);
}

std::string join(const std::vector<std::string> & lines)
{
  std::string out;
  for (const auto & line : lines) {
    out += line;
    out += '\n';
  }
  return out;
}

constexpr bagwiz::core::tui::Size kWide{24, 300, 0, 0};   // nothing wraps
constexpr bagwiz::core::tui::Size kNarrow{24, 80, 0, 0};  // the legend wraps

TEST(WalkBuildYamlFrame, HeaderCarriesTimestampAndSize)
{
  std::string status;
  auto cursor = make_cursor(1, status);
  ASSERT_TRUE(cursor.load_next());
  ASSERT_FALSE(cursor.load_next());  // drain: exhaust the source
  StubDecoder decoder;

  const auto frame =
    build_yaml_frame(0, kWide, cursor, decoder, false, "/topic", "pkg/msg/Type", status, false);
  ASSERT_EQ(frame.header.size(), 3U);
  EXPECT_EQ(frame.header[0], "timestamp: " + bagwiz::core::format_timestamp(0));
  EXPECT_EQ(frame.header[1], "size:      42 bytes");
  EXPECT_TRUE(frame.header[2].empty());  // blank separator
}

TEST(WalkBuildYamlFrame, BodyRendersDecodedMessage)
{
  std::string status;
  auto cursor = make_cursor(1, status);
  ASSERT_TRUE(cursor.load_next());
  StubDecoder decoder;

  const auto frame =
    build_yaml_frame(0, kWide, cursor, decoder, false, "/topic", "pkg/msg/Type", status, false);
  EXPECT_EQ(frame.body, (std::vector<std::string>{"data: 'hello'"}));
}

TEST(WalkBuildYamlFrame, DecodeFailureRendersWarningLine)
{
  std::string status;
  auto cursor = make_cursor(1, status);
  ASSERT_TRUE(cursor.load_next());
  StubDecoder decoder;
  decoder.fail = true;

  const auto frame =
    build_yaml_frame(0, kWide, cursor, decoder, false, "/topic", "pkg/msg/Type", status, false);
  EXPECT_EQ(frame.body, (std::vector<std::string>{"⚠  Could not decode this message: boom"}));
}

TEST(WalkBuildYamlFrame, FooterLayoutAndLegend)
{
  std::string status;
  auto cursor = make_cursor(3, status);
  while (cursor.load_next()) {
  }
  StubDecoder decoder;

  const auto frame =
    build_yaml_frame(0, kWide, cursor, decoder, false, "/topic", "pkg/msg/Type", status, false);
  // Wide terminal: blank separator, index row, one legend line, status row.
  ASSERT_EQ(frame.footer.size(), 4U);
  EXPECT_TRUE(frame.footer[0].empty());
  EXPECT_EQ(frame.footer[1], "  [0 / 2]  /topic  pkg/msg/Type");
  EXPECT_TRUE(frame.footer[3].empty());  // empty status renders a blank row
  const std::string legend = frame.footer[2];
  EXPECT_NE(legend.find("[s] save as yaml"), std::string::npos) << legend;
  EXPECT_NE(legend.find("[a] expand arrays"), std::string::npos) << legend;
  EXPECT_NE(legend.find("[q] quit"), std::string::npos) << legend;
  EXPECT_EQ(legend.find("[i] preview"), std::string::npos) << legend;
}

TEST(WalkBuildYamlFrame, NotExhaustedIndexRowGetsPlusSuffix)
{
  std::string status;
  auto cursor = make_cursor(3, status);
  ASSERT_TRUE(cursor.load_next());
  ASSERT_TRUE(cursor.load_next());
  ASSERT_TRUE(cursor.load_next());  // stop before the failing pull
  StubDecoder decoder;

  const auto frame =
    build_yaml_frame(0, kWide, cursor, decoder, false, "/topic", "pkg/msg/Type", status, false);
  ASSERT_EQ(frame.footer.size(), 4U);
  EXPECT_EQ(frame.footer[1], "  [0 / 2+]  /topic  pkg/msg/Type");
}

TEST(WalkBuildYamlFrame, StatusRowShowsTransientMessage)
{
  std::string status = "saved /tmp/x.yaml";
  auto cursor = make_cursor(1, status);
  ASSERT_TRUE(cursor.load_next());
  StubDecoder decoder;

  const auto frame =
    build_yaml_frame(0, kWide, cursor, decoder, false, "/topic", "pkg/msg/Type", status, false);
  ASSERT_EQ(frame.footer.size(), 4U);
  EXPECT_EQ(frame.footer[3], "  saved /tmp/x.yaml");
}

TEST(WalkBuildYamlFrame, PreviewHintIsRainbowColored)
{
  std::string status;
  auto cursor = make_cursor(1, status);
  ASSERT_TRUE(cursor.load_next());
  StubDecoder decoder;

  const auto with_preview =
    build_yaml_frame(0, kWide, cursor, decoder, false, "/topic", "pkg/msg/Type", status, true);
  // Each character of "[i] preview" carries its own SGR escape.
  EXPECT_NE(join(with_preview.footer).find("\x1B[31m"), std::string::npos);

  const auto without_preview =
    build_yaml_frame(0, kWide, cursor, decoder, false, "/topic", "pkg/msg/Type", status, false);
  EXPECT_EQ(join(without_preview.footer).find("\x1B["), std::string::npos);
}

TEST(WalkBuildYamlFrame, ScrollHintAppearsWhenBodyOverflows)
{
  std::string status;
  auto cursor = make_cursor(1, status);
  ASSERT_TRUE(cursor.load_next());
  ASSERT_FALSE(cursor.load_next());
  StubDecoder decoder;
  decoder.field_count = 30;  // 30 body lines

  const auto frame =
    build_yaml_frame(0, kWide, cursor, decoder, false, "/topic", "pkg/msg/Type", status, false);
  ASSERT_EQ(frame.footer.size(), 4U);
  // body_rows = 24 rows - 3 header - 4 footer = 17.
  EXPECT_EQ(frame.footer[1], "  [0 / 0]  /topic  pkg/msg/Type    lines 1-17 of 30");
}

TEST(WalkBuildYamlFrame, ScrollHintTracksScrollOffset)
{
  std::string status;
  auto cursor = make_cursor(1, status);
  ASSERT_TRUE(cursor.load_next());
  ASSERT_FALSE(cursor.load_next());
  StubDecoder decoder;
  decoder.field_count = 30;

  const auto frame =
    build_yaml_frame(5, kWide, cursor, decoder, false, "/topic", "pkg/msg/Type", status, false);
  ASSERT_EQ(frame.footer.size(), 4U);
  EXPECT_EQ(frame.footer[1], "  [0 / 0]  /topic  pkg/msg/Type    lines 6-22 of 30");
}

TEST(WalkBuildYamlFrame, ScrollHintStaysConsistentWithWrappedFooter)
{
  std::string status;
  auto cursor = make_cursor(1, status);
  ASSERT_TRUE(cursor.load_next());
  ASSERT_FALSE(cursor.load_next());
  StubDecoder decoder;
  decoder.field_count = 30;

  // Narrow terminal: the legend wraps, shrinking the body window; the hint
  // must still describe exactly the rows left between header and footer.
  const auto frame =
    build_yaml_frame(0, kNarrow, cursor, decoder, false, "/topic", "pkg/msg/Type", status, false);
  const std::size_t body_rows = kNarrow.rows - frame.header.size() - frame.footer.size();
  const std::string expected = "lines 1-" + std::to_string(body_rows) + " of 30";
  // The index row itself may wrap at 80 columns; compare the joined footer.
  EXPECT_NE(join(frame.footer).find(expected), std::string::npos) << join(frame.footer);
}

TEST(WalkAppendWrapped, WrapsToWidthAndReindentsContinuations)
{
  std::vector<std::string> out;
  bagwiz::commands::append_wrapped(out, "    abcdefgh", 6);
  // First fragment takes 2 content columns after the 4-column indent;
  // continuations inherit the indent (see wrap_to_width).
  EXPECT_EQ(out, (std::vector<std::string>{"    ab", "    cd", "    ef", "    gh"}));
}

TEST(WalkAppendWrapped, EmptyLineSurvives)
{
  std::vector<std::string> out;
  bagwiz::commands::append_wrapped(out, "", 10);
  EXPECT_EQ(out, (std::vector<std::string>{""}));
}

}  // namespace
