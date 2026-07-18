// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/msg_yaml/message_formatter.hpp"

#include "bagwiz/core/cdr_walker/value.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

namespace cdr = bagwiz::core::cdr_walker;
namespace bw = bagwiz::core;

namespace
{

// Helper: build the std_msgs/String shape `{ data: "..." }` directly so
// the formatter test stays decoupled from the schema parser and the
// introspection typesupport. The decoder layer has its own tests for
// the Value-construction step.
cdr::Object string_message(const std::string & value)
{
  cdr::Object obj;
  obj.fields.emplace_back("data", cdr::Value{value});
  return obj;
}

}  // namespace

TEST(MessageFormatter, RendersSingleStringField)
{
  const auto obj = string_message("hello");
  const auto result = bw::format_message(cdr::Value{obj});
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(result.text, "data: 'hello'\n");
}

TEST(MessageFormatter, RendersAllScalarPrimitives)
{
  // Construct a synthetic message covering every Value primitive variant
  // so number formatting (signed vs unsigned widths, float precision,
  // string escaping) is locked down at the formatter level rather than
  // inferred from one or two real schemas.
  cdr::Object obj;
  obj.fields.emplace_back("b", cdr::Value{true});
  obj.fields.emplace_back("i8", cdr::Value{std::int8_t{-5}});
  obj.fields.emplace_back("u8", cdr::Value{std::uint8_t{200}});
  obj.fields.emplace_back("i16", cdr::Value{std::int16_t{-1234}});
  obj.fields.emplace_back("u16", cdr::Value{std::uint16_t{56789}});
  obj.fields.emplace_back("i32", cdr::Value{std::int32_t{-100000}});
  obj.fields.emplace_back("u32", cdr::Value{std::uint32_t{0xDEADBEEFU}});
  obj.fields.emplace_back("i64", cdr::Value{std::int64_t{-1LL << 40}});
  obj.fields.emplace_back("u64", cdr::Value{std::uint64_t{0xCAFEBABE12345678ULL}});
  obj.fields.emplace_back("f32", cdr::Value{3.5F});
  obj.fields.emplace_back("f64", cdr::Value{2.5});
  obj.fields.emplace_back("s", cdr::Value{std::string{"hi"}});

  const auto result = bw::format_message(cdr::Value{obj});
  ASSERT_TRUE(result.ok()) << result.error;
  // Spot-check a few representative lines instead of asserting the full
  // body — the formatter's per-field code path is uniform so once each
  // primitive renders correctly the rest follows.
  EXPECT_NE(result.text.find("b: true\n"), std::string::npos) << result.text;
  EXPECT_NE(result.text.find("i8: -5\n"), std::string::npos) << result.text;
  EXPECT_NE(result.text.find("u8: 200\n"), std::string::npos) << result.text;
  EXPECT_NE(result.text.find("u32: 3735928559\n"), std::string::npos) << result.text;
  EXPECT_NE(result.text.find("f32: 3.5\n"), std::string::npos) << result.text;
  EXPECT_NE(result.text.find("s: 'hi'\n"), std::string::npos) << result.text;
}

TEST(MessageFormatter, RendersNestedObject)
{
  // header { stamp { sec, nanosec } frame_id } pattern.
  cdr::Object stamp;
  stamp.fields.emplace_back("sec", cdr::Value{std::int32_t{42}});
  stamp.fields.emplace_back("nanosec", cdr::Value{std::uint32_t{500}});

  cdr::Object header;
  header.fields.emplace_back("stamp", cdr::Value{std::move(stamp)});
  header.fields.emplace_back("frame_id", cdr::Value{std::string{"map"}});

  cdr::Object root;
  root.fields.emplace_back("header", cdr::Value{std::move(header)});

  const auto result = bw::format_message(cdr::Value{root});
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(
    result.text,
    "header:\n"
    "  stamp:\n"
    "    sec: 42\n"
    "    nanosec: 500\n"
    "  frame_id: 'map'\n");
}

TEST(MessageFormatter, RendersPrimitiveArrayInline)
{
  cdr::Sequence seq;
  seq.elements.emplace_back(std::uint8_t{1});
  seq.elements.emplace_back(std::uint8_t{2});
  seq.elements.emplace_back(std::uint8_t{3});

  cdr::Object root;
  root.fields.emplace_back("data", cdr::Value{std::move(seq)});

  const auto result = bw::format_message(cdr::Value{root});
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(result.text, "data: [1, 2, 3]\n");
}

TEST(MessageFormatter, SummarizesLargePrimitiveArray)
{
  cdr::Sequence seq;
  for (int i = 0; i < 100; ++i) {
    seq.elements.emplace_back(std::uint8_t{0});
  }
  cdr::Object root;
  root.fields.emplace_back("data", cdr::Value{std::move(seq)});

  bw::FormatOptions opts;
  opts.max_inline_array = 32;
  const auto result = bw::format_message(cdr::Value{root}, opts);
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(result.text, "data: [<100 items>]\n");
}

TEST(MessageFormatter, EmptyArrayRendersBrackets)
{
  cdr::Object root;
  root.fields.emplace_back("data", cdr::Value{cdr::Sequence{}});
  const auto result = bw::format_message(cdr::Value{root});
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.text, "data: []\n");
}

TEST(MessageFormatter, RendersSequenceOfNestedMessages)
{
  // tf2_msgs/TFMessage shape: transforms = [TransformStamped, ...]
  cdr::Object t1;
  t1.fields.emplace_back("frame_id", cdr::Value{std::string{"map"}});
  t1.fields.emplace_back("x", cdr::Value{1.0});
  cdr::Object t2;
  t2.fields.emplace_back("frame_id", cdr::Value{std::string{"odom"}});
  t2.fields.emplace_back("x", cdr::Value{2.0});

  cdr::Sequence seq;
  seq.elements.emplace_back(std::move(t1));
  seq.elements.emplace_back(std::move(t2));

  cdr::Object root;
  root.fields.emplace_back("transforms", cdr::Value{std::move(seq)});

  const auto result = bw::format_message(cdr::Value{root});
  ASSERT_TRUE(result.ok()) << result.error;
  // YAML "block sequence" style: the `- ` marker sits at the same indent
  // as the parent key. Continuation lines align under the dash with two
  // spaces of additional indent.
  EXPECT_EQ(
    result.text,
    "transforms:\n"
    "  - frame_id: 'map'\n"
    "    x: 1\n"
    "  - frame_id: 'odom'\n"
    "    x: 2\n");
}

TEST(MessageFormatter, SequenceOfNestedMessagesIsYamlParsable)
{
  cdr::Object stamp;
  stamp.fields.emplace_back("sec", cdr::Value{std::int32_t{1}});
  stamp.fields.emplace_back("nanosec", cdr::Value{std::uint32_t{2}});

  cdr::Object header;
  header.fields.emplace_back("stamp", cdr::Value{std::move(stamp)});
  header.fields.emplace_back("frame_id", cdr::Value{std::string{"base_link"}});

  cdr::Object transform;
  transform.fields.emplace_back("header", cdr::Value{std::move(header)});
  transform.fields.emplace_back("child_frame_id", cdr::Value{std::string{"camera_link"}});

  cdr::Sequence transforms;
  transforms.elements.emplace_back(std::move(transform));

  cdr::Object root;
  root.fields.emplace_back("transforms", cdr::Value{std::move(transforms)});

  const auto result = bw::format_message(cdr::Value{root});
  ASSERT_TRUE(result.ok()) << result.error;
  YAML::Node parsed;
  EXPECT_NO_THROW(parsed = YAML::Load(result.text));
  ASSERT_TRUE(parsed["transforms"].IsSequence());
  ASSERT_EQ(parsed["transforms"].size(), 1U);
}

TEST(MessageFormatter, EscapesProblematicStringChars)
{
  cdr::Object root;
  root.fields.emplace_back("msg", cdr::Value{std::string{"line1\nline2'quote'\ttab"}});
  const auto result = bw::format_message(cdr::Value{root});
  ASSERT_TRUE(result.ok());
  // Single quotes are doubled (YAML single-quoted scalar convention),
  // newlines / tabs become escape sequences.
  EXPECT_EQ(result.text, "msg: 'line1\\nline2''quote''\\ttab'\n");
}

TEST(MessageFormatter, RejectsNonObjectRoot)
{
  // A sequence at the top level isn't a valid decoded message.
  const auto result = bw::format_message(cdr::Value{cdr::Sequence{}});
  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.error.find("not an Object"), std::string::npos) << result.error;
}

TEST(MessageFormatter, ExpandsLargeArrayAsBlockSequence)
{
  // Walk's `[a] expand arrays` toggle sets expand_long_arrays. Long
  // primitive arrays must render as a YAML block sequence (one element
  // per line) instead of the `[<N items>]` summary — staying within the
  // terminal width and producing output that PyYAML can parse verbatim.
  cdr::Sequence seq;
  for (int i = 0; i < 100; ++i) {
    seq.elements.emplace_back(std::uint8_t{0});
  }
  cdr::Object root;
  root.fields.emplace_back("data", cdr::Value{std::move(seq)});

  bw::FormatOptions opts;
  opts.expand_long_arrays = true;
  const auto result = bw::format_message(cdr::Value{root}, opts);
  ASSERT_TRUE(result.ok()) << result.error;
  std::string expected = "data:\n";
  for (int i = 0; i < 100; ++i) {
    expected += "  - 0\n";
  }
  EXPECT_EQ(result.text, expected);
  EXPECT_EQ(result.text.find("items>]"), std::string::npos) << "must not be summarized";
}

TEST(MessageFormatter, ExpandsLongArrayInsideListItem)
{
  // Guard the indent calculation in emit_list_item_child_value: a long
  // primitive array under a list item must align under the list item's
  // field key, two spaces deeper than that key. Threshold is set to 2
  // so a 3-element array trips the block-expand path without needing a
  // large corpus.
  cdr::Sequence data;
  data.elements.emplace_back(std::uint8_t{1});
  data.elements.emplace_back(std::uint8_t{2});
  data.elements.emplace_back(std::uint8_t{3});

  cdr::Object item;
  item.fields.emplace_back("data", cdr::Value{std::move(data)});

  cdr::Sequence items;
  items.elements.emplace_back(std::move(item));

  cdr::Object root;
  root.fields.emplace_back("items", cdr::Value{std::move(items)});

  bw::FormatOptions opts;
  opts.max_inline_array = 2;
  opts.expand_long_arrays = true;
  const auto result = bw::format_message(cdr::Value{root}, opts);
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(
    result.text,
    "items:\n"
    "  - data:\n"
    "      - 1\n"
    "      - 2\n"
    "      - 3\n");
}
