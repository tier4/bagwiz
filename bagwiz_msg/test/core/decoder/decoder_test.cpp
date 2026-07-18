// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/decoder/decoder.hpp"

#include "bagwiz/core/cdr_walker/value.hpp"
#include "bagwiz/core/decoder/message_view.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <span>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace dec = bagwiz::core::decoder;
namespace cdr = bagwiz::core::cdr_walker;

namespace
{

// CDR-1 little-endian payload builder (same shape as the cdr_walker
// test, duplicated here to keep tests self-contained without exposing a
// production CDR writer).
class CdrBuilder
{
public:
  CdrBuilder() { bytes_ = {std::byte{0x00}, std::byte{0x01}, std::byte{0x00}, std::byte{0x00}}; }

  void align(std::size_t size)
  {
    while ((bytes_.size() - 4) % size != 0) {
      bytes_.push_back(std::byte{0});
    }
  }

  void put_u8(std::uint8_t v) { bytes_.push_back(std::byte{v}); }

  template <typename T>
  void put_le(T v)
  {
    align(sizeof(T));
    std::array<std::byte, sizeof(T)> buf{};
    std::memcpy(buf.data(), &v, sizeof(T));
    for (auto b : buf) {
      bytes_.push_back(b);
    }
  }

  void put_u32(std::uint32_t v) { put_le(v); }

  void put_string(const std::string & s)
  {
    put_u32(static_cast<std::uint32_t>(s.size() + 1));
    for (const char c : s) {
      put_u8(static_cast<std::uint8_t>(c));
    }
    put_u8(0);
  }

  std::span<const std::byte> span() const { return bytes_; }

private:
  std::vector<std::byte> bytes_;
};

bagwiz::io::TopicInfo make_topic(
  std::string name, std::string type, std::string schema_text = "",
  std::string schema_encoding = "")
{
  bagwiz::io::TopicInfo t;
  t.name = std::move(name);
  t.type = std::move(type);
  t.serialization_format = "cdr";
  t.schema_text = std::move(schema_text);
  t.schema_encoding = std::move(schema_encoding);
  return t;
}

}  // namespace

// --- MessageView -------------------------------------------------------

TEST(MessageView, NameLookupAndPrimitive)
{
  cdr::Object obj;
  obj.fields.emplace_back("a", cdr::Value{std::int32_t{42}});
  obj.fields.emplace_back("b", cdr::Value{std::string{"hello"}});

  dec::MessageView view{obj};
  EXPECT_EQ(view.size(), 2U);
  EXPECT_EQ(view.name_at(0), "a");
  EXPECT_EQ(view.index_of("b"), 1U);
  EXPECT_EQ(view.index_of("missing"), view.size());

  EXPECT_EQ(view.primitive<std::int32_t>("a").value(), 42);
  EXPECT_EQ(view.primitive<std::string>("b").value(), "hello");

  // Type mismatch → nullopt instead of std::bad_variant_access.
  EXPECT_FALSE(view.primitive<std::string>("a").has_value());
  EXPECT_FALSE(view.primitive<double>("a").has_value());

  // Missing field → nullopt.
  EXPECT_FALSE(view.primitive<int>("missing").has_value());
}

TEST(MessageView, NestedAndSequence)
{
  // Construct: { stamp: { sec: 1, nanosec: 2 }, transforms: [obj1, obj2] }
  cdr::Object stamp;
  stamp.fields.emplace_back("sec", cdr::Value{std::int32_t{1}});
  stamp.fields.emplace_back("nanosec", cdr::Value{std::uint32_t{2}});

  cdr::Object t1;
  t1.fields.emplace_back("frame_id", cdr::Value{std::string{"map"}});
  cdr::Object t2;
  t2.fields.emplace_back("frame_id", cdr::Value{std::string{"odom"}});

  cdr::Sequence seq;
  seq.elements.emplace_back(std::move(t1));
  seq.elements.emplace_back(std::move(t2));

  cdr::Object root;
  root.fields.emplace_back("stamp", cdr::Value{std::move(stamp)});
  root.fields.emplace_back("transforms", cdr::Value{std::move(seq)});

  dec::MessageView view{root};
  const auto nested_stamp = view.nested("stamp");
  ASSERT_TRUE(nested_stamp.has_value());
  EXPECT_EQ(nested_stamp->primitive<std::int32_t>("sec").value(), 1);

  EXPECT_EQ(view.sequence_length("transforms").value(), 2U);
  const auto first = view.nested_element("transforms", 0);
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->primitive<std::string>("frame_id").value(), "map");

  // Out-of-range and wrong-kind paths → nullopt.
  EXPECT_FALSE(view.nested_element("transforms", 99).has_value());
  EXPECT_FALSE(view.nested("transforms").has_value());      // sequence, not object
  EXPECT_FALSE(view.sequence_length("stamp").has_value());  // object, not sequence
  EXPECT_FALSE(view.nested("missing").has_value());
}

// --- SchemaDecoder -----------------------------------------------------

TEST(SchemaDecoder, RoundtripsStdMsgsString)
{
  const auto topic = make_topic("/foo", "std_msgs/msg/String", "string data\n", "ros2msg");
  auto open = dec::open_decoder(topic);
  ASSERT_TRUE(open.ok()) << open.error;
  EXPECT_EQ(open.decoder->backend(), "schema");

  CdrBuilder b;
  b.put_string("hello");
  const auto result = open.decoder->decode(b.span());
  ASSERT_TRUE(result.ok()) << result.error;

  const auto * obj = std::get_if<cdr::Object>(&result.value->v);
  ASSERT_NE(obj, nullptr);
  dec::MessageView view{*obj};
  EXPECT_EQ(view.primitive<std::string>("data").value(), "hello");
}

TEST(SchemaDecoder, MissingSchemaFallsThroughToIntrospection)
{
  // When schema_text is empty the schema backend can't be used; the
  // factory must transparently fall through to introspection. std_msgs
  // is on AMENT_PREFIX_PATH thanks to package.xml's <depend>.
  const auto topic = make_topic("/foo", "std_msgs/msg/String", /*schema_text=*/"");
  auto open = dec::open_decoder(topic);
  ASSERT_TRUE(open.ok()) << open.error;
  EXPECT_EQ(open.decoder->backend(), "introspection");
}

TEST(SchemaDecoder, WstringSchemaFallsThroughToIntrospection)
{
  // A schema referencing wstring is treated as unsupported by the
  // schema-driven backend (cdr_walker::Value has no variant for
  // wstring) and routed to introspection. We only have one wstring-
  // using type (a synthetic one), but std_msgs/String is on PATH so
  // the fallback resolves. We assert backend is "introspection".
  //
  // Use std_msgs/msg/String as the type name (so introspection succeeds)
  // but pass a schema_text that would otherwise parse as a wstring
  // field. The factory should reject the schema and pick introspection.
  const auto topic = make_topic("/foo", "std_msgs/msg/String", "wstring data\n", "ros2msg");
  auto open = dec::open_decoder(topic);
  ASSERT_TRUE(open.ok()) << open.error;
  EXPECT_EQ(open.decoder->backend(), "introspection");
}

// --- IntrospectionDecoder ---------------------------------------------

TEST(IntrospectionDecoder, RoundtripsStdMsgsString)
{
  // No schema_text → introspection is the only choice.
  const auto topic = make_topic("/foo", "std_msgs/msg/String");
  auto open = dec::open_decoder(topic);
  ASSERT_TRUE(open.ok()) << open.error;
  ASSERT_EQ(open.decoder->backend(), "introspection");

  CdrBuilder b;
  b.put_string("hello");
  const auto result = open.decoder->decode(b.span());
  ASSERT_TRUE(result.ok()) << result.error;

  const auto * obj = std::get_if<cdr::Object>(&result.value->v);
  ASSERT_NE(obj, nullptr);
  dec::MessageView view{*obj};
  EXPECT_EQ(view.primitive<std::string>("data").value(), "hello");
}

TEST(IntrospectionDecoder, FailsOnUnknownType)
{
  // A type that has no typesupport `.so` on AMENT_PREFIX_PATH should
  // surface as an open() error, not a decode crash.
  const auto topic = make_topic("/foo", "this_pkg_does_not_exist/msg/Nope");
  auto open = dec::open_decoder(topic);
  EXPECT_FALSE(open.ok());
  EXPECT_NE(open.error.find("introspection"), std::string::npos) << open.error;
}

// --- Cross-backend equivalence ----------------------------------------

TEST(Equivalence, BothBackendsProduceEqualValueTrees)
{
  // Same payload, same type, decoded both ways. The two Value trees
  // should be structurally equal — the YAML formatter relies on this
  // contract to produce byte-identical output regardless of which
  // backend decoded the message.
  CdrBuilder b;
  b.put_string("a roundtripped string");

  const auto schema_topic = make_topic("/foo", "std_msgs/msg/String", "string data\n", "ros2msg");
  auto schema_open = dec::open_decoder(schema_topic);
  ASSERT_TRUE(schema_open.ok()) << schema_open.error;
  ASSERT_EQ(schema_open.decoder->backend(), "schema");

  const auto intro_topic = make_topic("/foo", "std_msgs/msg/String");
  auto intro_open = dec::open_decoder(intro_topic);
  ASSERT_TRUE(intro_open.ok()) << intro_open.error;
  ASSERT_EQ(intro_open.decoder->backend(), "introspection");

  const auto schema_result = schema_open.decoder->decode(b.span());
  const auto intro_result = intro_open.decoder->decode(b.span());
  ASSERT_TRUE(schema_result.ok()) << schema_result.error;
  ASSERT_TRUE(intro_result.ok()) << intro_result.error;

  const auto * schema_obj = std::get_if<cdr::Object>(&schema_result.value->v);
  const auto * intro_obj = std::get_if<cdr::Object>(&intro_result.value->v);
  ASSERT_NE(schema_obj, nullptr);
  ASSERT_NE(intro_obj, nullptr);
  ASSERT_EQ(schema_obj->fields.size(), intro_obj->fields.size());
  for (std::size_t i = 0; i < schema_obj->fields.size(); ++i) {
    EXPECT_EQ(schema_obj->fields[i].first, intro_obj->fields[i].first);
    EXPECT_EQ(
      std::get<std::string>(schema_obj->fields[i].second.v),
      std::get<std::string>(intro_obj->fields[i].second.v));
  }
}

// --- BAGWIZ_DECODER environment override -------------------------------

class DecoderEnvOverride : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const char * existing = std::getenv("BAGWIZ_DECODER");
    if (existing != nullptr) {
      saved_ = existing;
    }
  }
  void TearDown() override
  {
    if (saved_.has_value()) {
      ::setenv("BAGWIZ_DECODER", saved_->c_str(), 1);
    } else {
      ::unsetenv("BAGWIZ_DECODER");
    }
  }

private:
  std::optional<std::string> saved_;
};

TEST_F(DecoderEnvOverride, ForcesIntrospectionEvenWhenSchemaAvailable)
{
  ::setenv("BAGWIZ_DECODER", "introspection", 1);
  const auto topic = make_topic("/foo", "std_msgs/msg/String", "string data\n", "ros2msg");
  auto open = dec::open_decoder(topic);
  ASSERT_TRUE(open.ok()) << open.error;
  EXPECT_EQ(open.decoder->backend(), "introspection");
}

TEST_F(DecoderEnvOverride, EmptyEnvFallsBackToAutoPolicy)
{
  ::setenv("BAGWIZ_DECODER", "", 1);
  const auto topic = make_topic("/foo", "std_msgs/msg/String", "string data\n", "ros2msg");
  auto open = dec::open_decoder(topic);
  ASSERT_TRUE(open.ok()) << open.error;
  EXPECT_EQ(open.decoder->backend(), "schema");
}
