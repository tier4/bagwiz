// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/cdr_walker/walker.hpp"

#include "bagwiz/core/cdr_walker/value.hpp"
#include "bagwiz/core/msg_schema/parser.hpp"
#include "bagwiz/core/msg_schema/schema_model.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace cdr = bagwiz::core::cdr_walker;
namespace ms = bagwiz::core::msg_schema;

namespace
{

// Tiny CDR-1 payload builder used only by these tests. The bagwiz core
// CDR walker is decode-only (no CDR writer is shipped); we re-implement
// the minimum needed to construct realistic test inputs without
// depending on rmw_serialize. Mirrors the same alignment rules as the
// reader so both sides exercise the (offset - 4) % size convention.
class CdrBuilder
{
public:
  CdrBuilder()
  {
    // CDR encapsulation header: kind=CDR_LE, options=0.
    bytes_ = {std::byte{0x00}, std::byte{0x01}, std::byte{0x00}, std::byte{0x00}};
  }

  // Construct a payload that uses the BE header so the reader exercises
  // its big-endian path. Caller is responsible for writing the data
  // big-endian-style using big_endian variants.
  static CdrBuilder big_endian()
  {
    CdrBuilder b;
    b.bytes_[1] = std::byte{0x00};  // CDR_BE
    b.little_endian_ = false;
    return b;
  }

  void align(std::size_t size)
  {
    while ((bytes_.size() - 4) % size != 0) {
      bytes_.push_back(std::byte{0});
    }
  }

  void put_u8(std::uint8_t v) { bytes_.push_back(std::byte{v}); }
  void put_i8(std::int8_t v) { put_u8(static_cast<std::uint8_t>(v)); }
  void put_bool(bool v) { put_u8(v ? 1 : 0); }

  template <typename T>
  void put_le(T v)
  {
    align(sizeof(T));
    std::array<std::byte, sizeof(T)> buf{};
    std::memcpy(buf.data(), &v, sizeof(T));
    if (!little_endian_) {
      for (std::size_t i = 0; i < sizeof(T) / 2; ++i) {
        std::swap(buf[i], buf[sizeof(T) - 1 - i]);
      }
    }
    for (auto b : buf) {
      bytes_.push_back(b);
    }
  }

  void put_u16(std::uint16_t v) { put_le(v); }
  void put_i16(std::int16_t v) { put_le(v); }
  void put_u32(std::uint32_t v) { put_le(v); }
  void put_i32(std::int32_t v) { put_le(v); }
  void put_u64(std::uint64_t v) { put_le(v); }
  void put_i64(std::int64_t v) { put_le(v); }
  void put_f32(float v) { put_le(v); }
  void put_f64(double v) { put_le(v); }

  // Length-prefixed string with trailing NUL. Length includes the NUL.
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
  bool little_endian_ = true;
};

// Helper: parse a single .msg, decode the payload, return the Value.
const cdr::Value & expect_decode_ok(
  const ms::SchemaModel & schema, std::span<const std::byte> payload, cdr::DecodeResult & holder)
{
  holder = cdr::decode(schema, payload);
  EXPECT_TRUE(holder.ok()) << "decode failed: " << holder.error;
  return *holder.value;
}

const cdr::Object & as_object(const cdr::Value & v)
{
  EXPECT_TRUE(v.is_object());
  return std::get<cdr::Object>(v.v);
}

const cdr::Sequence & as_sequence(const cdr::Value & v)
{
  EXPECT_TRUE(v.is_sequence());
  return std::get<cdr::Sequence>(v.v);
}

const cdr::Value & field(const cdr::Object & obj, std::string_view name)
{
  for (const auto & [k, val] : obj.fields) {
    if (k == name) {
      return val;
    }
  }
  throw std::logic_error("field not found: " + std::string(name));
}

}  // namespace

// --- String -------------------------------------------------------------

TEST(CdrWalker, SingleStringField)
{
  // Schema: `string data` — std_msgs/String.
  const auto parse = ms::parse_message("std_msgs", "String", "string data\n");
  ASSERT_TRUE(parse.ok()) << parse.error;

  CdrBuilder b;
  b.put_string("hello");

  cdr::DecodeResult result;
  const auto & root = as_object(expect_decode_ok(*parse.schema, b.span(), result));
  ASSERT_EQ(root.fields.size(), 1U);
  EXPECT_EQ(root.fields[0].first, "data");
  EXPECT_EQ(std::get<std::string>(root.fields[0].second.v), "hello");
}

TEST(CdrWalker, AcceptsTrailingPaddingFromEncapsulationOptions)
{
  // OMG DDS-XTYPES 1.3 §7.6.3.1.2: the lower two bits of
  // representation_options encode the count of pad bytes (0-3) appended
  // after the body so the total encapsulation ends on a 4-byte
  // boundary. The reader must exclude those bytes from the effective
  // body so a schema-correct decode succeeds rather than treating them
  // as part of the payload.
  const auto parse = ms::parse_message("std_msgs", "String", "string data\n");
  ASSERT_TRUE(parse.ok()) << parse.error;

  CdrBuilder b;
  b.put_string("hi");
  const auto base = b.span();
  std::vector<std::byte> padded(base.begin(), base.end());
  padded[3] = std::byte{2};  // 2 pad bytes claimed in options LSB
  padded.push_back(std::byte{0});
  padded.push_back(std::byte{0});

  cdr::DecodeResult result;
  const auto & root = as_object(expect_decode_ok(*parse.schema, padded, result));
  EXPECT_EQ(std::get<std::string>(field(root, "data").v), "hi");
}

TEST(CdrWalker, EmptyString)
{
  // Length-1 (just NUL) and length-0 must both decode to empty string.
  const auto parse = ms::parse_message("std_msgs", "String", "string data\n");
  ASSERT_TRUE(parse.ok());

  CdrBuilder b1;
  b1.put_u32(1);
  b1.put_u8(0);  // NUL
  cdr::DecodeResult result;
  const auto & root1 = as_object(expect_decode_ok(*parse.schema, b1.span(), result));
  EXPECT_EQ(std::get<std::string>(root1.fields[0].second.v), "");

  CdrBuilder b2;
  b2.put_u32(0);
  const auto & root2 = as_object(expect_decode_ok(*parse.schema, b2.span(), result));
  EXPECT_EQ(std::get<std::string>(root2.fields[0].second.v), "");
}

// --- Primitives end to end ---------------------------------------------

TEST(CdrWalker, AllScalarPrimitives)
{
  const std::string text =
    "bool b\n"
    "int8 i8\n"
    "uint8 u8\n"
    "int16 i16\n"
    "uint16 u16\n"
    "int32 i32\n"
    "uint32 u32\n"
    "int64 i64\n"
    "uint64 u64\n"
    "float32 f32\n"
    "float64 f64\n";
  const auto parse = ms::parse_message("test_pkg", "AllScalars", text);
  ASSERT_TRUE(parse.ok()) << parse.error;

  // Build payload in declared order, reproducing CDR alignment by hand.
  CdrBuilder b;
  b.put_bool(true);    // 1 byte at offset 0
  b.put_i8(-5);        // 1 byte at offset 1
  b.put_u8(200);       // 1 byte at offset 2
  b.put_i16(-1234);    // align(2): pad to offset 4 (already 3+1)
  b.put_u16(56789);    // already aligned
  b.put_i32(-100000);  // align(4)
  b.put_u32(0xDEADBEEFU);
  b.put_i64(-1LL << 40);
  b.put_u64(0xCAFEBABE12345678ULL);
  b.put_f32(3.14F);
  b.put_f64(2.718281828);

  cdr::DecodeResult result;
  const auto & root = as_object(expect_decode_ok(*parse.schema, b.span(), result));
  ASSERT_EQ(root.fields.size(), 11U);
  EXPECT_EQ(std::get<bool>(field(root, "b").v), true);
  EXPECT_EQ(std::get<std::int8_t>(field(root, "i8").v), -5);
  EXPECT_EQ(std::get<std::uint8_t>(field(root, "u8").v), 200);
  EXPECT_EQ(std::get<std::int16_t>(field(root, "i16").v), -1234);
  EXPECT_EQ(std::get<std::uint16_t>(field(root, "u16").v), 56789);
  EXPECT_EQ(std::get<std::int32_t>(field(root, "i32").v), -100000);
  EXPECT_EQ(std::get<std::uint32_t>(field(root, "u32").v), 0xDEADBEEFU);
  EXPECT_EQ(std::get<std::int64_t>(field(root, "i64").v), -1LL << 40);
  EXPECT_EQ(std::get<std::uint64_t>(field(root, "u64").v), 0xCAFEBABE12345678ULL);
  EXPECT_FLOAT_EQ(std::get<float>(field(root, "f32").v), 3.14F);
  EXPECT_DOUBLE_EQ(std::get<double>(field(root, "f64").v), 2.718281828);
}

// --- Arrays ------------------------------------------------------------

TEST(CdrWalker, FixedByteArray)
{
  const auto parse = ms::parse_message("test", "Foo", "uint8[4] bytes\n");
  ASSERT_TRUE(parse.ok());

  CdrBuilder b;
  b.put_u8(0x01);
  b.put_u8(0x02);
  b.put_u8(0x03);
  b.put_u8(0x04);

  cdr::DecodeResult result;
  const auto & root = as_object(expect_decode_ok(*parse.schema, b.span(), result));
  const auto & seq = as_sequence(field(root, "bytes"));
  ASSERT_EQ(seq.elements.size(), 4U);
  EXPECT_EQ(std::get<std::uint8_t>(seq.elements[0].v), 0x01);
  EXPECT_EQ(std::get<std::uint8_t>(seq.elements[3].v), 0x04);
}

TEST(CdrWalker, UnboundedByteArray)
{
  const auto parse = ms::parse_message("test", "Foo", "uint8[] bytes\n");
  ASSERT_TRUE(parse.ok());

  CdrBuilder b;
  b.put_u32(3);  // length prefix
  b.put_u8(0xAA);
  b.put_u8(0xBB);
  b.put_u8(0xCC);

  cdr::DecodeResult result;
  const auto & root = as_object(expect_decode_ok(*parse.schema, b.span(), result));
  const auto & seq = as_sequence(field(root, "bytes"));
  ASSERT_EQ(seq.elements.size(), 3U);
  EXPECT_EQ(std::get<std::uint8_t>(seq.elements[1].v), 0xBB);
}

TEST(CdrWalker, UnboundedFloat64Array)
{
  const auto parse = ms::parse_message("test", "Foo", "float64[] xs\n");
  ASSERT_TRUE(parse.ok());

  CdrBuilder b;
  b.put_u32(3);  // length, 4 bytes
  // align(8): the 4-byte length leaves us at file offset 8 (header+len),
  // i.e. (8-4) = 4 mod 8 = 4 → need 4 bytes pad before first float64.
  b.put_f64(1.0);
  b.put_f64(2.0);
  b.put_f64(3.0);

  cdr::DecodeResult result;
  const auto & root = as_object(expect_decode_ok(*parse.schema, b.span(), result));
  const auto & seq = as_sequence(field(root, "xs"));
  ASSERT_EQ(seq.elements.size(), 3U);
  EXPECT_DOUBLE_EQ(std::get<double>(seq.elements[0].v), 1.0);
  EXPECT_DOUBLE_EQ(std::get<double>(seq.elements[2].v), 3.0);
}

TEST(CdrWalker, BoundedSequenceUsesLengthPrefix)
{
  // Bounded sequences carry a runtime length just like unbounded; the
  // bound is only a schema-level constraint that the walker doesn't
  // currently enforce (no consumer needs it).
  const auto parse = ms::parse_message("test", "Foo", "int32[<=4] xs\n");
  ASSERT_TRUE(parse.ok());

  CdrBuilder b;
  b.put_u32(2);
  b.put_i32(-1);
  b.put_i32(2);

  cdr::DecodeResult result;
  const auto & root = as_object(expect_decode_ok(*parse.schema, b.span(), result));
  const auto & seq = as_sequence(field(root, "xs"));
  ASSERT_EQ(seq.elements.size(), 2U);
  EXPECT_EQ(std::get<std::int32_t>(seq.elements[0].v), -1);
}

// --- Nested types ------------------------------------------------------

TEST(CdrWalker, BuiltinTimeNested)
{
  // builtin_interfaces/Time = {int32 sec, uint32 nanosec}; the parser
  // injects this even when the schema text doesn't redeclare it, so a
  // schema like `builtin_interfaces/Time stamp` resolves cleanly.
  const auto parse = ms::parse_message("test", "WithTime", "builtin_interfaces/Time stamp\n");
  ASSERT_TRUE(parse.ok()) << parse.error;

  CdrBuilder b;
  b.put_i32(1700000000);
  b.put_u32(123456789U);

  cdr::DecodeResult result;
  const auto & root = as_object(expect_decode_ok(*parse.schema, b.span(), result));
  const auto & stamp = as_object(field(root, "stamp"));
  ASSERT_EQ(stamp.fields.size(), 2U);
  EXPECT_EQ(std::get<std::int32_t>(field(stamp, "sec").v), 1700000000);
  EXPECT_EQ(std::get<std::uint32_t>(field(stamp, "nanosec").v), 123456789U);
}

TEST(CdrWalker, NestedMessageVectorThroughConcatenatedSchema)
{
  // tf2_msgs/TFMessage shape: vector<TransformStamped>, which itself
  // contains a Header (with Time + frame_id), child_frame_id, and a
  // Transform with two Vector3-shaped sub-messages. Exercises the full
  // recursive decode path against a concatenated schema.
  const std::string text =
    "geometry_msgs/TransformStamped[] transforms\n"
    "================================================================================\n"
    "MSG: geometry_msgs/TransformStamped\n"
    "std_msgs/Header header\n"
    "string child_frame_id\n"
    "Transform transform\n"
    "================================================================================\n"
    "MSG: std_msgs/Header\n"
    "builtin_interfaces/Time stamp\n"
    "string frame_id\n"
    "================================================================================\n"
    "MSG: geometry_msgs/Transform\n"
    "Vector3 translation\n"
    "Quaternion rotation\n"
    "================================================================================\n"
    "MSG: geometry_msgs/Vector3\n"
    "float64 x\n"
    "float64 y\n"
    "float64 z\n"
    "================================================================================\n"
    "MSG: geometry_msgs/Quaternion\n"
    "float64 x\n"
    "float64 y\n"
    "float64 z\n"
    "float64 w\n";

  const auto parse = ms::parse_schema("tf2_msgs/msg/TFMessage", text);
  ASSERT_TRUE(parse.ok()) << parse.error;

  CdrBuilder b;
  b.put_u32(1);  // one TransformStamped
  // header: Time(sec=1, nanosec=0), frame_id="map"
  b.put_i32(1);
  b.put_u32(0);
  b.put_string("map");
  // child_frame_id
  b.put_string("base_link");
  // transform: translation Vector3(1,2,3), rotation Quaternion(0,0,0,1)
  b.put_f64(1.0);
  b.put_f64(2.0);
  b.put_f64(3.0);
  b.put_f64(0.0);
  b.put_f64(0.0);
  b.put_f64(0.0);
  b.put_f64(1.0);

  cdr::DecodeResult result;
  const auto & root = as_object(expect_decode_ok(*parse.schema, b.span(), result));
  const auto & seq = as_sequence(field(root, "transforms"));
  ASSERT_EQ(seq.elements.size(), 1U);

  const auto & ts = as_object(seq.elements[0]);
  EXPECT_EQ(std::get<std::string>(field(ts, "child_frame_id").v), "base_link");

  const auto & header = as_object(field(ts, "header"));
  EXPECT_EQ(std::get<std::string>(field(header, "frame_id").v), "map");
  const auto & stamp = as_object(field(header, "stamp"));
  EXPECT_EQ(std::get<std::int32_t>(field(stamp, "sec").v), 1);

  const auto & transform = as_object(field(ts, "transform"));
  const auto & translation = as_object(field(transform, "translation"));
  EXPECT_DOUBLE_EQ(std::get<double>(field(translation, "x").v), 1.0);
  EXPECT_DOUBLE_EQ(std::get<double>(field(translation, "z").v), 3.0);
  const auto & rotation = as_object(field(transform, "rotation"));
  EXPECT_DOUBLE_EQ(std::get<double>(field(rotation, "w").v), 1.0);
}

// --- Empty messages ----------------------------------------------------

TEST(CdrWalker, EmptyMessageReadsPlaceholderByte)
{
  const auto parse = ms::parse_message("std_srvs", "Empty", "# truly empty\n");
  ASSERT_TRUE(parse.ok());

  CdrBuilder b;
  b.put_u8(0);  // structure_needs_at_least_one_member

  cdr::DecodeResult result;
  const auto & root = as_object(expect_decode_ok(*parse.schema, b.span(), result));
  EXPECT_EQ(root.fields.size(), 0U);
}

// --- Endianness --------------------------------------------------------

TEST(CdrWalker, BigEndianPayload)
{
  const auto parse = ms::parse_message("test", "Foo", "uint32 v\n");
  ASSERT_TRUE(parse.ok());

  auto b = CdrBuilder::big_endian();
  b.put_u32(0x11223344U);

  cdr::DecodeResult result;
  const auto & root = as_object(expect_decode_ok(*parse.schema, b.span(), result));
  EXPECT_EQ(std::get<std::uint32_t>(field(root, "v").v), 0x11223344U);
}

// --- Error paths -------------------------------------------------------

TEST(CdrWalker, RejectsPlCdrEncapsulation)
{
  const auto parse = ms::parse_message("test", "Foo", "uint32 v\n");
  ASSERT_TRUE(parse.ok());

  // Hand-craft a header with kind=0x02 (PL_CDR_BE).
  std::vector<std::byte> payload = {std::byte{0x00}, std::byte{0x02}, std::byte{0}, std::byte{0}};
  auto result = cdr::decode(*parse.schema, payload);
  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.error.find("PL_CDR"), std::string::npos) << result.error;
}

TEST(CdrWalker, RejectsWstring)
{
  const auto parse = ms::parse_message("test", "Foo", "wstring s\n");
  ASSERT_TRUE(parse.ok());

  CdrBuilder b;
  b.put_u32(0);  // length-0 wstring (would be "valid" if we supported it)
  auto result = cdr::decode(*parse.schema, b.span());
  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.error.find("wstring"), std::string::npos) << result.error;
}

TEST(CdrWalker, RejectsLongDouble)
{
  const auto parse = ms::parse_message("test", "Foo", "float128 v\n");
  ASSERT_TRUE(parse.ok());

  CdrBuilder b;
  b.put_u64(0);  // would-be float128, 16 bytes
  b.put_u64(0);
  auto result = cdr::decode(*parse.schema, b.span());
  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.error.find("float128"), std::string::npos) << result.error;
}

TEST(CdrWalker, UnderflowOnTruncatedPayload)
{
  const auto parse = ms::parse_message("test", "Foo", "uint32 a\nuint32 b\n");
  ASSERT_TRUE(parse.ok());

  CdrBuilder b;
  b.put_u32(1);  // only one of the two fields

  auto result = cdr::decode(*parse.schema, b.span());
  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.error.find("CDR underflow"), std::string::npos) << result.error;
}

TEST(CdrWalker, ShortHeaderRejected)
{
  const auto parse = ms::parse_message("test", "Foo", "uint8 a\n");
  ASSERT_TRUE(parse.ok());

  std::vector<std::byte> payload = {std::byte{0x00}, std::byte{0x01}};  // only 2 bytes
  auto result = cdr::decode(*parse.schema, payload);
  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.error.find("encapsulation header"), std::string::npos) << result.error;
}
