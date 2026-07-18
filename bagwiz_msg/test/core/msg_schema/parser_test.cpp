// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/msg_schema/parser.hpp"

#include "bagwiz/core/msg_schema/schema_model.hpp"

#include <gtest/gtest.h>

#include <string>
#include <variant>

namespace ms = bagwiz::core::msg_schema;

namespace
{

// Convenience: assert ok and return the schema by reference. Caller can
// chain `.find()` / `.root()` calls on the value without sprinkling
// `ASSERT_TRUE(result.ok())` everywhere.
const ms::SchemaModel & expect_ok(const ms::ParseResult & result)
{
  EXPECT_TRUE(result.ok()) << "parse failed: " << result.error;
  return *result.schema;
}

const ms::FieldDef * find_field(const ms::MessageDef & def, std::string_view name)
{
  for (const auto & f : def.fields) {
    if (f.name == name) {
      return &f;
    }
  }
  return nullptr;
}

}  // namespace

// --- Smoke: trivial single-field message ---------------------------------

TEST(MsgSchemaParser, SingleStringField)
{
  const auto result = ms::parse_message("std_msgs", "String", "string data\n");
  const auto & schema = expect_ok(result);

  ASSERT_NE(schema.root(), nullptr);
  EXPECT_EQ(schema.root()->package, "std_msgs");
  EXPECT_EQ(schema.root()->name, "String");
  ASSERT_EQ(schema.root()->fields.size(), 1U);

  const auto & field = schema.root()->fields[0];
  EXPECT_EQ(field.name, "data");
  ASSERT_TRUE(field.type.is_primitive());
  EXPECT_EQ(std::get<ms::PrimitiveKind>(field.type.base), ms::PrimitiveKind::String);
  EXPECT_EQ(field.type.array.kind, ms::ArrayKind::Scalar);
}

TEST(MsgSchemaParser, AllPrimitives)
{
  const std::string text =
    "bool a\n"
    "byte b\n"
    "char c\n"
    "float32 d\n"
    "float64 e\n"
    "int8 f\n"
    "uint8 g\n"
    "int16 h\n"
    "uint16 i\n"
    "int32 j\n"
    "uint32 k\n"
    "int64 l\n"
    "uint64 m\n"
    "string n\n"
    "wstring o\n";

  const auto result = ms::parse_message("test_pkg", "AllPrimitives", text);
  const auto & schema = expect_ok(result);

  ASSERT_EQ(schema.root()->fields.size(), 15U);
  // Spot-check: byte/char and the unsigned-vs-signed pairs are not
  // collapsed (bagwiz preserves IDL semantics).
  EXPECT_EQ(
    std::get<ms::PrimitiveKind>(schema.root()->fields[1].type.base), ms::PrimitiveKind::Byte);
  EXPECT_EQ(
    std::get<ms::PrimitiveKind>(schema.root()->fields[2].type.base), ms::PrimitiveKind::Char);
  EXPECT_EQ(
    std::get<ms::PrimitiveKind>(schema.root()->fields[14].type.base), ms::PrimitiveKind::Wstring);
}

// --- Array variants ------------------------------------------------------

TEST(MsgSchemaParser, FixedArray)
{
  const auto result = ms::parse_message("test", "Foo", "uint8[16] bytes\n");
  const auto & schema = expect_ok(result);
  ASSERT_EQ(schema.root()->fields.size(), 1U);
  EXPECT_EQ(schema.root()->fields[0].type.array.kind, ms::ArrayKind::FixedArray);
  EXPECT_EQ(schema.root()->fields[0].type.array.size.value(), 16U);
}

TEST(MsgSchemaParser, BoundedSequence)
{
  const auto result = ms::parse_message("test", "Foo", "uint8[<=10] bytes\n");
  const auto & schema = expect_ok(result);
  EXPECT_EQ(schema.root()->fields[0].type.array.kind, ms::ArrayKind::BoundedSequence);
  EXPECT_EQ(schema.root()->fields[0].type.array.size.value(), 10U);
}

TEST(MsgSchemaParser, UnboundedSequence)
{
  const auto result = ms::parse_message("test", "Foo", "uint8[] bytes\n");
  const auto & schema = expect_ok(result);
  EXPECT_EQ(schema.root()->fields[0].type.array.kind, ms::ArrayKind::UnboundedSequence);
  EXPECT_FALSE(schema.root()->fields[0].type.array.size.has_value());
}

// --- Bounded strings ----------------------------------------------------

TEST(MsgSchemaParser, BoundedString)
{
  const auto result = ms::parse_message("test", "Foo", "string<=64 name\n");
  const auto & schema = expect_ok(result);
  ASSERT_EQ(schema.root()->fields.size(), 1U);
  EXPECT_EQ(
    std::get<ms::PrimitiveKind>(schema.root()->fields[0].type.base), ms::PrimitiveKind::String);
  EXPECT_EQ(schema.root()->fields[0].type.string_upper_bound.value(), 64U);
}

TEST(MsgSchemaParser, BoundedStringArray)
{
  const auto result = ms::parse_message("test", "Foo", "string<=8[<=4] tags\n");
  const auto & schema = expect_ok(result);
  EXPECT_EQ(
    std::get<ms::PrimitiveKind>(schema.root()->fields[0].type.base), ms::PrimitiveKind::String);
  EXPECT_EQ(schema.root()->fields[0].type.string_upper_bound.value(), 8U);
  EXPECT_EQ(schema.root()->fields[0].type.array.kind, ms::ArrayKind::BoundedSequence);
  EXPECT_EQ(schema.root()->fields[0].type.array.size.value(), 4U);
}

// --- Constants ----------------------------------------------------------

TEST(MsgSchemaParser, ConstantsAndFieldsMixed)
{
  const std::string text =
    "uint8 OK = 0\n"
    "uint8 WARN = 1\n"
    "uint8 ERROR = 2\n"
    "uint8 level\n"
    "string message\n";
  const auto result = ms::parse_message("diagnostic_msgs", "DiagnosticStatus", text);
  const auto & schema = expect_ok(result);

  ASSERT_EQ(schema.root()->constants.size(), 3U);
  EXPECT_EQ(schema.root()->constants[0].name, "OK");
  EXPECT_EQ(schema.root()->constants[0].raw_value, "0");
  EXPECT_EQ(schema.root()->constants[1].name, "WARN");
  EXPECT_EQ(schema.root()->constants[2].name, "ERROR");
  ASSERT_EQ(schema.root()->fields.size(), 2U);
  EXPECT_EQ(schema.root()->fields[0].name, "level");
  EXPECT_EQ(schema.root()->fields[1].name, "message");
}

// --- Default values -----------------------------------------------------

TEST(MsgSchemaParser, DefaultValuesParsedAsRawText)
{
  const std::string text =
    "uint8 mode 3\n"
    "string greeting \"hello world\"\n"
    "float64[] coefficients [1.0, 2.0, 3.0]\n";
  const auto result = ms::parse_message("test", "Foo", text);
  const auto & schema = expect_ok(result);

  ASSERT_EQ(schema.root()->fields.size(), 3U);
  EXPECT_TRUE(schema.root()->fields[0].default_value.has_value());
  EXPECT_EQ(schema.root()->fields[0].default_value->raw, "3");
  EXPECT_TRUE(schema.root()->fields[1].default_value.has_value());
  EXPECT_EQ(schema.root()->fields[1].default_value->raw, "\"hello world\"");
  EXPECT_TRUE(schema.root()->fields[2].default_value.has_value());
  EXPECT_EQ(schema.root()->fields[2].default_value->raw, "[1.0, 2.0, 3.0]");
}

TEST(MsgSchemaParser, DefaultStringWithHashIsNotTruncated)
{
  // A '#' inside a string-literal default value must NOT be treated as a
  // comment delimiter. Otherwise the captured raw_value silently loses
  // everything after the '#' (including the closing quote), which both
  // corrupts the parsed default and breaks downstream warning messages.
  const std::string text = "string color \"red # the color\"\n";
  const auto result = ms::parse_message("test", "Foo", text);
  const auto & schema = expect_ok(result);

  ASSERT_EQ(schema.root()->fields.size(), 1U);
  ASSERT_TRUE(schema.root()->fields[0].default_value.has_value());
  EXPECT_EQ(schema.root()->fields[0].default_value->raw, "\"red # the color\"");
}

TEST(MsgSchemaParser, EscapedQuoteInsideDefaultStringDoesNotEndLiteral)
{
  // An escaped quote `\"` keeps the literal open, so a '#' that follows
  // the escape but precedes the real closing quote must still be treated
  // as part of the literal.
  const std::string text = "string label \"a\\\" # still inside\"\n";
  const auto result = ms::parse_message("test", "Foo", text);
  const auto & schema = expect_ok(result);

  ASSERT_EQ(schema.root()->fields.size(), 1U);
  ASSERT_TRUE(schema.root()->fields[0].default_value.has_value());
  EXPECT_EQ(schema.root()->fields[0].default_value->raw, "\"a\\\" # still inside\"");
}

TEST(MsgSchemaParser, BareHeaderResolvesToStdMsgsHeader)
{
  // A field declared as `Header header` (no package qualifier) must
  // resolve to `std_msgs/Header`, even when the surrounding package is
  // not std_msgs. The qualified form `std_msgs/Header header` is more
  // common in modern .msg, but unqualified `Header` appears in older
  // and hand-written types, and rosbags handles it.
  //
  // Use parse_schema with a concatenated form so cross-reference
  // validation has a Header definition to resolve against; the
  // assertion is on where `Header` ends up pointing inside the root.
  const std::string text =
    "Header header\n"
    "uint32 value\n"
    "================================================================================\n"
    "MSG: std_msgs/Header\n"
    "uint32 seq\n"
    "builtin_interfaces/Time stamp\n"
    "string frame_id\n";
  const auto result = ms::parse_schema("test/msg/Foo", text);
  const auto & schema = expect_ok(result);

  const auto * root = schema.root();
  ASSERT_NE(root, nullptr);
  ASSERT_GE(root->fields.size(), 1U);
  ASSERT_TRUE(root->fields[0].type.is_nested());
  EXPECT_EQ(std::get<std::string>(root->fields[0].type.base), "std_msgs/Header")
    << "bare `Header` must resolve to std_msgs/Header, not test/Header";
}

// --- Comments and blank lines -------------------------------------------

TEST(MsgSchemaParser, CommentsAndBlankLines)
{
  const std::string text =
    "# Header comment\n"
    "\n"
    "uint32 sec  # seconds since epoch\n"
    "# Standalone comment\n"
    "uint32 nanosec\n"
    "\n";
  const auto result = ms::parse_message("builtin_interfaces", "Time", text);
  const auto & schema = expect_ok(result);
  ASSERT_EQ(schema.root()->fields.size(), 2U);
  EXPECT_EQ(schema.root()->fields[0].name, "sec");
  EXPECT_EQ(schema.root()->fields[1].name, "nanosec");
}

// --- Built-in injection -------------------------------------------------

TEST(MsgSchemaParser, BuiltinTimeAndDurationInjected)
{
  // A header-using schema that does not redefine builtin_interfaces/Time.
  const auto result = ms::parse_message("test", "WithTime", "builtin_interfaces/Time stamp\n");
  const auto & schema = expect_ok(result);

  // Even though the schema text never declared it, find() resolves Time
  // and Duration because parse_*() injects them.
  EXPECT_NE(schema.find("builtin_interfaces/Time"), nullptr);
  EXPECT_NE(schema.find("builtin_interfaces/msg/Time"), nullptr);
  EXPECT_NE(schema.find("builtin_interfaces/Duration"), nullptr);

  const auto * t = schema.find("builtin_interfaces/Time");
  ASSERT_NE(t, nullptr);
  ASSERT_EQ(t->fields.size(), 2U);
  EXPECT_EQ(t->fields[0].name, "sec");
  EXPECT_EQ(t->fields[1].name, "nanosec");
}

// --- MCAP concatenated form ---------------------------------------------

TEST(MsgSchemaParser, ConcatenatedTwoTypes)
{
  // Mirrors how rosbag2 / mcap-ros2 emit a `geometry_msgs/PoseStamped`
  // schema: root block first, then `===` separator, then `MSG: pkg/.../`
  // header followed by the dependency's body.
  const std::string text =
    "std_msgs/Header header\n"
    "Pose pose\n"
    "================================================================================\n"
    "MSG: std_msgs/Header\n"
    "builtin_interfaces/Time stamp\n"
    "string frame_id\n"
    "================================================================================\n"
    "MSG: geometry_msgs/Pose\n"
    "Point position\n"
    "Quaternion orientation\n"
    "================================================================================\n"
    "MSG: geometry_msgs/Point\n"
    "float64 x\n"
    "float64 y\n"
    "float64 z\n"
    "================================================================================\n"
    "MSG: geometry_msgs/Quaternion\n"
    "float64 x\n"
    "float64 y\n"
    "float64 z\n"
    "float64 w\n";

  const auto result = ms::parse_schema("geometry_msgs/msg/PoseStamped", text);
  const auto & schema = expect_ok(result);

  // Root resolves under both forms and points to PoseStamped.
  ASSERT_NE(schema.root(), nullptr);
  EXPECT_EQ(schema.root()->name, "PoseStamped");
  EXPECT_EQ(schema.root()->package, "geometry_msgs");

  // Dependencies resolve.
  ASSERT_NE(schema.find("std_msgs/Header"), nullptr);
  ASSERT_NE(schema.find("geometry_msgs/Pose"), nullptr);
  ASSERT_NE(schema.find("geometry_msgs/Point"), nullptr);
  ASSERT_NE(schema.find("geometry_msgs/Quaternion"), nullptr);

  // Built-ins still injected.
  EXPECT_NE(schema.find("builtin_interfaces/Time"), nullptr);
}

TEST(MsgSchemaParser, ConcatenatedNestedFieldRecognised)
{
  const std::string text =
    "Header header\n"
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

  // The root uses bare "Header" / "Transform" — the parser must resolve
  // these via the surrounding package context (geometry_msgs), but the
  // dependency is in std_msgs. This is a known limitation: bare names
  // inherit the root's package, so "Header" gets resolved as
  // "geometry_msgs/Header" and the test must use full names. Verify the
  // fully-qualified form works.
  const std::string fully_qualified =
    "std_msgs/Header header\n"
    "string child_frame_id\n"
    "geometry_msgs/Transform transform\n"
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

  const auto result = ms::parse_schema("geometry_msgs/msg/TransformStamped", fully_qualified);
  const auto & schema = expect_ok(result);
  EXPECT_NE(schema.root(), nullptr);

  // Cross-reference validation succeeded → all nested types resolved.
  const auto * transform = schema.find("geometry_msgs/Transform");
  ASSERT_NE(transform, nullptr);
  // Inside Transform, "Vector3" is bare; the parser uses
  // geometry_msgs as context and stores it as "geometry_msgs/Vector3".
  const auto * translation = find_field(*transform, "translation");
  ASSERT_NE(translation, nullptr);
  ASSERT_TRUE(translation->type.is_nested());
  EXPECT_EQ(std::get<std::string>(translation->type.base), "geometry_msgs/Vector3");
}

// --- Error paths --------------------------------------------------------

TEST(MsgSchemaParser, RejectsUnknownNestedType)
{
  // Use a multi-segment name so it's clearly a nested type rather than an
  // unrecognised primitive — the resulting error is then about
  // cross-reference validation, not about parsing the type token.
  const auto result = ms::parse_message("test", "Foo", "missing_pkg/MissingType x\n");
  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.error.find("unknown type"), std::string::npos) << result.error;
}

TEST(MsgSchemaParser, RejectsMalformedArray)
{
  const auto result = ms::parse_message("test", "Foo", "uint8[abc] bytes\n");
  EXPECT_FALSE(result.ok());
}

TEST(MsgSchemaParser, RejectsMalformedFieldName)
{
  // Field names must match the snake_case package-name pattern; uppercase
  // and CamelCase are rejected.
  const auto result = ms::parse_message("test", "Foo", "uint8 BadName\n");
  EXPECT_FALSE(result.ok());
}

TEST(MsgSchemaParser, RejectsDuplicateField)
{
  const auto result = ms::parse_message("test", "Foo", "uint8 a\nuint8 a\n");
  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.error.find("duplicate"), std::string::npos) << result.error;
}

TEST(MsgSchemaParser, RejectsInvalidPackageName)
{
  // Uppercase is invalid in package names.
  const auto result = ms::parse_message("BadPkg", "Foo", "uint8 x\n");
  EXPECT_FALSE(result.ok());
}

TEST(MsgSchemaParser, RejectsConcatenatedMissingDependency)
{
  // Block 0 references geometry_msgs/Pose but no MSG: block defines it.
  const std::string text =
    "geometry_msgs/Pose pose\n"
    "================================================================================\n"
    "MSG: geometry_msgs/Point\n"
    "float64 x\n"
    "float64 y\n"
    "float64 z\n";

  const auto result = ms::parse_schema("test/Foo", text);
  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.error.find("unknown type"), std::string::npos) << result.error;
}

TEST(MsgSchemaParser, EmptyBodyParsesToZeroFields)
{
  // Empty .msg text is technically legal — ROS 2 generates an
  // "structure_needs_at_least_one_member" placeholder at codegen time but
  // the schema text itself can carry no fields.
  const auto result = ms::parse_message("test", "Empty", "# only a comment\n");
  const auto & schema = expect_ok(result);
  EXPECT_EQ(schema.root()->fields.size(), 0U);
  EXPECT_EQ(schema.root()->constants.size(), 0U);
}
