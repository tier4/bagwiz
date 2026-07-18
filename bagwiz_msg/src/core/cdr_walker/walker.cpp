// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/cdr_walker/walker.hpp"

#include "bagwiz/core/cdr_walker/cdr_reader.hpp"
#include "bagwiz/core/cdr_walker/value.hpp"
#include "bagwiz/core/msg_schema/schema_model.hpp"

#include <cstddef>
#include <exception>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>

namespace bagwiz::core::cdr_walker
{

namespace
{

namespace ms = bagwiz::core::msg_schema;

// Forward declaration: messages can nest other messages arbitrarily.
Value decode_message(
  const ms::SchemaModel & schema, const ms::MessageDef & def, CdrReader & reader);

// Read one primitive of the given kind. Throws on Wstring / LongDouble
// (the decoder factory filters those out before calling here, but we
// defend against the parser handing us a schema with one slipping
// through).
Value decode_primitive(ms::PrimitiveKind kind, CdrReader & reader)
{
  switch (kind) {
    case ms::PrimitiveKind::Bool:
      return Value{reader.read_bool()};
    case ms::PrimitiveKind::Byte:
      // CDR `byte` is unsigned 8-bit; ROS 1 distinguished it from uint8
      // by signedness intent but the wire format is identical.
      return Value{reader.read_u8()};
    case ms::PrimitiveKind::Char:
      // Legacy ROS 1 alias for int8; same wire format.
      return Value{reader.read_i8()};
    case ms::PrimitiveKind::Float32:
      return Value{reader.read_f32()};
    case ms::PrimitiveKind::Float64:
      return Value{reader.read_f64()};
    case ms::PrimitiveKind::Int8:
      return Value{reader.read_i8()};
    case ms::PrimitiveKind::Uint8:
      return Value{reader.read_u8()};
    case ms::PrimitiveKind::Int16:
      return Value{reader.read_i16()};
    case ms::PrimitiveKind::Uint16:
      return Value{reader.read_u16()};
    case ms::PrimitiveKind::Int32:
      return Value{reader.read_i32()};
    case ms::PrimitiveKind::Uint32:
      return Value{reader.read_u32()};
    case ms::PrimitiveKind::Int64:
      return Value{reader.read_i64()};
    case ms::PrimitiveKind::Uint64:
      return Value{reader.read_u64()};
    case ms::PrimitiveKind::String:
      return Value{reader.read_string()};
    case ms::PrimitiveKind::Wstring:
      throw std::runtime_error(
        "wstring is not supported by the schema-driven decoder; route through introspection");
    case ms::PrimitiveKind::LongDouble:
      throw std::runtime_error(
        "float128 / long double is not supported by the schema-driven decoder; route through "
        "introspection");
  }
  throw std::runtime_error(
    "unhandled PrimitiveKind in decode_primitive (parser produced an unknown enum value)");
}

// Decode a non-array field. Either a primitive or a nested message.
Value decode_scalar_field(
  const ms::SchemaModel & schema, const ms::FieldType & ft, CdrReader & reader)
{
  if (std::holds_alternative<ms::PrimitiveKind>(ft.base)) {
    return decode_primitive(std::get<ms::PrimitiveKind>(ft.base), reader);
  }
  const auto & ref = std::get<std::string>(ft.base);
  const auto * nested = schema.find(ref);
  if (nested == nullptr) {
    throw std::runtime_error(
      "schema-driven decode hit unknown nested type '" + ref +
      "' (cross-reference check should have caught this)");
  }
  return decode_message(schema, *nested, reader);
}

// Decode an array field of any kind: fixed, bounded sequence, unbounded
// sequence. Sequences (bounded + unbounded) are length-prefixed; fixed
// arrays use the schema-declared size.
Value decode_array_field(
  const ms::SchemaModel & schema, const ms::FieldType & ft, CdrReader & reader)
{
  std::size_t length = 0;
  if (ft.array.is_fixed()) {
    length = ft.array.size.value_or(0);
  } else {
    length = reader.read_sequence_length();
  }

  Sequence seq;
  seq.elements.reserve(length);

  // Fast path for byte-sized primitive arrays. uint8/int8/byte/char are
  // 1-byte per element so we can pull them in a single span without per-
  // element alignment work. Important for sensor_msgs/Image.data and
  // sensor_msgs/PointCloud2.data which are routinely megabytes.
  if (std::holds_alternative<ms::PrimitiveKind>(ft.base)) {
    const auto kind = std::get<ms::PrimitiveKind>(ft.base);
    const bool byte_sized = kind == ms::PrimitiveKind::Uint8 || kind == ms::PrimitiveKind::Byte ||
                            kind == ms::PrimitiveKind::Int8 || kind == ms::PrimitiveKind::Char;
    if (byte_sized && length > 0) {
      auto bytes = reader.read_bytes(length);
      const bool signed_kind = kind == ms::PrimitiveKind::Int8 || kind == ms::PrimitiveKind::Char;
      for (std::size_t i = 0; i < length; ++i) {
        if (signed_kind) {
          seq.elements.emplace_back(static_cast<std::int8_t>(bytes[i]));
        } else {
          seq.elements.emplace_back(static_cast<std::uint8_t>(bytes[i]));
        }
      }
      return Value{std::move(seq)};
    }
  }

  // Generic path: per-element decode.
  // Build a scalar version of the field type (same base, no array) to
  // reuse decode_scalar_field for each element.
  ms::FieldType scalar_ft = ft;
  scalar_ft.array.kind = ms::ArrayKind::Scalar;
  scalar_ft.array.size.reset();
  for (std::size_t i = 0; i < length; ++i) {
    seq.elements.push_back(decode_scalar_field(schema, scalar_ft, reader));
  }
  return Value{std::move(seq)};
}

Value decode_message(const ms::SchemaModel & schema, const ms::MessageDef & def, CdrReader & reader)
{
  Object obj;
  obj.fields.reserve(def.fields.size());

  // Empty messages: ROS 2 codegen inserts `uint8 structure_needs_at_least_one_member`
  // at IDL conversion time so the wire payload always has at least one
  // byte even when the .msg text is empty. Mirror that here so empty
  // messages decode without underflow.
  if (def.fields.empty()) {
    (void)reader.read_u8();
    return Value{std::move(obj)};
  }

  for (const auto & field : def.fields) {
    Value v;
    if (field.type.array.is_array()) {
      v = decode_array_field(schema, field.type, reader);
    } else {
      v = decode_scalar_field(schema, field.type, reader);
    }
    obj.fields.emplace_back(field.name, std::move(v));
  }
  return Value{std::move(obj)};
}

}  // namespace

DecodeResult decode(const msg_schema::SchemaModel & schema, std::span<const std::byte> payload)
{
  DecodeResult result;
  try {
    const auto * root = schema.root();
    if (root == nullptr) {
      throw std::runtime_error("schema has no root message; was the parser given empty input?");
    }
    CdrReader reader(payload);
    result.value = decode_message(schema, *root, reader);
  } catch (const std::exception & e) {
    result.error = e.what();
  }
  return result;
}

}  // namespace bagwiz::core::cdr_walker
