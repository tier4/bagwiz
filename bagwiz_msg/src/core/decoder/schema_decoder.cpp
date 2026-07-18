// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/decoder/schema_decoder.hpp"

#include "bagwiz/core/cdr_walker/walker.hpp"
#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/msg_schema/parser.hpp"
#include "bagwiz/core/msg_schema/schema_model.hpp"

#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <variant>

namespace bagwiz::core::decoder
{

namespace
{

namespace ms = bagwiz::core::msg_schema;

// Walk every field of every reachable definition and report whether
// any references wstring or float128. cdr_walker::Value has no variant
// for either, so the schema-driven walker rejects them; the factory
// must route schemas containing them to the introspection backend
// instead. Cheap (a few hundred ns even on the largest TF-style
// schemas) so safe to call at open() time.
bool has_unsupported_primitive(const ms::SchemaModel & schema)
{
  for (const auto & def : schema.definitions()) {
    for (const auto & field : def.fields) {
      if (!field.type.is_primitive()) {
        continue;
      }
      const auto kind = std::get<ms::PrimitiveKind>(field.type.base);
      if (kind == ms::PrimitiveKind::Wstring || kind == ms::PrimitiveKind::LongDouble) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace

DecodeResult SchemaDecoder::decode(std::span<const std::byte> payload) const
{
  const auto walker_result = cdr_walker::decode(schema_, payload);
  DecodeResult out;
  if (walker_result.ok()) {
    out.value = walker_result.value;
  } else {
    out.error = walker_result.error;
  }
  return out;
}

OpenDecoderResult SchemaDecoder::open(const io::TopicInfo & topic)
{
  OpenDecoderResult result;
  if (topic.schema_text.empty()) {
    result.error = "schema_text is empty";
    return result;
  }
  // ros2idl is the other valid encoding but bagwiz's schema parser only
  // handles ros2msg today. Surface as a routing error so the factory
  // falls through to introspection.
  if (!topic.schema_encoding.empty() && topic.schema_encoding != "ros2msg") {
    result.error = "unsupported schema_encoding '" + topic.schema_encoding + "'";
    return result;
  }

  // The MCAP Schema.name is the canonical "pkg/msg/Type" form; the
  // parser accepts either form, so we pass it through unchanged.
  const auto parse = ms::parse_schema(topic.type, topic.schema_text);
  if (!parse.ok()) {
    result.error = "schema parse failed: " + parse.error;
    return result;
  }
  if (has_unsupported_primitive(*parse.schema)) {
    result.error =
      "schema references wstring or float128 — the decoder factory "
      "should route this channel to the introspection backend";
    return result;
  }

  result.decoder = std::make_unique<SchemaDecoder>(std::move(*parse.schema));
  return result;
}

}  // namespace bagwiz::core::decoder
