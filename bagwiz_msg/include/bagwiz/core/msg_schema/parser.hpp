// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__MSG_SCHEMA__PARSER_HPP_
#define BAGWIZ__CORE__MSG_SCHEMA__PARSER_HPP_

#include "bagwiz/core/msg_schema/schema_model.hpp"

#include <optional>
#include <string>
#include <string_view>

// Parser entry points for ROS 2 .msg schema text. The grammar matches
// what `rosidl_adapter` accepts (Apache-2.0, ROS 2 Humble); the
// concatenated multi-type form is the one MCAP emits for self-describing
// channels.
//
// Errors are reported as a result struct (no exceptions on the public
// boundary) so the decoder factory and the YAML formatter can fall
// back to the introspection path without try/catch noise.
namespace bagwiz::core::msg_schema
{

// Outcome of a parse call. On success, `schema` carries the built model
// and `error` is empty. On failure, `schema` is empty and `error` carries
// a human-readable message including the offending package/type name and,
// when available, the line context.
struct ParseResult
{
  std::optional<SchemaModel> schema;
  std::string error;

  bool ok() const noexcept { return schema.has_value() && error.empty(); }
};

// Parse a single-message .msg text (no `===` separators, no `MSG:`
// headers). The returned SchemaModel has exactly one definition (plus
// the auto-injected built-ins) and `root()` points at it.
//
// `pkg_name` and `msg_name` are taken from outside the text — in MCAP,
// from the channel's Schema.name; in tests, supplied explicitly. They
// are validated against the ROS 2 naming rules.
//
// Parameters take `const std::string_view &` (rather than by-value, the
// usual idiom) so that multi-line declarations stay friendly to
// cppcheck's per-line `cppcheck-suppress` scoping.
ParseResult parse_message(
  const std::string_view & pkg_name, const std::string_view & msg_name,
  const std::string_view & text);

// Parse the MCAP concatenated form. The root block (the text before any
// `===`-prefixed separator line) uses `root_schema_name`; subsequent
// blocks must declare their own type via a `MSG: pkg/msg/Type` header
// line. After parsing, every nested type referenced by any field must
// resolve via SchemaModel::find() or the call fails with an
// UnknownMessageType error.
//
// Accepts both "pkg/Type" and "pkg/msg/Type" forms in `root_schema_name`
// and in MSG: headers, matching how rosbag2 writes schemas.
ParseResult parse_schema(const std::string_view & root_schema_name, const std::string_view & text);

}  // namespace bagwiz::core::msg_schema

#endif  // BAGWIZ__CORE__MSG_SCHEMA__PARSER_HPP_
