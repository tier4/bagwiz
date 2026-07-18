// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__CDR_WALKER__WALKER_HPP_
#define BAGWIZ__CORE__CDR_WALKER__WALKER_HPP_

#include "bagwiz/core/cdr_walker/value.hpp"
#include "bagwiz/core/msg_schema/schema_model.hpp"

#include <cstddef>
#include <optional>
#include <span>
#include <string>

namespace bagwiz::core::cdr_walker
{

// Outcome of decode(). On success, `value` carries the decoded message
// (always wrapping an Object at the top level) and `error` is empty.
// Failure modes:
//   - schema has no root (parser was given an empty / malformed input)
//   - payload is shorter than the encapsulation header
//   - payload uses PL_CDR encapsulation (XCDR-2 mutable types)
//   - schema references an unsupported type for the schema-driven
//     path: wstring or float128. cdr_walker::Value has no variant for
//     either, so the decoder factory pre-checks the schema and routes
//     such channels to the introspection backend before ever calling
//     decode().
//   - schema references an unknown nested type that the cross-reference
//     check missed (defensive — should not happen with parser output).
//   - CDR underflow (payload truncated mid-field).
struct DecodeResult
{
  std::optional<Value> value;
  std::string error;

  bool ok() const noexcept { return value.has_value() && error.empty(); }
};

// Decode a single CDR-1 payload against the given schema's root message.
// Schema must be the parser output for the channel's TopicInfo type.
DecodeResult decode(const msg_schema::SchemaModel & schema, std::span<const std::byte> payload);

}  // namespace bagwiz::core::cdr_walker

#endif  // BAGWIZ__CORE__CDR_WALKER__WALKER_HPP_
