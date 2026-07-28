// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__DECODER__DECODER_HPP_
#define BAGWIZ__CORE__DECODER__DECODER_HPP_

#include "bagwiz/core/cdr_walker/value.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <cstddef>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>

// Top-level decoder abstraction for bagwiz. Hides the choice between
// the schema-driven path (msg_schema parser + cdr_walker) and the
// introspection-typesupport path (rmw_deserialize + MessageMembers
// walk) behind a single interface.
//
// Per-channel decoders are constructed once via open_decoder() and then
// invoked once per message; both backends front-load their setup costs
// (schema parse, .so dlopen) into construction so per-message work is just
// the CDR walk.
namespace bagwiz::core::decoder
{

// Outcome of one decode call. On success, `value` is the decoded message
// (always wrapping an Object at the top level) and `error` is empty. On
// failure, `value` is empty and `error` carries a human-readable message
// suitable for INFO/WARN logging.
struct DecodeResult
{
  std::optional<cdr_walker::Value> value;
  std::string error;

  bool ok() const noexcept { return value.has_value() && error.empty(); }
};

// One decoder per channel. Stateful (caches schema model / typesupport
// handle) but thread-incompatible — callers should not share an instance
// across threads without external synchronisation.
class Decoder
{
public:
  Decoder() = default;
  virtual ~Decoder() = default;

  // Rule of Five (C.21) on a polymorphic interface: explicit defaults so
  // clang-tidy can see them. Concrete derived implementations decide
  // whether copy/move is supported; the abstract base has no state.
  Decoder(const Decoder &) = default;
  Decoder & operator=(const Decoder &) = default;
  Decoder(Decoder &&) noexcept = default;
  Decoder & operator=(Decoder &&) noexcept = default;

  virtual DecodeResult decode(std::span<const std::byte> payload) const = 0;

  // Identifies which backend served this decoder: "schema" when the
  // schema-driven path was chosen, "introspection" when the rosidl
  // typesupport path was used. Surfaced for one-line per-topic DEBUG
  // logs so users can confirm whether their bag exercises the
  // self-describing path (BAGWIZ_LOG_LEVEL=debug to see them).
  virtual std::string_view backend() const noexcept = 0;
};

// Outcome of open_decoder(). On success, `decoder` is non-null and
// `error` is empty; on failure, both backends were unavailable (no
// schema, no typesupport `.so` on AMENT_PREFIX_PATH) and the error
// string explains what was tried.
struct OpenDecoderResult
{
  std::unique_ptr<Decoder> decoder;
  std::string error;

  bool ok() const noexcept { return decoder != nullptr && error.empty(); }
};

// Choose the best decoder for the given topic. Selection policy:
//
//   1. If the BAGWIZ_DECODER environment variable is "introspection",
//      always use the introspection path. Empty / "auto" / "schema" all
//      fall through to the schema-first policy.
//   2. If `topic.schema_text` is non-empty AND `topic.schema_encoding`
//      is "ros2msg" (the only encoding the schema path supports today)
//      AND the parsed schema does not transitively reference wstring
//      or float128 (cdr_walker::Value has no variant for either, so
//      such schemas must take the introspection path) — pick the
//      schema backend.
//   3. Otherwise — pick the introspection backend.
//
// On step 2 failure to parse, the factory falls through to step 3
// silently; on step 3 failure to load typesupport, the factory returns
// an error.
//
// Logs the chosen backend at DEBUG once per call so the user can correlate
// `bagwiz walk` output with which path served it. Run with
// BAGWIZ_LOG_LEVEL=debug to surface those lines; they are suppressed at the
// default INFO level.
OpenDecoderResult open_decoder(const io::TopicInfo & topic);

}  // namespace bagwiz::core::decoder

#endif  // BAGWIZ__CORE__DECODER__DECODER_HPP_
