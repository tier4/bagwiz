// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__MSG_YAML__MESSAGE_FORMATTER_HPP_
#define BAGWIZ__CORE__MSG_YAML__MESSAGE_FORMATTER_HPP_

#include "bagwiz/core/cdr_walker/value.hpp"

#include <cstddef>
#include <string>

namespace bagwiz::core
{

// Options controlling how format_message renders large values. By default,
// primitive arrays with up to `max_inline_array` elements are rendered
// inline as `[a, b, c]`; longer arrays are summarized as `[<N items>]` to
// keep terminal output scannable.
//
// When `expand_long_arrays` is set, the same long arrays are instead
// rendered as a YAML block sequence (one element per line, with `- `
// markers) so every value is visible without flowing past the right edge
// of the terminal.
struct FormatOptions
{
  std::size_t max_inline_array = 32;
  bool expand_long_arrays = false;
};

// Outcome of a format_message() call. On success `text` holds the rendered
// YAML-ish string; on failure `error` explains what went wrong (always a
// shape mismatch — the bytes-to-Value step is the decoder's job).
struct FormatResult
{
  std::string text;
  std::string error;
  bool ok() const { return error.empty(); }
};

// Render a decoded message to a YAML-ish string mirroring `ros2 topic
// echo`. Input is the Value produced by the decoder factory (either
// the schema-driven or the introspection-based backend — both yield
// the same shape).
//
// The Value MUST wrap a top-level Object (a struct). Primitive or
// sequence Values at the root are rejected; the shape contract for
// "decoded message" is always an Object.
FormatResult format_message(const cdr_walker::Value & root, const FormatOptions & options = {});

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__MSG_YAML__MESSAGE_FORMATTER_HPP_
