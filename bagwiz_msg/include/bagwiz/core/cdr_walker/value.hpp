// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__CDR_WALKER__VALUE_HPP_
#define BAGWIZ__CORE__CDR_WALKER__VALUE_HPP_

#include <cstdint>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

// In-memory representation of a single decoded ROS 2 message. Produced
// by the CDR walker (decode()); consumed by the decoder factory's
// MessageView wrapper and the downstream YAML / pose / TF formatters.
//
// Numeric primitives are kept in their original IDL widths instead of
// being widened to int64/double — the YAML formatter and the tf math
// helpers need exact bit-pattern preservation (e.g. float32 vs float64
// quaternions, uint8 enums on diagnostic_msgs).
//
// Object and Sequence reference Value recursively. std::vector is
// complete-type-tolerant since C++17 so we model these inline rather
// than via unique_ptr indirection.
namespace bagwiz::core::cdr_walker
{

struct Value;

// Object: ordered list of (field_name, value) pairs. Order matches the
// declaration order in the .msg schema; lookup is O(N) which is fine for
// schemas with single-digit field counts (the common case).
struct Object
{
  std::vector<std::pair<std::string, Value>> fields;
};

// Sequence: ordered list of elements. Used for fixed arrays, bounded
// sequences, and unbounded sequences alike — the schema-side ArraySpec
// already distinguishes them.
struct Sequence
{
  std::vector<Value> elements;
};

// Tagged union. The numeric variants are present in their full set so a
// downstream consumer can match on the original CDR type. `std::monostate`
// represents an uninitialized / null Value (only used as a sentinel
// during construction; the walker never produces one).
struct Value
{
  std::variant<
    std::monostate, bool, std::int8_t, std::uint8_t, std::int16_t, std::uint16_t, std::int32_t,
    std::uint32_t, std::int64_t, std::uint64_t, float, double,
    std::string,  // utf-8; wstring is intentionally absent — the
                  // decoder factory routes wstring schemas to the
                  // introspection backend instead.
    Object, Sequence>
    v;

  Value() = default;
  // Direct-init forwarding constructor: lets `Value{x}` compile for any
  // alternative of the underlying variant. SFINAE-guarded so it does
  // NOT participate in overload resolution when T is Value itself —
  // otherwise the universal reference would shadow the implicit copy
  // and move constructors and break std::optional<Value> /
  // std::vector<Value>. Marked explicit (linter requirement); call
  // sites use brace-init form, so the constraint is invisible at use.
  template <typename T, typename = std::enable_if_t<!std::is_same_v<std::decay_t<T>, Value>>>
  explicit Value(T && x) : v(std::forward<T>(x))
  {
  }

  bool is_object() const noexcept { return std::holds_alternative<Object>(v); }
  bool is_sequence() const noexcept { return std::holds_alternative<Sequence>(v); }
};

}  // namespace bagwiz::core::cdr_walker

#endif  // BAGWIZ__CORE__CDR_WALKER__VALUE_HPP_
