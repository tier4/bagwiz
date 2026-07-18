// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__MSG_SCHEMA__SCHEMA_MODEL_HPP_
#define BAGWIZ__CORE__MSG_SCHEMA__SCHEMA_MODEL_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <variant>
#include <vector>

// In-memory model of a parsed ROS 2 .msg schema. Designed as the input
// to the schema-driven CDR walker (cdr_walker::decode) — every field
// carries enough information to drive alignment, length-prefix
// handling, and nested-type recursion without re-parsing the schema
// text.
//
// The shapes mirror Foxglove `mcap-ros2-support` (Apache-2.0):
// `python/mcap-ros2-support/mcap_ros2/_vendor/rosidl_adapter/parser.py`,
// which itself vendors ROS 2 `rosidl_adapter`. Names are kept close to the
// reference so the port is easy to audit; C++-idiomatic shapes (variants,
// optionals) replace Python's duck-typed attributes.
namespace bagwiz::core::msg_schema
{

// All ROS 2 IDL primitive scalar types. Scope matches Python
// `PRIMITIVE_TYPES` in parser.py except that `duration` and `time` are
// modelled as built-in nested message references (`builtin_interfaces/Time`,
// `builtin_interfaces/Duration`) — see SchemaModel::find().
//
// `Wstring` and `LongDouble` (== `float128`) parse cleanly here but the
// schema-driven CDR decoder will reject them — cdr_walker::Value has
// no variant for either — and the decoder factory falls back to the
// introspection path for channels whose schemas use these types.
enum class PrimitiveKind : std::uint8_t {
  Bool,
  Byte,  // unsigned 8-bit; semantically distinct from Uint8 in IDL
  Char,  // signed 8-bit; legacy, deprecated in favour of Int8
  Float32,
  Float64,
  LongDouble,  // 128-bit IEEE / x86 80-bit (platform-dependent)
  Int8,
  Uint8,
  Int16,
  Uint16,
  Int32,
  Uint32,
  Int64,
  Uint64,
  String,
  Wstring,
};

enum class ArrayKind : std::uint8_t {
  Scalar,             // T
  FixedArray,         // T[N]            -- size known at compile time
  BoundedSequence,    // T[<=N]          -- variable length, capped at N
  UnboundedSequence,  // T[]             -- variable length, no cap
};

// Array attributes for a field. `size` carries N for FixedArray /
// BoundedSequence, and is empty for Scalar / UnboundedSequence.
struct ArraySpec
{
  ArrayKind kind = ArrayKind::Scalar;
  std::optional<std::size_t> size;

  bool is_array() const { return kind != ArrayKind::Scalar; }
  bool is_fixed() const { return kind == ArrayKind::FixedArray; }
  bool is_bounded_sequence() const { return kind == ArrayKind::BoundedSequence; }
  bool is_unbounded_sequence() const { return kind == ArrayKind::UnboundedSequence; }
};

// Either a primitive (PrimitiveKind) or a nested message reference
// stored as the canonical short name "pkg/Type" (matches what
// SchemaModel::find() looks up).
using FieldBase = std::variant<PrimitiveKind, std::string>;

struct FieldType
{
  FieldBase base;
  ArraySpec array;

  // Only populated when base == PrimitiveKind::String / Wstring and the
  // field declared an upper bound (e.g. `string<=10`). Empty otherwise.
  std::optional<std::size_t> string_upper_bound;

  bool is_primitive() const { return std::holds_alternative<PrimitiveKind>(base); }
  bool is_nested() const { return std::holds_alternative<std::string>(base); }
};

// Default values are parsed but never interpreted by the .msg parser
// or the schema-driven CDR decoder — they only matter to encoders.
// Stored as the raw post-`field_name` text so a future encoder can
// re-parse them without us having to settle the integer/float/bool/
// string ambiguity here.
struct DefaultValue
{
  std::string raw;
};

struct FieldDef
{
  std::string name;
  FieldType type;
  std::optional<DefaultValue> default_value;
};

// Constants are always primitive (the IDL grammar rejects nested types and
// arrays for constants). Value text is kept raw, same reasoning as
// DefaultValue.
struct ConstantDef
{
  std::string name;
  PrimitiveKind type = PrimitiveKind::Bool;
  std::string raw_value;
};

// One full message definition. `package` and `name` together form the
// canonical "pkg/msg/Name"; the short form "pkg/Name" is what the parser
// uses for cross-referencing nested types.
struct MessageDef
{
  std::string package;  // "geometry_msgs"
  std::string name;     // "PoseStamped"
  std::vector<FieldDef> fields;
  std::vector<ConstantDef> constants;

  // "geometry_msgs/PoseStamped" — used for SchemaModel lookup keys.
  std::string short_name() const { return package + "/" + name; }

  // "geometry_msgs/msg/PoseStamped" — the canonical name used in MCAP
  // channel `Schema.name` and ROS 2 introspection symbols.
  std::string canonical_name() const { return package + "/msg/" + name; }
};

// A complete parsed schema: the root MessageDef plus every nested
// definition referenced by it (transitively). MCAP concatenated-form
// schemas (one root + dependent definitions separated by `===` and
// `MSG: pkg/msg/Type` headers) parse into this shape.
//
// Lookups accept both "pkg/Type" and "pkg/msg/Type" forms. The two
// always-available built-in types `builtin_interfaces/Time` and
// `builtin_interfaces/Duration` are auto-injected: ROS 2 omits them from
// MCAP schema text on the assumption that decoders know them.
class SchemaModel
{
public:
  SchemaModel() = default;

  // The root message (the type named in the MCAP `Schema.name`).
  // Returns nullptr if no root has been set.
  //
  // Resolved via index lookup, not a stored pointer, because add() may
  // reallocate the underlying vector while building up the model — a
  // raw pointer captured at insertion would dangle by the time the
  // caller asked for it.
  const MessageDef * root() const noexcept
  {
    if (!root_index_.has_value()) {
      return nullptr;
    }
    return &definitions_[*root_index_];
  }
  std::string_view root_short_name() const noexcept { return root_short_name_; }

  // Look up a definition by either short ("pkg/Type") or canonical
  // ("pkg/msg/Type") form. Returns nullptr when not found.
  const MessageDef * find(std::string_view type_name) const noexcept;

  // Insert a definition. The first inserted definition with
  // `is_root == true` becomes root(). Both lookup forms are registered.
  void add(MessageDef def, bool is_root = false);

  // Number of stored definitions (root + dependencies + built-ins).
  std::size_t size() const noexcept { return definitions_.size(); }

  // Iteration: order is insertion order so callers can rebuild MCAP
  // concatenated form deterministically.
  const std::vector<MessageDef> & definitions() const noexcept { return definitions_; }

private:
  std::vector<MessageDef> definitions_;
  std::unordered_map<std::string, std::size_t> by_short_name_;
  std::unordered_map<std::string, std::size_t> by_canonical_name_;
  std::optional<std::size_t> root_index_;
  std::string root_short_name_;
};

}  // namespace bagwiz::core::msg_schema

#endif  // BAGWIZ__CORE__MSG_SCHEMA__SCHEMA_MODEL_HPP_
