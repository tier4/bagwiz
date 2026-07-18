// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/msg_schema/parser.hpp"

#include "bagwiz/core/msg_schema/schema_model.hpp"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <exception>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

// Port of `mcap-ros2-support/_vendor/rosidl_adapter/parser.py` (Apache-2.0,
// Open Source Robotics Foundation, 2014-2018), restricted to the message
// subset bagwiz needs. Service / action grammars and the comment /
// annotation processing are intentionally not ported.

namespace bagwiz::core::msg_schema
{

namespace
{

// --- Naming rules (mirror VALID_*_PATTERN regexes) -----------------------

bool is_lower(char c)
{
  return c >= 'a' && c <= 'z';
}
bool is_upper(char c)
{
  return c >= 'A' && c <= 'Z';
}
bool is_digit(char c)
{
  return c >= '0' && c <= '9';
}

// "^(?!.*__)(?!.*_$)[a-z][a-z0-9_]*$" — package/field name pattern.
bool is_valid_package_name(std::string_view s)
{
  if (s.empty() || !is_lower(s.front()) || s.back() == '_') {
    return false;
  }
  for (std::size_t i = 0; i < s.size(); ++i) {
    const char c = s[i];
    if (!(is_lower(c) || is_digit(c) || c == '_')) {
      return false;
    }
    if (c == '_' && i + 1 < s.size() && s[i + 1] == '_') {
      return false;
    }
  }
  return true;
}

// cppcheck-suppress passedByValue
bool is_valid_field_name(std::string_view s)
{
  return is_valid_package_name(s);
}

// "^[A-Z][A-Za-z0-9]*$"
bool is_valid_message_name(std::string_view s)
{
  if (s.empty() || !is_upper(s.front())) {
    return false;
  }
  for (const char c : s) {
    if (!(is_upper(c) || is_lower(c) || is_digit(c))) {
      return false;
    }
  }
  return true;
}

// "^[A-Z]([A-Z0-9_]?[A-Z0-9]+)*$" — constants are SCREAMING_SNAKE.
bool is_valid_constant_name(std::string_view s)
{
  if (s.empty() || !is_upper(s.front())) {
    return false;
  }
  bool prev_underscore = false;
  for (std::size_t i = 1; i < s.size(); ++i) {
    const char c = s[i];
    if (c == '_') {
      if (prev_underscore) {
        return false;  // disallow consecutive underscores
      }
      prev_underscore = true;
      continue;
    }
    if (!(is_upper(c) || is_digit(c))) {
      return false;
    }
    prev_underscore = false;
  }
  // trailing underscore not allowed
  return s.back() != '_';
}

// --- String helpers ------------------------------------------------------

std::string_view ltrim(std::string_view s)
{
  while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) {
    s.remove_prefix(1);
  }
  return s;
}

std::string_view rtrim(std::string_view s)
{
  while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r')) {
    s.remove_suffix(1);
  }
  return s;
}

// cppcheck-suppress passedByValue
std::string_view trim(std::string_view s)
{
  return rtrim(ltrim(s));
}

// Split on the first occurrence of `delim`. Returns (before, after);
// `after` is empty if the delimiter is absent.
std::pair<std::string_view, std::string_view> partition(std::string_view s, char delim)
{
  const auto pos = s.find(delim);
  if (pos == std::string_view::npos) {
    return {s, std::string_view{}};
  }
  return {s.substr(0, pos), s.substr(pos + 1)};
}

// Split on every newline; trailing '\r' is stripped from each line so
// CRLF input round-trips identically to LF input.
std::vector<std::string_view> split_lines(std::string_view s)
{
  std::vector<std::string_view> out;
  std::size_t start = 0;
  for (std::size_t i = 0; i < s.size(); ++i) {
    if (s[i] == '\n') {
      auto line = s.substr(start, i - start);
      if (!line.empty() && line.back() == '\r') {
        line.remove_suffix(1);
      }
      out.push_back(line);
      start = i + 1;
    }
  }
  if (start < s.size()) {
    auto line = s.substr(start);
    if (!line.empty() && line.back() == '\r') {
      line.remove_suffix(1);
    }
    out.push_back(line);
  }
  return out;
}

bool starts_with(std::string_view s, std::string_view prefix)
{
  return s.size() >= prefix.size() && s.substr(0, prefix.size()) == prefix;
}

// --- Type parsing --------------------------------------------------------

// Map a primitive name to its enum value. Returns nullopt for non-primitives.
// cppcheck-suppress passedByValue
std::optional<PrimitiveKind> primitive_from_name(std::string_view name)
{
  // ordered roughly by frequency in real schemas
  if (name == "bool") return PrimitiveKind::Bool;
  if (name == "byte") return PrimitiveKind::Byte;
  if (name == "char") return PrimitiveKind::Char;
  if (name == "float32") return PrimitiveKind::Float32;
  if (name == "float64") return PrimitiveKind::Float64;
  if (name == "float128") return PrimitiveKind::LongDouble;
  if (name == "int8") return PrimitiveKind::Int8;
  if (name == "uint8") return PrimitiveKind::Uint8;
  if (name == "int16") return PrimitiveKind::Int16;
  if (name == "uint16") return PrimitiveKind::Uint16;
  if (name == "int32") return PrimitiveKind::Int32;
  if (name == "uint32") return PrimitiveKind::Uint32;
  if (name == "int64") return PrimitiveKind::Int64;
  if (name == "uint64") return PrimitiveKind::Uint64;
  if (name == "string") return PrimitiveKind::String;
  if (name == "wstring") return PrimitiveKind::Wstring;
  // Legacy ROS 1 aliases. Python parser includes `time` and `duration` in
  // PRIMITIVE_TYPES "for compatibility only" — they should never appear
  // in real ROS 2 .msg text but we accept them as their canonical 8-byte
  // representation by routing through builtin_interfaces.
  return std::nullopt;
}

// cppcheck-suppress passedByValue
bool name_is_legacy_time_or_duration(std::string_view name)
{
  return name == "time" || name == "duration";
}

// Parse a non-negative decimal integer. Returns nullopt on malformed input.
std::optional<std::size_t> parse_size(std::string_view s)
{
  if (s.empty()) {
    return std::nullopt;
  }
  std::size_t value = 0;
  for (const char c : s) {
    if (!is_digit(c)) {
      return std::nullopt;
    }
    const auto d = static_cast<std::size_t>(c - '0');
    // Overflow check: value * 10 + d.
    if (value > (static_cast<std::size_t>(-1) - d) / 10) {
      return std::nullopt;
    }
    value = value * 10 + d;
  }
  return value;
}

// Split off the array suffix `[...]` if present, returning (base, array).
// Throws std::runtime_error on malformed array syntax.
std::pair<std::string_view, ArraySpec> split_array_suffix(std::string_view type_str)
{
  ArraySpec array;
  if (type_str.empty() || type_str.back() != ']') {
    return {type_str, array};
  }

  // Scan from the end to find the matching '['. ROS 2 .msg array syntax
  // does not nest, so a simple rfind is sufficient.
  const auto open = type_str.rfind('[');
  if (open == std::string_view::npos) {
    throw std::runtime_error(
      "type ends with ']' but has no matching '[': " + std::string(type_str));
  }

  const auto base = type_str.substr(0, open);
  auto inside = type_str.substr(open + 1, type_str.size() - open - 2);

  if (inside.empty()) {
    array.kind = ArrayKind::UnboundedSequence;
  } else if (starts_with(inside, "<=")) {
    inside.remove_prefix(2);
    auto size = parse_size(inside);
    if (!size || *size == 0) {
      throw std::runtime_error(
        "bounded sequence size must be a positive integer: " + std::string(type_str));
    }
    array.kind = ArrayKind::BoundedSequence;
    array.size = size;
  } else {
    auto size = parse_size(inside);
    if (!size || *size == 0) {
      throw std::runtime_error(
        "fixed array size must be a positive integer: " + std::string(type_str));
    }
    array.kind = ArrayKind::FixedArray;
    array.size = size;
  }
  return {base, array};
}

// Parse the base part (after stripping any `[...]` suffix). Handles:
//   - primitive name ("bool", "uint32", ...)
//   - bounded string ("string<=N", "wstring<=N")
//   - nested type name ("pkg/Type" or "pkg/msg/Type")
// `context_pkg` supplies the package when the field type omits it (a
// short-form reference like "Header" inside std_msgs).
//
// Throws std::runtime_error on malformed input.
FieldType parse_base_type(std::string_view base, std::string_view context_pkg)
{
  FieldType result;

  if (auto prim = primitive_from_name(base)) {
    result.base = *prim;
    return result;
  }

  // Bounded string / wstring: "string<=N" or "wstring<=N".
  for (const auto * prefix : {"string<=", "wstring<="}) {
    const std::string_view p{prefix};
    if (starts_with(base, p)) {
      const auto bound_str = base.substr(p.size());
      auto bound = parse_size(bound_str);
      if (!bound || *bound == 0) {
        throw std::runtime_error(
          "string upper bound must be a positive integer: " + std::string(base));
      }
      result.base = (p == "string<=") ? PrimitiveKind::String : PrimitiveKind::Wstring;
      result.string_upper_bound = bound;
      return result;
    }
  }

  if (name_is_legacy_time_or_duration(base)) {
    // Legacy ROS 1 names map to builtin_interfaces equivalents. Resolution
    // happens at decode time via SchemaModel::find().
    result.base =
      std::string(base == "time" ? "builtin_interfaces/Time" : "builtin_interfaces/Duration");
    return result;
  }

  // Nested type. Accept "pkg/Type", "pkg/msg/Type", or bare "Type" (using
  // the surrounding message's package). Always normalise to "pkg/Type"
  // before storing so SchemaModel::find() lookups are consistent.
  std::string pkg;
  std::string type;

  std::size_t slash_count = 0;
  for (const char c : base) {
    if (c == '/') {
      ++slash_count;
    }
  }
  if (slash_count == 0) {
    // Special case: bare `Header` always means `std_msgs/Header`,
    // independently of the surrounding package. ROS 2 .msg files
    // typically write the qualified form, but some hand-written and
    // older types use the unqualified shortcut. Mirrors rosbags'
    // `if name == 'Header': name = 'std_msgs/msg/Header'`.
    if (base == "Header") {
      pkg = "std_msgs";
      type = "Header";
    } else if (context_pkg.empty()) {
      throw std::runtime_error(
        "nested type '" + std::string(base) + "' has no package and no context package");
    } else {
      pkg = std::string(context_pkg);
      type = std::string(base);
    }
  } else if (slash_count == 1) {
    auto [a, b] = partition(base, '/');
    pkg = std::string(a);
    type = std::string(b);
  } else if (slash_count == 2) {
    // "pkg/msg/Type" — strip the middle "msg".
    auto [a, rest] = partition(base, '/');
    auto [mid, c] = partition(rest, '/');
    if (mid != "msg") {
      throw std::runtime_error(
        "nested type middle segment must be 'msg', got: " + std::string(base));
    }
    pkg = std::string(a);
    type = std::string(c);
  } else {
    throw std::runtime_error("malformed nested type name: " + std::string(base));
  }

  if (!is_valid_package_name(pkg)) {
    throw std::runtime_error("invalid package name in nested type: '" + pkg + "'");
  }
  if (!is_valid_message_name(type)) {
    throw std::runtime_error("invalid message name in nested type: '" + type + "'");
  }
  result.base = pkg + "/" + type;
  return result;
}

// Parse a complete field type string ("uint8[]", "string<=10", "Header",
// "geometry_msgs/Pose", ...). Throws on malformed input.
// cppcheck-suppress passedByValue
FieldType parse_field_type(std::string_view type_str, std::string_view context_pkg)
{
  auto [base, array] = split_array_suffix(type_str);
  FieldType ft = parse_base_type(base, context_pkg);
  ft.array = array;
  return ft;
}

// --- Line dispatch -------------------------------------------------------

// Strip an inline `# comment` from `line` and return the surviving prefix.
// Anything after the first '#' that is OUTSIDE a string literal is discarded.
// The returned view is rtrimmed.
//
// String-literal awareness matters because ROS 2 .msg fields can carry a
// quoted default value (`string color "red # not a comment"`). Treating the
// first '#' as the comment delimiter would silently truncate that default.
// Default values are wire-irrelevant so md5 / message_definition output is
// unaffected, but the captured raw_value (used in warning messages and any
// future tooling that reads it back) would be wrong.
std::string_view strip_comment(std::string_view line)
{
  char quote = '\0';
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (quote != '\0') {
      if (c == '\\' && i + 1 < line.size()) {
        // Skip the escaped char so an escaped quote (\" or \') does not close
        // the literal.
        ++i;
        continue;
      }
      if (c == quote) {
        quote = '\0';
      }
      continue;
    }
    if (c == '"' || c == '\'') {
      quote = c;
      continue;
    }
    if (c == '#') {
      line = line.substr(0, i);
      break;
    }
  }
  return rtrim(line);
}

// Parse one non-blank, non-comment-only line. Pushes either a FieldDef or
// a ConstantDef onto the appropriate vector. Throws on malformed input.
//
// string_view parameters use const-ref form because the multi-line
// declaration would otherwise place them outside the scope of any
// `cppcheck-suppress passedByValue` comment we could attach.
void parse_line(
  const std::string_view & line, const std::string_view & context_pkg,
  std::vector<FieldDef> & fields, std::vector<ConstantDef> & constants)
{
  // Replace tabs with spaces (Python parser does the same upfront).
  std::string buf;
  buf.reserve(line.size());
  for (const char c : line) {
    buf.push_back(c == '\t' ? ' ' : c);
  }
  const std::string_view normalized{buf};

  auto [type_str, rest_after_type] = partition(normalized, ' ');
  rest_after_type = ltrim(rest_after_type);
  if (rest_after_type.empty()) {
    throw std::runtime_error("field/constant line missing name: " + std::string(line));
  }

  // Is this a constant ("type NAME = value") or a field ("type name [default]")?
  // The presence of '=' AFTER the type token decides it.
  const auto eq_in_rest = rest_after_type.find('=');
  if (eq_in_rest != std::string_view::npos) {
    auto [name_view, value_view] = partition(rest_after_type, '=');
    const auto name = trim(name_view);
    const auto value = trim(value_view);
    if (name.empty()) {
      throw std::runtime_error("constant name is empty: " + std::string(line));
    }
    if (!is_valid_constant_name(name)) {
      throw std::runtime_error(
        "invalid constant name '" + std::string(name) + "' (must match SCREAMING_SNAKE pattern)");
    }
    auto prim = primitive_from_name(type_str);
    if (!prim) {
      throw std::runtime_error("constant type '" + std::string(type_str) + "' must be a primitive");
    }
    ConstantDef c;
    c.name = std::string(name);
    c.type = *prim;
    c.raw_value = std::string(value);
    constants.push_back(std::move(c));
    return;
  }

  auto [name_view, default_view] = partition(rest_after_type, ' ');
  const auto name = trim(name_view);
  const auto default_text = trim(default_view);
  if (name.empty()) {
    throw std::runtime_error("field name is empty: " + std::string(line));
  }
  if (!is_valid_field_name(name)) {
    throw std::runtime_error(
      "invalid field name '" + std::string(name) +
      "' (must match snake_case package-name pattern)");
  }

  FieldDef f;
  f.name = std::string(name);
  f.type = parse_field_type(type_str, context_pkg);
  if (!default_text.empty()) {
    f.default_value = DefaultValue{std::string(default_text)};
  }
  fields.push_back(std::move(f));
}

// Parse the body of one message definition (no MSG: header, no ===
// separators) into a MessageDef.
//
// Parameters take `const std::string_view &` rather than by-value because
// the multi-line declaration breaks cppcheck's `cppcheck-suppress` line
// scoping; the const-ref form sidesteps the warning entirely. Same for
// parse_message() / parse_schema() below.
MessageDef parse_message_body(
  const std::string_view & pkg_name, const std::string_view & msg_name,
  const std::string_view & text)
{
  if (!is_valid_package_name(pkg_name)) {
    throw std::runtime_error("invalid package name: '" + std::string(pkg_name) + "'");
  }
  if (!is_valid_message_name(msg_name)) {
    throw std::runtime_error("invalid message name: '" + std::string(msg_name) + "'");
  }

  MessageDef def;
  def.package = std::string(pkg_name);
  def.name = std::string(msg_name);

  for (auto raw_line : split_lines(text)) {
    auto line = strip_comment(raw_line);
    line = ltrim(line);
    if (line.empty()) {
      continue;
    }
    parse_line(line, pkg_name, def.fields, def.constants);
  }

  // Detect duplicate field/constant names (mirrors Python check).
  std::unordered_set<std::string> seen_fields;
  for (const auto & f : def.fields) {
    if (!seen_fields.insert(f.name).second) {
      throw std::runtime_error(
        std::string(pkg_name) + "/" + std::string(msg_name) +
        " has duplicate field name: " + f.name);
    }
  }
  std::unordered_set<std::string> seen_consts;
  for (const auto & c : def.constants) {
    if (!seen_consts.insert(c.name).second) {
      throw std::runtime_error(
        std::string(pkg_name) + "/" + std::string(msg_name) +
        " has duplicate constant name: " + c.name);
    }
  }

  return def;
}

// --- Concatenated form (MSG: separators) --------------------------------

// Split the schema text into blocks. The first block has no MSG: header
// (it uses the root name); subsequent blocks must start with one. The
// separator is a line whose stripped content is `===...` (3 or more `=`).
struct Block
{
  std::optional<std::string> msg_header;  // "pkg/msg/Type" if present
  std::string body;
};

// cppcheck-suppress passedByValue
bool is_separator_line(std::string_view line)
{
  const auto trimmed = trim(line);
  if (trimmed.size() < 3) {
    return false;
  }
  for (const char c : trimmed) {
    if (c != '=') {
      return false;
    }
  }
  return true;
}

// cppcheck-suppress passedByValue
std::vector<Block> split_blocks(std::string_view text)
{
  std::vector<Block> blocks;
  blocks.emplace_back();

  for (auto line : split_lines(text)) {
    if (is_separator_line(line)) {
      blocks.emplace_back();
      continue;
    }
    auto trimmed = ltrim(line);
    if (starts_with(trimmed, "MSG:") && !blocks.back().msg_header && blocks.back().body.empty()) {
      auto rest = ltrim(trimmed.substr(4));
      blocks.back().msg_header = std::string(trim(rest));
      continue;
    }
    blocks.back().body.append(line.data(), line.size());
    blocks.back().body.push_back('\n');
  }

  // Drop any blocks that are completely empty (can happen with trailing
  // separator lines or all-whitespace input between separators).
  blocks.erase(
    std::remove_if(
      blocks.begin(), blocks.end(),
      [](const Block & b) { return !b.msg_header && trim(std::string_view(b.body)).empty(); }),
    blocks.end());

  return blocks;
}

// Split "pkg/msg/Type" or "pkg/Type" into (pkg, type). Throws on malformed
// input.
std::pair<std::string, std::string> split_full_type(std::string_view full)
{
  std::size_t slash_count = 0;
  for (const char c : full) {
    if (c == '/') {
      ++slash_count;
    }
  }
  if (slash_count == 1) {
    auto [a, b] = partition(full, '/');
    return {std::string(a), std::string(b)};
  }
  if (slash_count == 2) {
    auto [a, rest] = partition(full, '/');
    auto [mid, c] = partition(rest, '/');
    if (mid != "msg") {
      throw std::runtime_error(
        "schema name middle segment must be 'msg', got: " + std::string(full));
    }
    return {std::string(a), std::string(c)};
  }
  throw std::runtime_error(
    "schema name must be 'pkg/Type' or 'pkg/msg/Type', got: " + std::string(full));
}

// Inject the always-available builtin_interfaces messages. ROS 2 omits
// these from MCAP schema text on the assumption decoders know them.
void inject_builtins(SchemaModel & model)
{
  for (const auto * type_name : {"Time", "Duration"}) {
    const std::string short_name = std::string("builtin_interfaces/") + type_name;
    if (model.find(short_name) != nullptr) {
      continue;
    }
    MessageDef def;
    def.package = "builtin_interfaces";
    def.name = type_name;
    {
      FieldDef f;
      f.name = "sec";
      f.type.base = PrimitiveKind::Int32;  // ROS 2 builtin_interfaces/Time uses int32 sec
      def.fields.push_back(std::move(f));
    }
    {
      FieldDef f;
      f.name = "nanosec";
      f.type.base = PrimitiveKind::Uint32;
      def.fields.push_back(std::move(f));
    }
    model.add(std::move(def));
  }
}

// Walk every field of every definition; ensure nested type references all
// resolve in the model. Mirrors `validate_field_types` from the Python
// parser. Throws on the first unresolved reference.
void validate_cross_references(const SchemaModel & model)
{
  for (const auto & def : model.definitions()) {
    for (const auto & field : def.fields) {
      if (!field.type.is_nested()) {
        continue;
      }
      const auto & ref = std::get<std::string>(field.type.base);
      if (model.find(ref) == nullptr) {
        throw std::runtime_error(
          def.short_name() + ": field '" + field.name + "' references unknown type '" + ref + "'");
      }
    }
  }
}

}  // namespace

ParseResult parse_message(
  const std::string_view & pkg_name, const std::string_view & msg_name,
  const std::string_view & text)
{
  ParseResult result;
  try {
    auto def = parse_message_body(pkg_name, msg_name, text);
    SchemaModel model;
    model.add(std::move(def), /*is_root=*/true);
    inject_builtins(model);
    validate_cross_references(model);
    result.schema = std::move(model);
  } catch (const std::exception & e) {
    result.error = e.what();
  }
  return result;
}

ParseResult parse_schema(const std::string_view & root_schema_name, const std::string_view & text)
{
  ParseResult result;
  try {
    const auto blocks = split_blocks(text);
    if (blocks.empty()) {
      throw std::runtime_error("schema text contains no message definitions");
    }

    SchemaModel model;
    for (std::size_t i = 0; i < blocks.size(); ++i) {
      const std::string full_name =
        i == 0 ? std::string(root_schema_name)
               : (blocks[i].msg_header ? *blocks[i].msg_header : std::string{});
      if (full_name.empty()) {
        throw std::runtime_error(
          "block " + std::to_string(i) + " has no MSG: header and is not the root");
      }
      auto [pkg, type] = split_full_type(full_name);
      auto def = parse_message_body(pkg, type, blocks[i].body);
      model.add(std::move(def), /*is_root=*/i == 0);
    }

    inject_builtins(model);
    validate_cross_references(model);
    result.schema = std::move(model);
  } catch (const std::exception & e) {
    result.error = e.what();
  }
  return result;
}

}  // namespace bagwiz::core::msg_schema
