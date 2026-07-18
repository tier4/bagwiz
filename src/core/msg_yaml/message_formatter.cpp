// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/msg_yaml/message_formatter.hpp"

#include "bagwiz/core/cdr_walker/value.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <variant>

namespace bagwiz::core
{

namespace
{

namespace cdr = bagwiz::core::cdr_walker;

// --- helpers --------------------------------------------------------------

std::string float_to_string(float value)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.7g", static_cast<double>(value));
  return std::string(buf);
}

std::string double_to_string(double value)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.17g", value);
  return std::string(buf);
}

std::string escape_for_yaml(const std::string & s)
{
  std::string out;
  out.reserve(s.size() + 2);
  out += '\'';
  for (const char c : s) {
    if (c == '\'') {
      out += "''";
    } else if (c == '\n') {
      out += "\\n";
    } else if (c == '\r') {
      out += "\\r";
    } else if (c == '\t') {
      out += "\\t";
    } else {
      out += c;
    }
  }
  out += '\'';
  return out;
}

// True when the Value holds one of the leaf primitive variants. Object
// and Sequence are not primitives.
bool is_primitive(const cdr::Value & v)
{
  return !std::holds_alternative<cdr::Object>(v.v) && !std::holds_alternative<cdr::Sequence>(v.v) &&
         !std::holds_alternative<std::monostate>(v.v);
}

// Render a single primitive Value. Number formatting matches the legacy
// MessageMembers walker exactly so existing test corpora and any
// downstream tools that grep YAML keep working.
std::string primitive_to_string(const cdr::Value & v)
{
  return std::visit(
    [](const auto & x) -> std::string {
      using T = std::decay_t<decltype(x)>;
      constexpr bool kIsWideInt =
        std::is_same_v<T, std::uint16_t> || std::is_same_v<T, std::uint32_t> ||
        std::is_same_v<T, std::int16_t> || std::is_same_v<T, std::int32_t> ||
        std::is_same_v<T, std::uint64_t> || std::is_same_v<T, std::int64_t>;
      if constexpr (std::is_same_v<T, bool>) {
        return x ? "true" : "false";
      } else if constexpr (std::is_same_v<T, std::uint8_t>) {
        return std::to_string(static_cast<unsigned>(x));
      } else if constexpr (std::is_same_v<T, std::int8_t>) {
        return std::to_string(static_cast<int>(x));
      } else if constexpr (kIsWideInt) {
        return std::to_string(x);
      } else if constexpr (std::is_same_v<T, float>) {
        return float_to_string(x);
      } else if constexpr (std::is_same_v<T, double>) {
        return double_to_string(x);
      } else if constexpr (std::is_same_v<T, std::string>) {
        return escape_for_yaml(x);
      } else {
        // monostate, Object, Sequence — caller must filter via
        // is_primitive() before calling.
        return "<non-primitive>";
      }
    },
    v.v);
}

// --- emitter --------------------------------------------------------------

class Emitter
{
public:
  Emitter(std::string & out, const FormatOptions & opts) : out_(out), opts_(opts) {}

  // Top-level entry: emit each field of the root Object at no indent.
  void emit_object(const cdr::Object & obj, const std::string & indent)
  {
    for (const auto & [name, value] : obj.fields) {
      emit_field(name, value, indent);
    }
  }

private:
  void emit_field(const std::string & name, const cdr::Value & value, const std::string & indent)
  {
    out_ += indent;
    out_ += name;
    out_ += ':';

    if (const auto * obj = std::get_if<cdr::Object>(&value.v)) {
      out_ += '\n';
      emit_object(*obj, indent + "  ");
      return;
    }
    if (const auto * seq = std::get_if<cdr::Sequence>(&value.v)) {
      emit_sequence(*seq, indent);
      return;
    }
    out_ += ' ';
    out_ += primitive_to_string(value);
    out_ += '\n';
  }

  void emit_sequence(const cdr::Sequence & seq, const std::string & indent)
  {
    if (seq.elements.empty()) {
      out_ += " []\n";
      return;
    }
    // Treat as primitive-array if first element is primitive. Sequences
    // are homogeneous in ROS 2 so checking element 0 is sufficient.
    if (is_primitive(seq.elements.front())) {
      emit_primitive_array(seq, indent);
      return;
    }
    // Sequence of nested objects: block style with `- ` markers.
    out_ += '\n';
    const std::string list_indent = indent + "  ";
    const std::string item_indent = indent + "    ";
    for (const auto & elem : seq.elements) {
      const auto * obj = std::get_if<cdr::Object>(&elem.v);
      if (obj == nullptr) {
        // Non-primitive, non-object element (e.g. a nested sequence).
        // ROS 2 does not allow this in the wire format but render
        // defensively.
        out_ += list_indent + "- <unsupported nested element>\n";
        continue;
      }
      emit_message_list_item(*obj, list_indent, item_indent);
    }
  }

  // `indent` is the indent of the parent field's line (the column where
  // the key was emitted). When the array is too long for inline rendering
  // and expand_long_arrays is set, each element is emitted on its own line
  // as a block-sequence item indented by two more spaces — matching the
  // style used for sequences of nested messages, so downstream YAML
  // parsers see a uniform shape.
  void emit_primitive_array(const cdr::Sequence & seq, const std::string & indent)
  {
    const std::size_t count = seq.elements.size();
    if (count > opts_.max_inline_array) {
      if (opts_.expand_long_arrays) {
        out_ += '\n';
        const std::string list_indent = indent + "  ";
        for (const auto & elem : seq.elements) {
          out_ += list_indent;
          out_ += "- ";
          out_ += primitive_to_string(elem);
          out_ += '\n';
        }
        return;
      }
      out_ += " [<";
      out_ += std::to_string(count);
      out_ += " items>]\n";
      return;
    }
    out_ += " [";
    for (std::size_t i = 0; i < count; ++i) {
      if (i != 0) {
        out_ += ", ";
      }
      out_ += primitive_to_string(seq.elements[i]);
    }
    out_ += "]\n";
  }

  // One element of a list of messages. The first child field uses the
  // "- " dash marker; subsequent ones align under it. Mirrors the
  // YAML-ish style the legacy formatter emitted so reviewers can diff
  // outputs verbatim against the previous formatter output.
  void emit_message_list_item(
    const cdr::Object & obj, const std::string & list_indent, const std::string & item_indent)
  {
    if (obj.fields.empty()) {
      out_ += list_indent;
      out_ += "- {}\n";
      return;
    }
    for (std::size_t i = 0; i < obj.fields.size(); ++i) {
      const auto & entry = obj.fields[i];
      out_ += (i == 0) ? list_indent : item_indent;
      out_ += (i == 0) ? "- " : "";
      out_ += entry.first;
      out_ += ':';
      emit_list_item_child_value(entry.second, item_indent);
    }
  }

  void emit_list_item_child_value(const cdr::Value & value, const std::string & item_indent)
  {
    if (const auto * obj = std::get_if<cdr::Object>(&value.v)) {
      out_ += '\n';
      emit_object(*obj, item_indent + "  ");
      return;
    }
    if (const auto * seq = std::get_if<cdr::Sequence>(&value.v)) {
      if (seq->elements.empty()) {
        out_ += " []\n";
        return;
      }
      if (is_primitive(seq->elements.front())) {
        emit_primitive_array(*seq, item_indent);
        return;
      }
      out_ += '\n';
      const std::string inner_list_indent = item_indent + "  ";
      const std::string inner_item_indent = inner_list_indent + "  ";
      for (const auto & elem : seq->elements) {
        const auto * inner_obj = std::get_if<cdr::Object>(&elem.v);
        if (inner_obj == nullptr) {
          out_ += inner_list_indent + "- <unsupported nested element>\n";
          continue;
        }
        emit_message_list_item(*inner_obj, inner_list_indent, inner_item_indent);
      }
      return;
    }
    out_ += ' ';
    out_ += primitive_to_string(value);
    out_ += '\n';
  }

  std::string & out_;
  const FormatOptions & opts_;
};

}  // namespace

FormatResult format_message(const cdr_walker::Value & root, const FormatOptions & options)
{
  FormatResult result;
  const auto * obj = std::get_if<cdr_walker::Object>(&root.v);
  if (obj == nullptr) {
    result.error = "format_message: top-level Value is not an Object";
    return result;
  }
  Emitter emitter(result.text, options);
  emitter.emit_object(*obj, "");
  return result;
}

}  // namespace bagwiz::core
