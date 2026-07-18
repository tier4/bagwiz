// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/msg_yaml/yaml_msg_validate.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>

namespace bagwiz::core
{

namespace
{

namespace ms = bagwiz::core::msg_schema;

std::string path_prefix(std::string_view base)
{
  if (base.empty()) {
    return "";
  }
  return std::string(base) + ".";
}

bool is_integer_scalar(const YAML::Node & n)
{
  if (!n || !n.IsScalar()) {
    return false;
  }
  try {
    (void)n.as<std::int64_t>();
    return true;
  } catch (const YAML::Exception &) {
    return false;
  }
}

YamlMsgValidateResult fail_at(std::string_view path, std::string msg)
{
  if (!path.empty()) {
    return YamlMsgValidateResult::fail(std::string(path) + ": " + std::move(msg));
  }
  return YamlMsgValidateResult::fail(std::move(msg));
}

YamlMsgValidateResult yaml_fits_primitive(
  ms::PrimitiveKind k, std::size_t string_upper_bound, const YAML::Node & n,
  const std::string_view & path_prefix_sv);

YamlMsgValidateResult yaml_fits_message(
  const ms::MessageDef & def, const YAML::Node & map, const ms::SchemaModel & model,
  const std::string_view & path_base);

YamlMsgValidateResult yaml_fits_field_type(
  const ms::FieldType & ft, const YAML::Node & n, const ms::SchemaModel & model,
  const std::string_view & path_base);

YamlMsgValidateResult yaml_fits_sequence_of(
  const ms::FieldType & ft, std::optional<std::size_t> fixed_size,
  std::optional<std::size_t> bounded_max, std::optional<std::size_t> nested_string_bound,
  const YAML::Node & seq, const ms::SchemaModel & model, const std::string_view & path_base)
{
  if (!seq.IsSequence()) {
    return fail_at(path_base, "expected YAML sequence for array field");
  }
  const std::size_t sz = seq.size();
  if (fixed_size.has_value()) {
    if (sz != *fixed_size) {
      return fail_at(
        path_base, "fixed-length array expects " + std::to_string(*fixed_size) + " elements, got " +
                     std::to_string(sz));
    }
  }
  if (bounded_max.has_value() && sz > *bounded_max) {
    return fail_at(path_base, "bounded sequence exceeds max size " + std::to_string(*bounded_max));
  }

  for (std::size_t i = 0; i < sz; ++i) {
    const std::string sub = std::string(path_base) + "[" + std::to_string(i) + "]";
    if (ft.is_primitive()) {
      const ms::PrimitiveKind pk = std::get<ms::PrimitiveKind>(ft.base);
      const std::size_t sub_bound =
        (pk == ms::PrimitiveKind::String || pk == ms::PrimitiveKind::Wstring)
          ? nested_string_bound.value_or(0)
          : 0;
      const auto vr = yaml_fits_primitive(pk, sub_bound, seq[i], sub);
      if (!vr.ok) {
        return vr;
      }
    } else {
      const std::string nested = std::get<std::string>(ft.base);
      const ms::MessageDef * nested_def = model.find(nested);
      if (nested_def == nullptr) {
        return YamlMsgValidateResult::fail(
          std::string(path_base) + ": unknown nested type " + nested);
      }
      const auto vr = yaml_fits_message(*nested_def, seq[i], model, sub);
      if (!vr.ok) {
        return vr;
      }
    }
  }
  return YamlMsgValidateResult::success();
}

YamlMsgValidateResult yaml_fits_primitive(
  ms::PrimitiveKind k, std::size_t string_upper_bound, const YAML::Node & n,
  const std::string_view & path_prefix_sv)
{
  if (!n || (!n.IsScalar() && k != ms::PrimitiveKind::String && k != ms::PrimitiveKind::Wstring)) {
    return fail_at(path_prefix_sv, "expected a scalar YAML node for primitive field");
  }

  switch (k) {
    case ms::PrimitiveKind::Bool: {
      if (!n.IsScalar()) {
        return fail_at(path_prefix_sv, "expected bool scalar");
      }
      try {
        (void)n.as<bool>();
        return YamlMsgValidateResult::success();
      } catch (const YAML::Exception &) {
        return fail_at(path_prefix_sv, "value is not a bool");
      }
    }
    case ms::PrimitiveKind::Byte:
    case ms::PrimitiveKind::Uint8:
      if (!is_integer_scalar(n)) {
        return fail_at(path_prefix_sv, "expected integer in [0,255]");
      }
      {
        const auto u = n.as<std::uint64_t>();
        if (u > 255U) {
          return fail_at(path_prefix_sv, "uint8/byte value out of range");
        }
      }
      return YamlMsgValidateResult::success();

    case ms::PrimitiveKind::Char:
    case ms::PrimitiveKind::Int8:
      if (!is_integer_scalar(n)) {
        return fail_at(path_prefix_sv, "expected integer in [-128,127]");
      }
      {
        const auto v = n.as<std::int64_t>();
        if (v < -128 || v > 127) {
          return fail_at(path_prefix_sv, "int8/char value out of range");
        }
      }
      return YamlMsgValidateResult::success();

    case ms::PrimitiveKind::Uint16:
      if (!is_integer_scalar(n)) {
        return fail_at(path_prefix_sv, "expected uint16-compatible integer scalar");
      }
      {
        const auto u = n.as<std::uint64_t>();
        if (u > 65535U) {
          return fail_at(path_prefix_sv, "uint16 value out of range");
        }
      }
      return YamlMsgValidateResult::success();

    case ms::PrimitiveKind::Int16:
      if (!is_integer_scalar(n)) {
        return fail_at(path_prefix_sv, "expected int16-compatible integer scalar");
      }
      {
        const auto v = n.as<std::int64_t>();
        if (v < -32768 || v > 32767) {
          return fail_at(path_prefix_sv, "int16 value out of range");
        }
      }
      return YamlMsgValidateResult::success();

    case ms::PrimitiveKind::Uint32:
      if (!is_integer_scalar(n)) {
        return fail_at(path_prefix_sv, "expected uint32-compatible integer scalar");
      }
      {
        const auto u = n.as<std::uint64_t>();
        if (u > 4294967295ULL) {
          return fail_at(path_prefix_sv, "uint32 value out of range");
        }
      }
      return YamlMsgValidateResult::success();

    case ms::PrimitiveKind::Int32:
      if (!is_integer_scalar(n)) {
        return fail_at(path_prefix_sv, "expected int32-compatible integer scalar");
      }
      {
        const auto v = n.as<std::int64_t>();
        if (
          v < std::numeric_limits<std::int32_t>::min() ||
          v > std::numeric_limits<std::int32_t>::max()) {
          return fail_at(path_prefix_sv, "int32 value out of range");
        }
      }
      return YamlMsgValidateResult::success();

    case ms::PrimitiveKind::Uint64:
      if (!is_integer_scalar(n)) {
        return fail_at(path_prefix_sv, "expected uint64-compatible integer scalar");
      }
      return YamlMsgValidateResult::success();

    case ms::PrimitiveKind::Int64:
      if (!is_integer_scalar(n)) {
        return fail_at(path_prefix_sv, "expected int64-compatible integer scalar");
      }
      return YamlMsgValidateResult::success();

    case ms::PrimitiveKind::Float32:
      if (!(n.IsScalar())) {
        return fail_at(path_prefix_sv, "expected float-compatible scalar");
      }
      try {
        (void)n.as<float>();
        return YamlMsgValidateResult::success();
      } catch (const YAML::Exception &) {
        return fail_at(path_prefix_sv, "value is not float32-compatible");
      }

    case ms::PrimitiveKind::Float64:
      if (!(n.IsScalar())) {
        return fail_at(path_prefix_sv, "expected float-compatible scalar");
      }
      try {
        (void)n.as<double>();
        return YamlMsgValidateResult::success();
      } catch (const YAML::Exception &) {
        return fail_at(path_prefix_sv, "value is not float64-compatible");
      }

    case ms::PrimitiveKind::String: {
      if (!n.IsScalar()) {
        return fail_at(path_prefix_sv, "expected string scalar");
      }
      const std::string s = n.as<std::string>();
      if (string_upper_bound != 0 && s.size() > string_upper_bound) {
        return fail_at(
          path_prefix_sv,
          "string exceeds declared upper bound (" + std::to_string(string_upper_bound) + ")");
      }
      return YamlMsgValidateResult::success();
    }

    case ms::PrimitiveKind::Wstring:
    case ms::PrimitiveKind::LongDouble:
      return fail_at(path_prefix_sv, "field type not supported by bagwiz YAML pipeline");
    default:
      return fail_at(path_prefix_sv, "unknown primitive kind");
  }
}

YamlMsgValidateResult yaml_fits_field_type(
  const ms::FieldType & ft, const YAML::Node & n, const ms::SchemaModel & model,
  const std::string_view & path_base)
{
  const std::string path = std::string(path_base);

  if (!ft.array.is_array()) {
    if (ft.is_primitive()) {
      const auto pk = std::get<ms::PrimitiveKind>(ft.base);
      const std::size_t sb = ft.string_upper_bound.value_or(0);
      return yaml_fits_primitive(pk, sb, n, path_base);
    }
    const std::string nested = std::get<std::string>(ft.base);
    const ms::MessageDef * nested_def = model.find(nested);
    if (nested_def == nullptr) {
      return YamlMsgValidateResult::fail(path + ": unknown nested type reference '" + nested + "'");
    }
    return yaml_fits_message(*nested_def, n, model, path_base);
  }

  switch (ft.array.kind) {
    case ms::ArrayKind::FixedArray:
      return yaml_fits_sequence_of(
        ft, ft.array.size, std::nullopt, ft.string_upper_bound, n, model, path_base);
    case ms::ArrayKind::BoundedSequence:
      return yaml_fits_sequence_of(
        ft, std::nullopt, ft.array.size, ft.string_upper_bound, n, model, path_base);
    case ms::ArrayKind::UnboundedSequence:
      return yaml_fits_sequence_of(
        ft, std::nullopt, std::nullopt, ft.string_upper_bound, n, model, path_base);
    case ms::ArrayKind::Scalar:
    default:
      return YamlMsgValidateResult::fail(path + ": internal schema error (array kind)");
  }
}

YamlMsgValidateResult yaml_fits_message(
  const ms::MessageDef & def, const YAML::Node & map, const ms::SchemaModel & model,
  const std::string_view & path_base)
{
  if (!map.IsMap()) {
    return fail_at(path_base, "expected YAML mapping for nested message");
  }

  std::unordered_set<std::string> declared;
  declared.reserve(def.fields.size());
  for (const auto & f : def.fields) {
    declared.insert(f.name);
  }

  for (YAML::const_iterator it = map.begin(); it != map.end(); ++it) {
    const std::string k = it->first.as<std::string>();
    if (declared.find(k) == declared.end()) {
      const std::string at_path = path_prefix(path_base) + k;
      return YamlMsgValidateResult::fail(
        at_path + ": unknown field for message type " + def.canonical_name());
    }
  }

  for (const auto & field : def.fields) {
    const std::string subpath = path_prefix(path_base) + field.name;
    const YAML::Node child = map[field.name];
    if (!child) {
      return fail_at(subpath, "missing required field");
    }
    const auto vr = yaml_fits_field_type(field.type, child, model, subpath);
    if (!vr.ok) {
      return vr;
    }
  }
  return YamlMsgValidateResult::success();
}

}  // namespace

YamlMsgValidateResult validate_ros2_yaml_for_message_schema(
  const msg_schema::SchemaModel & schema, const YAML::Node & root)
{
  const ms::MessageDef * msg = schema.root();
  if (msg == nullptr) {
    return YamlMsgValidateResult::fail("schema has no root message definition");
  }
  if (!root.IsMap()) {
    return YamlMsgValidateResult::fail("YAML root must be a mapping keyed by message fields");
  }
  return yaml_fits_message(*msg, root, schema, {});
}

}  // namespace bagwiz::core
