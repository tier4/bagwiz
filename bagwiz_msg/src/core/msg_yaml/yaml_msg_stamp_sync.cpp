// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/msg_yaml/yaml_msg_stamp_sync.hpp"

#include <yaml-cpp/yaml.h>

#include <limits>
#include <optional>
#include <string>
#include <string_view>

namespace bagwiz::core
{

namespace
{

namespace ms = bagwiz::core::msg_schema;

const ms::FieldDef * find_field(const ms::MessageDef & def, const std::string_view & name)
{
  for (const auto & f : def.fields) {
    if (f.name == name) {
      return &f;
    }
  }
  return nullptr;
}

const ms::MessageDef * require_nested_message(
  const ms::SchemaModel & schema, const ms::FieldDef & field, std::string_view label,
  std::string & error)
{
  if (field.type.array.is_array()) {
    error = std::string(label) + " must be a scalar field";
    return nullptr;
  }
  if (!field.type.is_nested()) {
    error = std::string(label) + " must be a nested message";
    return nullptr;
  }
  const auto * nested = schema.find(std::get<std::string>(field.type.base));
  if (nested == nullptr) {
    error = std::string("schema is missing nested definition for ") + std::string(label);
    return nullptr;
  }
  return nested;
}

bool require_primitive_scalar(
  const ms::FieldDef & field, ms::PrimitiveKind expected, std::string_view label,
  std::string & error)
{
  if (field.type.array.is_array()) {
    error = std::string(label) + " must be a scalar field";
    return false;
  }
  if (!field.type.is_primitive()) {
    error = std::string(label) + " must be a primitive field";
    return false;
  }
  if (std::get<ms::PrimitiveKind>(field.type.base) != expected) {
    error = std::string(label) + " has incompatible type";
    return false;
  }
  return true;
}

}  // namespace

SyncMsgStampResult sync_top_level_header_stamp_to_time(
  const msg_schema::SchemaModel & schema, YAML::Node & root, std::int64_t stamp_ns)
{
  const auto * root_def = schema.root();
  if (root_def == nullptr) {
    return SyncMsgStampResult::fail("schema has no root message definition");
  }
  if (!root.IsMap()) {
    return SyncMsgStampResult::fail("YAML root must be a mapping");
  }

  const auto * header_field = find_field(*root_def, "header");
  if (header_field == nullptr) {
    return SyncMsgStampResult::fail(
      "message type has no top-level 'header' field for --sync-msg-stamp");
  }

  std::string type_error;
  const auto * header_def =
    require_nested_message(schema, *header_field, "top-level 'header'", type_error);
  if (header_def == nullptr) {
    return SyncMsgStampResult::fail(type_error);
  }

  const auto * stamp_field = find_field(*header_def, "stamp");
  if (stamp_field == nullptr) {
    return SyncMsgStampResult::fail("'header' has no 'stamp' field");
  }

  const auto * stamp_def =
    require_nested_message(schema, *stamp_field, "'header.stamp'", type_error);
  if (stamp_def == nullptr) {
    return SyncMsgStampResult::fail(type_error);
  }

  const auto * sec_field = find_field(*stamp_def, "sec");
  const auto * nanosec_field = find_field(*stamp_def, "nanosec");
  if (sec_field == nullptr || nanosec_field == nullptr) {
    return SyncMsgStampResult::fail("'header.stamp' must contain 'sec' and 'nanosec'");
  }
  if (!require_primitive_scalar(
        *sec_field, ms::PrimitiveKind::Int32, "'header.stamp.sec'", type_error)) {
    return SyncMsgStampResult::fail(type_error);
  }
  if (!require_primitive_scalar(
        *nanosec_field, ms::PrimitiveKind::Uint32, "'header.stamp.nanosec'", type_error)) {
    return SyncMsgStampResult::fail(type_error);
  }

  const std::int64_t kBillion = 1'000'000'000LL;
  std::int64_t sec = stamp_ns / kBillion;
  std::int64_t nanosec = stamp_ns % kBillion;
  if (nanosec < 0) {
    nanosec += kBillion;
    --sec;
  }

  if (
    sec < std::numeric_limits<std::int32_t>::min() ||
    sec > std::numeric_limits<std::int32_t>::max()) {
    return SyncMsgStampResult::fail("stamp is out of range for int32 seconds field");
  }

  YAML::Node header = root["header"];
  if (!header || !header.IsMap()) {
    return SyncMsgStampResult::fail(
      "YAML 'header' must be a mapping when --sync-msg-stamp is enabled");
  }

  YAML::Node stamp = header["stamp"];
  if (stamp && !stamp.IsMap()) {
    return SyncMsgStampResult::fail(
      "YAML 'header.stamp' must be a mapping when --sync-msg-stamp is enabled");
  }

  header["stamp"]["sec"] = static_cast<std::int32_t>(sec);
  header["stamp"]["nanosec"] = static_cast<std::uint32_t>(nanosec);
  return SyncMsgStampResult::success();
}

}  // namespace bagwiz::core
