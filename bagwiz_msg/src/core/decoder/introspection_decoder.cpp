// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/decoder/introspection_decoder.hpp"

#include "bagwiz/core/cdr_walker/value.hpp"
#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/introspection/introspection_loader.hpp"
#include "bagwiz/core/introspection/message_deserializer.hpp"

#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <utility>

namespace bagwiz::core::decoder
{

namespace
{

namespace ts_types = rosidl_typesupport_introspection_cpp;

// Forward decl: nested messages recurse into the same routine.
cdr_walker::Object walk_members(const ts_types::MessageMembers & members, const void * base);

// Read one primitive at the given pointer, tagging it with the matching
// cdr_walker::Value variant. Mirrors the type table in message_formatter
// but emits Value variants instead of formatted strings so consumers
// (the YAML formatter, traj, tf) can pattern-match on the original CDR
// type rather than parsing strings.
//
// Wstring and long double are out of scope for cdr_walker::Value, which
// has no variant for either. The introspection path nominally COULD
// handle them via the underlying typesupport, but we throw here to keep
// behaviour aligned with the schema-driven path (which rejects such
// schemas at parse time).
cdr_walker::Value read_primitive(const void * ptr, std::uint8_t type_id)
{
  switch (type_id) {
    case ts_types::ROS_TYPE_BOOLEAN:
      return cdr_walker::Value{*static_cast<const bool *>(ptr)};
    case ts_types::ROS_TYPE_OCTET:
    case ts_types::ROS_TYPE_UINT8:
      return cdr_walker::Value{*static_cast<const std::uint8_t *>(ptr)};
    case ts_types::ROS_TYPE_CHAR:
    case ts_types::ROS_TYPE_INT8:
      return cdr_walker::Value{*static_cast<const std::int8_t *>(ptr)};
    case ts_types::ROS_TYPE_UINT16:
      return cdr_walker::Value{*static_cast<const std::uint16_t *>(ptr)};
    case ts_types::ROS_TYPE_INT16:
      return cdr_walker::Value{*static_cast<const std::int16_t *>(ptr)};
    case ts_types::ROS_TYPE_UINT32:
      return cdr_walker::Value{*static_cast<const std::uint32_t *>(ptr)};
    case ts_types::ROS_TYPE_INT32:
      return cdr_walker::Value{*static_cast<const std::int32_t *>(ptr)};
    case ts_types::ROS_TYPE_UINT64:
      return cdr_walker::Value{*static_cast<const std::uint64_t *>(ptr)};
    case ts_types::ROS_TYPE_INT64:
      return cdr_walker::Value{*static_cast<const std::int64_t *>(ptr)};
    case ts_types::ROS_TYPE_FLOAT:
      return cdr_walker::Value{*static_cast<const float *>(ptr)};
    case ts_types::ROS_TYPE_DOUBLE:
      return cdr_walker::Value{*static_cast<const double *>(ptr)};
    case ts_types::ROS_TYPE_STRING:
      return cdr_walker::Value{*static_cast<const std::string *>(ptr)};
    case ts_types::ROS_TYPE_WSTRING:
      throw std::runtime_error(
        "wstring is not representable in cdr_walker::Value (introspection path)");
    case ts_types::ROS_TYPE_LONG_DOUBLE:
      throw std::runtime_error(
        "float128 / long double is not representable in cdr_walker::Value (introspection path)");
    case ts_types::ROS_TYPE_WCHAR:
      // wchar is a 16-bit type that no production message in the bagwiz
      // test corpus uses; keep parity with the schema decoder by not
      // including it in Value.
      throw std::runtime_error(
        "wchar is not representable in cdr_walker::Value (introspection path)");
    default:
      throw std::runtime_error(
        "unknown introspection type id " + std::to_string(static_cast<int>(type_id)));
  }
}

const ts_types::MessageMembers & nested_members_of(const ts_types::MessageMember & m)
{
  return *static_cast<const ts_types::MessageMembers *>(m.members_->data);
}

// Element count for an array/sequence member. Falls back to array_size_
// when size_function is unset (rosidl-generated code always sets it,
// but defensive).
std::size_t member_count(const ts_types::MessageMember & m, const void * field_ptr)
{
  if (m.size_function != nullptr) {
    return m.size_function(field_ptr);
  }
  return m.array_size_;
}

const void * member_element(
  const ts_types::MessageMember & m, const void * field_ptr, std::size_t index)
{
  // Universal accessor for std::array, std::vector and BoundedVector.
  return m.get_const_function(field_ptr, index);
}

cdr_walker::Value walk_field(const ts_types::MessageMember & m, const void * field)
{
  if (!m.is_array_) {
    if (m.type_id_ != ts_types::ROS_TYPE_MESSAGE) {
      return read_primitive(field, m.type_id_);
    }
    return cdr_walker::Value{walk_members(nested_members_of(m), field)};
  }

  const std::size_t count = member_count(m, field);
  cdr_walker::Sequence seq;
  seq.elements.reserve(count);
  if (m.type_id_ != ts_types::ROS_TYPE_MESSAGE) {
    for (std::size_t i = 0; i < count; ++i) {
      seq.elements.push_back(read_primitive(member_element(m, field, i), m.type_id_));
    }
  } else {
    const auto & sub = nested_members_of(m);
    for (std::size_t i = 0; i < count; ++i) {
      seq.elements.emplace_back(walk_members(sub, member_element(m, field, i)));
    }
  }
  return cdr_walker::Value{std::move(seq)};
}

cdr_walker::Object walk_members(const ts_types::MessageMembers & members, const void * base)
{
  cdr_walker::Object obj;
  obj.fields.reserve(members.member_count_);
  const auto * base_bytes = static_cast<const std::uint8_t *>(base);
  for (std::uint32_t i = 0; i < members.member_count_; ++i) {
    const ts_types::MessageMember & m = members.members_[i];
    const void * field = base_bytes + m.offset_;
    obj.fields.emplace_back(m.name_, walk_field(m, field));
  }
  return obj;
}

}  // namespace

DecodeResult IntrospectionDecoder::decode(std::span<const std::byte> payload) const
{
  DecodeResult result;
  try {
    DeserializedMessage deserialized(introspection_, payload);
    auto obj = walk_members(deserialized.members(), deserialized.data());
    result.value = cdr_walker::Value{std::move(obj)};
  } catch (const std::exception & e) {
    result.error = e.what();
  }
  return result;
}

OpenDecoderResult IntrospectionDecoder::open(const io::TopicInfo & topic)
{
  OpenDecoderResult result;
  if (topic.type.empty()) {
    result.error = "topic.type is empty; cannot resolve introspection typesupport";
    return result;
  }
  auto introspection = load_introspection(topic.type);
  if (!introspection.ok()) {
    result.error = "introspection load failed for '" + topic.type + "' (tried " +
                   introspection.library_name + "): " + introspection.error;
    return result;
  }
  result.decoder = std::make_unique<IntrospectionDecoder>(introspection);
  return result;
}

}  // namespace bagwiz::core::decoder
