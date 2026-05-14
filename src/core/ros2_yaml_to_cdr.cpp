// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/ros2_yaml_to_cdr.hpp"

#include "bagwiz/core/introspection_loader.hpp"
#include "bagwiz/core/yaml_msg_validate.hpp"

#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <rcutils/allocator.h>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::core
{

namespace
{

namespace ts_types = rosidl_typesupport_introspection_cpp;

struct AlignedAllocFreer
{
  void operator()(unsigned char * p) const noexcept { std::free(p); }
};

using AlignedBuffer = std::unique_ptr<unsigned char, AlignedAllocFreer>;

AlignedBuffer aligned_alloc_or_throw(std::size_t alignment, std::size_t size)
{
  void * p = nullptr;
  const std::size_t bytes = size == 0 ? 1 : size;
  if (::posix_memalign(&p, alignment, bytes) != 0 || p == nullptr) {
    throw std::bad_alloc();
  }
  return AlignedBuffer{static_cast<unsigned char *>(p)};
}

class MessageInitGuard
{
public:
  MessageInitGuard(const ts_types::MessageMembers * members, void * buffer)
  : members_(members), buffer_(buffer)
  {
    members_->init_function(buffer_, rosidl_runtime_cpp::MessageInitialization::ALL);
  }
  ~MessageInitGuard()
  {
    if (members_ != nullptr) {
      members_->fini_function(buffer_);
    }
  }

  MessageInitGuard(const MessageInitGuard &) = delete;
  MessageInitGuard & operator=(const MessageInitGuard &) = delete;
  MessageInitGuard(MessageInitGuard &&) = delete;
  MessageInitGuard & operator=(MessageInitGuard &&) = delete;
  void release() noexcept { members_ = nullptr; }

private:
  const ts_types::MessageMembers * members_ = nullptr;
  void * buffer_ = nullptr;
};

class SerializedMessageRmw
{
public:
  explicit SerializedMessageRmw(std::size_t capacity)
  {
    rcutils_allocator_t alloc = rcutils_get_default_allocator();
    if (rmw_serialized_message_init(&msg_, capacity, &alloc) != RMW_RET_OK) {
      throw std::runtime_error("rmw_serialized_message_init failed");
    }
  }
  ~SerializedMessageRmw() { rmw_serialized_message_fini(&msg_); }

  SerializedMessageRmw(const SerializedMessageRmw &) = delete;
  SerializedMessageRmw & operator=(const SerializedMessageRmw &) = delete;
  SerializedMessageRmw(SerializedMessageRmw &&) = delete;
  SerializedMessageRmw & operator=(SerializedMessageRmw &&) = delete;

  rmw_serialized_message_t & get() noexcept { return msg_; }

private:
  rmw_serialized_message_t msg_ = rmw_get_zero_initialized_serialized_message();
};

const ts_types::MessageMembers & nested_members_of(const ts_types::MessageMember & m)
{
  return *static_cast<const ts_types::MessageMembers *>(m.members_->data);
}

void primitive_temp_from_yaml_non_string(std::uint8_t type_id, const YAML::Node & n, void * dst)
{
  switch (type_id) {
    case ts_types::ROS_TYPE_BOOLEAN:
      *static_cast<bool *>(dst) = n.as<bool>();
      return;
    case ts_types::ROS_TYPE_OCTET:
    case ts_types::ROS_TYPE_UINT8:
      *static_cast<std::uint8_t *>(dst) = static_cast<std::uint8_t>(n.as<std::uint64_t>() & 0xFF);
      return;
    case ts_types::ROS_TYPE_CHAR:
    case ts_types::ROS_TYPE_INT8:
      *static_cast<std::int8_t *>(dst) = static_cast<std::int8_t>(n.as<std::int64_t>());
      return;
    case ts_types::ROS_TYPE_UINT16:
      *static_cast<std::uint16_t *>(dst) = static_cast<std::uint16_t>(n.as<std::uint64_t>());
      return;
    case ts_types::ROS_TYPE_INT16:
      *static_cast<std::int16_t *>(dst) = static_cast<std::int16_t>(n.as<std::int64_t>());
      return;
    case ts_types::ROS_TYPE_UINT32:
      *static_cast<std::uint32_t *>(dst) = static_cast<std::uint32_t>(n.as<std::uint64_t>());
      return;
    case ts_types::ROS_TYPE_INT32:
      *static_cast<std::int32_t *>(dst) = static_cast<std::int32_t>(n.as<std::int64_t>());
      return;
    case ts_types::ROS_TYPE_UINT64:
      *static_cast<std::uint64_t *>(dst) = n.as<std::uint64_t>();
      return;
    case ts_types::ROS_TYPE_INT64:
      *static_cast<std::int64_t *>(dst) = n.as<std::int64_t>();
      return;
    case ts_types::ROS_TYPE_FLOAT:
      *static_cast<float *>(dst) = n.as<float>();
      return;
    case ts_types::ROS_TYPE_DOUBLE:
      *static_cast<double *>(dst) = n.as<double>();
      return;
    default:
      throw std::runtime_error(
        "unsupported primitive type_id " + std::to_string(static_cast<int>(type_id)));
  }
}

void fill_message_members_for_yaml(
  const ts_types::MessageMembers & members, void * msg_base, const YAML::Node & yaml_map);

void fill_primitive_field_scalar(
  const ts_types::MessageMember & m, void * field_ptr, const YAML::Node & n)
{
  switch (m.type_id_) {
    case ts_types::ROS_TYPE_BOOLEAN:
      *static_cast<bool *>(field_ptr) = n.as<bool>();
      return;
    case ts_types::ROS_TYPE_OCTET:
    case ts_types::ROS_TYPE_UINT8:
      *static_cast<std::uint8_t *>(field_ptr) =
        static_cast<std::uint8_t>(n.as<std::uint64_t>() & 0xFF);
      return;
    case ts_types::ROS_TYPE_CHAR:
    case ts_types::ROS_TYPE_INT8:
      *static_cast<std::int8_t *>(field_ptr) = static_cast<std::int8_t>(n.as<std::int64_t>());
      return;
    case ts_types::ROS_TYPE_UINT16:
      *static_cast<std::uint16_t *>(field_ptr) = static_cast<std::uint16_t>(n.as<std::uint64_t>());
      return;
    case ts_types::ROS_TYPE_INT16:
      *static_cast<std::int16_t *>(field_ptr) = static_cast<std::int16_t>(n.as<std::int64_t>());
      return;
    case ts_types::ROS_TYPE_UINT32:
      *static_cast<std::uint32_t *>(field_ptr) = static_cast<std::uint32_t>(n.as<std::uint64_t>());
      return;
    case ts_types::ROS_TYPE_INT32:
      *static_cast<std::int32_t *>(field_ptr) = static_cast<std::int32_t>(n.as<std::int64_t>());
      return;
    case ts_types::ROS_TYPE_UINT64:
      *static_cast<std::uint64_t *>(field_ptr) = n.as<std::uint64_t>();
      return;
    case ts_types::ROS_TYPE_INT64:
      *static_cast<std::int64_t *>(field_ptr) = n.as<std::int64_t>();
      return;
    case ts_types::ROS_TYPE_FLOAT:
      *static_cast<float *>(field_ptr) = n.as<float>();
      return;
    case ts_types::ROS_TYPE_DOUBLE:
      *static_cast<double *>(field_ptr) = n.as<double>();
      return;
    case ts_types::ROS_TYPE_STRING:
      *static_cast<std::string *>(field_ptr) = n.as<std::string>();
      return;
    case ts_types::ROS_TYPE_WSTRING:
      throw std::runtime_error("wstring is not supported for YAML ingestion");
    case ts_types::ROS_TYPE_LONG_DOUBLE:
      throw std::runtime_error("float128 / long double is not supported for YAML ingestion");
    case ts_types::ROS_TYPE_WCHAR:
      throw std::runtime_error("wchar is not supported for YAML ingestion");
    default:
      throw std::runtime_error(
        "unknown primitive type id " + std::to_string(static_cast<int>(m.type_id_)));
  }
}

void resize_sequence_if_needed(
  const ts_types::MessageMember & m, void * field_ptr, std::size_t new_len)
{
  if (m.is_upper_bound_ && new_len > m.array_size_) {
    throw std::runtime_error(std::string("sequence size exceeds bound for field ") + m.name_);
  }
  if (!m.is_array_) {
    return;
  }
  if (m.resize_function != nullptr) {
    m.resize_function(field_ptr, new_len);
    return;
  }
  if (new_len != m.array_size_) {
    throw std::runtime_error(
      std::string("YAML sequence length does not match fixed array size on field ") + m.name_);
  }
}

void fill_field(const ts_types::MessageMember & m, void * msg_base, const YAML::Node & yaml_field)
{
  void * field_ptr = static_cast<std::uint8_t *>(msg_base) + m.offset_;

  if (!m.is_array_) {
    if (m.type_id_ == ts_types::ROS_TYPE_MESSAGE) {
      fill_message_members_for_yaml(nested_members_of(m), field_ptr, yaml_field);
      return;
    }
    fill_primitive_field_scalar(m, field_ptr, yaml_field);
    return;
  }

  const std::size_t seq_len = yaml_field.size();
  resize_sequence_if_needed(m, field_ptr, seq_len);

  if (m.type_id_ != ts_types::ROS_TYPE_MESSAGE) {
    alignas(std::max_align_t) std::byte
      scratch[sizeof(std::uint64_t) > sizeof(double) ? sizeof(std::uint64_t) : sizeof(double)];
    for (std::size_t i = 0; i < seq_len; ++i) {
      if (m.type_id_ == ts_types::ROS_TYPE_STRING) {
        auto tmp = yaml_field[i].as<std::string>();
        m.assign_function(field_ptr, i, &tmp);
      } else {
        primitive_temp_from_yaml_non_string(m.type_id_, yaml_field[i], scratch);
        m.assign_function(field_ptr, i, scratch);
      }
    }
    return;
  }

  const auto & sub = nested_members_of(m);
  for (std::size_t i = 0; i < seq_len; ++i) {
    void * elem = m.get_function(field_ptr, i);
    fill_message_members_for_yaml(sub, elem, yaml_field[i]);
  }
}

void fill_message_members_for_yaml(
  const ts_types::MessageMembers & members, void * msg_base, const YAML::Node & yaml_map)
{
  if (!yaml_map.IsMap()) {
    throw std::runtime_error("expected YAML mapping for ROS message fields");
  }
  for (std::uint32_t i = 0; i < members.member_count_; ++i) {
    const ts_types::MessageMember & m = members.members_[i];
    const YAML::Node child = yaml_map[m.name_];
    if (!child.IsDefined()) {
      throw std::runtime_error(std::string("YAML missing required field '") + m.name_ + "'");
    }
    fill_field(m, msg_base, child);
  }
}

}  // namespace

Ros2YamlToCdrResult ros2_yaml_to_cdr_bytes(
  std::string_view ros2_type_name, const msg_schema::SchemaModel & schema_for_validate,
  const YAML::Node & root_map)
{
  Ros2YamlToCdrResult out;
  const auto vr = validate_ros2_yaml_for_message_schema(schema_for_validate, root_map);
  if (!vr.ok) {
    out.ok = false;
    out.error = vr.error;
    return out;
  }

  auto intro = load_introspection(ros2_type_name);
  if (!intro.ok()) {
    out.error = intro.error;
    return out;
  }

  const auto * members = intro.members;

  auto buffer_storage = aligned_alloc_or_throw(alignof(std::max_align_t), members->size_of_);
  void * const native = static_cast<void *>(buffer_storage.get());
  MessageInitGuard guard(members, native);

  try {
    fill_message_members_for_yaml(*members, native, root_map);
  } catch (const std::exception & e) {
    out.error = e.what();
    return out;
  }

  SerializedMessageRmw serialized(0);
  const rmw_ret_t rc = rmw_serialize(native, intro.typesupport, &serialized.get());
  if (rc != RMW_RET_OK) {
    const rcutils_error_state_t * s = rcutils_get_error_state();
    out.error = "rmw_serialize failed: ";
    out.error += s != nullptr ? s->message : "(no error message)";
    rcutils_reset_error();
    return out;
  }

  const auto * sm = &serialized.get();
  out.cdr.resize(sm->buffer_length);
  if (sm->buffer_length > 0 && sm->buffer != nullptr) {
    std::memcpy(out.cdr.data(), sm->buffer, sm->buffer_length);
  }
  guard.release();
  members->fini_function(native);
  out.ok = true;
  buffer_storage.reset();
  return out;
}

}  // namespace bagwiz::core
