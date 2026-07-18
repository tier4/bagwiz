// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__INTROSPECTION__MESSAGE_DESERIALIZER_HPP_
#define BAGWIZ__CORE__INTROSPECTION__MESSAGE_DESERIALIZER_HPP_

#include <cstddef>
#include <span>

namespace rosidl_typesupport_introspection_cpp
{
struct MessageMembers_s;
using MessageMembers = MessageMembers_s;
}  // namespace rosidl_typesupport_introspection_cpp

namespace bagwiz::core
{

struct IntrospectionLoad;

// RAII owner of a single deserialized ROS 2 message.
//
// Construction sequence:
//   1. posix_memalign an aligned buffer of members.size_of_ bytes
//   2. members.init_function(buffer, ALL) -> default-construct all
//      C++ fields (std::string, std::vector, ...) so the CDR bytes
//      land on valid objects
//   3. rmw_deserialize(payload, typesupport, buffer) -> the active RMW
//      walks the CDR stream and populates the fields
//
// Destruction runs members.fini_function(buffer) then frees the memory.
// The constructor throws std::runtime_error on any failure (including a
// non-OK rmw_deserialize return); the error message includes the first
// few payload bytes for diagnosis.
class DeserializedMessage
{
public:
  DeserializedMessage(
    const IntrospectionLoad & introspection, std::span<const std::byte> cdr_payload);
  ~DeserializedMessage();

  DeserializedMessage(const DeserializedMessage &) = delete;
  DeserializedMessage & operator=(const DeserializedMessage &) = delete;
  DeserializedMessage(DeserializedMessage &&) = delete;
  DeserializedMessage & operator=(DeserializedMessage &&) = delete;

  const void * data() const { return buffer_; }
  const rosidl_typesupport_introspection_cpp::MessageMembers & members() const { return *members_; }

private:
  const rosidl_typesupport_introspection_cpp::MessageMembers * members_ = nullptr;
  void * buffer_ = nullptr;
  bool initialized_ = false;
};

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__INTROSPECTION__MESSAGE_DESERIALIZER_HPP_
