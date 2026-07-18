// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__INTROSPECTION__INTROSPECTION_LOADER_HPP_
#define BAGWIZ__CORE__INTROSPECTION__INTROSPECTION_LOADER_HPP_

#include <string>
#include <string_view>

struct rosidl_message_type_support_t;

namespace rosidl_typesupport_introspection_cpp
{
struct MessageMembers_s;
using MessageMembers = MessageMembers_s;
}  // namespace rosidl_typesupport_introspection_cpp

namespace bagwiz::core
{

// Outcome of resolving a ROS 2 message type for bagwiz's use.
//
// We load only the introspection typesupport for the package and use it
// for both purposes:
//   * `typesupport` is passed to rmw_deserialize. Every supported RMW
//     (cyclonedds, fastrtps) accepts an introspection typesupport handle
//     directly, so we skip the `rosidl_typesupport_cpp` router — that
//     router would otherwise try to dlopen another per-RMW library,
//     which the per-package cpp typesupport on Humble is not linked
//     against and which fails on minimal installations.
//   * `members` is the walker view used to emit YAML.
struct IntrospectionLoad
{
  const rosidl_message_type_support_t * typesupport = nullptr;
  const rosidl_typesupport_introspection_cpp::MessageMembers * members = nullptr;

  std::string library_name;  // the .so we attempted to load
  std::string error;         // empty on success

  bool ok() const { return typesupport != nullptr && members != nullptr; }
};

// Resolve the introspection typesupport for a type name like
// "sensor_msgs/msg/Image" (preferred) or legacy "sensor_msgs/Image".
//
// Intentionally leaks the dlopen handle: CLI runs are short-lived and
// the returned pointers must outlive all use of the decoded messages.
IntrospectionLoad load_introspection(std::string_view type_name);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__INTROSPECTION__INTROSPECTION_LOADER_HPP_
