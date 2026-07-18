// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/introspection/introspection_loader.hpp"

#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <dlfcn.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::core
{

namespace
{

// Split "pkg/msg/Type" (or legacy "pkg/Type") into its three components.
bool split_type_name(
  std::string_view full, std::string & package, std::string & sub, std::string & type)
{
  std::vector<std::string_view> parts;
  std::size_t start = 0;
  for (std::size_t i = 0; i < full.size(); ++i) {
    if (full[i] == '/') {
      parts.emplace_back(full.substr(start, i - start));
      start = i + 1;
    }
  }
  parts.emplace_back(full.substr(start));
  if (parts.size() == 2) {
    package = std::string(parts[0]);
    sub = "msg";
    type = std::string(parts[1]);
    return !package.empty() && !type.empty();
  }
  if (parts.size() == 3) {
    package = std::string(parts[0]);
    sub = std::string(parts[1]);
    type = std::string(parts[2]);
    return !package.empty() && !sub.empty() && !type.empty();
  }
  return false;
}

using GetTypeSupportFn = const rosidl_message_type_support_t * (*)();

}  // namespace

IntrospectionLoad load_introspection(std::string_view type_name)
{
  IntrospectionLoad result;

  std::string package;
  std::string sub;
  std::string type;
  if (!split_type_name(type_name, package, sub, type)) {
    result.error = "type name must look like 'pkg/msg/Type' (got '" + std::string(type_name) + "')";
    return result;
  }

  // We intentionally load only the introspection typesupport. rmw_deserialize
  // in both rmw_cyclonedds_cpp and rmw_fastrtps_cpp (via the introspection
  // path) accepts this handle directly; the rosidl_typesupport_cpp wrapper
  // would otherwise insist on dlopen'ing an RMW-specific library that the
  // per-package cpp typesupport on Humble is not linked against.
  result.library_name = "lib" + package + "__rosidl_typesupport_introspection_cpp.so";
  const std::string symbol =
    "rosidl_typesupport_introspection_cpp__get_message_type_support_handle__" + package + "__" +
    sub + "__" + type;

  // RTLD_GLOBAL so the symbols from this typesupport are visible to any
  // later-loaded RMW library when it walks the handle graph.
  void * handle = ::dlopen(result.library_name.c_str(), RTLD_LAZY | RTLD_GLOBAL);
  if (handle == nullptr) {
    const char * err = ::dlerror();
    result.error = err != nullptr ? err : "dlopen failed";
    return result;
  }

  ::dlerror();  // clear stale error state
  auto * raw = ::dlsym(handle, symbol.c_str());
  const char * sym_err = ::dlerror();
  if (sym_err != nullptr || raw == nullptr) {
    result.error = sym_err != nullptr ? sym_err : "symbol not found";
    return result;
  }

  auto fn = reinterpret_cast<GetTypeSupportFn>(raw);
  const rosidl_message_type_support_t * ts = fn();
  if (ts == nullptr || ts->data == nullptr) {
    result.error = "typesupport function returned null";
    return result;
  }

  result.typesupport = ts;
  result.members =
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(ts->data);
  // handle is intentionally leaked; see header note.
  return result;
}

}  // namespace bagwiz::core
