// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__MSG_YAML__ROS2_YAML_TO_CDR_HPP_
#define BAGWIZ__CORE__MSG_YAML__ROS2_YAML_TO_CDR_HPP_

#include "bagwiz/core/msg_schema/schema_model.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace YAML
{
class Node;
}  // namespace YAML

namespace bagwiz::core
{

// Validates `root_map` against `schema_for_validate` (must be the resolved
// .msg-derived model for `ros2_type_name`), assigns each field via the
// introspection runtime layout for that type, and serialises using the
// active RMW implementation.
struct Ros2YamlToCdrResult
{
  bool ok = false;
  std::vector<std::byte> cdr;
  std::string error;
};

Ros2YamlToCdrResult ros2_yaml_to_cdr_bytes(
  std::string_view ros2_type_name, const msg_schema::SchemaModel & schema_for_validate,
  const YAML::Node & root_map);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__MSG_YAML__ROS2_YAML_TO_CDR_HPP_
