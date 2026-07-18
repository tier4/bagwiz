// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__MSG_YAML__YAML_MSG_VALIDATE_HPP_
#define BAGWIZ__CORE__MSG_YAML__YAML_MSG_VALIDATE_HPP_

#include "bagwiz/core/msg_schema/schema_model.hpp"

#include <string>
#include <utility>

namespace YAML
{
class Node;
}  // namespace YAML

namespace bagwiz::core
{

// Validates that `root` matches the ROS 2 message shape emitted by ROS 2
// stack YAML tools (nested maps, primitives, sequences mirroring `.msg`).
// Requires the root YAML node be a mapping whose keys correspond to the
// non-constant wire fields on `schema.root()`.
struct YamlMsgValidateResult
{
  bool ok = false;
  std::string error;

  static YamlMsgValidateResult success()
  {
    YamlMsgValidateResult r;
    r.ok = true;
    return r;
  }

  static YamlMsgValidateResult fail(std::string msg)
  {
    YamlMsgValidateResult r;
    r.ok = false;
    r.error = std::move(msg);
    return r;
  }
};

YamlMsgValidateResult validate_ros2_yaml_for_message_schema(
  const msg_schema::SchemaModel & schema, const YAML::Node & root);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__MSG_YAML__YAML_MSG_VALIDATE_HPP_
