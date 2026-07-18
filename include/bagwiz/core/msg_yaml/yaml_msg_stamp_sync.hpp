// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__MSG_YAML__YAML_MSG_STAMP_SYNC_HPP_
#define BAGWIZ__CORE__MSG_YAML__YAML_MSG_STAMP_SYNC_HPP_

#include "bagwiz/core/msg_schema/schema_model.hpp"

#include <cstdint>
#include <string>
#include <utility>

namespace YAML
{
class Node;
}  // namespace YAML

namespace bagwiz::core
{

struct SyncMsgStampResult
{
  bool ok = false;
  std::string error;

  static SyncMsgStampResult success()
  {
    SyncMsgStampResult r;
    r.ok = true;
    return r;
  }

  static SyncMsgStampResult fail(std::string msg)
  {
    SyncMsgStampResult r;
    r.ok = false;
    r.error = std::move(msg);
    return r;
  }
};

// Syncs top-level `header.stamp` in `root` to `stamp_ns` (receive-time
// nanoseconds since Unix epoch). This function validates that the root
// schema has a top-level `header` field with `stamp.sec` / `stamp.nanosec`
// shape and returns an error when the shape is missing or incompatible.
SyncMsgStampResult sync_top_level_header_stamp_to_time(
  const msg_schema::SchemaModel & schema, YAML::Node & root, std::int64_t stamp_ns);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__MSG_YAML__YAML_MSG_STAMP_SYNC_HPP_
