// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__MSG_YAML__MSG_DEFINITION_RESOLVER_HPP_
#define BAGWIZ__CORE__MSG_YAML__MSG_DEFINITION_RESOLVER_HPP_

#include <string>
#include <string_view>

namespace bagwiz::core
{

// Result of resolving a ROS 2 type name into the concatenated .msg
// representation that rosbag2 / mcap consumers expect.
struct ResolvedMessageDefinition
{
  // Concatenated .msg text following the rosbag2 mcap convention:
  //
  //   <top-level body>
  //   ================================================================================
  //   MSG: dep_pkg/dep_Type
  //   <dep body>
  //   ================================================================================
  //   MSG: dep_pkg2/dep_Type2
  //   <dep body>
  //   ...
  //
  // Empty string when resolution failed (some required .msg could not
  // be located on disk).
  std::string text;

  // "ros2msg" when text is non-empty; empty when resolution failed.
  // Callers writing MCAP Schema records should pair empty text with
  // empty encoding (per the MCAP convention for "no schema known").
  std::string encoding;
};

// Resolve a ROS 2 type name (e.g. "sensor_msgs/msg/Imu") to its
// canonical .msg text by reading
// `$AMENT_PREFIX_PATH/share/<pkg>/msg/<Type>.msg` and recursively
// expanding dependencies. Returns an empty struct when any required
// .msg file cannot be located.
//
// Result is cached per-process so repeated lookups for the same type
// (the common case during a bag conversion) are O(1).
//
// Thread-safe: an internal mutex guards the cache.
ResolvedMessageDefinition resolve_message_definition(std::string_view ros2_type);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__MSG_YAML__MSG_DEFINITION_RESOLVER_HPP_
