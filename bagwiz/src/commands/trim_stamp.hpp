// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__TRIM_STAMP_HPP_
#define COMMANDS__TRIM_STAMP_HPP_

#include "bagwiz/io/bag_io.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

// header.stamp helpers for `bagwiz trim --stamp header` (CLI-internal, lives
// next to trim.cpp; tests add src/commands to their include path — the
// traj_common idiom).
namespace bagwiz::commands
{

// True when the message definition's top-level block declares a leading
// std_msgs/Header field — the ROS 2 convention for stamped messages — i.e.
// the serialized CDR payload starts with header.stamp right after the 4-byte
// encapsulation. `schema_text` is a .msg body, optionally with concatenated
// dependency blocks after '===' separators (only the first block is read).
[[nodiscard]] bool schema_leads_with_header(std::string_view schema_text);

// Read the leading std_msgs/Header stamp (sec * 1e9 + nanosec) of a
// CDR-encapsulated ROS 2 message. The caller must have verified the type
// leads with a Header (schema_leads_with_header); the bytes are not validated
// beyond length. Returns nullopt when the payload is too short to hold the
// encapsulation plus the stamp.
[[nodiscard]] std::optional<std::int64_t> read_leading_header_stamp_ns(
  std::span<const std::byte> payload);

// Result of classifying a bag's topics by leading-Header presence.
struct HeaderedTopics
{
  // Topic names whose type leads with std_msgs/Header.
  std::unordered_set<std::string> topics;
  // Types that could not be classified (no embedded schema and no resolvable
  // .msg on $AMENT_PREFIX_PATH), deduplicated. Their topics are treated as
  // headerless (receive-time clock).
  std::vector<std::string> unresolved_types;
};

// Classify `topics` from their embedded schema text, falling back to
// core::resolve_message_definition for types without one (SQLite3 bags do not
// embed definitions). Callers should run BagReader::populate_schemas() first
// so MCAP shard readers carry their schema text.
[[nodiscard]] HeaderedTopics classify_headered_topics(std::span<const io::TopicInfo> topics);

}  // namespace bagwiz::commands

#endif  // COMMANDS__TRIM_STAMP_HPP_
