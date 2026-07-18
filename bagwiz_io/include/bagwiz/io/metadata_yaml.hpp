// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__IO__METADATA_YAML_HPP_
#define BAGWIZ__IO__METADATA_YAML_HPP_

#include "bagwiz/io/bag_io.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace bagwiz::io
{

// Minimal view of a rosbag2 metadata.yaml file, carrying just the fields
// bagwiz needs to open a directory bag without re-scanning.
//
// The summary fields (`has_summary` and below) let `compute_stats()` answer
// from metadata alone without touching the underlying shards. They are only
// trusted as a unit: if any required field is missing, `has_summary` stays
// false and callers fall back to scanning shards.
struct BagMetadata
{
  std::string storage_identifier;                          // "mcap" or "sqlite3"
  std::vector<std::filesystem::path> relative_file_paths;  // in play order
  std::vector<TopicInfo> topics;                           // may be empty

  // True when message_count + starting_time + duration are all present.
  bool has_summary = false;
  int64_t total_messages = 0;
  int64_t start_ns = 0;
  int64_t end_ns = 0;
  // Per-topic counts keyed by topic name. Populated alongside `topics`
  // from `topics_with_message_count`. May be empty even when has_summary
  // is true (older writers).
  std::unordered_map<std::string, int64_t> per_topic_counts;

  // rosbag2-layer compression metadata (separate from any storage-native
  // compression like mcap chunk compression). Empty string == not present
  // in the YAML, which the reader treats as equivalent to "NONE".
  std::string compression_mode;    // "" / "NONE" / "FILE" / "MESSAGE"
  std::string compression_format;  // "" / "zstd" / ...
};

// Parse `<dir>/metadata.yaml`. Throws on IO or schema errors.
BagMetadata load_metadata_yaml(const std::filesystem::path & yaml_path);

}  // namespace bagwiz::io

#endif  // BAGWIZ__IO__METADATA_YAML_HPP_
