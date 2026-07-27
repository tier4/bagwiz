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
#include <optional>
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

// Inputs for emitting a rosbag2-compatible metadata.yaml for a single-shard
// directory bag. The single emitter below is shared by every producer (the
// mcap and sqlite3 directory writers, and the chunk pass-through rewrite)
// so the YAML schema cannot drift between them.
struct MetadataYamlInfo
{
  std::string storage_identifier;  // "mcap" or "sqlite3"
  std::vector<TopicInfo> topics;   // declaration order
  // Per-topic counts keyed by topic name; topics absent from the map are
  // written with a count of 0.
  std::unordered_map<std::string, int64_t> per_topic_counts;
  int64_t total_messages = 0;
  int64_t start_ns = 0;  // earliest message time; ignored when total_messages == 0
  int64_t end_ns = 0;    // latest message time; ignored when total_messages == 0
  // Storage-native compression, written verbatim to `compression_format`.
  // `compression_mode` derives from it: "" when empty or "none", else "file"
  // (mcap chunk compression is declared as FILE-mode; see the mcap directory
  // writer for the compatibility rationale).
  std::string compression_format;
  std::string shard_relative_path;  // the single shard, relative to the dir
};

// Emit `<dir>/metadata.yaml` (schema version 5, single shard). Throws on IO
// errors.
void write_metadata_yaml(const std::filesystem::path & dir, const MetadataYamlInfo & info);

// Emit the same summary as `write_metadata_yaml` but as the bare inner mapping
// (no `rosbag2_bagfile_information:` wrapper), which is how rosbag2 stores it
// in a single-file .db3's `metadata` table.
//
// Both emitters share one body writer on purpose. rosbag2 dispatches its YAML
// decode on the declared `version`, so a document whose structure does not
// match the version it claims is not merely wrong — jazzy's storage plugin
// then fails to open the bag at all ("No plugin detected that could open
// file"). Never hand-roll this YAML at a call site.
std::string emit_metadata_yaml_body(const MetadataYamlInfo & info);

// Parse the inner mapping produced by `emit_metadata_yaml_body` (or written by
// rosbag2 iron+ into a .db3 `metadata` row).
//
// Returns nullopt rather than throwing when the document is unusable, so
// callers fall back to scanning instead of failing the read:
//   - unparseable YAML, or a document that is not a map
//   - the open-time template row rosbag2 writes before recording starts
//     (message_count 0 with starting_time == INT64_MAX)
//   - a document missing any of the message_count / starting_time / duration
//     trio, in which case `has_summary` could not be set anyway
//
// `relative_file_paths` is not required here: the caller already holds the
// file it read this from.
std::optional<BagMetadata> parse_metadata_yaml_body(const std::string & yaml);

}  // namespace bagwiz::io

#endif  // BAGWIZ__IO__METADATA_YAML_HPP_
