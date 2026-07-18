// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/metadata_yaml.hpp"

#include "bagwiz/core/base/logging.hpp"

#include <yaml-cpp/yaml.h>

#include <cctype>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace bagwiz::io
{

namespace
{
constexpr const char * kLogger = "bagwiz.io.metadata";

// Read a scalar i64 field if present; otherwise return nullopt. We don't
// throw on missing fields because older bags may legitimately omit them;
// the caller decides whether the absence is fatal.
std::optional<int64_t> read_i64(const YAML::Node & parent, const char * key)
{
  if (auto n = parent[key]; n && n.IsScalar()) {
    try {
      return n.as<int64_t>();
    } catch (const YAML::Exception &) {
      return std::nullopt;
    }
  }
  return std::nullopt;
}

std::string to_lower_copy(std::string s)
{
  for (auto & c : s) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return s;
}

}  // namespace

BagMetadata load_metadata_yaml(const std::filesystem::path & yaml_path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path.string());
  } catch (const YAML::Exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to parse %s: %s", yaml_path.c_str(), e.what());
    throw std::runtime_error("failed to parse metadata.yaml: " + std::string(e.what()));
  }

  auto info = root["rosbag2_bagfile_information"];
  if (!info) {
    throw std::runtime_error(
      "metadata.yaml missing `rosbag2_bagfile_information`: " + yaml_path.string());
  }

  BagMetadata md;

  if (auto node = info["storage_identifier"]; node) {
    md.storage_identifier = node.as<std::string>();
  } else {
    throw std::runtime_error("metadata.yaml missing storage_identifier");
  }

  // Compression metadata must be read before choosing the shard-path list:
  // FILE-mode bags list the decompressed logical name in `files[].path`
  // (e.g. `shard_0.db3`) but the *on-disk* name in `relative_file_paths`
  // (e.g. `shard_0.db3.zstd`). For those we must use `relative_file_paths`
  // so the reader opens the file that actually exists.
  if (auto cm = info["compression_mode"]; cm && cm.IsScalar()) {
    md.compression_mode = cm.as<std::string>("");
  }
  if (auto cf = info["compression_format"]; cf && cf.IsScalar()) {
    md.compression_format = cf.as<std::string>("");
  }
  const bool file_compressed = to_lower_copy(md.compression_mode) == "file";

  // Newer rosbag2 versions emit `files:` with per-shard metadata; older
  // versions (and simple single-shard bags) emit `relative_file_paths:`.
  // Prefer `files:` when present since it preserves order explicitly — except
  // for FILE-compressed bags, where only `relative_file_paths` carries the
  // real `.zstd` on-disk names.
  auto files = info["files"];
  if (!file_compressed && files && files.IsSequence() && files.size() > 0) {
    for (const auto & f : files) {
      if (auto p = f["path"]; p) {
        md.relative_file_paths.emplace_back(p.as<std::string>());
      }
    }
  } else if (auto paths = info["relative_file_paths"]; paths && paths.IsSequence()) {
    for (const auto & p : paths) {
      md.relative_file_paths.emplace_back(p.as<std::string>());
    }
  } else if (files && files.IsSequence() && files.size() > 0) {
    // FILE-compressed bag with no `relative_file_paths` — fall back to the
    // logical names; the reader will surface a clear open error if they do
    // not exist on disk.
    for (const auto & f : files) {
      if (auto p = f["path"]; p) {
        md.relative_file_paths.emplace_back(p.as<std::string>());
      }
    }
  }

  if (md.relative_file_paths.empty()) {
    throw std::runtime_error("metadata.yaml has no files listed");
  }

  if (auto topics = info["topics_with_message_count"]; topics && topics.IsSequence()) {
    for (const auto & t : topics) {
      auto tmeta = t["topic_metadata"];
      if (!tmeta) {
        continue;
      }
      TopicInfo topic;
      topic.name = tmeta["name"].as<std::string>("");
      topic.type = tmeta["type"].as<std::string>("");
      topic.serialization_format = tmeta["serialization_format"].as<std::string>("cdr");
      if (auto qos = tmeta["offered_qos_profiles"]; qos) {
        topic.offered_qos_profiles = qos.as<std::string>("");
      }
      if (auto count = read_i64(t, "message_count")) {
        md.per_topic_counts[topic.name] = *count;
      }
      md.topics.push_back(std::move(topic));
    }
  }

  // Summary is the trio of total count, start time and duration. All three
  // are written together by both bagwiz and rosbag2; treat them as a unit
  // so callers can rely on `has_summary` without re-validating each field.
  const auto total = read_i64(info, "message_count");
  std::optional<int64_t> start;
  std::optional<int64_t> duration;
  if (auto st = info["starting_time"]; st && st.IsMap()) {
    start = read_i64(st, "nanoseconds_since_epoch");
  }
  if (auto dur = info["duration"]; dur && dur.IsMap()) {
    duration = read_i64(dur, "nanoseconds");
  }
  if (total && start && duration) {
    md.has_summary = true;
    md.total_messages = *total;
    md.start_ns = *start;
    md.end_ns = *start + *duration;
  }

  return md;
}

}  // namespace bagwiz::io
