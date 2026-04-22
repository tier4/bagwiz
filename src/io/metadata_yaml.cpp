// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagcli/io/metadata_yaml.hpp"

#include "bagcli/core/logging.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>
#include <utility>

namespace bagcli::io
{

namespace
{
constexpr const char * kLogger = "bagcli.io.metadata";
}

BagMetadata load_metadata_yaml(const std::filesystem::path & yaml_path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path.string());
  } catch (const YAML::Exception & e) {
    BAGCLI_LOG_ERROR(kLogger, "Failed to parse %s: %s", yaml_path.c_str(), e.what());
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

  // Newer rosbag2 versions emit `files:` with per-shard metadata; older
  // versions (and simple single-shard bags) emit `relative_file_paths:`.
  // Prefer `files:` when present since it preserves order explicitly.
  if (auto files = info["files"]; files && files.IsSequence() && files.size() > 0) {
    for (const auto & f : files) {
      if (auto p = f["path"]; p) {
        md.relative_file_paths.emplace_back(p.as<std::string>());
      }
    }
  } else if (auto paths = info["relative_file_paths"]; paths && paths.IsSequence()) {
    for (const auto & p : paths) {
      md.relative_file_paths.emplace_back(p.as<std::string>());
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
      md.topics.push_back(std::move(topic));
    }
  }

  return md;
}

}  // namespace bagcli::io
