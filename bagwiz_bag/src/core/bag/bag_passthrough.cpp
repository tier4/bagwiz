// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/bag/bag_passthrough.hpp"

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/mcap_passthrough.hpp"
#include "bagwiz/io/metadata_yaml.hpp"

#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <system_error>

namespace bagwiz::core
{

namespace
{

bool passthrough_disabled_by_env()
{
  const char * raw = std::getenv("BAGWIZ_PASSTHROUGH");
  if (raw == nullptr) {
    return false;
  }
  std::string value(raw);
  for (auto & c : value) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return value == "off" || value == "0" || value == "false" || value == "no";
}

std::string to_lower_copy(std::string s)
{
  for (auto & c : s) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return s;
}

}  // namespace

std::optional<PassthroughCounts> try_bag_passthrough_rewrite(
  const std::filesystem::path & input_path, const RewriteTarget & target,
  const PassthroughEdit & edit, const char * logger)
{
  if (passthrough_disabled_by_env()) {
    BAGWIZ_LOG_DEBUG(logger, "chunk pass-through disabled via BAGWIZ_PASSTHROUGH");
    return std::nullopt;
  }
  // Splitting re-chunks by definition; the rewrite commands never request it,
  // so this is a defensive gate for future callers.
  if (
    target.create_options.split_bytes.has_value() ||
    target.create_options.split_duration_ns.has_value()) {
    return std::nullopt;
  }
  if (io::detect_format(input_path) != io::Format::Mcap) {
    return std::nullopt;
  }
  const auto resolved = io::resolve_write_layout(target.path, target.create_options);
  if (resolved.format != io::Format::Mcap) {
    return std::nullopt;
  }

  // Resolve the single input shard. Directory bags must carry exactly one
  // shard: channel/schema ids are per-file, so a verbatim multi-shard merge
  // would collide them.
  std::filesystem::path input_file = input_path;
  std::error_code ec;
  if (std::filesystem::is_directory(input_path, ec)) {
    io::BagMetadata metadata;
    try {
      metadata = io::load_metadata_yaml(input_path / "metadata.yaml");
    } catch (const std::exception & e) {
      BAGWIZ_LOG_DEBUG(logger, "chunk pass-through declined: %s", e.what());
      return std::nullopt;
    }
    if (metadata.relative_file_paths.size() != 1) {
      BAGWIZ_LOG_DEBUG(
        logger, "chunk pass-through declined: input has %zu shards",
        metadata.relative_file_paths.size());
      return std::nullopt;
    }
    // MESSAGE-mode bags carry per-message zstd envelopes that the decoded
    // pipeline strips on read; a verbatim copy would keep them. FILE-mode
    // on mcap storage is bagwiz's own marker for chunk compression (the
    // shard is a plain readable mcap), which the engine handles natively.
    if (to_lower_copy(metadata.compression_mode) == "message") {
      BAGWIZ_LOG_DEBUG(logger, "chunk pass-through declined: MESSAGE-mode compression");
      return std::nullopt;
    }
    input_file = input_path / metadata.relative_file_paths.front();
  }

  // Resolve the output file; directory outputs get the rosbag2 shard naming
  // McapDirectoryWriter uses, plus a metadata.yaml once the engine is done.
  const bool directory_output = resolved.layout == io::Layout::Directory;
  std::filesystem::path output_file = target.path;
  bool created_directory = false;
  if (directory_output) {
    created_directory = std::filesystem::create_directories(target.path, ec) && !ec;
    output_file = target.path / (target.path.filename().string() + "_0.mcap");
  }

  io::McapPassthroughEdit engine_edit;
  engine_edit.drop_topics = edit.suppress_topics;
  engine_edit.rename = edit.rename;
  engine_edit.start_ns = edit.start_ns;
  engine_edit.end_ns = edit.end_ns;

  std::string reason;
  const auto result = io::mcap_passthrough_rewrite(input_file, output_file, engine_edit, &reason);
  if (!result.has_value()) {
    BAGWIZ_LOG_DEBUG(logger, "chunk pass-through declined: %s", reason.c_str());
    // Leave the target as the fallback expects it: the decoded pipeline's
    // writer factory recreates the directory itself.
    if (created_directory) {
      std::filesystem::remove_all(target.path, ec);
    }
    return std::nullopt;
  }

  if (directory_output) {
    io::MetadataYamlInfo info;
    info.storage_identifier = "mcap";
    info.topics = result->topics;
    info.per_topic_counts = result->per_topic_counts;
    info.total_messages = static_cast<int64_t>(result->messages_written);
    info.start_ns = result->start_ns;
    info.end_ns = result->end_ns;
    info.compression_format = result->chunk_compression;
    info.shard_relative_path = output_file.filename().string();
    io::write_metadata_yaml(target.path, info);
  }

  // The decoded pipeline loses attachment/metadata records silently; the
  // pass-through matches its output but says so.
  if (result->attachments_skipped > 0 || result->metadata_skipped > 0) {
    BAGWIZ_LOG_WARN(
      logger,
      "input carries %lu attachment and %lu metadata record(s); rewrites do not preserve them.",
      static_cast<unsigned long>(result->attachments_skipped),  // NOLINT(runtime/int)
      static_cast<unsigned long>(result->metadata_skipped));    // NOLINT(runtime/int)
  }
  BAGWIZ_LOG_INFO(
    logger, "chunk pass-through: copied %lu chunk(s) verbatim, re-encoded %lu, dropped %lu",
    static_cast<unsigned long>(result->chunks_copied),     // NOLINT(runtime/int)
    static_cast<unsigned long>(result->chunks_reencoded),  // NOLINT(runtime/int)
    static_cast<unsigned long>(result->chunks_dropped));   // NOLINT(runtime/int)
  return PassthroughCounts{result->messages_written, result->messages_renamed};
}

}  // namespace bagwiz::core
