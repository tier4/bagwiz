// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/mcap_writer.hpp"

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <mcap/writer.hpp>

#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bagwiz::io::detail
{

namespace
{
constexpr const char * kLogger = "bagwiz.io.mcap";

mcap::Compression parse_compression(std::string_view name)
{
  if (name == "zstd") {
    return mcap::Compression::Zstd;
  }
  if (name == "lz4") {
    return mcap::Compression::Lz4;
  }
  if (name.empty() || name == "none") {
    return mcap::Compression::None;
  }
  throw std::runtime_error("unknown mcap compression: " + std::string(name));
}

// ---------------------------------------------------------------------------
// Single .mcap file writer.
// ---------------------------------------------------------------------------
class McapFileWriter : public BagWriter
{
public:
  McapFileWriter(const std::filesystem::path & path, const CreateOptions & options)
  {
    mcap::McapWriterOptions wopts("ros2");
    wopts.compression = parse_compression(options.mcap_compression);
    wopts.chunkSize = options.mcap_chunk_size;

    const auto status = writer_.open(path.string(), wopts);
    if (!status.ok()) {
      throw std::runtime_error(
        "mcap writer open failed for " + path.string() + ": " + status.message);
    }
  }

  ~McapFileWriter() override
  {
    if (!closed_) {
      try {
        writer_.close();
      } catch (...) {
        // Destructors must not throw; mcap::close itself can't throw either
        // but a user subclass or future revision might. Swallow silently.
      }
    }
  }

  McapFileWriter(const McapFileWriter &) = delete;
  McapFileWriter & operator=(const McapFileWriter &) = delete;
  McapFileWriter(McapFileWriter &&) = delete;
  McapFileWriter & operator=(McapFileWriter &&) = delete;

  void declare_topic(const TopicInfo & topic) override
  {
    mcap::SchemaId schema_id{};
    if (auto it = type_to_schema_.find(topic.type); it != type_to_schema_.end()) {
      schema_id = it->second;
    } else {
      // Embed the message definition when the caller provided one. The
      // encoding defaults to "ros2msg" because that is what every ROS 2
      // toolchain (rosbag2, foxglove, mcap_ros2) emits today; if a caller
      // ever passes raw IDL, they must set schema_encoding explicitly.
      //
      // When schema_text is empty (caller has no definition handy),
      // also emit an empty encoding. Pairing `encoding="ros2msg"` with
      // `data=""` is misleading: strict readers (e.g. rosbags-convert)
      // parse the empty payload as a zero-field type and conflict it
      // with their built-in `builtin_interfaces/msg/Time` definition,
      // surfacing as `TypesysError("...already present with different
      // definition.")`. An empty encoding is the MCAP convention for
      // "no schema known" — readers fall back to their default
      // typestore instead of treating it as a malformed schema.
      const bool has_text = !topic.schema_text.empty();
      const std::string encoding =
        has_text ? (topic.schema_encoding.empty() ? std::string("ros2msg") : topic.schema_encoding)
                 : std::string{};
      mcap::Schema schema(topic.type, encoding, topic.schema_text);
      writer_.addSchema(schema);
      schema_id = schema.id;
      type_to_schema_[topic.type] = schema_id;
    }

    mcap::Channel channel(topic.name, topic.serialization_format, schema_id);
    if (!topic.offered_qos_profiles.empty()) {
      channel.metadata["offered_qos_profiles"] = topic.offered_qos_profiles;
    }
    writer_.addChannel(channel);
    topic_to_channel_[topic.name] = channel.id;
  }

  void write(
    std::string_view topic, int64_t timestamp_ns, std::span<const std::byte> payload) override
  {
    auto it = topic_to_channel_.find(std::string(topic));
    if (it == topic_to_channel_.end()) {
      throw std::runtime_error(
        "mcap write on undeclared topic: " + std::string(topic) + " (call declare_topic() first)");
    }
    mcap::Message msg;
    msg.channelId = it->second;
    msg.sequence = 0;
    msg.logTime = static_cast<mcap::Timestamp>(timestamp_ns);
    msg.publishTime = msg.logTime;
    msg.data = reinterpret_cast<const std::byte *>(payload.data());
    msg.dataSize = payload.size();

    const auto status = writer_.write(msg);
    if (!status.ok()) {
      throw std::runtime_error("mcap write failed: " + status.message);
    }
  }

  void close() override
  {
    if (closed_) {
      return;
    }
    writer_.close();
    closed_ = true;
  }

private:
  mcap::McapWriter writer_;
  std::unordered_map<std::string, mcap::ChannelId> topic_to_channel_;
  std::unordered_map<std::string, mcap::SchemaId> type_to_schema_;
  bool closed_ = false;
};

// ---------------------------------------------------------------------------
// Directory writer: wraps a single McapFileWriter shard and emits a
// metadata.yaml compatible with rosbag2's expected schema on close().
// ---------------------------------------------------------------------------
class McapDirectoryWriter final : public BagWriter
{
public:
  McapDirectoryWriter(const std::filesystem::path & dir, const CreateOptions & options)
  : dir_(dir), options_(options)
  {
    std::filesystem::create_directories(dir);

    // rosbag2 uses "<dirname>_<index>.mcap" for shards; match that layout so
    // the output is interchangeable with `ros2 bag record`.
    const auto stem = dir.filename().string();
    shard_rel_ = stem + "_0.mcap";
    inner_ = std::make_unique<McapFileWriter>(dir_ / shard_rel_, options);
  }

  ~McapDirectoryWriter() override
  {
    if (!closed_) {
      try {
        McapDirectoryWriter::close();
      } catch (const std::exception & e) {
        BAGWIZ_LOG_WARN(kLogger, "McapDirectoryWriter close failed: %s", e.what());
      } catch (...) {
        // Never throw from destructor.
      }
    }
  }

  McapDirectoryWriter(const McapDirectoryWriter &) = delete;
  McapDirectoryWriter & operator=(const McapDirectoryWriter &) = delete;
  McapDirectoryWriter(McapDirectoryWriter &&) = delete;
  McapDirectoryWriter & operator=(McapDirectoryWriter &&) = delete;

  void declare_topic(const TopicInfo & topic) override
  {
    inner_->declare_topic(topic);
    topics_.push_back(topic);
    topic_counts_[topic.name] = 0;
  }

  void write(
    std::string_view topic, int64_t timestamp_ns, std::span<const std::byte> payload) override
  {
    inner_->write(topic, timestamp_ns, payload);
    ++topic_counts_[std::string(topic)];
    ++total_messages_;
    if (timestamp_ns < start_ns_) {
      start_ns_ = timestamp_ns;
    }
    if (timestamp_ns > end_ns_) {
      end_ns_ = timestamp_ns;
    }
  }

  void close() override
  {
    if (closed_) {
      return;
    }
    inner_->close();
    write_metadata_yaml();
    closed_ = true;
  }

private:
  void write_metadata_yaml()
  {
    const int64_t duration_ns =
      (total_messages_ > 0 && end_ns_ >= start_ns_) ? (end_ns_ - start_ns_) : 0;
    const int64_t starting_ns = total_messages_ > 0 ? start_ns_ : 0;

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "rosbag2_bagfile_information";
    out << YAML::Value << YAML::BeginMap;

    // rosbag2_storage::BagMetadata defaults `version` to 5 on humble, and 5 is
    // the minimum version where rosbag2 considers `files:` part of the schema.
    // Lower versions (1-4) still parse but lose per-file timing; humble's
    // writer emits 5 by default so we match that.
    out << YAML::Key << "version" << YAML::Value << 5;
    out << YAML::Key << "storage_identifier" << YAML::Value << "mcap";
    out << YAML::Key << "duration" << YAML::Value << YAML::BeginMap << YAML::Key << "nanoseconds"
        << YAML::Value << duration_ns << YAML::EndMap;
    out << YAML::Key << "starting_time" << YAML::Value << YAML::BeginMap << YAML::Key
        << "nanoseconds_since_epoch" << YAML::Value << starting_ns << YAML::EndMap;
    out << YAML::Key << "message_count" << YAML::Value << total_messages_;

    out << YAML::Key << "topics_with_message_count" << YAML::Value << YAML::BeginSeq;
    for (const auto & t : topics_) {
      const auto count_it = topic_counts_.find(t.name);
      const int64_t count = count_it != topic_counts_.end() ? count_it->second : 0;
      out << YAML::BeginMap;
      out << YAML::Key << "topic_metadata" << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "name" << YAML::Value << t.name;
      out << YAML::Key << "type" << YAML::Value << t.type;
      out << YAML::Key << "serialization_format" << YAML::Value << t.serialization_format;
      out << YAML::Key << "offered_qos_profiles" << YAML::Value << t.offered_qos_profiles;
      out << YAML::EndMap;
      out << YAML::Key << "message_count" << YAML::Value << count;
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;

    out << YAML::Key << "compression_format" << YAML::Value << options_.mcap_compression;
    out << YAML::Key << "compression_mode" << YAML::Value
        << (options_.mcap_compression.empty() || options_.mcap_compression == "none" ? "" : "file");
    out << YAML::Key << "relative_file_paths" << YAML::Value << YAML::BeginSeq;
    out << shard_rel_;
    out << YAML::EndSeq;

    // `files:` is required for metadata version >= 5; rosbag2's reader throws
    // "invalid node; first invalid key: \"files\"" without it. Single shard,
    // so the per-file timing equals the bag-level summary.
    out << YAML::Key << "files" << YAML::Value << YAML::BeginSeq;
    out << YAML::BeginMap;
    out << YAML::Key << "path" << YAML::Value << shard_rel_;
    out << YAML::Key << "starting_time" << YAML::Value << YAML::BeginMap << YAML::Key
        << "nanoseconds_since_epoch" << YAML::Value << starting_ns << YAML::EndMap;
    out << YAML::Key << "duration" << YAML::Value << YAML::BeginMap << YAML::Key << "nanoseconds"
        << YAML::Value << duration_ns << YAML::EndMap;
    out << YAML::Key << "message_count" << YAML::Value << total_messages_;
    out << YAML::EndMap;
    out << YAML::EndSeq;

    out << YAML::EndMap;
    out << YAML::EndMap;

    std::ofstream f(dir_ / "metadata.yaml");
    if (!f) {
      throw std::runtime_error("failed to open metadata.yaml for writing in " + dir_.string());
    }
    f << out.c_str() << '\n';
  }

  std::filesystem::path dir_;
  CreateOptions options_;
  std::string shard_rel_;
  std::unique_ptr<McapFileWriter> inner_;

  std::vector<TopicInfo> topics_;
  std::unordered_map<std::string, int64_t> topic_counts_;
  int64_t total_messages_ = 0;
  int64_t start_ns_ = std::numeric_limits<int64_t>::max();
  int64_t end_ns_ = std::numeric_limits<int64_t>::min();
  bool closed_ = false;
};

}  // namespace

std::unique_ptr<BagWriter> create_mcap_file(
  const std::filesystem::path & path, const CreateOptions & options)
{
  return std::make_unique<McapFileWriter>(path, options);
}

std::unique_ptr<BagWriter> create_mcap_directory(
  const std::filesystem::path & dir, const CreateOptions & options)
{
  return std::make_unique<McapDirectoryWriter>(dir, options);
}

}  // namespace bagwiz::io::detail
