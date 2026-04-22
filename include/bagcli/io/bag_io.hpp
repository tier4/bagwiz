// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGCLI__IO__BAG_IO_HPP_
#define BAGCLI__IO__BAG_IO_HPP_

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace bagcli::io
{

// Per-topic metadata. Populated lazily by BagReader::topics(); never triggers
// a full-message scan.
struct TopicInfo
{
  std::string name;
  std::string type;                  // e.g. "sensor_msgs/msg/Image"
  std::string serialization_format;  // usually "cdr"
  std::string offered_qos_profiles;  // raw YAML string, unparsed
};

// Zero-copy view of a single message returned by BagReader::next(). Pointers
// and spans are invalidated by the next call to next() or by reader
// destruction.
struct RawMessage
{
  const TopicInfo * topic = nullptr;
  int64_t timestamp_ns = 0;
  std::span<const std::byte> payload;
};

// Pre-iteration filter pushed down into the storage layer. SQLite3 uses it
// in WHERE clauses; MCAP uses it to skip chunks via the chunk index.
struct ReadFilter
{
  std::vector<std::string> topics;  // empty = all topics
  std::optional<int64_t> start_ns;
  std::optional<int64_t> end_ns;
};

// Read-only bag interface. Opening is cheap: implementations must never
// scan message records during construction. Statistics are opt-in via
// compute_stats().
class BagReader
{
public:
  virtual ~BagReader() = default;

  // Cheap: returns the topic list without scanning messages.
  virtual std::span<const TopicInfo> topics() const = 0;

  // Must be called before the first next(). Calling after iteration has
  // started is undefined behaviour.
  virtual void set_filter(const ReadFilter & filter) = 0;

  // Stream the next message. Returns false at EOF; throws on IO error.
  virtual bool next(RawMessage & out) = 0;

  struct Stats
  {
    int64_t start_ns = 0;
    int64_t end_ns = 0;
    int64_t total_messages = 0;
    std::unordered_map<std::string, int64_t> per_topic;
    // True if the numbers came from a summary/index; false if a scan was
    // required.
    bool from_summary = false;
  };

  // May be expensive. Implementations should prefer summary/index data
  // (MCAP summary, SQLite aggregate queries) and only fall back to a full
  // scan when unavoidable.
  virtual Stats compute_stats() = 0;
};

// Write-only bag interface. close() finalizes the bag (and writes
// metadata.yaml for directory layouts).
class BagWriter
{
public:
  virtual ~BagWriter() = default;

  virtual void declare_topic(const TopicInfo & topic) = 0;
  virtual void write(
    std::string_view topic, int64_t timestamp_ns, std::span<const std::byte> payload) = 0;
  virtual void close() = 0;
};

enum class Format { Auto, Mcap, Sqlite3 };

enum class Layout { Auto, SingleFile, Directory };

struct OpenOptions
{
  Format format = Format::Auto;
  Layout layout = Layout::Auto;
};

struct CreateOptions
{
  Format format = Format::Mcap;
  Layout layout = Layout::Directory;
  std::optional<uint64_t> split_bytes;
  std::optional<int64_t> split_duration_ns;

  // MCAP-specific
  std::string mcap_compression = "zstd";  // "", "lz4", "zstd"
  uint32_t mcap_chunk_size = 4 * 1024 * 1024;
};

// Factory functions. Format/layout are auto-detected from magic bytes and
// directory contents unless overridden in options.
std::unique_ptr<BagReader> open_read(const std::filesystem::path & path, OpenOptions options = {});
std::unique_ptr<BagWriter> open_write(
  const std::filesystem::path & path, CreateOptions options = {});

}  // namespace bagcli::io

#endif  // BAGCLI__IO__BAG_IO_HPP_
