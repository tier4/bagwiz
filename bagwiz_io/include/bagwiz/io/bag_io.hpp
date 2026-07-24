// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__IO__BAG_IO_HPP_
#define BAGWIZ__IO__BAG_IO_HPP_

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

namespace bagwiz::io
{

// Per-topic metadata. Populated lazily by BagReader::topics(); never triggers
// a full-message scan.
//
// Schema fields carry the embedded message definition when the storage format
// preserves it (MCAP `Schema` records) and stays empty otherwise (SQLite3,
// metadata.yaml-derived listings before populate_schemas() is called). When
// non-empty they let downstream code decode messages without loading the
// rosidl introspection typesupport for the type — the central payoff of
// MCAP's self-describing nature.
struct TopicInfo
{
  std::string name;
  std::string type;                  // e.g. "sensor_msgs/msg/Image"
  std::string serialization_format;  // usually "cdr"
  std::string offered_qos_profiles;  // raw YAML string, unparsed

  std::string schema_encoding;  // "ros2msg", "ros2idl", or empty if unknown
  std::string schema_text;      // raw bytes from the storage layer, decoded as
                                // UTF-8; empty when the storage carries no
                                // schema (legacy SQLite3 v3 bags, MCAPs
                                // written with empty Schema.data).

  // ROS 2 Iron-and-later type description hash ("RIHS01_<sha256>", per the
  // rosidl_runtime_c type description spec). Iron+ rosbag2 SQLite3 v4 and
  // some MCAPs propagate this; older formats leave it empty. bagwiz
  // preserves it on round-trip but does not currently compute it for
  // bags where it's missing.
  std::string type_description_hash;
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
  // Time range as the half-open interval [start_ns, end_ns): start_ns is
  // inclusive, end_ns is exclusive (matching mcap ReadMessageOptions), on
  // every backend. Unset bound = unbounded on that side.
  std::optional<int64_t> start_ns;
  std::optional<int64_t> end_ns;

  // When non-empty, only these topics' messages are guaranteed to carry their
  // payload; messages of other topics passed by `topics` may come back with an
  // empty payload span. This lets the storage layer skip BLOB materialization
  // for rows the caller only needs timestamps from (SQLite3 avoids reading the
  // overflow pages of large messages, which is the dominant I/O cost when
  // collecting timestamps of multi-GB point-cloud topics). A non-empty list
  // that names no topic in the bag therefore skips every payload — the
  // timestamps-only scan idiom. MCAP exposes payloads unconditionally (chunks
  // are decompressed wholesale), which the contract permits: callers must
  // treat the payload of a non-listed topic as unconditionally absent, never
  // as an empty message.
  std::vector<std::string> payload_topics;
};

// Read-only bag interface. Opening is cheap: implementations must never
// scan message records during construction. Statistics are opt-in via
// compute_stats().
class BagReader
{
public:
  BagReader() = default;
  virtual ~BagReader() = default;

  // Rule of Five (C.21) on a polymorphic interface: explicit defaults so
  // clang-tidy can see them. Concrete derived implementations decide
  // whether copy/move is supported; the abstract base has no state.
  BagReader(const BagReader &) = default;
  BagReader & operator=(const BagReader &) = default;
  BagReader(BagReader &&) noexcept = default;
  BagReader & operator=(BagReader &&) noexcept = default;

  // Cheap: returns the topic list without scanning messages.
  virtual std::span<const TopicInfo> topics() const = 0;

  // Must be called before the first next(). Calling after iteration has
  // started throws std::runtime_error.
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

  // Compute message counts for a subset of topics.
  //
  // Implementations should prefer summary/index data (MCAP summary,
  // metadata.yaml per-topic counts, SQLite covering index) and only scan
  // the requested topics when unavoidable. Topics that are absent or have
  // zero messages are omitted from the result.
  virtual std::unordered_map<std::string, int64_t> compute_topic_counts(
    std::span<const std::string> topics) = 0;

  // Lightweight bag-level time extent. Implementations prefer
  // summary/index data (MCAP summary, metadata.yaml, SQLite timestamp index).
  // When no summary is available, the returned extent is zero and has_data is
  // false; a full scan is not performed automatically.
  struct TimeExtent
  {
    int64_t start_ns = 0;
    int64_t end_ns = 0;
    bool has_data = false;
  };

  virtual TimeExtent compute_time_extent() = 0;

  // Backfill TopicInfo::schema_text / schema_encoding for all topics when the
  // storage embeds schemas but the cheap topics() path did not load them.
  //
  // Default: no-op — single-file readers populate schemas at construction.
  // McapShardReader overrides to open the first shard once and copy schema
  // bytes into its cached topic list. Idempotent.
  //
  // Callers that need decode-ready TopicInfo (decoders, self-describing
  // writers) must call this before consuming topics(); callers that only
  // want names / types / counts (`bagwiz ls`) should not, since it forces a
  // shard open on multi-shard bags.
  virtual void populate_schemas() {}
};

// Write-only bag interface. close() finalizes the bag (and writes
// metadata.yaml for directory layouts).
class BagWriter
{
public:
  BagWriter() = default;
  virtual ~BagWriter() = default;

  // Rule of Five (C.21) on a polymorphic interface: explicit defaults so
  // clang-tidy can see them. Concrete derived implementations decide
  // whether copy/move is supported; the abstract base has no state.
  BagWriter(const BagWriter &) = default;
  BagWriter & operator=(const BagWriter &) = default;
  BagWriter(BagWriter &&) noexcept = default;
  BagWriter & operator=(BagWriter &&) noexcept = default;

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
  // 1 MiB keeps libmcap's chunk staging buffer L2-resident: every payload is
  // memcpy'd into that buffer and read back by the chunk flush, and once the
  // buffer outgrows the per-core cache both passes round-trip DRAM instead
  // (a 4 MiB buffer measured ~9% slower, 64 MiB ~26% slower, on a 4.7 GB
  // uncompressed rewrite). Upstream mcap defaults to 768 KiB for the same
  // regime; 512 KiB measured no faster than 1 MiB.
  uint32_t mcap_chunk_size = 1024 * 1024;
};

// Factory functions. Format/layout are auto-detected from magic bytes and
// directory contents unless overridden in options.
std::unique_ptr<BagReader> open_read(const std::filesystem::path & path, OpenOptions options = {});
std::unique_ptr<BagWriter> open_write(
  const std::filesystem::path & path, CreateOptions options = {});

// Identify the storage format of an existing bag without opening a reader.
//
// - For directory layouts: parses metadata.yaml's storage_identifier.
// - For single-file inputs: reads the first 16 bytes and matches the
//   format-specific magic prefix (MCAP `\x89MCAP0`, SQLite3
//   `SQLite format 3\0`). Extensions are not consulted, so renamed or
//   extensionless files are still classified correctly.
//
// Returns Format::Auto when the input does not exist or no signal
// matches. This function never throws.
Format detect_format(const std::filesystem::path & path) noexcept;

// Infer Format from a path's extension only. The path does not need to
// exist — this is intended for output paths where the user has signalled
// intent through the filename (e.g. `out.mcap` -> Mcap, `out.db3` ->
// Sqlite3). Any other extension (or none, e.g. directory targets) yields
// Format::Auto so callers can fall back to an explicit `--storage` flag.
Format infer_format_from_extension(const std::filesystem::path & path) noexcept;

// True when `path` is a rosbag2 FILE-mode (whole-database zstd envelope) bag:
// a directory whose metadata.yaml declares `compression_mode: FILE`, or a bare
// `.db3.zstd` single file. Reading such a bag's contents (even its topic list,
// for a bare single file) requires decompressing the entire database first, so
// latency-sensitive callers use this to skip work that would otherwise block.
// MESSAGE-mode bags return false. Inspects only the extension / metadata.yaml —
// never decompresses, never throws.
bool is_file_compressed_bag(const std::filesystem::path & path) noexcept;

// Compose CreateOptions for a directory-style output path that should
// inherit the storage format of a reference bag when the user did not
// pick a single-file format via the output extension. Intended for
// rewrite-style commands where the user's mental model is "the output
// should look like the input unless I named a `.mcap` / `.db3` file".
//
// The reference bag may be a directory bag OR a single-file bag — only
// its format is inherited. The output is always Layout::Directory when
// inheritance kicks in, since the output path's lack of a single-file
// extension is what signals the user's directory intent.
//
// Behavior:
//   - `output_path` ends in `.mcap` or `.db3`: returns
//     Format::Auto + Layout::Auto so factory-level extension resolution
//     honours the user's choice (including cross-format conversions).
//   - else if `reference_path`'s format can be detected via
//     `detect_format` (works on both directory and single-file bags):
//     returns Format::<detected> + Layout::Directory.
//   - otherwise: returns Format::Auto + Layout::Auto and callers fall
//     back to the factory's default (Directory + Mcap).
//
// Never throws and never touches the filesystem beyond `detect_format`.
CreateOptions create_options_inheriting_format(
  const std::filesystem::path & reference_path, const std::filesystem::path & output_path) noexcept;

// Compose CreateOptions that preserve BOTH the format and the layout of
// an existing bag. Used by in-place rewrites (e.g. `bagwiz traj join`
// without `-o`) where the output path is a synthetic tmp suffix that
// the factory's extension-based Auto resolution cannot interpret. A
// sibling to `create_options_inheriting_format`, with a stricter layout
// policy: single-file references stay single-file (not converted to a
// directory).
//
// Behavior:
//   - if `reference_path`'s format can be detected via `detect_format`:
//     returns Format::<detected> + Layout::<SingleFile|Directory>
//     depending on whether the reference path is a directory.
//   - otherwise: returns Format::Auto + Layout::Auto. The caller is
//     expected to detect this and surface a user-facing error rather
//     than open a writer with unresolved Auto/Auto.
//
// Never throws and never touches the filesystem beyond `detect_format`
// and `is_directory`.
CreateOptions create_options_preserving_storage(
  const std::filesystem::path & reference_path) noexcept;

}  // namespace bagwiz::io

#endif  // BAGWIZ__IO__BAG_IO_HPP_
