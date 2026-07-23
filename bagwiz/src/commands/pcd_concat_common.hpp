// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__PCD_CONCAT_COMMON_HPP_
#define COMMANDS__PCD_CONCAT_COMMON_HPP_

#include "bagwiz/core/pointcloud/cloud_transform.hpp"
#include "bagwiz/core/pointcloud/concat_sync.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Shared internals of `pcd concat`, split out of pcd_concat.cpp so the
// stamp-offset parsing and the concat assembly machinery can be unit-tested
// without a bag on disk. CLI-internal: this header lives with the command
// sources and is not installed.
namespace bagwiz::commands
{

// One input pcd topic's state: the resolved extrinsic and --stamp-offset, the
// Pass-A collected header stamps, and the Pass-A diagnostics that surface in
// the end-of-run summary.
struct TopicState
{
  std::string name;
  std::string frame_id;
  core::pointcloud::RigidTransform extrinsic;  // target(--frame) <- frame_id
  std::int64_t offset_ns = 0;                  // --stamp-offset
  std::vector<std::int64_t> stamps_ns;         // Pass-A collected header stamps
  std::int64_t header_fail = 0;                // undecodable header in Pass A
  bool non_monotonic = false;                  // header stamps went backwards
};

// Parse --stamp-offset "topic=value" entries into per-topic offsets (0 for
// topics without an entry), indexed like `topic_index`. Every malformed entry
// (no '=', topic not in --pcd, unparseable value) is logged to `logger` and
// yields std::nullopt.
[[nodiscard]] std::optional<std::vector<std::int64_t>> parse_stamp_offsets(
  const std::vector<std::string> & entries,
  const std::unordered_map<std::string, std::size_t> & topic_index, const char * logger);

// The Pass-B assembly machinery of `pcd concat`. Ingests the pcd input
// messages in bag order (on_message), parses and transforms the picked ones
// into the target frame, caches them with a refcount, and fires each sync
// group exactly once when its last pick has arrived — emitting the merged,
// serialized PointCloud2 for the caller to write. Picks whose message failed
// to parse or transform are missing from the cache, so their group fires as
// partial (or, when every pick failed, writes nothing).
class ConcatAssembler
{
public:
  // One fired group's merged output, ready for BagWriter::write on the output
  // topic: the group's output stamp and the serialized PointCloud2 payload.
  struct FiredOutput
  {
    std::int64_t stamp_ns = 0;
    std::vector<std::byte> payload;
  };

  // Per-run tallies surfaced in the end-of-run summary.
  struct Counters
  {
    std::int64_t written_groups = 0;           // groups whose merged cloud was emitted
    std::int64_t partial_groups = 0;           // fired groups missing at least one pick
    std::vector<std::int64_t> matched;         // per topic: picks joined into a fired group
    std::vector<std::int64_t> parse_fail;      // per topic: undecodable picked messages
    std::vector<std::int64_t> transform_fail;  // per topic: extrinsic transform failures
  };

  // Outcome of one on_message() call. `fired` holds every group this message
  // completed (one message can be the last missing pick of several groups), in
  // group order. `error` is set on a concat failure — the caller must still
  // write `fired` first (those groups completed successfully before the
  // failure), then log `error` and abort.
  struct IngestResult
  {
    std::vector<FiredOutput> fired;
    std::string error;
  };

  // `topics` supplies each input's resolved extrinsic and message count (the
  // stamps_ns size); `groups` is the plan_sync output for those topics.
  ConcatAssembler(
    const std::vector<TopicState> & topics, std::vector<core::pointcloud::SyncGroup> groups,
    std::string target_frame);

  // Ingest one message of pcd input `topic` (`index` = its 0-based position on
  // that topic in bag order). A message no group picked is ignored without
  // even parsing it, mirroring the streaming pass.
  [[nodiscard]] IngestResult on_message(
    std::size_t topic, std::size_t index, std::span<const std::byte> payload);

  [[nodiscard]] const Counters & counters() const { return counters_; }

private:
  // A transformed cloud awaiting its referencing groups, with a refcount so it
  // is freed once no pending group needs it.
  struct Cached
  {
    core::pointcloud::PointCloud2 cloud;
    std::int64_t header_stamp_ns = 0;
    std::size_t refcount = 0;
  };

  static std::uint64_t key(std::size_t topic, std::size_t index)
  {
    return (static_cast<std::uint64_t>(topic) << 40) | static_cast<std::uint64_t>(index);
  }

  // Fire one group whose picks have all arrived: gather the cached clouds in
  // --pcd order, concatenate, and append the merged output to `result`. On a
  // concat failure sets `result.error` and returns false WITHOUT releasing the
  // group's cache entries (the caller aborts the whole pass).
  bool fire(std::size_t group, IngestResult & result);

  // Drop the group's cache entries whose refcount hits zero.
  void release(std::size_t group);

  std::size_t num_topics_;
  std::vector<core::pointcloud::RigidTransform> extrinsics_;
  std::vector<core::pointcloud::SyncGroup> groups_;
  std::string target_frame_;
  // (topic, msg index) -> groups referencing it; sized to each topic's message
  // count so an out-of-plan index is detectably unpicked.
  std::vector<std::vector<std::vector<std::size_t>>> refs_;
  std::vector<std::size_t> group_remaining_;  // per-group not-yet-arrived picks
  std::vector<char> fired_;                   // per-group fire-once guard
  std::unordered_map<std::uint64_t, Cached> cache_;
  Counters counters_;
};

// ---------------------------------------------------------------------------
// Parallel Pass B building blocks. The reader thread runs ConcatGroupTracker
// (matching is arrival-driven only, so it needs no parsing); each fired group
// becomes one self-contained ConcatGroupJob a worker thread turns into a
// ConcatGroupResult via process_concat_group; the collector thread accumulates
// the serial-identical counters with ConcatCounterMerger.
// ---------------------------------------------------------------------------

// One planned pick of a fired group: which input message it is, plus shared
// ownership of its raw (unparsed) payload.
struct ConcatPick
{
  std::size_t topic = 0;
  std::size_t index = 0;
  std::shared_ptr<const std::vector<std::byte>> payload;
};

// One fired sync group as a self-contained worker job (picks in --pcd order).
struct ConcatGroupJob
{
  std::int64_t output_stamp_ns = 0;
  std::vector<ConcatPick> picks;
};

// Matching-only twin of ConcatAssembler for the parallel pass: the same
// arrival bookkeeping (refs / remaining / fire-once / refcounted cache), but
// it stores raw payloads and never parses. All calls happen on the reader
// thread; the returned jobs carry shared payload ownership into the workers.
class ConcatGroupTracker
{
public:
  // `topics` supplies each input's message count (the stamps_ns size);
  // `groups` is the plan_sync output for those topics.
  ConcatGroupTracker(
    const std::vector<TopicState> & topics, std::vector<core::pointcloud::SyncGroup> groups);

  // Ingest one message of pcd input `topic` (`index` = its 0-based position on
  // that topic in bag order). Returns one job per group this arrival
  // completed, in group order — empty when none fired. Unpicked messages are
  // not even copied, mirroring the serial pass.
  [[nodiscard]] std::vector<ConcatGroupJob> on_message(
    std::size_t topic, std::size_t index, std::span<const std::byte> payload);

private:
  // A raw payload awaiting its referencing groups, with a refcount so the
  // cache entry is dropped once every referencing group's job took its
  // shared_ptr.
  struct Entry
  {
    std::shared_ptr<const std::vector<std::byte>> payload;
    std::size_t refcount = 0;
  };

  [[nodiscard]] ConcatGroupJob build_job(std::size_t group);

  std::size_t num_topics_;
  std::vector<core::pointcloud::SyncGroup> groups_;
  std::vector<std::vector<std::vector<std::size_t>>> refs_;
  std::vector<std::size_t> group_remaining_;
  std::vector<char> fired_;
  std::unordered_map<std::uint64_t, Entry> cache_;
};

// Per-pick processing outcome, keyed by (topic, index) so the collector can
// dedupe failures of a pick shared between groups back to once-per-message.
struct ConcatPickOutcome
{
  enum class Status { kOk, kParseFail, kTransformFail };
  std::size_t topic = 0;
  std::size_t index = 0;
  Status status = Status::kOk;
};

// One processed group: the serialized merged cloud (empty when every pick
// failed, or on error) plus everything the collector needs to reproduce the
// serial pass's counters.
struct ConcatGroupResult
{
  std::int64_t stamp_ns = 0;
  std::vector<std::byte> payload;  // empty => nothing to write
  bool partial = false;            // fewer surviving picks than input topics
  std::string error;               // non-empty => concat failed (abort the pass)
  std::vector<ConcatPickOutcome> picks;
};

// Parse, transform, and concatenate one fired group. Touches only the job and
// the read-only extrinsics/target frame, so it is safe on any worker thread.
// A pick shared between groups is re-parsed per group; parsing is
// deterministic, so every group sees the identical cloud.
[[nodiscard]] ConcatGroupResult process_concat_group(
  const ConcatGroupJob & job, const std::vector<core::pointcloud::RigidTransform> & extrinsics,
  const std::string & target_frame, std::size_t num_topics);

// Collector-side counter accumulation replicating the serial counters:
// matched per (group, surviving pick); parse/transform failures once per
// unique message even when a shared pick was re-processed by several groups.
class ConcatCounterMerger
{
public:
  explicit ConcatCounterMerger(std::size_t num_topics);

  void merge(const ConcatGroupResult & result);

  [[nodiscard]] const ConcatAssembler::Counters & counters() const { return counters_; }

private:
  ConcatAssembler::Counters counters_;
  std::unordered_set<std::uint64_t> failed_seen_;
};

}  // namespace bagwiz::commands

#endif  // COMMANDS__PCD_CONCAT_COMMON_HPP_
