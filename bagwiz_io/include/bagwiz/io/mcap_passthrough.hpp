// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__IO__MCAP_PASSTHROUGH_HPP_
#define BAGWIZ__IO__MCAP_PASSTHROUGH_HPP_

#include "bagwiz/io/bag_io.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace bagwiz::io
{

// The edit a pure-copy rewrite applies to a single mcap file. All fields
// compose: a channel survives when its topic is not in `drop_topics`; a
// message survives when its channel survives and its log time falls inside
// the half-open window [start_ns, end_ns); surviving channels listed in
// `rename` are re-published under the mapped topic name.
struct McapPassthroughEdit
{
  std::unordered_set<std::string> drop_topics;
  std::unordered_map<std::string, std::string> rename;  // old topic -> new topic
  // Half-open [start_ns, end_ns) on message log time, matching
  // ReadFilter's convention. Unset bound = unbounded on that side.
  std::optional<std::int64_t> start_ns;
  std::optional<std::int64_t> end_ns;
};

// What a successful pass-through run produced, with enough detail for the
// caller to log command counts and emit a directory bag's metadata.yaml.
struct McapPassthroughResult
{
  std::uint64_t messages_written = 0;
  std::uint64_t messages_renamed = 0;  // subset of messages_written
  std::uint64_t chunks_copied = 0;     // copied byte-for-byte
  std::uint64_t chunks_reencoded = 0;  // decoded and re-encoded (edit touched them)
  std::uint64_t chunks_dropped = 0;    // omitted entirely
  // Surviving channels in ascending channel-id order, with rename applied.
  std::vector<TopicInfo> topics;
  // Message counts keyed by (post-rename) topic name; zero-count topics are
  // omitted.
  std::unordered_map<std::string, std::int64_t> per_topic_counts;
  std::int64_t start_ns = 0;  // output time extent; valid when messages_written > 0
  std::int64_t end_ns = 0;
  // The single chunk codec of the output ("zstd" / "lz4") when every written
  // chunk shares it; "" when the output is uncompressed, empty, or mixed.
  std::string chunk_compression;
  // Top-level Attachment/Metadata records found in the input and omitted from
  // the output. The decoded rewrite pipeline drops these silently (it reads
  // messages only); the pass-through matches that behavior but reports the
  // counts so the caller can warn. rosbag2 stamps every recording with one
  // Metadata record, so a non-zero count is the norm, not a corruption sign.
  std::uint64_t attachments_skipped = 0;
  std::uint64_t metadata_skipped = 0;
};

// Rewrite the single mcap file `input` into `output` at the mcap record
// level, without decoding message payloads:
//
//   - chunks the edit does not touch are copied byte-for-byte together with
//     their MessageIndex records, preserving the input's chunk compression;
//   - chunks containing a dropped or renamed channel, or straddling the time
//     window, are decompressed, filtered record-by-record, and re-encoded
//     with the chunk's own codec;
//   - chunks entirely outside the window (or carrying only dropped channels)
//     are omitted;
//   - the summary section is rebuilt: schema and channel ids are preserved
//     verbatim (which keeps the copied chunks' message records and
//     MessageIndex offsets valid), ChunkIndex offsets are recomputed, and
//     Statistics are recomputed from the output.
//
// The surviving Schema/Channel set is additionally re-emitted as top-level
// data-section records right after the Header. Chunked mcap embeds these
// records inside chunk payloads, so an edit that drops the embedding chunk
// (a trim window past the head of the bag is the common case) would
// otherwise leave surviving messages with no preceding channel definition
// for file-order readers and `mcap recover`. Duplicates inside verbatim
// chunks are legal per the mcap specification.
//
// Top-level Attachment/Metadata records are omitted from the output and
// counted in the result — the same content loss the decoded pipeline
// inflicts silently, made visible for the caller to warn about.
//
// Returns nullopt — with a human-readable reason in `fallback_reason` when
// provided — whenever the input needs the decoded rewrite pipeline instead.
// That covers, among others: a missing/damaged summary section, unchunked
// or index-less content, duplicate topic names across channels (the decoded
// pipeline merges them), a rename whose embedded Channel record cannot be
// located and stripped, and unknown records or codecs in chunks that must
// be re-encoded. Every check except the rename verification runs before the
// first output byte; the rename check can abort late, in which case the
// partial output file is removed before returning. Genuine I/O errors on an
// eligible input throw, like the rest of the io layer.
std::optional<McapPassthroughResult> mcap_passthrough_rewrite(
  const std::filesystem::path & input, const std::filesystem::path & output,
  const McapPassthroughEdit & edit, std::string * fallback_reason = nullptr);

}  // namespace bagwiz::io

#endif  // BAGWIZ__IO__MCAP_PASSTHROUGH_HPP_
