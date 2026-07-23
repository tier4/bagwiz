// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__BAG__BAG_COPY_HPP_
#define BAGWIZ__CORE__BAG__BAG_COPY_HPP_

#include "bagwiz/core/pipeline/backend_select.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

// Stream-copy a bag through an already-open reader / writer, optionally
// skipping messages on a set of topic names. Used by rewrite-style
// commands (e.g. `bagwiz traj join`) that need to forward every message
// from an input bag into an output bag except for the topics they are
// about to replace.
//
// The reader's filter (set via `BagReader::set_filter`) and the
// writer's topic declarations are the caller's responsibility. This
// helper only owns the `next()` -> `write()` loop.
namespace bagwiz::core
{

struct BagCopyCounts
{
  // Messages forwarded to the writer.
  std::uint64_t copied = 0;
  // Messages skipped because their topic was in the suppress set.
  std::uint64_t suppressed = 0;
};

// Per-message keep decision with access to the full message (see
// Processor::keep_message). Runs on the reading thread; the RawMessage view is
// only valid for the duration of the call.
using MessagePredicate = std::function<bool(const io::RawMessage &)>;

// Iterate `reader` to exhaustion. Messages whose `topic->name` appears
// in `suppress` are dropped; the rest are forwarded to `writer` with
// the original `timestamp_ns` and payload preserved.
//
// When `keep` is set, it is evaluated for every message before the suppress
// set; a message it rejects is dropped (counted in `suppressed`). Used by
// content-dependent filters such as `trim`'s header.stamp window.
//
// When `profile_label` is non-empty AND the BAGWIZ_PROFILE environment
// variable is set, the loop times the read and write stages and logs a
// per-stage bottleneck report under that label on completion. The
// instrumentation is off the hot path otherwise (no clock reads).
//
// Throws whatever the underlying reader / writer throws on I/O error;
// partial progress is visible through the returned counters when an
// exception propagates.
BagCopyCounts bag_copy_filtered(
  io::BagReader & reader, io::BagWriter & writer, const std::unordered_set<std::string> & suppress,
  std::string_view profile_label = "",
  pipeline::BackendKind backend = pipeline::BackendKind::Sequential,
  const MessagePredicate & keep = {});

struct BagCopyRenameCounts
{
  // Total messages forwarded to the writer.
  std::uint64_t copied = 0;
  // Subset of `copied` whose topic name was remapped via `rename`.
  std::uint64_t renamed = 0;
};

// Iterate `reader` to exhaustion, forwarding every message to `writer`. A
// message whose `topic->name` is a key in `rename` is written under the mapped
// name (and counted in `renamed`); every other message keeps its original name.
// Timestamps and payloads are preserved verbatim. Used by `bagwiz topic rename`,
// which supplies a single old->new entry; the map form keeps the helper general
// and trivially testable.
//
// `profile_label` behaves as in bag_copy_filtered: a non-empty label plus
// BAGWIZ_PROFILE enables a per-stage timing report.
//
// The caller is responsible for declaring the destination topic(s) on `writer`
// before calling this (writers reject a write to an undeclared topic). Throws
// whatever the underlying reader / writer throws on I/O error; partial progress
// is visible through the returned counters when an exception propagates.
BagCopyRenameCounts bag_copy_renamed(
  io::BagReader & reader, io::BagWriter & writer,
  const std::unordered_map<std::string, std::string> & rename, std::string_view profile_label = "",
  pipeline::BackendKind backend = pipeline::BackendKind::Sequential);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__BAG__BAG_COPY_HPP_
