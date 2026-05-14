// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__BAG_COPY_HPP_
#define BAGWIZ__CORE__BAG_COPY_HPP_

#include "bagwiz/io/bag_io.hpp"

#include <cstdint>
#include <string>
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

// Iterate `reader` to exhaustion. Messages whose `topic->name` appears
// in `suppress` are dropped; the rest are forwarded to `writer` with
// the original `timestamp_ns` and payload preserved.
//
// Throws whatever the underlying reader / writer throws on I/O error;
// partial progress is visible through the returned counters when an
// exception propagates.
BagCopyCounts bag_copy_filtered(
  io::BagReader & reader, io::BagWriter & writer, const std::unordered_set<std::string> & suppress);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__BAG_COPY_HPP_
