// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TF__TF_CHAIN_HPP_
#define BAGWIZ__CORE__TF__TF_CHAIN_HPP_

#include <tf2/buffer_core.hpp>
#include <tf2/time.hpp>

#include <string>
#include <utility>
#include <vector>

namespace bagwiz::core
{

// Resolve the chain of frames between `of_frame` and `ref_frame` at
// `time` by walking parent links via tf2::BufferCore::_getParent. We
// use this instead of tf2's `_chainAsVector` because the latter
// returned empty for static-only buffers in our testing (the chain
// search starts from a fixed frame and can produce an empty result
// when source == fixed). _getParent works uniformly for static and
// dynamic cache entries.
//
// The returned vector is ordered front=of_frame, back=ref_frame, i.e.
// of -> ... -> ref. Returns empty when no common ancestor exists in the
// buffer or when the walk is suspiciously deep (cycle guard).
std::vector<std::string> resolve_chain(
  const tf2::BufferCore & buffer, const std::string & of_frame, const std::string & ref_frame,
  tf2::TimePoint time);

// Map a chain of frames (front=of, back=ref) into the underlying TF
// edges as (parent, child) pairs. For each adjacent pair (a, b) in
// `chain`, `_getParent(a, time)` is consulted to determine which side
// is the parent in the buffer. Returns chain.size() - 1 entries; an
// empty input chain yields an empty result.
std::vector<std::pair<std::string, std::string>> chain_to_edges(
  const tf2::BufferCore & buffer, const std::vector<std::string> & chain, tf2::TimePoint time);

// Return whichever of `of_frame` and `ref_frame` is NOT a frame known to
// `buffer` (i.e. not among buffer.getAllFrameNames()), in {of, ref} order and
// de-duplicated when both are the same missing frame. Returns empty when both
// frames exist.
//
// This guards against tf2::BufferCore::lookupTransform's same-frame fast path:
// when target == source it returns an identity transform WITHOUT verifying the
// frame exists, so `tf walk <f> <f>` for a frame absent from the bag would
// otherwise display a bogus identity transform instead of erroring.
std::vector<std::string> missing_frames(
  const tf2::BufferCore & buffer, const std::string & of_frame, const std::string & ref_frame);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TF__TF_CHAIN_HPP_
