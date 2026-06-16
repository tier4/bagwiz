// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__PIPELINE__REWRITE_BACKEND_HPP_
#define BAGWIZ__CORE__PIPELINE__REWRITE_BACKEND_HPP_

#include "bagwiz/io/bag_io.hpp"

#include <cstdint>
#include <string>
#include <string_view>

// The shared "read -> process -> write" rewrite seam. A rewrite command pairs a
// Processor (the per-message routing decision) with a Backend (which owns the
// read/process/write loop) via run_pipeline(). The only Backend today is the
// zero-copy SequentialBackend; Milestone #3 adds a threaded PipelinedBackend
// behind the same interface, so commands gain parallelism without changing
// their routing logic.
namespace bagwiz::core::pipeline
{

// Per-message routing decision produced by a Processor. For pure-copy rewrites
// this only selects the OUTPUT topic name or drops the message; the payload is
// forwarded verbatim. The shape is forward-compatible with transform commands
// (a future payload-aware Processor overload can rewrite the bytes too).
struct Emit
{
  // false -> drop the message entirely (neither written nor counted as copied).
  bool keep = true;
  // Output topic name when kept. Must stay valid for the duration of the
  // corresponding writer.write() call; routers return a view into the input
  // topic name or into their caller-owned selector containers.
  std::string_view out_topic;
};

// Maps an input message to an Emit decision. Implementations must be
// thread-compatible: a Backend may call route() from a worker thread
// (Milestone #3), so route() is const and must not mutate shared state without
// synchronization. The pure-copy routers are trivially safe (const lookups).
class Processor
{
public:
  Processor() = default;
  virtual ~Processor() = default;
  Processor(const Processor &) = default;
  Processor & operator=(const Processor &) = default;
  Processor(Processor &&) noexcept = default;
  Processor & operator=(Processor &&) noexcept = default;

  // Decide how to route a message published on `in_topic`. Taken by const
  // reference (not string_view) so set/map lookups stay allocation-free, since
  // the reader hands back the topic name as a std::string.
  [[nodiscard]] virtual Emit route(const std::string & in_topic) const = 0;
};

// Outcome counters for a rewrite run.
struct RewriteCounts
{
  std::uint64_t copied = 0;   // messages forwarded to the writer
  std::uint64_t dropped = 0;  // messages dropped by the processor (keep == false)
  std::uint64_t renamed = 0;  // subset of `copied` written under a changed name
};

// Owns the read -> process -> write loop. SequentialBackend runs it inline and
// zero-copy (the correctness oracle and the zero-cost default). Milestone #3
// adds a PipelinedBackend that overlaps read and write on separate threads
// behind this same interface.
//
// Contract for every Backend: messages are processed and written in the
// reader's emission order; `profile_label`, when non-empty and BAGWIZ_PROFILE
// is set, drives the per-stage StageProfiler report. Throws whatever the
// reader/writer throw on I/O error (partial counts are lost on throw).
class Backend
{
public:
  Backend() = default;
  virtual ~Backend() = default;
  Backend(const Backend &) = default;
  Backend & operator=(const Backend &) = default;
  Backend(Backend &&) noexcept = default;
  Backend & operator=(Backend &&) noexcept = default;

  virtual RewriteCounts run(
    io::BagReader & reader, io::BagWriter & writer, const Processor & processor,
    std::string_view profile_label) = 0;
};

// Convenience entry point: forward to `backend`. Keeps call sites uniform and
// gives a single place to add cross-backend bookkeeping later.
RewriteCounts run_pipeline(
  io::BagReader & reader, io::BagWriter & writer, const Processor & processor, Backend & backend,
  std::string_view profile_label = "");

}  // namespace bagwiz::core::pipeline

#endif  // BAGWIZ__CORE__PIPELINE__REWRITE_BACKEND_HPP_
