// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__PIPELINE__PIPELINED_BACKEND_HPP_
#define BAGWIZ__CORE__PIPELINE__PIPELINED_BACKEND_HPP_

#include "bagwiz/core/pipeline/bounded_message_queue.hpp"
#include "bagwiz/core/pipeline/rewrite_backend.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <cstddef>
#include <string_view>

namespace bagwiz::core::pipeline
{

// A two-stage read || write Backend. The calling thread reads, routes, and
// copies each kept message into a bounded queue; a single writer thread drains
// the queue in FIFO order and writes. Overlapping the read and write stages is
// the whole win on the write-dominant pure-copy trio (read ~32% / write ~68% /
// process 0%), where SequentialBackend leaves one of the two stages idle at any
// instant.
//
// Output is byte-identical to SequentialBackend: a single consumer draining a
// FIFO preserves the reader's emission order, and the payload is copied
// verbatim. The only object shared between the two threads is the queue, so the
// reader and writer are each touched by exactly one thread. An exception on
// either stage is latched in the queue, unblocks the other stage, and is
// rethrown from run() after both threads join (so the process never hangs on a
// read/write error). Counts mirror SequentialBackend exactly.
//
// PipelinedBackend is for pure-copy rewrites only: it never decodes payloads, so
// the decoders' thread-incompatibility is irrelevant here. Transform/inject
// commands stay on SequentialBackend.
class PipelinedBackend : public Backend
{
public:
  // `queue_bytes` caps the total payload bytes buffered between the stages
  // (backpressure / RSS bound). Defaults to kDefaultQueueBytes; tests pass a
  // tiny value to exercise the backpressure path.
  explicit PipelinedBackend(std::size_t queue_bytes = kDefaultQueueBytes)
  : queue_bytes_(queue_bytes)
  {
  }

  RewriteCounts run(
    io::BagReader & reader, io::BagWriter & writer, const Processor & processor,
    std::string_view profile_label) override;

private:
  std::size_t queue_bytes_;
};

}  // namespace bagwiz::core::pipeline

#endif  // BAGWIZ__CORE__PIPELINE__PIPELINED_BACKEND_HPP_
