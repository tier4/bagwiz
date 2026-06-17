// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pipeline/pipelined_backend.hpp"

#include "bagwiz/core/pipeline/bounded_message_queue.hpp"
#include "bagwiz/core/pipeline/owned_message.hpp"
#include "bagwiz/core/pipeline/rewrite_backend.hpp"
#include "bagwiz/core/pipeline/stage_profiler.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <string>
#include <thread>
#include <utility>

namespace bagwiz::core::pipeline
{

namespace
{

// Read stage: pull every message, route it, and copy each kept message into the
// queue. Owns all the counters and the read/byte profiling so the write stage
// stays a pure drain. The keep/drop/rename accounting mirrors SequentialBackend
// line for line, so the counts are identical regardless of backend.
RewriteCounts read_loop(
  io::BagReader & reader, const Processor & processor, BoundedMessageQueue & queue,
  StageProfiler & prof)
{
  RewriteCounts counts;
  io::RawMessage raw;
  while (true) {
    bool got = false;
    {
      auto s = prof.time(Stage::kRead);
      got = reader.next(raw);
    }
    if (!got) {
      break;
    }
    // raw.topic is non-null when next() returns true (zero-copy view documented
    // by BagReader::next).
    const auto size = static_cast<std::uint64_t>(raw.payload.size());
    const Emit emit = processor.route(raw.topic->name);
    if (!emit.keep) {
      prof.add_message(size, 0);  // read+decompressed but not written
      ++counts.dropped;
      continue;
    }
    // Copy the payload and the resolved topic name into an owned record BEFORE
    // the next next() invalidates the reader's zero-copy view, then hand it to
    // the write thread. The rename test (out_topic != input name) must happen
    // here too, while the input name is still valid.
    const bool renamed = emit.out_topic != raw.topic->name;
    OwnedMessage msg;
    msg.out_topic = std::string(emit.out_topic);
    msg.timestamp_ns = raw.timestamp_ns;
    msg.payload.assign(raw.payload.begin(), raw.payload.end());
    if (!queue.push(std::move(msg))) {
      break;  // the writer failed; stop producing (its error is rethrown below)
    }
    if (renamed) {
      ++counts.renamed;
    }
    prof.add_message(size, size);
    ++counts.copied;
  }
  return counts;
}

}  // namespace

RewriteCounts PipelinedBackend::run(
  io::BagReader & reader, io::BagWriter & writer, const Processor & processor,
  std::string_view profile_label)
{
  // Resolve the profile flag once on this thread, then hand each stage its own
  // profiler — StageProfiler has no internal locking, so the two threads must
  // not share one instance.
  const bool profiling = profile_value_enabled(std::getenv("BAGWIZ_PROFILE"));
  StageProfiler read_prof(profiling);
  StageProfiler write_prof(profiling);
  BoundedMessageQueue queue(queue_bytes_);

  // Write stage on a worker thread; the read stage stays on the calling thread.
  // A throw inside the writer is latched in the queue (which also unblocks a
  // reader blocked on backpressure) and rethrown from run() after the join.
  std::thread writer_thread([&writer, &queue, &write_prof] {
    try {
      OwnedMessage msg;
      while (queue.pop(msg)) {
        auto s = write_prof.time(Stage::kWrite);
        writer.write(msg.out_topic, msg.timestamp_ns, msg.payload);
      }
    } catch (...) {
      queue.fail(std::current_exception());
    }
  });

  RewriteCounts counts;
  try {
    counts = read_loop(reader, processor, queue, read_prof);
  } catch (...) {
    queue.fail(std::current_exception());
  }
  queue.close();  // no more messages: let the writer drain and stop
  writer_thread.join();

  // Propagate the first fatal error from either stage (reader and writer both
  // route their exceptions through queue.fail()).
  if (auto err = queue.error()) {
    std::rethrow_exception(err);
  }

  // Merge the two per-thread profilers into a single report. read_ns and
  // write_ns overlap in wall-clock (that overlap IS the speedup), so the
  // reported "wall" sum exceeds the real elapsed time by design.
  read_prof.add(Stage::kWrite, std::chrono::nanoseconds(write_prof.totals().write_ns));
  read_prof.report(profile_label);
  return counts;
}

}  // namespace bagwiz::core::pipeline
