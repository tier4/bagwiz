// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pipeline/sequential_backend.hpp"

#include "bagwiz/core/pipeline/rewrite_backend.hpp"
#include "bagwiz/core/pipeline/stage_profiler.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

namespace bagwiz::core::pipeline
{

RewriteCounts SequentialBackend::run(
  io::BagReader & reader, io::BagWriter & writer, const Processor & processor,
  std::string_view profile_label)
{
  StageProfiler prof;
  RewriteCounts counts;
  const bool transforming = processor.transforms();
  std::vector<std::byte> xform_buf;  // reused across transformed messages
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
    const auto in_size = static_cast<std::uint64_t>(raw.payload.size());
    if (!processor.keep_message(raw)) {
      prof.add_message(in_size, 0);  // read+decompressed but not written
      ++counts.dropped;
      continue;
    }
    const Emit emit = processor.route(raw.topic->name);
    if (!emit.keep) {
      prof.add_message(in_size, 0);  // read+decompressed but not written
      ++counts.dropped;
      continue;
    }

    // For a pure-copy processor the reader's payload is written verbatim (zero
    // copy). A transforming processor produces the output bytes (kWrite), opts
    // to forward verbatim (kPassthrough), or drops the message (kSkip).
    std::span<const std::byte> out_payload = raw.payload;
    bool transformed = false;
    if (transforming) {
      xform_buf.clear();
      const TransformAction action = [&] {
        auto s = prof.time(Stage::kProcess);
        return processor.transform(raw.topic->name, raw.payload, xform_buf);
      }();
      if (action == TransformAction::kSkip) {
        prof.add_message(in_size, 0);
        ++counts.skipped;
        continue;
      }
      if (action == TransformAction::kWrite) {
        out_payload = xform_buf;
        transformed = true;
      }
    }

    {
      auto s = prof.time(Stage::kWrite);
      writer.write(emit.out_topic, raw.timestamp_ns, out_payload);
    }
    if (emit.out_topic != raw.topic->name) {
      ++counts.renamed;
    }
    if (transformed) {
      ++counts.transformed;
    }
    prof.add_message(in_size, static_cast<std::uint64_t>(out_payload.size()));
    ++counts.copied;
  }
  prof.report(profile_label);
  return counts;
}

}  // namespace bagwiz::core::pipeline
