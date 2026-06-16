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

#include <cstdint>
#include <string_view>

namespace bagwiz::core::pipeline
{

RewriteCounts SequentialBackend::run(
  io::BagReader & reader, io::BagWriter & writer, const Processor & processor,
  std::string_view profile_label)
{
  StageProfiler prof;
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
    {
      auto s = prof.time(Stage::kWrite);
      writer.write(emit.out_topic, raw.timestamp_ns, raw.payload);
    }
    if (emit.out_topic != raw.topic->name) {
      ++counts.renamed;
    }
    prof.add_message(size, size);
    ++counts.copied;
  }
  prof.report(profile_label);
  return counts;
}

}  // namespace bagwiz::core::pipeline
