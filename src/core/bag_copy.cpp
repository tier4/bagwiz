// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/bag_copy.hpp"

#include "bagwiz/core/pipeline/stage_profiler.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

namespace bagwiz::core
{

BagCopyCounts bag_copy_filtered(
  io::BagReader & reader, io::BagWriter & writer, const std::unordered_set<std::string> & suppress,
  std::string_view profile_label)
{
  pipeline::StageProfiler prof;
  BagCopyCounts counts;
  io::RawMessage raw;
  while (true) {
    bool got = false;
    {
      auto s = prof.time(pipeline::Stage::kRead);
      got = reader.next(raw);
    }
    if (!got) {
      break;
    }
    // raw.topic is non-null when next() returns true (zero-copy view
    // documented by BagReader::next).
    const auto size = static_cast<std::uint64_t>(raw.payload.size());
    if (suppress.count(raw.topic->name) != 0) {
      prof.add_message(size, 0);  // read+decompressed but not written
      ++counts.suppressed;
      continue;
    }
    {
      auto s = prof.time(pipeline::Stage::kWrite);
      writer.write(raw.topic->name, raw.timestamp_ns, raw.payload);
    }
    prof.add_message(size, size);
    ++counts.copied;
  }
  prof.report(profile_label);
  return counts;
}

BagCopyRenameCounts bag_copy_renamed(
  io::BagReader & reader, io::BagWriter & writer,
  const std::unordered_map<std::string, std::string> & rename, std::string_view profile_label)
{
  pipeline::StageProfiler prof;
  BagCopyRenameCounts counts;
  io::RawMessage raw;
  while (true) {
    bool got = false;
    {
      auto s = prof.time(pipeline::Stage::kRead);
      got = reader.next(raw);
    }
    if (!got) {
      break;
    }
    // raw.topic is non-null when next() returns true (zero-copy view
    // documented by BagReader::next).
    const auto size = static_cast<std::uint64_t>(raw.payload.size());
    const auto it = rename.find(raw.topic->name);
    {
      auto s = prof.time(pipeline::Stage::kWrite);
      if (it != rename.end()) {
        writer.write(it->second, raw.timestamp_ns, raw.payload);
        ++counts.renamed;
      } else {
        writer.write(raw.topic->name, raw.timestamp_ns, raw.payload);
      }
    }
    prof.add_message(size, size);
    ++counts.copied;
  }
  prof.report(profile_label);
  return counts;
}

}  // namespace bagwiz::core
