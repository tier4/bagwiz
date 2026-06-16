// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/bag_copy.hpp"

#include "bagwiz/io/bag_io.hpp"

#include <string>
#include <unordered_map>
#include <unordered_set>

namespace bagwiz::core
{

BagCopyCounts bag_copy_filtered(
  io::BagReader & reader, io::BagWriter & writer, const std::unordered_set<std::string> & suppress)
{
  BagCopyCounts counts;
  io::RawMessage raw;
  while (reader.next(raw)) {
    // raw.topic is non-null when next() returns true (zero-copy view
    // documented by BagReader::next).
    if (suppress.count(raw.topic->name) != 0) {
      ++counts.suppressed;
      continue;
    }
    writer.write(raw.topic->name, raw.timestamp_ns, raw.payload);
    ++counts.copied;
  }
  return counts;
}

BagCopyRenameCounts bag_copy_renamed(
  io::BagReader & reader, io::BagWriter & writer,
  const std::unordered_map<std::string, std::string> & rename)
{
  BagCopyRenameCounts counts;
  io::RawMessage raw;
  while (reader.next(raw)) {
    // raw.topic is non-null when next() returns true (zero-copy view
    // documented by BagReader::next).
    const auto it = rename.find(raw.topic->name);
    if (it != rename.end()) {
      writer.write(it->second, raw.timestamp_ns, raw.payload);
      ++counts.renamed;
    } else {
      writer.write(raw.topic->name, raw.timestamp_ns, raw.payload);
    }
    ++counts.copied;
  }
  return counts;
}

}  // namespace bagwiz::core
