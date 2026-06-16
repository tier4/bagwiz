// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/bag_copy.hpp"

#include "bagwiz/core/pipeline/rewrite_backend.hpp"
#include "bagwiz/core/pipeline/sequential_backend.hpp"
#include "bagwiz/core/pipeline/topic_router.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

// bag_copy_filtered / bag_copy_renamed are thin adapters over the shared
// pipeline seam: they build the matching pure-copy Processor and run it on the
// zero-copy SequentialBackend, then map the generic RewriteCounts back onto the
// command-facing count structs. The historical read/process/write loop now
// lives in SequentialBackend, so these keep working byte-identically while the
// trio (and future commands) gain access to alternative backends.
namespace bagwiz::core
{

BagCopyCounts bag_copy_filtered(
  io::BagReader & reader, io::BagWriter & writer, const std::unordered_set<std::string> & suppress,
  std::string_view profile_label)
{
  pipeline::SuppressRouter router(suppress);
  pipeline::SequentialBackend backend;
  const auto counts = pipeline::run_pipeline(reader, writer, router, backend, profile_label);
  return BagCopyCounts{counts.copied, counts.dropped};
}

BagCopyRenameCounts bag_copy_renamed(
  io::BagReader & reader, io::BagWriter & writer,
  const std::unordered_map<std::string, std::string> & rename, std::string_view profile_label)
{
  pipeline::RenameRouter router(rename);
  pipeline::SequentialBackend backend;
  const auto counts = pipeline::run_pipeline(reader, writer, router, backend, profile_label);
  return BagCopyRenameCounts{counts.copied, counts.renamed};
}

}  // namespace bagwiz::core
