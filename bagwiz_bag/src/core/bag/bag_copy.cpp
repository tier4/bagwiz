// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/bag/bag_copy.hpp"

#include "bagwiz/core/pipeline/backend_select.hpp"
#include "bagwiz/core/pipeline/rewrite_backend.hpp"
#include "bagwiz/core/pipeline/topic_router.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <cstdint>
#include <exception>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// bag_copy_filtered / bag_copy_renamed are thin adapters over the shared
// pipeline seam: they build the matching pure-copy Processor, run it on the
// caller-selected Backend (Sequential by default; the pure-copy trio asks for
// Pipelined, overridable via BAGWIZ_BACKEND), then map the generic RewriteCounts
// back onto the command-facing count structs. The read/process/write loop lives
// in the backends, so these keep working byte-identically while every caller can
// pick its acceleration strategy.
namespace bagwiz::core
{

namespace
{

// SuppressRouter plus a caller-supplied per-message predicate (see
// Processor::keep_message). Only instantiated when `keep` is set, so the
// predicate-free path keeps the plain router's virtual-call profile.
class PredicateSuppressRouter : public pipeline::SuppressRouter
{
public:
  PredicateSuppressRouter(
    const std::unordered_set<std::string> & suppress, const MessagePredicate & keep)
  : pipeline::SuppressRouter(suppress), keep_(keep)
  {
  }

  [[nodiscard]] bool keep_message(const io::RawMessage & msg) const override { return keep_(msg); }

private:
  const MessagePredicate & keep_;
};

}  // namespace

BagCopyCounts bag_copy_filtered(
  io::BagReader & reader, io::BagWriter & writer, const std::unordered_set<std::string> & suppress,
  std::string_view profile_label, pipeline::BackendKind backend, const MessagePredicate & keep)
{
  const auto backend_impl = pipeline::make_backend(backend);
  if (keep) {
    PredicateSuppressRouter router(suppress, keep);
    const auto counts =
      pipeline::run_pipeline(reader, writer, router, *backend_impl, profile_label);
    return BagCopyCounts{counts.copied, counts.dropped};
  }
  pipeline::SuppressRouter router(suppress);
  const auto counts = pipeline::run_pipeline(reader, writer, router, *backend_impl, profile_label);
  return BagCopyCounts{counts.copied, counts.dropped};
}

std::optional<std::int64_t> count_topic_messages(
  io::BagReader & reader, const std::unordered_set<std::string> & topics)
{
  if (topics.empty()) {
    return 0;
  }
  const std::vector<std::string> names(topics.begin(), topics.end());
  std::int64_t total = 0;
  try {
    // cppcheck-suppress unassignedVariable
    for (const auto & [name, count] : reader.compute_topic_counts(names)) {
      (void)name;
      total += count;
    }
  } catch (const std::exception &) {
    return std::nullopt;
  }
  return total;
}

BagCopyRenameCounts bag_copy_renamed(
  io::BagReader & reader, io::BagWriter & writer,
  const std::unordered_map<std::string, std::string> & rename, std::string_view profile_label,
  pipeline::BackendKind backend)
{
  pipeline::RenameRouter router(rename);
  const auto backend_impl = pipeline::make_backend(backend);
  const auto counts = pipeline::run_pipeline(reader, writer, router, *backend_impl, profile_label);
  return BagCopyRenameCounts{counts.copied, counts.renamed};
}

}  // namespace bagwiz::core
