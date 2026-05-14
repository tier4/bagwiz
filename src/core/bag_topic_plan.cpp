// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/bag_topic_plan.hpp"

#include "bagwiz/io/bag_io.hpp"

#include <cstdint>
#include <span>
#include <string>
#include <string_view>

namespace bagwiz::core
{

TopicWriteDecision decide_topic_write(
  std::span<const io::TopicInfo> existing_topics, std::string_view target_topic,
  std::string_view expected_type, std::int64_t existing_count, bool force)
{
  TopicWriteDecision out;
  out.existing_count = existing_count;

  // Concatenate reason messages without fmt::format so this translation
  // unit compiles cleanly under both header-only fmt (clang-tidy's view
  // of bagwiz_core) and the regular linked fmt mode.
  const std::string target_str(target_topic);

  // The helper used to live in an anon namespace as `find_topic` returning
  // `const TopicInfo *`, but cppcheck (jazzy) misreads `std::span` as
  // owning its data and flags returning `&t` as `returnDanglingLifetime`.
  // Inlining the search here keeps the analysis local and avoids the
  // false positive without needing a suppression.
  const io::TopicInfo * existing = nullptr;
  for (const auto & t : existing_topics) {
    if (t.name == target_topic) {
      existing = &t;
      break;
    }
  }
  if (existing == nullptr) {
    out.action = TopicWriteAction::kDeclareNew;
    out.reason = "Topic '" + target_str + "' is absent from the destination; declaring it new.";
    return out;
  }

  if (existing->type != expected_type) {
    out.action = TopicWriteAction::kTypeMismatch;
    out.reason = "Topic '" + target_str + "' exists with type '" + existing->type +
                 "', but the rewrite expects '" + std::string(expected_type) +
                 "'; refusing to write incompatible payloads.";
    return out;
  }

  if (existing_count == 0) {
    out.action = TopicWriteAction::kDeclareKeep;
    out.reason = "Topic '" + target_str +
                 "' exists in the destination with zero messages; keeping its declaration.";
    return out;
  }

  if (!force) {
    out.action = TopicWriteAction::kConflictAbort;
    out.reason = "Destination already has " + std::to_string(existing_count) + " message(s) on '" +
                 target_str + "'. Pass --force to overwrite (existing messages will be dropped).";
    return out;
  }

  out.action = TopicWriteAction::kDeclareAndSuppress;
  out.reason = "--force: dropping " + std::to_string(existing_count) + " message(s) on '" +
               target_str + "' and replacing with new payloads.";
  return out;
}

}  // namespace bagwiz::core
