// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__BAG__BAG_TOPIC_PLAN_HPP_
#define BAGWIZ__CORE__BAG__BAG_TOPIC_PLAN_HPP_

#include "bagwiz/io/bag_io.hpp"

#include <cstdint>
#include <span>
#include <string>
#include <string_view>

// Conflict-resolution policy for "introduce a topic into a bag we are
// rewriting": given the destination bag's existing topic list (with
// types) and an existing-message count for the target topic, decide
// whether to declare it new, keep an empty declaration, replace
// existing messages under --force, or abort.
//
// This codifies a shared topic-rewrite policy so rewrite-style
// commands (e.g. `bagwiz traj join`) can apply identical semantics
// without duplicating the branching.
namespace bagwiz::core
{

// One of five mutually-exclusive outcomes. `reason` on
// TopicWriteDecision carries the human-readable explanation used for
// log messages; it is not a stable machine-parseable string.
enum class TopicWriteAction {
  // Target topic is absent from the destination — declare it from
  // scratch using a synthesised TopicInfo provided by the caller.
  kDeclareNew,

  // Target topic exists in the destination but carries zero messages.
  // Keep the existing TopicInfo as-is (so QoS / schema metadata is
  // preserved on round-trip), and append the new payloads.
  kDeclareKeep,

  // Target topic exists with messages and `force` is true — declare
  // the topic, but suppress the existing payloads during stream-copy
  // and replace them with the new payloads.
  kDeclareAndSuppress,

  // Target topic exists with messages and `force` is false. The
  // caller must surface `reason` and abort.
  kConflictAbort,

  // Target topic exists with a different message type than expected.
  // Always an error — `force` does not relax this.
  kTypeMismatch,
};

struct TopicWriteDecision
{
  TopicWriteAction action = TopicWriteAction::kDeclareNew;
  std::string reason;
  std::int64_t existing_count = 0;
};

// Decide how a rewrite should treat `target_topic`.
//
// `existing_topics`   : destination bag's topic list (snapshot).
// `target_topic`      : name of the topic the rewrite wants to write.
// `expected_type`     : message type the rewrite plans to emit on it.
// `existing_count`    : number of existing messages on that topic in
//                       the destination (0 if absent or empty).
// `force`             : whether the user passed --force.
//
// Pure function — no I/O. The caller is responsible for stream-copy
// suppression, writer declarations, and surfacing `reason` to the
// user.
TopicWriteDecision decide_topic_write(
  std::span<const io::TopicInfo> existing_topics, std::string_view target_topic,
  std::string_view expected_type, std::int64_t existing_count, bool force);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__BAG__BAG_TOPIC_PLAN_HPP_
