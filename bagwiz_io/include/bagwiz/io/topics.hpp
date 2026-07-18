// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__IO__TOPICS_HPP_
#define BAGWIZ__IO__TOPICS_HPP_

#include "bagwiz/io/bag_io.hpp"

#include <filesystem>
#include <string>
#include <vector>

// Small utilities over a reader's topic list, covering the two patterns the
// commands repeat: snapshotting topic names so they outlive the reader, and
// looking a topic up by name with a uniform "not present" error.
namespace bagwiz::io
{

// Copy the reader's topic names into a vector, preserving the reader's order.
// Use when the names must outlive the reader: the span returned by
// BagReader::topics() is invalidated once the reader is destroyed.
std::vector<std::string> snapshot_topic_names(const BagReader & reader);

// Return the TopicInfo for `name`, or nullptr when the bag has no such topic.
// The pointer aliases the reader's internal topic list and stays valid until
// the reader is destroyed. Never logs — callers that want the uniform error
// message use find_topic_or_log().
const TopicInfo * find_topic(const BagReader & reader, const std::string & name);

// Return the TopicInfo for `name`. When absent, log exactly
// "Topic '%s' is not present in %s" (name, bag_path) to `logger` and return
// nullptr. `bag_path` is only used for the message text. The returned pointer
// aliases the reader's internal topic list (see find_topic()).
const TopicInfo * find_topic_or_log(
  const BagReader & reader, const std::string & name, const std::filesystem::path & bag_path,
  const char * logger);

}  // namespace bagwiz::io

#endif  // BAGWIZ__IO__TOPICS_HPP_
