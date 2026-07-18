// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/topics.hpp"

#include "bagwiz/core/base/logging.hpp"

#include <string>
#include <vector>

namespace bagwiz::io
{

std::vector<std::string> snapshot_topic_names(const BagReader & reader)
{
  std::vector<std::string> names;
  names.reserve(reader.topics().size());
  for (const auto & t : reader.topics()) {
    names.push_back(t.name);
  }
  return names;
}

const TopicInfo * find_topic(const BagReader & reader, const std::string & name)
{
  for (const auto & t : reader.topics()) {
    if (t.name == name) {
      return &t;
    }
  }
  return nullptr;
}

const TopicInfo * find_topic_or_log(
  const BagReader & reader, const std::string & name, const std::filesystem::path & bag_path,
  const char * logger)
{
  const TopicInfo * info = find_topic(reader, name);
  if (info == nullptr) {
    BAGWIZ_LOG_ERROR(logger, "Topic '%s' is not present in %s", name.c_str(), bag_path.c_str());
  }
  return info;
}

}  // namespace bagwiz::io
