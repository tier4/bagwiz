// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "pcd_concat_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/duration_parse.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/pointcloud/cloud_concat.hpp"

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

std::optional<std::vector<std::int64_t>> parse_stamp_offsets(
  const std::vector<std::string> & entries,
  const std::unordered_map<std::string, std::size_t> & topic_index, const char * logger)
{
  std::vector<std::int64_t> offsets(topic_index.size(), 0);
  for (const auto & entry : entries) {
    const auto eq = entry.find('=');
    if (eq == std::string::npos) {
      BAGWIZ_LOG_ERROR(
        logger, "pcd concat: --stamp-offset must be topic=value (got '%s')", entry.c_str());
      return std::nullopt;
    }
    const std::string topic = entry.substr(0, eq);
    const std::string value = entry.substr(eq + 1);
    const auto it = topic_index.find(topic);
    if (it == topic_index.end()) {
      BAGWIZ_LOG_ERROR(
        logger, "pcd concat: --stamp-offset topic '%s' is not in --pcd", topic.c_str());
      return std::nullopt;
    }
    const auto ns = core::parse_duration_ns(value);
    if (!ns.has_value()) {
      BAGWIZ_LOG_ERROR(
        logger, "pcd concat: could not parse --stamp-offset value '%s' (e.g. 50ms, -500ns, 0.05s)",
        value.c_str());
      return std::nullopt;
    }
    offsets[it->second] = *ns;
  }
  return offsets;
}

ConcatAssembler::ConcatAssembler(
  const std::vector<TopicState> & topics, std::vector<core::pointcloud::SyncGroup> groups,
  std::string target_frame)
: num_topics_(topics.size()), groups_(std::move(groups)), target_frame_(std::move(target_frame))
{
  extrinsics_.reserve(num_topics_);
  refs_.resize(num_topics_);
  for (std::size_t i = 0; i < num_topics_; ++i) {
    extrinsics_.push_back(topics[i].extrinsic);
    refs_[i].resize(topics[i].stamps_ns.size());
  }
  // (topic i, msg index) -> groups referencing it; and per-group remaining picks.
  group_remaining_.assign(groups_.size(), 0);
  for (std::size_t g = 0; g < groups_.size(); ++g) {
    for (std::size_t t = 0; t < num_topics_; ++t) {
      if (groups_[g].picks[t].has_value()) {
        refs_[t][*groups_[g].picks[t]].push_back(g);
        ++group_remaining_[g];
      }
    }
  }
  fired_.assign(groups_.size(), 0);
  counters_.matched.assign(num_topics_, 0);
  counters_.parse_fail.assign(num_topics_, 0);
  counters_.transform_fail.assign(num_topics_, 0);
}

ConcatAssembler::IngestResult ConcatAssembler::on_message(
  std::size_t topic, std::size_t index, std::span<const std::byte> payload)
{
  IngestResult result;
  if (index >= refs_[topic].size() || refs_[topic][index].empty()) {
    return result;  // this message is not picked by any group
  }

  // parse + transform + stash
  auto parsed = core::pointcloud::parse_pointcloud2(payload);
  if (parsed.ok()) {
    auto cloud = std::move(*parsed.cloud);
    const auto tr = core::pointcloud::transform_cloud_xyz(cloud, extrinsics_[topic]);
    if (tr.ok) {
      Cached c;
      c.header_stamp_ns = cloud.timestamp_ns;
      c.refcount = refs_[topic][index].size();
      c.cloud = std::move(cloud);
      cache_.emplace(key(topic, index), std::move(c));
    } else {
      ++counters_.transform_fail[topic];
    }
  } else {
    ++counters_.parse_fail[topic];
  }

  // notify referencing groups; fire the ones now complete
  for (const std::size_t g : refs_[topic][index]) {
    if (group_remaining_[g] > 0) {
      --group_remaining_[g];
    }
    if (group_remaining_[g] != 0 || fired_[g] != 0) {
      continue;
    }
    if (!fire(g, result)) {
      return result;
    }
  }
  return result;
}

bool ConcatAssembler::fire(std::size_t group, IngestResult & result)
{
  fired_[group] = 1;
  // assemble inputs in --pcd order from cached picks
  std::vector<core::pointcloud::ConcatInput> inputs;
  for (std::size_t k = 0; k < num_topics_; ++k) {
    if (!groups_[group].picks[k].has_value()) {
      continue;
    }
    const auto ci = cache_.find(key(k, *groups_[group].picks[k]));
    if (ci == cache_.end()) {
      continue;  // parse/transform failed -> partial
    }
    inputs.push_back({&ci->second.cloud, ci->second.header_stamp_ns});
    ++counters_.matched[k];
  }
  if (inputs.size() < num_topics_) {
    ++counters_.partial_groups;
  }
  if (!inputs.empty()) {
    const auto merged =
      core::pointcloud::concat_clouds(inputs, groups_[group].output_stamp_ns, target_frame_);
    if (!merged.ok()) {
      result.error = merged.error;
      return false;
    }
    FiredOutput output;
    output.stamp_ns = groups_[group].output_stamp_ns;
    output.payload = core::pointcloud::serialize_pointcloud2(*merged.cloud);
    result.fired.push_back(std::move(output));
    ++counters_.written_groups;
  }
  release(group);
  return true;
}

void ConcatAssembler::release(std::size_t group)
{
  // release cached picks whose refcount hits zero
  for (std::size_t k = 0; k < num_topics_; ++k) {
    if (!groups_[group].picks[k].has_value()) {
      continue;
    }
    const auto ci = cache_.find(key(k, *groups_[group].picks[k]));
    if (ci != cache_.end() && --ci->second.refcount == 0) {
      cache_.erase(ci);
    }
  }
}

}  // namespace bagwiz::commands
