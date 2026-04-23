// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/mcap_reader.hpp"

#include "bagwiz/core/logging.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/metadata_yaml.hpp"

// mcap_vendor ships MCAP as a pre-compiled library, so MCAP_IMPLEMENTATION
// must NOT be defined here — the symbols are provided via linkage against
// mcap_vendor::mcap. Defining it would try to pull in .inl files that the
// vendor package does not install.
#include <mcap/reader.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bagwiz::io::detail
{

namespace
{
constexpr const char * kLogger = "bagwiz.io.mcap";

// ---------------------------------------------------------------------------
// Single .mcap file reader.
// ---------------------------------------------------------------------------
class McapFileReader : public BagReader
{
public:
  explicit McapFileReader(const std::filesystem::path & path) : path_(path)
  {
    auto status = reader_.open(path.string());
    if (!status.ok()) {
      throw std::runtime_error("mcap open failed: " + path.string() + ": " + status.message);
    }

    // Prefer reading the summary. AllowFallbackScan makes this correct even
    // for bags that were not properly closed (no summary written), at the
    // cost of a one-time linear scan in that failure mode.
    auto summary_status = reader_.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
    if (!summary_status.ok()) {
      BAGWIZ_LOG_WARN(
        kLogger, "readSummary failed for %s: %s; continuing", path.c_str(),
        summary_status.message.c_str());
    }

    populate_topics();
  }

  std::span<const TopicInfo> topics() const override { return topics_; }

  void set_filter(const ReadFilter & f) override
  {
    if (iteration_started_) {
      throw std::runtime_error("BagReader::set_filter called after iteration started");
    }
    filter_ = f;
  }

  bool next(RawMessage & out) override
  {
    ensure_iterator();
    if (!it_) {
      return false;
    }

    auto & iter = *it_;
    while (iter != view_->end()) {
      const auto & mv = *iter;
      auto idx_it = channel_to_topic_idx_.find(mv.channel->id);
      if (idx_it == channel_to_topic_idx_.end()) {
        // Channel appeared mid-stream without a schema entry we know about.
        ++iter;
        continue;
      }
      out.topic = &topics_[idx_it->second];
      out.timestamp_ns = static_cast<int64_t>(mv.message.logTime);
      out.payload = std::span<const std::byte>(
        reinterpret_cast<const std::byte *>(mv.message.data),
        static_cast<std::size_t>(mv.message.dataSize));
      ++iter;
      return true;
    }
    return false;
  }

  Stats compute_stats() override
  {
    Stats stats;
    const auto & statistics = reader_.statistics();
    if (statistics) {
      stats.from_summary = true;
      stats.total_messages = static_cast<int64_t>(statistics->messageCount);
      stats.start_ns = static_cast<int64_t>(statistics->messageStartTime);
      stats.end_ns = static_cast<int64_t>(statistics->messageEndTime);
      for (const auto & [channel_id, count] : statistics->channelMessageCounts) {
        auto idx_it = channel_to_topic_idx_.find(channel_id);
        if (idx_it != channel_to_topic_idx_.end()) {
          stats.per_topic[topics_[idx_it->second].name] = static_cast<int64_t>(count);
        }
      }
    } else {
      BAGWIZ_LOG_WARN(kLogger, "Statistics unavailable for %s; stats will be zero", path_.c_str());
    }
    return stats;
  }

private:
  void populate_topics()
  {
    const auto & channels = reader_.channels();
    const auto & schemas = reader_.schemas();

    topics_.reserve(channels.size());
    for (const auto & [channel_id, channel] : channels) {
      TopicInfo info;
      info.name = channel->topic;
      info.serialization_format = channel->messageEncoding;
      if (auto schema_it = schemas.find(channel->schemaId); schema_it != schemas.end()) {
        info.type = schema_it->second->name;
      }
      if (auto qos_it = channel->metadata.find("offered_qos_profiles");
          qos_it != channel->metadata.end()) {
        info.offered_qos_profiles = qos_it->second;
      }
      channel_to_topic_idx_[channel_id] = topics_.size();
      topics_.push_back(std::move(info));
    }
  }

  void ensure_iterator()
  {
    if (iteration_started_) {
      return;
    }
    iteration_started_ = true;

    mcap::ReadMessageOptions opts;
    if (filter_.start_ns) {
      opts.startTime = static_cast<mcap::Timestamp>(*filter_.start_ns);
    }
    if (filter_.end_ns) {
      opts.endTime = static_cast<mcap::Timestamp>(*filter_.end_ns);
    }
    if (!filter_.topics.empty()) {
      auto topics_copy = filter_.topics;
      opts.topicFilter = [topics_copy = std::move(topics_copy)](std::string_view t) {
        return std::find(topics_copy.begin(), topics_copy.end(), std::string(t)) !=
               topics_copy.end();
      };
    }

    // mcap requires a problem callback alongside options; surface problems
    // as warnings and continue.
    auto problem_cb = [](const mcap::Status & s) {
      BAGWIZ_LOG_WARN(kLogger, "mcap read problem: %s", s.message.c_str());
    };

    view_ = std::make_unique<mcap::LinearMessageView>(reader_.readMessages(problem_cb, opts));
    it_.emplace(view_->begin());
  }

  std::filesystem::path path_;
  mcap::McapReader reader_;
  std::vector<TopicInfo> topics_;
  std::unordered_map<uint16_t, std::size_t> channel_to_topic_idx_;

  ReadFilter filter_;
  bool iteration_started_ = false;
  std::unique_ptr<mcap::LinearMessageView> view_;
  std::optional<mcap::LinearMessageView::Iterator> it_;
};

// ---------------------------------------------------------------------------
// Multi-shard reader: concatenates McapFileReaders in declared order.
// rosbag2 writes shards in time order without overlap so simple concat is
// equivalent to time-ordered merge.
// ---------------------------------------------------------------------------
class McapShardReader : public BagReader
{
public:
  McapShardReader(
    std::vector<std::unique_ptr<McapFileReader>> shards, std::vector<TopicInfo> topics)
  : shards_(std::move(shards)), topics_(std::move(topics))
  {
  }

  std::span<const TopicInfo> topics() const override { return topics_; }

  void set_filter(const ReadFilter & f) override
  {
    for (auto & s : shards_) {
      s->set_filter(f);
    }
  }

  bool next(RawMessage & out) override
  {
    while (current_ < shards_.size()) {
      if (shards_[current_]->next(out)) {
        // Remap the shard-local TopicInfo pointer to our owned vector so
        // callers see a stable pointer for the whole bag.
        for (auto & t : topics_) {
          if (t.name == out.topic->name) {
            out.topic = &t;
            return true;
          }
        }
        return true;
      }
      ++current_;
    }
    return false;
  }

  Stats compute_stats() override
  {
    Stats combined;
    combined.from_summary = true;
    bool first = true;
    for (auto & s : shards_) {
      auto st = s->compute_stats();
      combined.from_summary = combined.from_summary && st.from_summary;
      combined.total_messages += st.total_messages;
      if (first || st.start_ns < combined.start_ns) {
        combined.start_ns = st.start_ns;
      }
      if (first || st.end_ns > combined.end_ns) {
        combined.end_ns = st.end_ns;
      }
      first = false;
      for (const auto & [k, v] : st.per_topic) {
        combined.per_topic[k] += v;
      }
    }
    return combined;
  }

private:
  std::vector<std::unique_ptr<McapFileReader>> shards_;
  std::vector<TopicInfo> topics_;
  std::size_t current_ = 0;
};

}  // namespace

std::unique_ptr<BagReader> open_mcap_file(const std::filesystem::path & path)
{
  return std::make_unique<McapFileReader>(path);
}

std::unique_ptr<BagReader> open_mcap_directory(const std::filesystem::path & dir)
{
  const auto metadata_path = dir / "metadata.yaml";
  auto md = load_metadata_yaml(metadata_path);

  std::vector<std::unique_ptr<McapFileReader>> shards;
  shards.reserve(md.relative_file_paths.size());
  for (const auto & rel : md.relative_file_paths) {
    shards.push_back(std::make_unique<McapFileReader>(dir / rel));
  }

  std::vector<TopicInfo> topics;
  if (!md.topics.empty()) {
    topics = std::move(md.topics);
  } else if (!shards.empty()) {
    auto shard_topics = shards.front()->topics();
    topics.assign(shard_topics.begin(), shard_topics.end());
  }

  return std::make_unique<McapShardReader>(std::move(shards), std::move(topics));
}

}  // namespace bagwiz::io::detail
