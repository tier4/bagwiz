// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef IO__SHARD_MULTIPLEXER_HPP_
#define IO__SHARD_MULTIPLEXER_HPP_

#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/metadata_yaml.hpp"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bagwiz::io::detail
{

// Multi-shard directory reader: concatenates per-shard file readers in
// declared order. Matches rosbag2's monotonic shard ordering (shards are
// written in time order without overlap, so simple concat is equivalent to a
// time-ordered merge). Shared by the MCAP and SQLite3 directory readers;
// src-local (lives with the reader sources, not installed).
//
// Shards are opened lazily. When metadata.yaml carries a complete summary
// (total count, start time, duration, per-topic counts) and the topic list,
// `topics()` and the compute_*() methods answer from metadata alone —
// `bagwiz ls` against a multi-shard bag avoids opening every shard.
//
// `FileReader` is the per-shard reader type (McapFileReader /
// SqliteFileReader). The genuinely format-specific behavior stays in the
// derived class through three hooks:
//   - open_shard(): how one shard file becomes a FileReader (decompressor
//     sharing, `.db3.zstd` envelope handling),
//   - on_topic_remapped(): per-message fixup after a message's topic pointer
//     is remapped to the bag-level TopicInfo (MCAP backfills schema bytes;
//     SQLite has nothing to backfill),
//   - scan_from_summary(): the Stats::from_summary policy when compute_stats()
//     must combine per-shard scans (no metadata summary): SQLite shard stats
//     read raw tables so the combined flag is always false; MCAP shard stats
//     read summary records so the combined flag ANDs the per-shard flags.
template <typename FileReader>
class ShardMultiplexer : public BagReader
{
public:
  ShardMultiplexer(
    std::filesystem::path dir, std::vector<std::filesystem::path> shard_rel_paths,
    std::vector<TopicInfo> topics, BagMetadata metadata)
  : dir_(std::move(dir)),
    shard_rel_paths_(std::move(shard_rel_paths)),
    topics_(std::move(topics)),
    metadata_(std::move(metadata))
  {
    shards_.resize(shard_rel_paths_.size());
  }

  std::span<const TopicInfo> topics() const override
  {
    if (!topics_.empty()) {
      return topics_;
    }
    // Last-resort fallback: derive topics from the first shard. Cached into
    // topics_ so subsequent calls stay O(1).
    if (!shard_rel_paths_.empty()) {
      const auto & first = ensure_shard(0);
      auto shard_topics = first.topics();
      topics_.assign(shard_topics.begin(), shard_topics.end());
    }
    return topics_;
  }

  void set_filter(const ReadFilter & f) override
  {
    if (iteration_started_) {
      throw std::runtime_error("BagReader::set_filter called after iteration started");
    }
    pending_filter_ = f;
    has_pending_filter_ = true;
  }

  bool next(RawMessage & out) override
  {
    iteration_started_ = true;
    while (current_ < shard_rel_paths_.size()) {
      auto & shard = ensure_shard(current_);
      if (shards_filter_applied_.size() <= current_) {
        shards_filter_applied_.resize(current_ + 1, false);
      }
      if (has_pending_filter_ && !shards_filter_applied_[current_]) {
        shard.set_filter(pending_filter_);
        shards_filter_applied_[current_] = true;
      }
      if (shard.next(out)) {
        // Remap the shard-local TopicInfo pointer to our owned vector so
        // callers see a stable pointer for the whole bag.
        for (auto & t : topics_) {
          if (t.name == out.topic->name) {
            on_topic_remapped(t, *out.topic);
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
    if (metadata_.has_summary) {
      Stats stats;
      stats.from_summary = true;
      stats.total_messages = metadata_.total_messages;
      stats.start_ns = metadata_.start_ns;
      stats.end_ns = metadata_.end_ns;
      stats.per_topic = metadata_.per_topic_counts;
      return stats;
    }

    Stats combined;
    bool first = true;
    std::vector<bool> shard_flags;
    shard_flags.reserve(shard_rel_paths_.size());
    for (std::size_t i = 0; i < shard_rel_paths_.size(); ++i) {
      auto st = ensure_shard(i).compute_stats();
      shard_flags.push_back(st.from_summary);
      combined.total_messages += st.total_messages;
      if (first || st.start_ns < combined.start_ns) {
        combined.start_ns = st.start_ns;
      }
      if (first || st.end_ns > combined.end_ns) {
        combined.end_ns = st.end_ns;
      }
      first = false;
      // cppcheck-suppress unassignedVariable
      for (const auto & [k, v] : st.per_topic) {
        combined.per_topic[k] += v;
      }
    }
    combined.from_summary = scan_from_summary(shard_flags);
    return combined;
  }

  std::unordered_map<std::string, int64_t> compute_topic_counts(
    std::span<const std::string> names) override
  {
    std::unordered_map<std::string, int64_t> result;
    if (names.empty()) {
      return result;
    }

    if (metadata_.has_summary) {
      for (const auto & topic : names) {
        if (auto it = metadata_.per_topic_counts.find(topic);
            it != metadata_.per_topic_counts.end()) {
          result[topic] = it->second;
        }
      }
      return result;
    }

    for (std::size_t i = 0; i < shard_rel_paths_.size(); ++i) {
      auto shard_counts = ensure_shard(i).compute_topic_counts(names);
      // cppcheck-suppress unassignedVariable
      for (const auto & [k, v] : shard_counts) {
        result[k] += v;
      }
    }
    return result;
  }

  TimeExtent compute_time_extent() override
  {
    TimeExtent extent;
    if (metadata_.has_summary) {
      extent.start_ns = metadata_.start_ns;
      extent.end_ns = metadata_.end_ns;
      extent.has_data = true;
      return extent;
    }

    for (std::size_t i = 0; i < shard_rel_paths_.size(); ++i) {
      auto shard_extent = ensure_shard(i).compute_time_extent();
      if (!shard_extent.has_data) {
        continue;
      }
      if (!extent.has_data || shard_extent.start_ns < extent.start_ns) {
        extent.start_ns = shard_extent.start_ns;
      }
      if (!extent.has_data || shard_extent.end_ns > extent.end_ns) {
        extent.end_ns = shard_extent.end_ns;
      }
      extent.has_data = true;
    }
    return extent;
  }

protected:
  // Open one shard file as a FileReader. Called at most once per shard (the
  // result is cached in shards_); `shard_path` is dir_ / shard_rel_paths_[i].
  virtual std::unique_ptr<FileReader> open_shard(
    const std::filesystem::path & shard_path) const = 0;

  // Per-message hook after a message's topic was matched to the bag-level
  // TopicInfo `bag_topic`: format-specific fixup pulled from the shard's own
  // TopicInfo `shard_topic`. Default no-op (SQLite); MCAP backfills schema
  // bytes the metadata-derived TopicInfo may lack.
  virtual void on_topic_remapped(TopicInfo & /*bag_topic*/, const TopicInfo & /*shard_topic*/) const
  {
  }

  // Stats::from_summary for the shard-scan combination of compute_stats()
  // when no metadata summary is available. `shard_flags` carries each shard's
  // own from_summary in shard order (empty when the bag has no shards).
  virtual bool scan_from_summary(const std::vector<bool> & shard_flags) const = 0;

  FileReader & ensure_shard(std::size_t i) const
  {
    if (!shards_[i]) {
      shards_[i] = open_shard(dir_ / shard_rel_paths_[i]);
    }
    return *shards_[i];
  }

  std::filesystem::path dir_;
  std::vector<std::filesystem::path> shard_rel_paths_;
  // mutable because topics() and ensure_shard() are logically const for
  // callers but cache lazily-derived state.
  mutable std::vector<TopicInfo> topics_;
  mutable std::vector<std::unique_ptr<FileReader>> shards_;
  BagMetadata metadata_;

private:
  ReadFilter pending_filter_;
  bool has_pending_filter_ = false;
  std::vector<bool> shards_filter_applied_;
  std::size_t current_ = 0;
  bool iteration_started_ = false;
};

}  // namespace bagwiz::io::detail

#endif  // IO__SHARD_MULTIPLEXER_HPP_
