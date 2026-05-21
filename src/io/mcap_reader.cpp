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
      // Copy the payload into a stable member buffer BEFORE advancing the
      // iterator. mcap::LinearMessageView::Iterator may release the
      // current chunk when incremented, invalidating mv.message.data. The
      // RawMessage contract says the span is valid until the next next()
      // call, which matches: we overwrite payload_buf_ on each call.
      const auto * src = reinterpret_cast<const std::byte *>(mv.message.data);
      const auto size = static_cast<std::size_t>(mv.message.dataSize);
      payload_buf_.assign(src, src + size);
      out.payload = std::span<const std::byte>(payload_buf_.data(), payload_buf_.size());
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
    // cppcheck-suppress unassignedVariable
    for (const auto & [channel_id, channel] : channels) {
      TopicInfo info;
      info.name = channel->topic;
      info.serialization_format = channel->messageEncoding;
      if (auto schema_it = schemas.find(channel->schemaId); schema_it != schemas.end()) {
        const auto & schema = *schema_it->second;
        info.type = schema.name;
        info.schema_encoding = schema.encoding;
        // schema.data is std::vector<std::byte>; ros2msg / ros2idl payloads
        // are UTF-8 text. Reinterpret the bytes as char and copy into a
        // string. Empty when the writer didn't embed schema bytes (legacy
        // bagwiz output, manually-crafted MCAPs).
        if (!schema.data.empty()) {
          info.schema_text.assign(
            reinterpret_cast<const char *>(schema.data.data()), schema.data.size());
        }
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

    // Prefer the indexed (log-time-ordered) reader when the bag carries
    // both a summary and chunk indexes: it lets mcap_cpp's
    // IndexedMessageReader prune whole chunks that do not contain any
    // matching channel, which is a large speedup for sparse topics like
    // /tf_static. Fall back to FileOrder when those preconditions are
    // missing (truncated/unchunked bags, or summary-via-fallback-scan
    // recoveries that produced statistics but no ChunkIndex records).
    const bool indexed_ok = reader_.statistics().has_value() && !reader_.chunkIndexes().empty();
    opts.readOrder = indexed_ok ? mcap::ReadMessageOptions::ReadOrder::LogTimeOrder
                                : mcap::ReadMessageOptions::ReadOrder::FileOrder;

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
  // Stable backing store for the payload returned by next(). Lives as
  // long as the reader; each next() overwrites it.
  std::vector<std::byte> payload_buf_;
};

// ---------------------------------------------------------------------------
// Multi-shard reader: concatenates McapFileReaders in declared order.
// rosbag2 writes shards in time order without overlap so simple concat is
// equivalent to time-ordered merge.
//
// Shards are opened lazily: when metadata.yaml carries a complete summary
// (total count, start time, duration, per-topic counts) and the topic list,
// `topics()` and `compute_stats()` answer from metadata alone, so commands
// like `bagwiz ls` never touch the shard files. Iteration via `next()` (or
// a stats query against an incomplete summary) still opens all shards in
// declared order.
// ---------------------------------------------------------------------------
class McapShardReader : public BagReader
{
public:
  McapShardReader(
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
    // Last-resort fallback: derive topics from the first shard. Cached
    // into topics_ so subsequent calls stay O(1).
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
        // callers see a stable pointer for the whole bag. Take the chance
        // to backfill schema bytes from the shard (which always carries
        // them) into our metadata-derived TopicInfo (which may not).
        for (auto & t : topics_) {
          if (t.name == out.topic->name) {
            if (t.schema_text.empty() && !out.topic->schema_text.empty()) {
              t.schema_text = out.topic->schema_text;
            }
            if (t.schema_encoding.empty() && !out.topic->schema_encoding.empty()) {
              t.schema_encoding = out.topic->schema_encoding;
            }
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
    combined.from_summary = true;
    bool first = true;
    for (std::size_t i = 0; i < shard_rel_paths_.size(); ++i) {
      auto st = ensure_shard(i).compute_stats();
      combined.from_summary = combined.from_summary && st.from_summary;
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
    return combined;
  }

  void populate_schemas() override
  {
    if (schemas_loaded_ || shard_rel_paths_.empty()) {
      schemas_loaded_ = true;
      return;
    }
    // Open shard 0 (rosbag2 writes shards in declared order with consistent
    // schema/channel definitions across shards, so the first shard is
    // sufficient) and copy its schema text into our owned topic list. This
    // is the only path that forces a shard open from the directory reader
    // and is opt-in via populate_schemas().
    const auto & first = ensure_shard(0);
    const auto shard_topics = first.topics();
    for (auto & t : topics_) {
      for (const auto & s : shard_topics) {
        if (t.name == s.name) {
          if (t.schema_text.empty()) {
            t.schema_text = s.schema_text;
          }
          if (t.schema_encoding.empty()) {
            t.schema_encoding = s.schema_encoding;
          }
          break;
        }
      }
    }
    schemas_loaded_ = true;
  }

private:
  McapFileReader & ensure_shard(std::size_t i) const
  {
    if (!shards_[i]) {
      shards_[i] = std::make_unique<McapFileReader>(dir_ / shard_rel_paths_[i]);
    }
    return *shards_[i];
  }

  std::filesystem::path dir_;
  std::vector<std::filesystem::path> shard_rel_paths_;
  // mutable because topics() and ensure_shard() are logically const for
  // callers but cache lazily-derived state.
  mutable std::vector<TopicInfo> topics_;
  mutable std::vector<std::unique_ptr<McapFileReader>> shards_;
  BagMetadata metadata_;
  ReadFilter pending_filter_;
  bool has_pending_filter_ = false;
  std::vector<bool> shards_filter_applied_;
  std::size_t current_ = 0;
  bool iteration_started_ = false;
  bool schemas_loaded_ = false;
};

}  // namespace

std::unique_ptr<BagReader> open_mcap_file(const std::filesystem::path & path)
{
  return std::make_unique<McapFileReader>(path);
}

std::unique_ptr<BagReader> open_mcap_directory(const std::filesystem::path & dir, BagMetadata md)
{
  std::vector<TopicInfo> topics = md.topics;  // copied; metadata_ retains its own
  std::vector<std::filesystem::path> rel_paths = md.relative_file_paths;
  return std::make_unique<McapShardReader>(
    dir, std::move(rel_paths), std::move(topics), std::move(md));
}

}  // namespace bagwiz::io::detail
