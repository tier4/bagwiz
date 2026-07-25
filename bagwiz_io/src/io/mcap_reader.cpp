// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/mcap_reader.hpp"

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/message_decompressor.hpp"
#include "bagwiz/io/metadata_yaml.hpp"
#include "mcap_indexed_stream.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "shard_multiplexer.hpp"    // NOLINT(build/include_subdir) src-local shared header

// mcap_vendor ships MCAP as a pre-compiled library, so MCAP_IMPLEMENTATION
// must NOT be defined here — the symbols are provided via linkage against
// mcap_vendor::mcap. Defining it would try to pull in .inl files that the
// vendor package does not install.
#include <mcap/reader.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::io::detail
{

namespace
{
constexpr const char * kLogger = "bagwiz.io.mcap";

// Decompress-worker count for the parallel indexed read path. Defaults to 8,
// capped at the host's hardware concurrency so low-core machines keep a
// smaller worker count (docs/benchmarks/mcap-read-threads.md has the sweep the
// default is based on). BAGWIZ_READ_THREADS overrides the default, and 0 or 1
// falls back to the synchronous libmcap iteration (the debugging escape
// hatch).
int resolve_read_threads()
{
  constexpr int kDefault = 8;
  constexpr int kMax = 16;
  const auto default_threads = [&] {
    const unsigned int hw = std::thread::hardware_concurrency();
    return hw == 0 ? kDefault : std::min<int>(kDefault, static_cast<int>(hw));
  };
  const char * env = std::getenv("BAGWIZ_READ_THREADS");
  if (env == nullptr || *env == '\0') {
    return default_threads();
  }
  char * end = nullptr;
  const long parsed = std::strtol(env, &end, 10);  // NOLINT(runtime/int) strtol API
  if (end == env || *end != '\0') {
    BAGWIZ_LOG_WARN(kLogger, "ignoring unparsable BAGWIZ_READ_THREADS='%s'", env);
    return default_threads();
  }
  return static_cast<int>(std::clamp<long>(parsed, 0, kMax));  // NOLINT(runtime/int)
}

// ---------------------------------------------------------------------------
// Single .mcap file reader.
// ---------------------------------------------------------------------------
class McapFileReader : public BagReader
{
public:
  // `decompressor` is null for an uncompressed bag and non-null when the
  // bag's metadata declares `compression_mode: MESSAGE`. When set, every
  // per-message payload is routed through it before being exposed via
  // `next()`.
  McapFileReader(
    const std::filesystem::path & path, std::shared_ptr<MessageDecompressor> decompressor)
  : path_(path), decompressor_(std::move(decompressor))
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
    if (parallel_stream_) {
      ParallelIndexedStream::Message msg;
      while (parallel_stream_->next(msg)) {
        auto idx_it = channel_to_topic_idx_.find(msg.channel_id);
        if (idx_it == channel_to_topic_idx_.end()) {
          continue;  // channel without a known schema entry, as below
        }
        out.topic = &topics_[idx_it->second];
        out.timestamp_ns = static_cast<int64_t>(msg.log_time);
        // The stream's payload span stays valid until its next next() call
        // (the retained chunk buffer is only reused then), which matches the
        // RawMessage contract — no copy needed on this path.
        if (decompressor_) {
          out.payload = decompressor_->decompress(msg.payload);
        } else {
          out.payload = msg.payload;
        }
        return true;
      }
      if (!parallel_stream_->error().empty()) {
        BAGWIZ_LOG_WARN(
          kLogger, "parallel mcap read failed for %s: %s", path_.c_str(),
          parallel_stream_->error().c_str());
      }
      return false;
    }
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
      // The payload must stay valid until the next next() call even though
      // ++iter may release the current chunk and invalidate mv.message.data.
      // Two stable backings cover both cases:
      //   - uncompressed: copy into payload_buf_ before advancing.
      //   - MESSAGE-compressed: decompress before advancing; the returned
      //     span points into the decompressor's reusable buffer, whose
      //     lifetime is "valid until the next decompress() call" — which
      //     matches our public contract.
      const auto * src = reinterpret_cast<const std::byte *>(mv.message.data);
      const auto size = static_cast<std::size_t>(mv.message.dataSize);
      if (decompressor_) {
        out.payload = decompressor_->decompress(std::span<const std::byte>(src, size));
      } else {
        payload_buf_.assign(src, src + size);
        out.payload = std::span<const std::byte>(payload_buf_.data(), payload_buf_.size());
      }
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

  std::unordered_map<std::string, int64_t> compute_topic_counts(
    std::span<const std::string> names) override
  {
    std::unordered_map<std::string, int64_t> result;
    if (names.empty()) {
      return result;
    }

    const auto & statistics = reader_.statistics();
    if (!statistics) {
      BAGWIZ_LOG_WARN(
        kLogger, "Statistics unavailable for %s; topic counts will be zero", path_.c_str());
      return result;
    }

    const std::unordered_set<std::string> requested(names.begin(), names.end());
    for (const auto & [channel_id, count] : statistics->channelMessageCounts) {
      auto idx_it = channel_to_topic_idx_.find(channel_id);
      if (idx_it == channel_to_topic_idx_.end()) {
        continue;
      }
      const std::string & name = topics_[idx_it->second].name;
      if (requested.count(name) != 0U) {
        result[name] = static_cast<int64_t>(count);
      }
    }
    return result;
  }

  TimeExtent compute_time_extent() override
  {
    TimeExtent extent;
    const auto & statistics = reader_.statistics();
    if (statistics) {
      extent.start_ns = static_cast<int64_t>(statistics->messageStartTime);
      extent.end_ns = static_cast<int64_t>(statistics->messageEndTime);
      extent.has_data = statistics->messageCount > 0;
    } else {
      BAGWIZ_LOG_WARN(
        kLogger, "Statistics unavailable for %s; time extent will be zero", path_.c_str());
    }
    return extent;
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

    // On the indexed path, serve the same log-time-ordered stream through
    // ParallelIndexedStream when possible: identical emission order, but the
    // chunk zstd decompression runs ahead on a small worker pool instead of
    // stalling the iterating thread — the dominant cost on multi-GB bags.
    const int read_threads = resolve_read_threads();
    if (indexed_ok && read_threads > 1 && ParallelIndexedStream::supported(reader_)) {
      ParallelIndexedStream::Options popts;
      if (filter_.start_ns) {
        popts.start_ns = static_cast<std::uint64_t>(*filter_.start_ns);
      }
      if (filter_.end_ns) {
        popts.end_ns = static_cast<std::uint64_t>(*filter_.end_ns);
      }
      popts.topic_filter = opts.topicFilter;
      popts.num_threads = read_threads;
      parallel_stream_ = std::make_unique<ParallelIndexedStream>(path_, reader_, std::move(popts));
      return;
    }

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
  std::unique_ptr<ParallelIndexedStream> parallel_stream_;
  // Stable backing store for the payload returned by next() in the
  // uncompressed path. Lives as long as the reader; each next() overwrites
  // it. Unused when decompressor_ is non-null (the decompressor's own
  // buffer plays the same role then).
  std::vector<std::byte> payload_buf_;
  std::shared_ptr<MessageDecompressor> decompressor_;
};

// ---------------------------------------------------------------------------
// Multi-shard MCAP reader: the shard multiplexing (lazy shard opening, filter
// push-down, topic-pointer remapping, stats/count/extent folding) is shared
// with the SQLite3 directory reader in ShardMultiplexer. What stays here is
// how a shard file becomes an McapFileReader, the MCAP-specific schema
// backfill (shards always carry schema bytes; a metadata-derived TopicInfo
// may not), populate_schemas(), and the from_summary policy: MCAP shard
// stats read summary records, so the combined flag ANDs the per-shard flags.
// ---------------------------------------------------------------------------
class McapShardReader : public ShardMultiplexer<McapFileReader>
{
public:
  McapShardReader(
    std::filesystem::path dir, std::vector<std::filesystem::path> shard_rel_paths,
    std::vector<TopicInfo> topics, BagMetadata metadata,
    std::shared_ptr<MessageDecompressor> decompressor)
  : ShardMultiplexer(
      std::move(dir), std::move(shard_rel_paths), std::move(topics), std::move(metadata)),
    decompressor_(std::move(decompressor))
  {
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
  std::unique_ptr<McapFileReader> open_shard(
    const std::filesystem::path & shard_path) const override
  {
    // Share the decompressor across shards so the ZSTD_DCtx is reused for
    // the entire iteration (per-thread context reuse is the hot-path
    // contract documented by rosbag2_compression_zstd).
    return std::make_unique<McapFileReader>(shard_path, decompressor_);
  }

  void on_topic_remapped(TopicInfo & bag_topic, const TopicInfo & shard_topic) const override
  {
    // Backfill schema bytes from the shard (which always carries them) into
    // our metadata-derived TopicInfo (which may not).
    if (bag_topic.schema_text.empty() && !shard_topic.schema_text.empty()) {
      bag_topic.schema_text = shard_topic.schema_text;
    }
    if (bag_topic.schema_encoding.empty() && !shard_topic.schema_encoding.empty()) {
      bag_topic.schema_encoding = shard_topic.schema_encoding;
    }
  }

  bool scan_from_summary(const std::vector<bool> & shard_flags) const override
  {
    return std::all_of(shard_flags.begin(), shard_flags.end(), [](bool f) { return f; });
  }

  bool schemas_loaded_ = false;
  std::shared_ptr<MessageDecompressor> decompressor_;
};

}  // namespace

std::unique_ptr<BagReader> open_mcap_file(
  const std::filesystem::path & path, std::shared_ptr<MessageDecompressor> decompressor)
{
  return std::make_unique<McapFileReader>(path, std::move(decompressor));
}

std::unique_ptr<BagReader> open_mcap_directory(
  const std::filesystem::path & dir, BagMetadata md,
  std::shared_ptr<MessageDecompressor> decompressor)
{
  std::vector<TopicInfo> topics = md.topics;  // copied; metadata_ retains its own
  std::vector<std::filesystem::path> rel_paths = md.relative_file_paths;
  return std::make_unique<McapShardReader>(
    dir, std::move(rel_paths), std::move(topics), std::move(md), std::move(decompressor));
}

}  // namespace bagwiz::io::detail
