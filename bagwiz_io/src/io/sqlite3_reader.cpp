// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/sqlite3_reader.hpp"

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/file_decompressor.hpp"
#include "bagwiz/io/message_decompressor.hpp"
#include "bagwiz/io/metadata_yaml.hpp"
#include "bagwiz/io/sqlite3_helpers.hpp"
#include "read_tuning.hpp"             // NOLINT(build/include_subdir) src-local shared header
#include "shard_multiplexer.hpp"       // NOLINT(build/include_subdir) src-local shared header
#include "sqlite3_slice_prefetch.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "sqlite3_slice_schedule.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <sqlite3.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::io::detail
{

namespace
{
constexpr const char * kLogger = "bagwiz.io.sqlite3";

// Upper bound on the parallel path's slice count, so a pathologically large
// bag cannot generate an unbounded schedule (each slice costs one prepare and
// one index seek).
constexpr std::size_t kMaxSlices = 4096;

// ---------------------------------------------------------------------------
// Single .db3 file reader.
// ---------------------------------------------------------------------------
class SqliteFileReader : public BagReader
{
public:
  // `decompressor` is null for an uncompressed bag and non-null when the
  // bag's metadata declares `compression_mode: MESSAGE`. When set, every
  // `messages.data` blob is routed through it before being exposed via
  // `next()`.
  SqliteFileReader(
    const std::filesystem::path & path, std::shared_ptr<MessageDecompressor> decompressor,
    TempFile temp = {})
  : path_(path),
    temp_(std::move(temp)),
    db_(sqlite_open_or_throw(
      path.string(), SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX, "sqlite3 open")),
    decompressor_(std::move(decompressor))
  {
    // Read-only streaming tuning. Failures are non-fatal (best-effort).
    sqlite3_exec(db_.get(), "PRAGMA query_only = 1;", nullptr, nullptr, nullptr);
    sqlite3_exec(db_.get(), "PRAGMA mmap_size = 268435456;", nullptr, nullptr, nullptr);
    sqlite3_exec(db_.get(), "PRAGMA cache_size = -65536;", nullptr, nullptr, nullptr);

    populate_topics();
  }

  ~SqliteFileReader() override = default;

  SqliteFileReader(const SqliteFileReader &) = delete;
  SqliteFileReader & operator=(const SqliteFileReader &) = delete;
  SqliteFileReader(SqliteFileReader &&) = delete;
  SqliteFileReader & operator=(SqliteFileReader &&) = delete;

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
    if (scanner_) {
      return next_sliced(out);
    }
    if (!cursors_.empty()) {
      return next_merged(out);
    }
    if (!read_stmt_) {
      return false;
    }

    for (;;) {
      const int rc = sqlite3_step(read_stmt_.get());
      if (rc == SQLITE_DONE) {
        return false;
      }
      if (rc != SQLITE_ROW) {
        throw std::runtime_error("sqlite3_step failed: " + sqlite_errmsg(db_.get()));
      }

      const int64_t topic_id = sqlite3_column_int64(read_stmt_.get(), 0);
      const int64_t timestamp = sqlite3_column_int64(read_stmt_.get(), 1);

      auto idx_it = topic_id_to_idx_.find(topic_id);
      if (idx_it == topic_id_to_idx_.end()) {
        // Row references a topic_id not in the topics table; skip.
        continue;
      }

      out.topic = &topics_[idx_it->second];
      out.timestamp_ns = timestamp;

      // Payload-skipped row (see ReadFilter::payload_topics): don't touch the
      // data column at all, so SQLite never reads the message's overflow
      // pages — the whole point of the flag on multi-GB message payloads.
      // Gate on the caller's name list, not the resolved id set: per the
      // ReadFilter contract, ANY non-empty allow-list means only the listed
      // topics carry payload — including a list that names no topic in the
      // bag, which callers use for timestamps-only scans.
      if (!filter_.payload_topics.empty() && payload_topic_ids_.count(topic_id) == 0) {
        out.payload = {};
        return true;
      }

      const void * data = sqlite3_column_blob(read_stmt_.get(), 2);
      const int data_size = sqlite3_column_bytes(read_stmt_.get(), 2);
      const auto src = std::span<const std::byte>(
        reinterpret_cast<const std::byte *>(data), static_cast<std::size_t>(data_size));
      // For uncompressed bags `data` points into SQLite's row buffer, which
      // stays valid until the next sqlite3_step / reset / finalize — that
      // matches the documented `next()` contract. For MESSAGE-compressed
      // bags the decompressor owns a reusable buffer with the same "valid
      // until the next next() call" lifetime, so we can return its span
      // directly without copying.
      out.payload = decompressor_ ? decompressor_->decompress(src) : src;
      return true;
    }
  }

  Stats compute_stats() override
  {
    Stats stats;
    // SQLite keeps no pre-computed summary, so the numbers come from the
    // messages table rather than an index/summary record.
    stats.from_summary = false;

    // Per-topic counts. bagwiz-written bags carry a (topic_id, timestamp)
    // index (see SqliteFileWriter::close()), which turns this GROUP BY into a
    // covering-index scan that never reads the BLOB-laden message rows. Bags
    // from other tools lack that index and fall back to a full table scan —
    // the reason `bagwiz ls` keeps per-topic stats behind `-l`. We select only
    // topic_id here (not MIN/MAX) so the covering index can satisfy the query.
    auto count_stmt = sqlite_prepare_or_throw(
      db_.get(), "SELECT topic_id, COUNT(*) FROM messages GROUP BY topic_id");
    for (;;) {
      const int rc = sqlite3_step(count_stmt.get());
      if (rc == SQLITE_DONE) {
        break;
      }
      if (rc != SQLITE_ROW) {
        throw std::runtime_error("stats query failed: " + sqlite_errmsg(db_.get()));
      }
      const int64_t topic_id = sqlite3_column_int64(count_stmt.get(), 0);
      const int64_t count = sqlite3_column_int64(count_stmt.get(), 1);

      auto idx_it = topic_id_to_idx_.find(topic_id);
      if (idx_it != topic_id_to_idx_.end()) {
        stats.per_topic[topics_[idx_it->second].name] = count;
      }
      stats.total_messages += count;
    }

    // Bag-level time extent. timestamp_idx is present in every rosbag2 SQLite
    // bag, so SQLite answers MIN/MAX from the index ends in O(1) — no scan,
    // even on bags that lack the topic_id index. MIN/MAX are NULL for an empty
    // bag, in which case start_ns/end_ns stay 0.
    auto extent_stmt =
      sqlite_prepare_or_throw(db_.get(), "SELECT MIN(timestamp), MAX(timestamp) FROM messages");
    if (
      sqlite3_step(extent_stmt.get()) == SQLITE_ROW &&
      sqlite3_column_type(extent_stmt.get(), 0) != SQLITE_NULL) {
      stats.start_ns = sqlite3_column_int64(extent_stmt.get(), 0);
      stats.end_ns = sqlite3_column_int64(extent_stmt.get(), 1);
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

    std::vector<int64_t> ids;
    ids.reserve(names.size());
    for (const auto & name : names) {
      for (const auto & [tid, idx] : topic_id_to_idx_) {
        if (topics_[idx].name == name) {
          ids.push_back(tid);
          break;
        }
      }
    }
    if (ids.empty()) {
      return result;
    }

    std::string sql = "SELECT topic_id, COUNT(*) FROM messages WHERE topic_id IN (";
    for (std::size_t i = 0; i < ids.size(); ++i) {
      sql += (i == 0 ? "" : ",") + std::to_string(ids[i]);
    }
    sql += ") GROUP BY topic_id";

    auto stmt = sqlite_prepare_or_throw(db_.get(), sql.c_str());
    for (;;) {
      const int rc = sqlite3_step(stmt.get());
      if (rc == SQLITE_DONE) {
        break;
      }
      if (rc != SQLITE_ROW) {
        throw std::runtime_error("topic count query failed: " + sqlite_errmsg(db_.get()));
      }
      const int64_t topic_id = sqlite3_column_int64(stmt.get(), 0);
      const int64_t count = sqlite3_column_int64(stmt.get(), 1);

      auto idx_it = topic_id_to_idx_.find(topic_id);
      if (idx_it != topic_id_to_idx_.end()) {
        result[topics_[idx_it->second].name] = count;
      }
    }
    return result;
  }

  TimeExtent compute_time_extent() override
  {
    TimeExtent extent;
    auto stmt =
      sqlite_prepare_or_throw(db_.get(), "SELECT MIN(timestamp), MAX(timestamp) FROM messages");
    if (
      sqlite3_step(stmt.get()) == SQLITE_ROW && sqlite3_column_type(stmt.get(), 0) != SQLITE_NULL) {
      extent.start_ns = sqlite3_column_int64(stmt.get(), 0);
      extent.end_ns = sqlite3_column_int64(stmt.get(), 1);
      extent.has_data = true;
    }
    return extent;
  }

private:
  // One per-topic statement of the k-way merge. `stmt` sits on its current row
  // whenever `live` is true, and that row's blob stays valid until the cursor
  // is stepped — which next_merged() defers until the call after the row was
  // emitted, so the payload span honours the next() contract.
  struct TopicCursor
  {
    SqliteStmtPtr stmt;
    std::size_t topic_idx = 0;  // index into topics_
    bool wants_payload = true;  // resolved once from ReadFilter::payload_topics
    bool live = false;
    int64_t timestamp = 0;
    int64_t rowid = 0;
  };

  static const char * column_text_or_empty(sqlite3_stmt * stmt, int col)
  {
    const auto * text = sqlite3_column_text(stmt, col);
    return text ? reinterpret_cast<const char *>(text) : "";
  }

  // Topic IDs for ReadFilter::topics, deduplicated so a name repeated by the
  // caller cannot open two cursors over the same rows (which would emit every
  // one of that topic's messages twice).
  std::vector<int64_t> resolve_filter_topic_ids() const
  {
    std::vector<int64_t> ids;
    ids.reserve(filter_.topics.size());
    for (const auto & name : filter_.topics) {
      for (const auto & [tid, idx] : topic_id_to_idx_) {
        if (topics_[idx].name != name) {
          continue;
        }
        if (std::find(ids.begin(), ids.end(), tid) == ids.end()) {
          ids.push_back(tid);
        }
        break;
      }
    }
    return ids;
  }

  void populate_topics()
  {
    // Detect Iron+ schema by probing for the type_description_hash
    // column on `topics`. We trust the column existence rather than
    // reading the `schema` table because some bags written by older
    // tooling claim schema_version=4 in the schema table while still
    // emitting the v3-shaped layout (a bug bagwiz itself exhibited
    // before this PR).
    const bool has_v4_topic_columns = column_exists("topics", "type_description_hash");

    const char * const sql_v4 =
      "SELECT id, name, type, serialization_format, offered_qos_profiles, "
      "type_description_hash FROM topics";
    const char * const sql_v3 =
      "SELECT id, name, type, serialization_format, offered_qos_profiles FROM topics";
    auto stmt = sqlite_prepare_or_throw(db_.get(), has_v4_topic_columns ? sql_v4 : sql_v3);
    for (;;) {
      const int rc = sqlite3_step(stmt.get());
      if (rc == SQLITE_DONE) {
        break;
      }
      if (rc != SQLITE_ROW) {
        throw std::runtime_error("topics query failed: " + sqlite_errmsg(db_.get()));
      }
      const int64_t topic_id = sqlite3_column_int64(stmt.get(), 0);
      TopicInfo info;
      info.name = column_text_or_empty(stmt.get(), 1);
      info.type = column_text_or_empty(stmt.get(), 2);
      info.serialization_format = column_text_or_empty(stmt.get(), 3);
      info.offered_qos_profiles = column_text_or_empty(stmt.get(), 4);
      if (has_v4_topic_columns) {
        info.type_description_hash = column_text_or_empty(stmt.get(), 5);
      }
      topic_id_to_idx_[topic_id] = topics_.size();
      topics_.push_back(std::move(info));
    }

    // Iron+ embeds per-type message_definitions. Backfill schema_text /
    // schema_encoding so the reader returns self-described topics
    // without callers having to invoke msg_definition_resolver.
    if (table_exists("message_definitions")) {
      auto def_stmt = sqlite_prepare_or_throw(
        db_.get(),
        "SELECT topic_type, encoding, encoded_message_definition, type_description_hash "
        "FROM message_definitions");
      // Build a fast lookup by type so we visit each topics_[i] at most once.
      for (;;) {
        const int rc = sqlite3_step(def_stmt.get());
        if (rc == SQLITE_DONE) {
          break;
        }
        if (rc != SQLITE_ROW) {
          throw std::runtime_error("message_definitions query failed: " + sqlite_errmsg(db_.get()));
        }
        const std::string topic_type = column_text_or_empty(def_stmt.get(), 0);
        const std::string encoding = column_text_or_empty(def_stmt.get(), 1);
        const std::string text = column_text_or_empty(def_stmt.get(), 2);
        const std::string hash = column_text_or_empty(def_stmt.get(), 3);
        for (auto & t : topics_) {
          if (t.type != topic_type) {
            continue;
          }
          if (t.schema_text.empty()) {
            t.schema_text = text;
          }
          if (t.schema_encoding.empty()) {
            t.schema_encoding = encoding;
          }
          if (t.type_description_hash.empty() && !hash.empty()) {
            t.type_description_hash = hash;
          }
        }
      }
    }
  }

  // Lightweight column / table existence probes. Both return false on
  // any sqlite error so `populate_topics` falls back gracefully when
  // the bag is shaped unexpectedly (e.g. test fixtures that omit the
  // `schema` table entirely).
  bool column_exists(const char * table, const char * column) const
  {
    auto stmt = sqlite_prepare_or_throw(
      db_.get(), (std::string("PRAGMA table_info('") + table + "')").c_str());
    while (sqlite3_step(stmt.get()) == SQLITE_ROW) {
      const auto * name = reinterpret_cast<const char *>(sqlite3_column_text(stmt.get(), 1));
      if (name != nullptr && std::string(name) == column) {
        return true;
      }
    }
    return false;
  }

  bool table_exists(const char * table) const
  {
    auto stmt = sqlite_prepare_or_throw(
      db_.get(), "SELECT 1 FROM sqlite_master WHERE type='table' AND name=? LIMIT 1");
    sqlite3_bind_text(stmt.get(), 1, table, -1, SQLITE_TRANSIENT);
    return sqlite3_step(stmt.get()) == SQLITE_ROW;
  }

  bool index_exists(const char * index) const
  {
    auto stmt = sqlite_prepare_or_throw(
      db_.get(), "SELECT 1 FROM sqlite_master WHERE type='index' AND name=? LIMIT 1");
    sqlite3_bind_text(stmt.get(), 1, index, -1, SQLITE_TRANSIENT);
    return sqlite3_step(stmt.get()) == SQLITE_ROW;
  }

  // Open one statement per selected topic for the k-way merge. Each is served
  // straight off topic_timestamp_idx in (timestamp, rowid) order — the index
  // key is (topic_id, timestamp) with the rowid appended, so SQLite satisfies
  // `ORDER BY timestamp, id` from the index and never builds a sorter.
  void open_merge_cursors(
    const std::vector<int64_t> & ids, const std::vector<std::string> & time_clauses)
  {
    cursors_.reserve(ids.size());
    for (const auto tid : ids) {
      std::string sql = "SELECT topic_id, timestamp, data, id FROM messages WHERE topic_id = " +
                        std::to_string(tid);
      for (const auto & clause : time_clauses) {
        sql += " AND " + clause;
      }
      sql += " ORDER BY timestamp, id";

      TopicCursor cursor;
      cursor.stmt = sqlite_prepare_or_throw(db_.get(), sql);
      cursor.topic_idx = topic_id_to_idx_.at(tid);
      // Same rule as the single-statement path: ANY non-empty allow-list means
      // only the listed topics carry payload.
      cursor.wants_payload = filter_.payload_topics.empty() || payload_topic_ids_.count(tid) != 0U;
      cursors_.push_back(std::move(cursor));
    }
    for (auto & cursor : cursors_) {
      advance_cursor(cursor);
    }
    pending_advance_ = cursors_.size();  // nothing consumed yet
  }

  void advance_cursor(TopicCursor & cursor)
  {
    const int rc = sqlite3_step(cursor.stmt.get());
    if (rc == SQLITE_ROW) {
      cursor.timestamp = sqlite3_column_int64(cursor.stmt.get(), 1);
      cursor.rowid = sqlite3_column_int64(cursor.stmt.get(), 3);
      cursor.live = true;
      return;
    }
    if (rc != SQLITE_DONE) {
      throw std::runtime_error("sqlite3_step failed: " + sqlite_errmsg(db_.get()));
    }
    cursor.live = false;
  }

  // Emit the smallest (timestamp, rowid) across the live cursors. Ties are
  // broken by rowid so a topic-filtered read emits messages in exactly the
  // order an unfiltered scan does: the unfiltered plan walks timestamp_idx,
  // whose key carries the rowid, so same-timestamp messages come out in
  // insertion order there too.
  bool next_merged(RawMessage & out)
  {
    // Step the cursor consumed by the previous call only now — its blob pointer
    // had to stay valid until this call, per the next() contract.
    if (pending_advance_ < cursors_.size()) {
      advance_cursor(cursors_[pending_advance_]);
      pending_advance_ = cursors_.size();
    }

    std::size_t best = cursors_.size();
    for (std::size_t i = 0; i < cursors_.size(); ++i) {
      const auto & cursor = cursors_[i];
      if (!cursor.live) {
        continue;
      }
      if (
        best == cursors_.size() || cursor.timestamp < cursors_[best].timestamp ||
        (cursor.timestamp == cursors_[best].timestamp && cursor.rowid < cursors_[best].rowid)) {
        best = i;
      }
    }
    if (best == cursors_.size()) {
      return false;
    }

    auto & cursor = cursors_[best];
    out.topic = &topics_[cursor.topic_idx];
    out.timestamp_ns = cursor.timestamp;
    if (!cursor.wants_payload) {
      // Never touch the data column, so SQLite leaves the overflow pages
      // unread — the whole point of ReadFilter::payload_topics.
      out.payload = {};
    } else {
      const void * data = sqlite3_column_blob(cursor.stmt.get(), 2);
      const int data_size = sqlite3_column_bytes(cursor.stmt.get(), 2);
      const auto src = std::span<const std::byte>(
        reinterpret_cast<const std::byte *>(data), static_cast<std::size_t>(data_size));
      out.payload = decompressor_ ? decompressor_->decompress(src) : src;
    }
    pending_advance_ = best;
    return true;
  }

  // Emit the current slice's records, then block for the next slice. Slices
  // are disjoint and ascending, so this is a plain concatenation — no merge,
  // and therefore no tiebreak that could diverge from the serial scan.
  bool next_sliced(RawMessage & out)
  {
    while (record_pos_ >= slice_.records.size()) {
      if (slice_index_ >= scanner_->size()) {
        return false;
      }
      // Hand the drained buffers back before blocking. Any payload span from
      // the previous next() dies here, which is exactly what the RawMessage
      // contract allows: this call invalidates it either way.
      scanner_->recycle(std::move(slice_.blobs), std::move(slice_.records));
      slice_ = scanner_->get(slice_index_++);
      if (!slice_.error.empty()) {
        throw std::runtime_error("db3 slice scan failed: " + slice_.error);
      }
      record_pos_ = 0;
    }

    const SliceRecord & rec = slice_.records[record_pos_++];
    out.topic = &topics_[topic_id_to_idx_.at(rec.topic_id)];
    out.timestamp_ns = rec.timestamp_ns;
    if (rec.size == 0) {
      // Either a payload-skipped row (ReadFilter::payload_topics) or a
      // genuinely empty blob; both surface as an empty span, as on the serial
      // path, and neither is worth routing through the decompressor.
      out.payload = {};
      return true;
    }
    const auto src = std::span<const std::byte>(slice_.blobs.data() + rec.offset, rec.size);
    // The span stays valid until a next() call crosses a slice boundary —
    // strictly longer than the "valid until the next next()" contract. For
    // MESSAGE-compressed bags the workers stage the compressed bytes and the
    // decompression stays here, on the consumer thread: MessageDecompressor
    // owns one ZSTD_DCtx and is not thread-safe.
    out.payload = decompressor_ ? decompressor_->decompress(src) : src;
    return true;
  }

  // Slice schedule for the parallel path, or an empty vector when the bag is
  // empty, the filter selects nothing, or the file is too small to be worth
  // splitting (in which case the caller falls back to the serial scan).
  std::vector<SliceRef> build_parallel_schedule() const
  {
    auto stmt =
      sqlite_prepare_or_throw(db_.get(), "SELECT MIN(timestamp), MAX(timestamp) FROM messages");
    // Both come off timestamp_idx's ends in O(1); NULL means an empty bag.
    if (
      sqlite3_step(stmt.get()) != SQLITE_ROW || sqlite3_column_type(stmt.get(), 0) == SQLITE_NULL) {
      return {};
    }

    SliceScheduleParams params;
    params.extent_start_ns = sqlite3_column_int64(stmt.get(), 0);
    params.extent_end_ns = sqlite3_column_int64(stmt.get(), 1);
    params.filter_start_ns = filter_.start_ns;
    params.filter_end_ns = filter_.end_ns;
    std::error_code ec;
    params.file_size_bytes = std::filesystem::file_size(path_, ec);
    if (ec) {
      return {};  // unknown size: no basis for sizing slices
    }
    params.target_slice_bytes = resolve_slice_bytes(kLogger);
    params.max_slices = kMaxSlices;
    return build_slice_schedule(params);
  }

  void ensure_iterator()
  {
    if (iteration_started_) {
      return;
    }
    iteration_started_ = true;

    // Resolve the payload allow-list (ReadFilter::payload_topics) to topic
    // IDs. An empty allow-list means every row materializes its payload.
    // Resolved before the statements are built because the merge cursors bake
    // the decision in per topic.
    payload_topic_ids_.clear();
    for (const auto & name : filter_.payload_topics) {
      for (const auto & [tid, idx] : topic_id_to_idx_) {
        if (topics_[idx].name == name) {
          payload_topic_ids_.insert(tid);
          break;
        }
      }
    }

    std::vector<int64_t> ids = resolve_filter_topic_ids();
    if (!filter_.topics.empty() && ids.empty()) {
      BAGWIZ_LOG_WARN(
        kLogger, "no topic IDs matched the filter for %s; iteration will be empty", path_.c_str());
      return;
    }

    std::vector<std::string> time_clauses;
    if (filter_.start_ns) {
      time_clauses.push_back("timestamp >= " + std::to_string(*filter_.start_ns));
    }
    if (filter_.end_ns) {
      // end_ns is exclusive (ReadFilter selects [start_ns, end_ns)), matching
      // the MCAP backend's ReadMessageOptions::endTime semantics.
      time_clauses.push_back("timestamp < " + std::to_string(*filter_.end_ns));
    }

    // A topic filter plus `ORDER BY timestamp` makes SQLite serve the rows off
    // topic_timestamp_idx (which orders by topic first) and then sort them back
    // into time order through a temp B-tree. That sorter carries the `data`
    // BLOB, so it spills gigabytes of payload to the temp store and reads them
    // back — measured ~9x slower than the same scan without the sort.
    //
    // Splitting into one statement per topic sidesteps it: each is served in
    // timestamp order straight off the index, and the merge below restores the
    // global order. Only worth it when topic_timestamp_idx actually exists
    // (bags bagwiz wrote). Without it SQLite already picks a sorter-free
    // timestamp_idx scan for the combined statement, and N per-topic statements
    // would scan that index N times instead of once.
    if (!ids.empty() && index_exists("topic_timestamp_idx")) {
      open_merge_cursors(ids, time_clauses);
      return;
    }

    std::string sql = "SELECT topic_id, timestamp, data FROM messages";
    std::vector<std::string> where;
    if (!ids.empty()) {
      std::string clause = "topic_id IN (";
      for (std::size_t i = 0; i < ids.size(); ++i) {
        clause += (i == 0 ? "" : ",") + std::to_string(ids[i]);
      }
      clause += ")";
      where.push_back(std::move(clause));
    }
    where.insert(where.end(), time_clauses.begin(), time_clauses.end());

    if (!where.empty()) {
      sql += " WHERE ";
      for (std::size_t i = 0; i < where.size(); ++i) {
        sql += (i == 0 ? "" : " AND ") + where[i];
      }
    }
    sql += " ORDER BY timestamp";

    // Parallel slice scan: split the bag's time extent into disjoint half-open
    // ranges and let a worker pool scan them ahead of us on its own
    // connections. Each slice is served straight off timestamp_idx, whose key
    // is (timestamp, rowid), so concatenating them in schedule order emits
    // exactly what this single statement would. Requires the index — without
    // it SQLite sorts through a temp B-tree, whose order among equal
    // timestamps is not guaranteed to match.
    if (const int read_threads = resolve_read_threads(kLogger);
        read_threads > 1 && index_exists("timestamp_idx")) {
      if (auto schedule = build_parallel_schedule(); !schedule.empty()) {
        SliceScanSpec spec;
        if (!ids.empty()) {
          spec.topic_clause = "topic_id IN (";
          for (std::size_t i = 0; i < ids.size(); ++i) {
            spec.topic_clause += (i == 0 ? "" : ",") + std::to_string(ids[i]);
          }
          spec.topic_clause += ")";
        }
        for (const auto & [tid, idx] : topic_id_to_idx_) {
          spec.known_topic_ids.insert(tid);
        }
        spec.payload_topic_ids = payload_topic_ids_;
        spec.payload_filter_active = !filter_.payload_topics.empty();
        scanner_ =
          std::make_unique<SliceScanner>(path_, std::move(spec), std::move(schedule), read_threads);
        return;
      }
    }

    read_stmt_ = sqlite_prepare_or_throw(db_.get(), sql);
  }

  std::filesystem::path path_;
  // Owned decompressed temp file for FILE-mode `.db3.zstd` envelopes; empty
  // for ordinary on-disk bags. Declared before db_ so that on destruction db_
  // is closed first and then the temp file is removed.
  TempFile temp_;
  SqlitePtr db_;
  // Exactly one of these drives iteration: `read_stmt_` for an unfiltered read
  // (or a bag without topic_timestamp_idx), `cursors_` for the per-topic merge.
  SqliteStmtPtr read_stmt_;
  std::vector<TopicCursor> cursors_;
  // Index of the cursor emitted by the previous next_merged() call, stepped at
  // the start of the following one; cursors_.size() means none is pending.
  std::size_t pending_advance_ = 0;
  // The third iteration driver, alongside read_stmt_ and cursors_: a worker
  // pool scanning disjoint timestamp slices ahead of us. Slices are emitted in
  // schedule order, which reproduces the serial scan's order exactly.
  std::unique_ptr<SliceScanner> scanner_;
  PrefetchedSlice slice_;        // the slice currently being drained
  std::size_t slice_index_ = 0;  // next schedule index to request
  std::size_t record_pos_ = 0;   // position within slice_.records
  std::vector<TopicInfo> topics_;
  std::unordered_map<int64_t, std::size_t> topic_id_to_idx_;
  std::unordered_set<int64_t> payload_topic_ids_;
  ReadFilter filter_;
  bool iteration_started_ = false;
  std::shared_ptr<MessageDecompressor> decompressor_;
};

// ---------------------------------------------------------------------------
// Multi-shard SQLite3 reader: the shard multiplexing (lazy shard opening,
// filter push-down, topic-pointer remapping, stats/count/extent folding) is
// shared with the MCAP directory reader in ShardMultiplexer. What stays here
// is how a shard file becomes a SqliteFileReader (MESSAGE-mode decompressor
// sharing, `.db3.zstd` FILE-mode envelope handling) and the from_summary
// policy: sqlite shard stats read raw tables, never a summary.
// ---------------------------------------------------------------------------
class SqliteShardReader : public ShardMultiplexer<SqliteFileReader>
{
public:
  SqliteShardReader(
    std::filesystem::path dir, std::vector<std::filesystem::path> shard_rel_paths,
    std::vector<TopicInfo> topics, BagMetadata metadata,
    std::shared_ptr<MessageDecompressor> decompressor, bool zstd_envelope)
  : ShardMultiplexer(
      std::move(dir), std::move(shard_rel_paths), std::move(topics), std::move(metadata)),
    decompressor_(std::move(decompressor)),
    zstd_envelope_(zstd_envelope)
  {
  }

private:
  std::unique_ptr<SqliteFileReader> open_shard(
    const std::filesystem::path & shard_path) const override
  {
    if (zstd_envelope_) {
      // FILE-mode `.db3.zstd` envelope: decompress this shard to a temp
      // `.db3` lazily (only now, when it is actually iterated) and hand
      // ownership of the temp file to the reader so it is removed on close.
      TempFile temp = decompress_zstd_file_to_temp(shard_path);
      const auto temp_path = temp.path();
      return std::make_unique<SqliteFileReader>(temp_path, decompressor_, std::move(temp));
    }
    // Share the decompressor across shards so the ZSTD_DCtx is reused for
    // the entire iteration (per-thread context reuse is the hot-path
    // contract documented by rosbag2_compression_zstd).
    return std::make_unique<SqliteFileReader>(shard_path, decompressor_);
  }

  bool scan_from_summary(const std::vector<bool> & /*shard_flags*/) const override { return false; }

  std::shared_ptr<MessageDecompressor> decompressor_;
  // True when each shard in shard_rel_paths_ is a `.db3.zstd` envelope that
  // must be decompressed to a temp `.db3` before SQLite can open it.
  bool zstd_envelope_ = false;
};

}  // namespace

std::unique_ptr<BagReader> open_sqlite3_file(
  const std::filesystem::path & path, std::shared_ptr<MessageDecompressor> decompressor,
  TempFile temp)
{
  return std::make_unique<SqliteFileReader>(path, std::move(decompressor), std::move(temp));
}

std::unique_ptr<BagReader> open_sqlite3_directory(
  const std::filesystem::path & dir, BagMetadata md,
  std::shared_ptr<MessageDecompressor> decompressor, bool zstd_file_envelope)
{
  std::vector<TopicInfo> topics = md.topics;
  std::vector<std::filesystem::path> rel_paths = md.relative_file_paths;
  return std::make_unique<SqliteShardReader>(
    dir, std::move(rel_paths), std::move(topics), std::move(md), std::move(decompressor),
    zstd_file_envelope);
}

}  // namespace bagwiz::io::detail
