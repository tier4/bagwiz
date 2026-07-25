// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "sqlite3_slice_prefetch.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/io/sqlite3_helpers.hpp"
#include "read_tuning.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <sqlite3.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::io::detail
{

namespace
{
constexpr const char * kLogger = "bagwiz.io.sqlite3";
constexpr std::uint64_t kDefaultSliceBytes = 32ULL * 1024ULL * 1024ULL;
constexpr std::uint64_t kMinSliceBytes = 1ULL;
constexpr std::uint64_t kMaxSliceBytes = 1024ULL * 1024ULL * 1024ULL;

// Compose one slice's statement. Every clause is a literal (the values come
// from the bag's own extent and the caller's filter, never from user text), so
// there is nothing to bind — matching how the serial reader builds its SQL.
std::string slice_sql(const SliceScanSpec & spec, const SliceRef & slice)
{
  std::vector<std::string> where;
  if (!spec.topic_clause.empty()) {
    where.push_back(spec.topic_clause);
  }
  if (slice.start_ns.has_value()) {
    where.push_back("timestamp >= " + std::to_string(*slice.start_ns));
  }
  if (slice.end_ns.has_value()) {
    where.push_back("timestamp < " + std::to_string(*slice.end_ns));
  }

  std::string sql = "SELECT topic_id, timestamp, data FROM messages";
  for (std::size_t i = 0; i < where.size(); ++i) {
    sql += (i == 0 ? " WHERE " : " AND ") + where[i];
  }
  // Served straight off timestamp_idx, whose key is (timestamp, rowid) — the
  // same order the unsliced scan walks, so concatenating the slices in
  // schedule order reproduces the serial emission order exactly.
  sql += " ORDER BY timestamp";
  return sql;
}

}  // namespace

std::uint64_t resolve_slice_bytes(const char * logger)
{
  return static_cast<std::uint64_t>(resolve_env_int(
    "BAGWIZ_DB3_SLICE_BYTES", static_cast<std::int64_t>(kDefaultSliceBytes),
    static_cast<std::int64_t>(kMinSliceBytes), static_cast<std::int64_t>(kMaxSliceBytes), logger));
}

SliceScanner::SliceScanner(
  std::filesystem::path path, SliceScanSpec spec, std::vector<SliceRef> schedule, int num_threads)
: path_(std::move(path)),
  spec_(std::move(spec)),
  schedule_(std::move(schedule)),
  lookahead_(static_cast<std::size_t>(std::max(num_threads, 1)) + 2)
{
  const int workers = std::max(num_threads, 1);
  workers_.reserve(static_cast<std::size_t>(workers));
  for (int i = 0; i < workers; ++i) {
    workers_.emplace_back([this]() { worker_loop(); });
  }
}

SliceScanner::~SliceScanner()
{
  {
    const std::lock_guard lock(mutex_);
    cancel_ = true;
  }
  cv_.notify_all();
  // jthread joins on destruction anyway; the explicit join keeps the worker
  // connections from outliving the object in an undefined order.
  for (auto & w : workers_) {
    w.join();
  }
}

PrefetchedSlice SliceScanner::get(std::size_t index)
{
  std::unique_lock lock(mutex_);
  cv_.wait(lock, [&] { return cancel_ || ready_.count(index) != 0; });
  if (cancel_ && ready_.count(index) == 0) {
    PrefetchedSlice out;
    out.error = worker_fatal_error_.empty() ? "db3 slice prefetch cancelled" : worker_fatal_error_;
    return out;
  }
  auto node = ready_.extract(index);
  consumed_ = index + 1;
  lock.unlock();
  cv_.notify_all();
  PrefetchedSlice out = std::move(node.mapped());
  return out;
}

void SliceScanner::recycle(std::vector<std::byte> && blobs, std::vector<SliceRecord> && records)
{
  const std::lock_guard lock(mutex_);
  if (blobs.capacity() != 0) {
    blob_pool_.push_back(std::move(blobs));
  }
  if (records.capacity() != 0) {
    record_pool_.push_back(std::move(records));
  }
}

std::vector<std::byte> SliceScanner::take_pooled_blobs()
{
  const std::lock_guard lock(mutex_);
  if (blob_pool_.empty()) {
    return {};
  }
  std::vector<std::byte> buf = std::move(blob_pool_.back());
  blob_pool_.pop_back();
  return buf;
}

std::vector<SliceRecord> SliceScanner::take_pooled_records()
{
  const std::lock_guard lock(mutex_);
  if (record_pool_.empty()) {
    return {};
  }
  std::vector<SliceRecord> buf = std::move(record_pool_.back());
  record_pool_.pop_back();
  return buf;
}

void SliceScanner::worker_loop()
{
  // One connection per worker: the point of the whole exercise is N
  // independent scans building device queue depth. jthread bodies must not
  // throw, so a failed open cancels the run instead of letting it escape.
  SqlitePtr db;
  try {
    db = sqlite_open_or_throw(
      path_.string(), SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX, "sqlite3 slice open");
  } catch (const std::exception & e) {
    const std::lock_guard lock(mutex_);
    cancel_ = true;
    if (worker_fatal_error_.empty()) {
      worker_fatal_error_ = e.what();
    }
    cv_.notify_all();
    return;
  }
  // Same read-only streaming tuning the serial reader applies; failures are
  // non-fatal (best-effort).
  sqlite3_exec(db.get(), "PRAGMA query_only = 1;", nullptr, nullptr, nullptr);
  sqlite3_exec(db.get(), "PRAGMA mmap_size = 268435456;", nullptr, nullptr, nullptr);
  sqlite3_exec(db.get(), "PRAGMA cache_size = -65536;", nullptr, nullptr, nullptr);

  while (true) {
    std::size_t index = 0;
    {
      std::unique_lock lock(mutex_);
      cv_.wait(lock, [&] {
        return cancel_ || (next_claim_ < schedule_.size() && next_claim_ < consumed_ + lookahead_);
      });
      if (cancel_) {
        return;
      }
      index = next_claim_++;
    }

    PrefetchedSlice result;
    result.blobs = take_pooled_blobs();
    result.records = take_pooled_records();
    result.blobs.clear();
    result.records.clear();
    try {
      auto stmt = sqlite_prepare_or_throw(db.get(), slice_sql(spec_, schedule_[index]));
      for (;;) {
        const int rc = sqlite3_step(stmt.get());
        if (rc == SQLITE_DONE) {
          break;
        }
        if (rc != SQLITE_ROW) {
          result.error = "sqlite3_step failed: " + sqlite_errmsg(db.get());
          break;
        }
        const std::int64_t topic_id = sqlite3_column_int64(stmt.get(), 0);
        if (spec_.known_topic_ids.count(topic_id) == 0) {
          continue;  // row references a topic_id not in the topics table
        }
        SliceRecord rec;
        rec.topic_id = topic_id;
        rec.timestamp_ns = sqlite3_column_int64(stmt.get(), 1);
        rec.offset = result.blobs.size();
        if (!spec_.payload_filter_active || spec_.payload_topic_ids.count(topic_id) != 0) {
          // The payload has to be copied: workers run ahead of the consumer,
          // so SQLite's row buffer (valid only until the next step) cannot be
          // handed out.
          const void * data = sqlite3_column_blob(stmt.get(), 2);
          const int data_size = sqlite3_column_bytes(stmt.get(), 2);
          if (data_size > 0) {
            const auto * src = reinterpret_cast<const std::byte *>(data);
            result.blobs.insert(result.blobs.end(), src, src + static_cast<std::size_t>(data_size));
            rec.size = static_cast<std::size_t>(data_size);
          }
        }
        result.records.push_back(rec);
      }
    } catch (const std::exception & e) {
      result.error = std::string("db3 slice prefetch worker exception: ") + e.what();
    }

    {
      const std::lock_guard lock(mutex_);
      ready_.emplace(index, std::move(result));
    }
    cv_.notify_all();
  }
}

}  // namespace bagwiz::io::detail
