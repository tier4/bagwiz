// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagcli/io/sqlite3_reader.hpp"

#include "bagcli/core/logging.hpp"
#include "bagcli/io/bag_io.hpp"
#include "bagcli/io/metadata_yaml.hpp"

#include <sqlite3.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bagcli::io::detail
{

namespace
{
constexpr const char * kLogger = "bagcli.io.sqlite3";

std::string sqlite_errmsg(sqlite3 * db)
{
  return db ? std::string(sqlite3_errmsg(db)) : "<null db>";
}

// RAII wrapper for sqlite3_stmt so prepare/step/finalize cannot leak a handle
// on exception paths.
class Statement
{
public:
  Statement(sqlite3 * db, std::string_view sql) : db_(db)
  {
    const int rc =
      sqlite3_prepare_v2(db, sql.data(), static_cast<int>(sql.size()), &stmt_, nullptr);
    if (rc != SQLITE_OK) {
      throw std::runtime_error(
        "sqlite3_prepare_v2 failed: " + sqlite_errmsg(db) + " (sql=" + std::string(sql) + ")");
    }
  }
  ~Statement()
  {
    if (stmt_) {
      sqlite3_finalize(stmt_);
    }
  }
  Statement(const Statement &) = delete;
  Statement & operator=(const Statement &) = delete;
  Statement(Statement && other) noexcept : db_(other.db_), stmt_(other.stmt_)
  {
    other.stmt_ = nullptr;
  }
  Statement & operator=(Statement && other) noexcept
  {
    if (this != &other) {
      if (stmt_) {
        sqlite3_finalize(stmt_);
      }
      db_ = other.db_;
      stmt_ = other.stmt_;
      other.stmt_ = nullptr;
    }
    return *this;
  }

  sqlite3_stmt * get() const { return stmt_; }

private:
  sqlite3 * db_ = nullptr;
  sqlite3_stmt * stmt_ = nullptr;
};

// ---------------------------------------------------------------------------
// Single .db3 file reader.
// ---------------------------------------------------------------------------
class SqliteFileReader : public BagReader
{
public:
  explicit SqliteFileReader(const std::filesystem::path & path) : path_(path)
  {
    const int rc = sqlite3_open_v2(
      path.string().c_str(), &db_, SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX, nullptr);
    if (rc != SQLITE_OK) {
      const std::string msg = sqlite_errmsg(db_);
      if (db_) {
        sqlite3_close(db_);
        db_ = nullptr;
      }
      throw std::runtime_error("sqlite3 open failed for " + path.string() + ": " + msg);
    }

    // Read-only streaming tuning. Failures are non-fatal (best-effort).
    sqlite3_exec(db_, "PRAGMA query_only = 1;", nullptr, nullptr, nullptr);
    sqlite3_exec(db_, "PRAGMA mmap_size = 268435456;", nullptr, nullptr, nullptr);
    sqlite3_exec(db_, "PRAGMA cache_size = -65536;", nullptr, nullptr, nullptr);

    populate_topics();
  }

  ~SqliteFileReader() override
  {
    if (read_stmt_) {
      sqlite3_finalize(read_stmt_);
      read_stmt_ = nullptr;
    }
    if (db_) {
      sqlite3_close(db_);
      db_ = nullptr;
    }
  }

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
    if (!read_stmt_) {
      return false;
    }

    for (;;) {
      const int rc = sqlite3_step(read_stmt_);
      if (rc == SQLITE_DONE) {
        return false;
      }
      if (rc != SQLITE_ROW) {
        throw std::runtime_error("sqlite3_step failed: " + sqlite_errmsg(db_));
      }

      const int64_t topic_id = sqlite3_column_int64(read_stmt_, 0);
      const int64_t timestamp = sqlite3_column_int64(read_stmt_, 1);
      const void * data = sqlite3_column_blob(read_stmt_, 2);
      const int data_size = sqlite3_column_bytes(read_stmt_, 2);

      auto idx_it = topic_id_to_idx_.find(topic_id);
      if (idx_it == topic_id_to_idx_.end()) {
        // Row references a topic_id not in the topics table; skip.
        continue;
      }

      out.topic = &topics_[idx_it->second];
      out.timestamp_ns = timestamp;
      out.payload = std::span<const std::byte>(
        reinterpret_cast<const std::byte *>(data), static_cast<std::size_t>(data_size));
      return true;
    }
  }

  Stats compute_stats() override
  {
    Stats stats;
    // SQLite does not keep a pre-computed summary; aggregates scan the
    // messages table (index-assisted but still O(n) for COUNT).
    stats.from_summary = false;

    Statement stmt(
      db_,
      "SELECT topic_id, COUNT(*), MIN(timestamp), MAX(timestamp) "
      "FROM messages GROUP BY topic_id");

    bool first = true;
    for (;;) {
      const int rc = sqlite3_step(stmt.get());
      if (rc == SQLITE_DONE) {
        break;
      }
      if (rc != SQLITE_ROW) {
        throw std::runtime_error("stats query failed: " + sqlite_errmsg(db_));
      }
      const int64_t topic_id = sqlite3_column_int64(stmt.get(), 0);
      const int64_t count = sqlite3_column_int64(stmt.get(), 1);
      const int64_t min_ts = sqlite3_column_int64(stmt.get(), 2);
      const int64_t max_ts = sqlite3_column_int64(stmt.get(), 3);

      auto idx_it = topic_id_to_idx_.find(topic_id);
      if (idx_it != topic_id_to_idx_.end()) {
        stats.per_topic[topics_[idx_it->second].name] = count;
      }
      stats.total_messages += count;
      if (first || min_ts < stats.start_ns) {
        stats.start_ns = min_ts;
      }
      if (first || max_ts > stats.end_ns) {
        stats.end_ns = max_ts;
      }
      first = false;
    }
    return stats;
  }

private:
  static const char * column_text_or_empty(sqlite3_stmt * stmt, int col)
  {
    const auto * text = sqlite3_column_text(stmt, col);
    return text ? reinterpret_cast<const char *>(text) : "";
  }

  void populate_topics()
  {
    Statement stmt(
      db_, "SELECT id, name, type, serialization_format, offered_qos_profiles FROM topics");
    for (;;) {
      const int rc = sqlite3_step(stmt.get());
      if (rc == SQLITE_DONE) {
        break;
      }
      if (rc != SQLITE_ROW) {
        throw std::runtime_error("topics query failed: " + sqlite_errmsg(db_));
      }
      const int64_t topic_id = sqlite3_column_int64(stmt.get(), 0);
      TopicInfo info;
      info.name = column_text_or_empty(stmt.get(), 1);
      info.type = column_text_or_empty(stmt.get(), 2);
      info.serialization_format = column_text_or_empty(stmt.get(), 3);
      info.offered_qos_profiles = column_text_or_empty(stmt.get(), 4);
      topic_id_to_idx_[topic_id] = topics_.size();
      topics_.push_back(std::move(info));
    }
  }

  void ensure_iterator()
  {
    if (iteration_started_) {
      return;
    }
    iteration_started_ = true;

    std::string sql = "SELECT topic_id, timestamp, data FROM messages";
    std::vector<std::string> where;

    if (!filter_.topics.empty()) {
      std::vector<int64_t> ids;
      for (const auto & name : filter_.topics) {
        for (const auto & [tid, idx] : topic_id_to_idx_) {
          if (topics_[idx].name == name) {
            ids.push_back(tid);
            break;
          }
        }
      }
      if (ids.empty()) {
        BAGCLI_LOG_WARN(
          kLogger, "no topic IDs matched the filter for %s; iteration will be empty",
          path_.c_str());
        return;
      }
      std::string clause = "topic_id IN (";
      for (std::size_t i = 0; i < ids.size(); ++i) {
        clause += (i == 0 ? "" : ",") + std::to_string(ids[i]);
      }
      clause += ")";
      where.push_back(std::move(clause));
    }
    if (filter_.start_ns) {
      where.push_back("timestamp >= " + std::to_string(*filter_.start_ns));
    }
    if (filter_.end_ns) {
      where.push_back("timestamp <= " + std::to_string(*filter_.end_ns));
    }

    if (!where.empty()) {
      sql += " WHERE ";
      for (std::size_t i = 0; i < where.size(); ++i) {
        sql += (i == 0 ? "" : " AND ") + where[i];
      }
    }
    sql += " ORDER BY timestamp";

    const int rc = sqlite3_prepare_v2(db_, sql.c_str(), -1, &read_stmt_, nullptr);
    if (rc != SQLITE_OK) {
      throw std::runtime_error("prepare read query failed: " + sqlite_errmsg(db_));
    }
  }

  std::filesystem::path path_;
  sqlite3 * db_ = nullptr;
  sqlite3_stmt * read_stmt_ = nullptr;
  std::vector<TopicInfo> topics_;
  std::unordered_map<int64_t, std::size_t> topic_id_to_idx_;
  ReadFilter filter_;
  bool iteration_started_ = false;
};

// ---------------------------------------------------------------------------
// Multi-shard SQLite3 reader: concatenates SqliteFileReaders in declared
// order. Matches rosbag2's monotonic shard ordering.
// ---------------------------------------------------------------------------
class SqliteShardReader : public BagReader
{
public:
  SqliteShardReader(
    std::vector<std::unique_ptr<SqliteFileReader>> shards, std::vector<TopicInfo> topics)
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
    combined.from_summary = false;
    bool first = true;
    for (auto & s : shards_) {
      auto st = s->compute_stats();
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
  std::vector<std::unique_ptr<SqliteFileReader>> shards_;
  std::vector<TopicInfo> topics_;
  std::size_t current_ = 0;
};

}  // namespace

std::unique_ptr<BagReader> open_sqlite3_file(const std::filesystem::path & path)
{
  return std::make_unique<SqliteFileReader>(path);
}

std::unique_ptr<BagReader> open_sqlite3_directory(const std::filesystem::path & dir)
{
  const auto metadata_path = dir / "metadata.yaml";
  auto md = load_metadata_yaml(metadata_path);

  std::vector<std::unique_ptr<SqliteFileReader>> shards;
  shards.reserve(md.relative_file_paths.size());
  for (const auto & rel : md.relative_file_paths) {
    shards.push_back(std::make_unique<SqliteFileReader>(dir / rel));
  }

  std::vector<TopicInfo> topics;
  if (!md.topics.empty()) {
    topics = std::move(md.topics);
  } else if (!shards.empty()) {
    auto shard_topics = shards.front()->topics();
    topics.assign(shard_topics.begin(), shard_topics.end());
  }

  return std::make_unique<SqliteShardReader>(std::move(shards), std::move(topics));
}

}  // namespace bagcli::io::detail
