// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/sqlite3_writer.hpp"

#include "bagwiz/core/logging.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <sqlite3.h>
#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bagwiz::io::detail
{

namespace
{
constexpr const char * kLogger = "bagwiz.io.sqlite3";
// Conservative batch size: keep transactions small enough that a crash loses
// at most a bounded number of messages but large enough to amortize commit
// overhead.
constexpr int kBatchSize = 1024;

std::string sqlite_errmsg(sqlite3 * db)
{
  return db ? std::string(sqlite3_errmsg(db)) : "<null db>";
}

void exec_or_throw(sqlite3 * db, const char * sql)
{
  char * err = nullptr;
  const int rc = sqlite3_exec(db, sql, nullptr, nullptr, &err);
  if (rc != SQLITE_OK) {
    std::string msg = err ? err : "<no message>";
    sqlite3_free(err);
    throw std::runtime_error(std::string("sqlite exec failed: ") + msg + " (sql=" + sql + ")");
  }
  sqlite3_free(err);
}

// ---------------------------------------------------------------------------
// Single .db3 file writer.
// ---------------------------------------------------------------------------
class SqliteFileWriter : public BagWriter
{
public:
  explicit SqliteFileWriter(const std::filesystem::path & path)
  {
    const int rc = sqlite3_open_v2(
      path.string().c_str(), &db_, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_NOMUTEX,
      nullptr);
    if (rc != SQLITE_OK) {
      const std::string msg = sqlite_errmsg(db_);
      if (db_) {
        sqlite3_close(db_);
        db_ = nullptr;
      }
      throw std::runtime_error("sqlite3 open failed for " + path.string() + ": " + msg);
    }

    // Write-side tuning. journal_mode=MEMORY keeps crash-consistency at the
    // cost of some durability; OFF would be faster but leaves a corrupt bag
    // on crash. bagwiz writes new bags so losing one on crash is acceptable,
    // but MEMORY is the better default.
    exec_or_throw(db_, "PRAGMA journal_mode = MEMORY;");
    exec_or_throw(db_, "PRAGMA synchronous = OFF;");
    exec_or_throw(db_, "PRAGMA temp_store = MEMORY;");
    exec_or_throw(db_, "PRAGMA cache_size = -65536;");

    create_schema();
    prepare_insert_stmt();
    begin_transaction();
  }

  ~SqliteFileWriter() override
  {
    if (!closed_) {
      try {
        close();
      } catch (const std::exception & e) {
        BAGWIZ_LOG_WARN(kLogger, "SqliteFileWriter close failed: %s", e.what());
      } catch (...) {
        // Never throw from destructor.
      }
    }
    if (insert_stmt_) {
      sqlite3_finalize(insert_stmt_);
      insert_stmt_ = nullptr;
    }
    if (db_) {
      sqlite3_close(db_);
      db_ = nullptr;
    }
  }

  SqliteFileWriter(const SqliteFileWriter &) = delete;
  SqliteFileWriter & operator=(const SqliteFileWriter &) = delete;
  SqliteFileWriter(SqliteFileWriter &&) = delete;
  SqliteFileWriter & operator=(SqliteFileWriter &&) = delete;

  void declare_topic(const TopicInfo & topic) override
  {
    if (topic_to_id_.count(topic.name) != 0U) {
      return;  // already declared
    }

    sqlite3_stmt * stmt = nullptr;
    const int rc = sqlite3_prepare_v2(
      db_,
      "INSERT INTO topics(name, type, serialization_format, offered_qos_profiles) "
      "VALUES (?, ?, ?, ?);",
      -1, &stmt, nullptr);
    if (rc != SQLITE_OK) {
      throw std::runtime_error("prepare topic insert failed: " + sqlite_errmsg(db_));
    }
    sqlite3_bind_text(stmt, 1, topic.name.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, topic.type.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 3, topic.serialization_format.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 4, topic.offered_qos_profiles.c_str(), -1, SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) != SQLITE_DONE) {
      const std::string msg = sqlite_errmsg(db_);
      sqlite3_finalize(stmt);
      throw std::runtime_error("topic insert failed: " + msg);
    }
    sqlite3_finalize(stmt);

    topic_to_id_[topic.name] = sqlite3_last_insert_rowid(db_);
  }

  void write(
    std::string_view topic, int64_t timestamp_ns, std::span<const std::byte> payload) override
  {
    auto it = topic_to_id_.find(std::string(topic));
    if (it == topic_to_id_.end()) {
      throw std::runtime_error(
        "sqlite3 write on undeclared topic: " + std::string(topic) +
        " (call declare_topic() first)");
    }

    sqlite3_bind_int64(insert_stmt_, 1, it->second);
    sqlite3_bind_int64(insert_stmt_, 2, timestamp_ns);
    sqlite3_bind_blob(
      insert_stmt_, 3, payload.data(), static_cast<int>(payload.size()), SQLITE_STATIC);
    if (sqlite3_step(insert_stmt_) != SQLITE_DONE) {
      throw std::runtime_error("message insert failed: " + sqlite_errmsg(db_));
    }
    sqlite3_reset(insert_stmt_);

    if (++pending_in_tx_ >= kBatchSize) {
      commit_transaction();
      begin_transaction();
      pending_in_tx_ = 0;
    }
  }

  void close() override
  {
    if (closed_) {
      return;
    }
    commit_transaction();
    closed_ = true;
  }

private:
  void create_schema()
  {
    exec_or_throw(
      db_,
      "CREATE TABLE IF NOT EXISTS schema("
      "  schema_version INTEGER PRIMARY KEY,"
      "  ros_distro TEXT);"
      "CREATE TABLE IF NOT EXISTS metadata("
      "  id INTEGER PRIMARY KEY,"
      "  metadata_version INTEGER,"
      "  metadata TEXT);"
      "CREATE TABLE IF NOT EXISTS topics("
      "  id INTEGER PRIMARY KEY,"
      "  name TEXT NOT NULL,"
      "  type TEXT NOT NULL,"
      "  serialization_format TEXT NOT NULL,"
      "  offered_qos_profiles TEXT NOT NULL);"
      "CREATE TABLE IF NOT EXISTS messages("
      "  id INTEGER PRIMARY KEY,"
      "  topic_id INTEGER NOT NULL,"
      "  timestamp INTEGER NOT NULL,"
      "  data BLOB NOT NULL);"
      "CREATE INDEX IF NOT EXISTS timestamp_idx ON messages (timestamp ASC);");

    // Record a schema version so downstream rosbag2 readers can negotiate.
    // Version 4 is the current stable schema in Humble/Jazzy.
    exec_or_throw(db_, "INSERT OR IGNORE INTO schema(schema_version, ros_distro) VALUES (4, '');");
  }

  void prepare_insert_stmt()
  {
    const int rc = sqlite3_prepare_v2(
      db_, "INSERT INTO messages(topic_id, timestamp, data) VALUES (?, ?, ?);", -1, &insert_stmt_,
      nullptr);
    if (rc != SQLITE_OK) {
      throw std::runtime_error("prepare message insert failed: " + sqlite_errmsg(db_));
    }
  }

  void begin_transaction() { exec_or_throw(db_, "BEGIN TRANSACTION;"); }

  void commit_transaction() { exec_or_throw(db_, "COMMIT;"); }

  sqlite3 * db_ = nullptr;
  sqlite3_stmt * insert_stmt_ = nullptr;
  std::unordered_map<std::string, int64_t> topic_to_id_;
  int pending_in_tx_ = 0;
  bool closed_ = false;
};

// ---------------------------------------------------------------------------
// Directory writer: single .db3 shard + metadata.yaml.
// ---------------------------------------------------------------------------
class SqliteDirectoryWriter : public BagWriter
{
public:
  SqliteDirectoryWriter(const std::filesystem::path & dir, const CreateOptions & options)
  : dir_(dir), options_(options)
  {
    std::filesystem::create_directories(dir);
    const auto stem = dir.filename().string();
    shard_rel_ = stem + "_0.db3";
    inner_ = std::make_unique<SqliteFileWriter>(dir_ / shard_rel_);
  }

  ~SqliteDirectoryWriter() override
  {
    if (!closed_) {
      try {
        close();
      } catch (const std::exception & e) {
        BAGWIZ_LOG_WARN(kLogger, "SqliteDirectoryWriter close failed: %s", e.what());
      } catch (...) {
      }
    }
  }

  SqliteDirectoryWriter(const SqliteDirectoryWriter &) = delete;
  SqliteDirectoryWriter & operator=(const SqliteDirectoryWriter &) = delete;
  SqliteDirectoryWriter(SqliteDirectoryWriter &&) = delete;
  SqliteDirectoryWriter & operator=(SqliteDirectoryWriter &&) = delete;

  void declare_topic(const TopicInfo & topic) override
  {
    inner_->declare_topic(topic);
    topics_.push_back(topic);
    topic_counts_[topic.name] = 0;
  }

  void write(
    std::string_view topic, int64_t timestamp_ns, std::span<const std::byte> payload) override
  {
    inner_->write(topic, timestamp_ns, payload);
    ++topic_counts_[std::string(topic)];
    ++total_messages_;
    if (timestamp_ns < start_ns_) {
      start_ns_ = timestamp_ns;
    }
    if (timestamp_ns > end_ns_) {
      end_ns_ = timestamp_ns;
    }
  }

  void close() override
  {
    if (closed_) {
      return;
    }
    inner_->close();
    write_metadata_yaml();
    closed_ = true;
  }

private:
  void write_metadata_yaml()
  {
    const int64_t duration_ns =
      (total_messages_ > 0 && end_ns_ >= start_ns_) ? (end_ns_ - start_ns_) : 0;
    const int64_t starting_ns = total_messages_ > 0 ? start_ns_ : 0;

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "rosbag2_bagfile_information";
    out << YAML::Value << YAML::BeginMap;

    out << YAML::Key << "version" << YAML::Value << 6;
    out << YAML::Key << "storage_identifier" << YAML::Value << "sqlite3";
    out << YAML::Key << "duration" << YAML::Value << YAML::BeginMap << YAML::Key << "nanoseconds"
        << YAML::Value << duration_ns << YAML::EndMap;
    out << YAML::Key << "starting_time" << YAML::Value << YAML::BeginMap << YAML::Key
        << "nanoseconds_since_epoch" << YAML::Value << starting_ns << YAML::EndMap;
    out << YAML::Key << "message_count" << YAML::Value << total_messages_;

    out << YAML::Key << "topics_with_message_count" << YAML::Value << YAML::BeginSeq;
    for (const auto & t : topics_) {
      const auto count_it = topic_counts_.find(t.name);
      const int64_t count = count_it != topic_counts_.end() ? count_it->second : 0;
      out << YAML::BeginMap;
      out << YAML::Key << "topic_metadata" << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "name" << YAML::Value << t.name;
      out << YAML::Key << "type" << YAML::Value << t.type;
      out << YAML::Key << "serialization_format" << YAML::Value << t.serialization_format;
      out << YAML::Key << "offered_qos_profiles" << YAML::Value << t.offered_qos_profiles;
      out << YAML::EndMap;
      out << YAML::Key << "message_count" << YAML::Value << count;
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;

    // sqlite3 storage does not have built-in compression; leave these empty.
    out << YAML::Key << "compression_format" << YAML::Value << "";
    out << YAML::Key << "compression_mode" << YAML::Value << "";
    out << YAML::Key << "relative_file_paths" << YAML::Value << YAML::BeginSeq;
    out << shard_rel_;
    out << YAML::EndSeq;

    out << YAML::EndMap;
    out << YAML::EndMap;

    std::ofstream f(dir_ / "metadata.yaml");
    if (!f) {
      throw std::runtime_error("failed to open metadata.yaml for writing in " + dir_.string());
    }
    f << out.c_str() << '\n';
  }

  std::filesystem::path dir_;
  CreateOptions options_;
  std::string shard_rel_;
  std::unique_ptr<SqliteFileWriter> inner_;

  std::vector<TopicInfo> topics_;
  std::unordered_map<std::string, int64_t> topic_counts_;
  int64_t total_messages_ = 0;
  int64_t start_ns_ = std::numeric_limits<int64_t>::max();
  int64_t end_ns_ = std::numeric_limits<int64_t>::min();
  bool closed_ = false;
};

}  // namespace

std::unique_ptr<BagWriter> create_sqlite3_file(
  const std::filesystem::path & path, const CreateOptions & options)
{
  (void)options;  // unused for sqlite3 (no format-specific options yet)
  return std::make_unique<SqliteFileWriter>(path);
}

std::unique_ptr<BagWriter> create_sqlite3_directory(
  const std::filesystem::path & dir, const CreateOptions & options)
{
  return std::make_unique<SqliteDirectoryWriter>(dir, options);
}

}  // namespace bagwiz::io::detail
