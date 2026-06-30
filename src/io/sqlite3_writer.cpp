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
#include "bagwiz/io/sqlite3_helpers.hpp"

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
class SqliteFileWriter final : public BagWriter
{
public:
  explicit SqliteFileWriter(const std::filesystem::path & path)
  : db_(sqlite_open_or_throw(
      path.string(), SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_NOMUTEX,
      "sqlite3 open"))
  {
    // Write-side tuning. journal_mode=MEMORY keeps crash-consistency at the
    // cost of some durability; OFF would be faster but leaves a corrupt bag
    // on crash. bagwiz writes new bags so losing one on crash is acceptable,
    // but MEMORY is the better default.
    exec_or_throw(db_.get(), "PRAGMA journal_mode = MEMORY;");
    exec_or_throw(db_.get(), "PRAGMA synchronous = OFF;");
    exec_or_throw(db_.get(), "PRAGMA temp_store = MEMORY;");
    exec_or_throw(db_.get(), "PRAGMA cache_size = -65536;");

    create_schema();
    prepare_insert_stmt();
    begin_transaction();
  }

  ~SqliteFileWriter() override
  {
    if (!closed_) {
      try {
        SqliteFileWriter::close();
      } catch (const std::exception & e) {
        BAGWIZ_LOG_WARN(kLogger, "SqliteFileWriter close failed: %s", e.what());
      } catch (...) {
        // Never throw from destructor.
      }
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

    auto stmt = sqlite_prepare_or_throw(
      db_.get(),
      "INSERT INTO topics(name, type, serialization_format, offered_qos_profiles, "
      "type_description_hash) VALUES (?, ?, ?, ?, ?);");
    sqlite3_bind_text(stmt.get(), 1, topic.name.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt.get(), 2, topic.type.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt.get(), 3, topic.serialization_format.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt.get(), 4, topic.offered_qos_profiles.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt.get(), 5, topic.type_description_hash.c_str(), -1, SQLITE_TRANSIENT);
    if (sqlite3_step(stmt.get()) != SQLITE_DONE) {
      throw std::runtime_error("topic insert failed: " + sqlite_errmsg(db_.get()));
    }

    topic_to_id_[topic.name] = sqlite3_last_insert_rowid(db_.get());

    // Insert message_definitions row once per type (deduped). Iron+ rosbag2
    // readers query this table directly for self-description; the row is
    // optional (rows with empty topic_type are ignored by the upstream
    // reader, which simply skips encoded_message_definition lookup). We
    // emit a row whenever we have a non-empty schema_text so the bag stays
    // self-describing across a repack.
    if (!topic.schema_text.empty() && type_to_msgdef_id_.count(topic.type) == 0U) {
      auto def_stmt = sqlite_prepare_or_throw(
        db_.get(),
        "INSERT INTO message_definitions("
        "  topic_type, encoding, encoded_message_definition, type_description_hash) "
        "VALUES (?, ?, ?, ?);");
      const std::string encoding =
        topic.schema_encoding.empty() ? std::string("ros2msg") : topic.schema_encoding;
      sqlite3_bind_text(def_stmt.get(), 1, topic.type.c_str(), -1, SQLITE_TRANSIENT);
      sqlite3_bind_text(def_stmt.get(), 2, encoding.c_str(), -1, SQLITE_TRANSIENT);
      sqlite3_bind_text(def_stmt.get(), 3, topic.schema_text.c_str(), -1, SQLITE_TRANSIENT);
      sqlite3_bind_text(
        def_stmt.get(), 4, topic.type_description_hash.c_str(), -1, SQLITE_TRANSIENT);
      if (sqlite3_step(def_stmt.get()) != SQLITE_DONE) {
        throw std::runtime_error("message_definitions insert failed: " + sqlite_errmsg(db_.get()));
      }
      type_to_msgdef_id_[topic.type] = sqlite3_last_insert_rowid(db_.get());
    }
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

    sqlite3_bind_int64(insert_stmt_.get(), 1, it->second);
    sqlite3_bind_int64(insert_stmt_.get(), 2, timestamp_ns);
    sqlite3_bind_blob(
      insert_stmt_.get(), 3, payload.data(), static_cast<int>(payload.size()), SQLITE_STATIC);
    if (sqlite3_step(insert_stmt_.get()) != SQLITE_DONE) {
      throw std::runtime_error("message insert failed: " + sqlite_errmsg(db_.get()));
    }
    sqlite3_reset(insert_stmt_.get());

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
    // Build the (topic_id, timestamp) covering index in one bulk pass now that
    // every row is inserted, rather than maintaining it on each insert (which
    // would slow the write hot path). It lets `bagwiz ls -l` / compute_stats()
    // answer per-topic COUNT and MIN/MAX(timestamp) straight from the index,
    // without scanning the BLOB-laden messages rows. rosbag2 itself never
    // creates this index but readers ignore the extra one, so round-trips stay
    // compatible. See SqliteFileReader::compute_stats() for the read side.
    exec_or_throw(
      db_.get(),
      "CREATE INDEX IF NOT EXISTS topic_timestamp_idx ON messages (topic_id, timestamp);");
    closed_ = true;
  }

private:
  void create_schema()
  {
    // Mirrors the rosbag2 sqlite3 plugin schema_version=4 layout
    // (Iron / Jazzy):
    //   - `topics` carries `type_description_hash` so the bag advertises
    //     its RIHS type identity to readers that care.
    //   - `message_definitions` stores per-type self-description so
    //     readers don't need a local typestore overlay sourced.
    // Older rosbag2 readers (Humble v3) tolerate the extra column and
    // unused table — they only SELECT the columns they know about. We
    // never write the v3-shaped schema; callers that need true v3
    // compatibility can decompose with rosbag2's reindex tool.
    exec_or_throw(
      db_.get(),
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
      "  offered_qos_profiles TEXT NOT NULL,"
      "  type_description_hash TEXT NOT NULL DEFAULT '');"
      "CREATE TABLE IF NOT EXISTS message_definitions("
      "  id INTEGER PRIMARY KEY,"
      "  topic_type TEXT NOT NULL,"
      "  encoding TEXT NOT NULL,"
      "  encoded_message_definition TEXT NOT NULL,"
      "  type_description_hash TEXT NOT NULL DEFAULT '');"
      "CREATE TABLE IF NOT EXISTS messages("
      "  id INTEGER PRIMARY KEY,"
      "  topic_id INTEGER NOT NULL,"
      "  timestamp INTEGER NOT NULL,"
      "  data BLOB NOT NULL);"
      "CREATE INDEX IF NOT EXISTS timestamp_idx ON messages (timestamp ASC);");

    // Iron / Jazzy schema_version. Older readers ignore unknown values.
    exec_or_throw(
      db_.get(), "INSERT OR IGNORE INTO schema(schema_version, ros_distro) VALUES (4, '');");
  }

  void prepare_insert_stmt()
  {
    insert_stmt_ = sqlite_prepare_or_throw(
      db_.get(), "INSERT INTO messages(topic_id, timestamp, data) VALUES (?, ?, ?);");
  }

  void begin_transaction() { exec_or_throw(db_.get(), "BEGIN TRANSACTION;"); }

  void commit_transaction() { exec_or_throw(db_.get(), "COMMIT;"); }

  SqlitePtr db_;
  SqliteStmtPtr insert_stmt_;
  std::unordered_map<std::string, int64_t> topic_to_id_;
  // Tracks message_definitions rows already written, keyed by ROS 2 type
  // name. Each type gets exactly one row regardless of how many topics
  // share it.
  std::unordered_map<std::string, int64_t> type_to_msgdef_id_;
  int pending_in_tx_ = 0;
  bool closed_ = false;
};

// ---------------------------------------------------------------------------
// Directory writer: single .db3 shard + metadata.yaml.
// ---------------------------------------------------------------------------
class SqliteDirectoryWriter final : public BagWriter
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
        SqliteDirectoryWriter::close();
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

    // rosbag2_storage::BagMetadata defaults `version` to 5 on humble, and 5 is
    // the minimum version where rosbag2 considers `files:` part of the schema.
    // Lower versions (1-4) still parse but lose per-file timing; humble's
    // writer emits 5 by default so we match that.
    out << YAML::Key << "version" << YAML::Value << 5;
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

    // `files:` is required for metadata version >= 5; rosbag2's reader throws
    // "invalid node; first invalid key: \"files\"" without it. Single shard,
    // so the per-file timing equals the bag-level summary.
    out << YAML::Key << "files" << YAML::Value << YAML::BeginSeq;
    out << YAML::BeginMap;
    out << YAML::Key << "path" << YAML::Value << shard_rel_;
    out << YAML::Key << "starting_time" << YAML::Value << YAML::BeginMap << YAML::Key
        << "nanoseconds_since_epoch" << YAML::Value << starting_ns << YAML::EndMap;
    out << YAML::Key << "duration" << YAML::Value << YAML::BeginMap << YAML::Key << "nanoseconds"
        << YAML::Value << duration_ns << YAML::EndMap;
    out << YAML::Key << "message_count" << YAML::Value << total_messages_;
    out << YAML::EndMap;
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
