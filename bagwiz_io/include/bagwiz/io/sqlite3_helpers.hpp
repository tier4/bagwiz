// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__IO__SQLITE3_HELPERS_HPP_
#define BAGWIZ__IO__SQLITE3_HELPERS_HPP_

// Internal RAII wrappers around the raw SQLite C handles used by both
// `sqlite3_reader.cpp` and `sqlite3_writer.cpp`. Lives under `bagwiz::io::detail`
// to flag it as a private implementation detail of the SQLite backend even
// though the file ships with the installed headers.

#include <sqlite3.h>

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>

namespace bagwiz::io::detail
{

struct SqliteCloser
{
  void operator()(sqlite3 * db) const noexcept
  {
    if (db != nullptr) {
      sqlite3_close(db);
    }
  }
};
using SqlitePtr = std::unique_ptr<sqlite3, SqliteCloser>;

struct SqliteStmtFinalizer
{
  void operator()(sqlite3_stmt * stmt) const noexcept
  {
    if (stmt != nullptr) {
      sqlite3_finalize(stmt);
    }
  }
};
using SqliteStmtPtr = std::unique_ptr<sqlite3_stmt, SqliteStmtFinalizer>;

inline std::string sqlite_errmsg(sqlite3 * db)
{
  return db != nullptr ? std::string(sqlite3_errmsg(db)) : std::string("<null db>");
}

// Open a SQLite database with the given flags. Always returns a SqlitePtr
// that owns whatever handle sqlite3 allocated, even on error — sqlite3 may
// allocate the connection object before reporting failure, and that handle
// must still be closed.
inline SqlitePtr sqlite_open_or_throw(const std::string & path, int flags, const char * description)
{
  sqlite3 * raw = nullptr;
  const int rc = sqlite3_open_v2(path.c_str(), &raw, flags, nullptr);
  SqlitePtr db(raw);
  if (rc != SQLITE_OK) {
    throw std::runtime_error(
      std::string(description) + " failed for " + path + ": " + sqlite_errmsg(db.get()));
  }
  return db;
}

// Prepare a SQLite statement and return it wrapped in a SqliteStmtPtr.
// On failure, throws and the partially-allocated statement (if any) is
// finalized via the unique_ptr.
inline SqliteStmtPtr sqlite_prepare_or_throw(sqlite3 * db, std::string_view sql)
{
  sqlite3_stmt * raw = nullptr;
  const int rc = sqlite3_prepare_v2(db, sql.data(), static_cast<int>(sql.size()), &raw, nullptr);
  SqliteStmtPtr stmt(raw);
  if (rc != SQLITE_OK) {
    throw std::runtime_error(
      "sqlite3_prepare_v2 failed: " + sqlite_errmsg(db) + " (sql=" + std::string(sql) + ")");
  }
  return stmt;
}

}  // namespace bagwiz::io::detail

#endif  // BAGWIZ__IO__SQLITE3_HELPERS_HPP_
